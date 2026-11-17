/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  Supervisor running on the ESP32-S31 LP core (RV32IMAC, 40 MHz from the
  crystal), from LP RAM. It keeps running while the HP cores are stalled by a
  flash operation or wedged, and costs the HP threads one store each. It
  only touches LP RAM, unless PC sampling is enabled.

  Every millisecond it reads the heartbeat counters the HP threads store in
  LP RAM and measures for each the time without progress:
  - statistics: longest gap since the start and per window (the HP closes a
    window each second), number of gaps longer than the channel threshold;
  - events: each gap longer than the threshold, with the main loop context
    (scheduler task, MAVLink command) and, with PC sampling, the PC of both
    HP cores, taken when the gap passed the threshold;
  - watchdog: when a vital channel (the main loop) makes no progress for
    wdt_ms, outside an expected delay of the HP, it writes a post-mortem to
    RTC no-init memory and raises reset_req: the HP supervisor task, on the
    other core, aborts (panic reboot). The HP reports the post-mortem after
    the reboot.
 */

#include <stdint.h>
#include <stdbool.h>
#include "sdkconfig.h"
#include "soc/soc.h"
#include "soc/bus_monitor_reg.h"
#include "ulp_lp_core_utils.h"
#include "ulp_lp_core_cpu_freq_shared.h"
#include "lpsv_shared.h"

#define TICK_CYCLES   (LP_CORE_CPU_FREQUENCY_HZ / 1000U)
#define CYCLES_PER_US (LP_CORE_CPU_FREQUENCY_HZ / 1000000U)

// shared with the HP: zeroed by the loader, then configured by the HP
lpsv_shared_t lpsv;

static uint32_t last_hb[LPSV_CH_MAX];
static uint32_t last_change[LPSV_CH_MAX];
static uint8_t seen[LPSV_CH_MAX];
static uint8_t in_stall[LPSV_CH_MAX];
static lpsv_event_t open_ev[LPSV_CH_MAX];

// PC samples of a vital channel stall, for the post-mortem
static uint32_t pm_pc[2][LPSV_PM_PCS];
static uint32_t pm_n;
static uint32_t pm_next_ms;

static uint32_t expect_seen, expect_start, expect_len;
static uint32_t clear_seen;

static inline uint32_t rd32(uint32_t addr)
{
    return *(volatile uint32_t *)addr;
}

// PC of an HP core, recorded in real time by the bus monitor (enabled by
// the bootloader for the reset reason report). An HP peripheral: read only
// with PC sampling enabled
static inline uint32_t hp_pc(int core)
{
    if (!lpsv.pc_sampling) {
        return 0;
    }
    return rd32(core ? BUS_MONITOR_CORE_1_RCD_PDEBUGPC_REG : BUS_MONITOR_CORE_0_RCD_PDEBUGPC_REG);
}

// an HP core halted by a debugger: no reset (known only with PC sampling)
static inline bool hp_debugging(void)
{
    if (!lpsv.pc_sampling) {
        return false;
    }
    return ((rd32(BUS_MONITOR_CORE_0_DEBUG_MODE_REG) | rd32(BUS_MONITOR_CORE_1_DEBUG_MODE_REG))
            & BUS_MONITOR_CORE_0_DEBUG_MODE) != 0;
}

static void publish(const lpsv_event_t *e)
{
    const uint32_t h = lpsv.ev_head;
    lpsv_event_t *d = &lpsv.ev[h & (LPSV_EV_N - 1)];
    d->t_ms = e->t_ms;
    d->ch = e->ch;
    d->flags = e->flags;
    d->gap_ms = e->gap_ms;
    d->task = e->task;
    d->mavcmd = e->mavcmd;
    d->pc[0] = e->pc[0];
    d->pc[1] = e->pc[1];
    __sync_synchronize();
    lpsv.ev_head = h + 1;
}

static bool expecting(uint32_t now)
{
    return expect_len != 0 && (now - expect_start) < expect_len;
}

__attribute__((noreturn)) static void fire(uint32_t ch, uint32_t now)
{
    lpsv_postmortem_t *pm = (lpsv_postmortem_t *)lpsv.pm_addr;
    if (pm != 0) {
        pm->magic = 0;
        __sync_synchronize();
        pm->version = LPSV_VERSION;
        pm->reason = LPSV_PM_REASON_STALL;
        pm->ch = ch;
        pm->gap_ms = now - last_change[ch];
        pm->uptime_ms = now;
        pm->task = open_ev[ch].task;
        pm->task_now = lpsv.hp_task;
        pm->mavcmd = lpsv.hp_mavcmd;
        pm->mavmsg = lpsv.hp_mavmsg;
        // oldest sample first
        const uint32_t n = pm_n < LPSV_PM_PCS ? pm_n : LPSV_PM_PCS;
        pm->n_pc = n;
        for (uint32_t i = 0; i < LPSV_PM_PCS; i++) {
            const uint32_t k = (pm_n - n + i) % LPSV_PM_PCS;
            pm->pc[0][i] = i < n ? pm_pc[0][k] : 0;
            pm->pc[1][i] = i < n ? pm_pc[1][k] : 0;
        }
        for (uint32_t c = 0; c < LPSV_CH_MAX; c++) {
            pm->age_ms[c] = (lpsv.warn_ms[c] != 0 && seen[c]) ? now - last_change[c] : UINT32_MAX;
        }
        pm->magic = LPSV_PM_MAGIC;
        pm->sum = lpsv_pm_sum(pm);
        __sync_synchronize();
    }
    lpsv.state = LPSV_STATE_RESETTING;
    __sync_synchronize();

    // the HP supervisor task aborts; the HP reloads this program after the
    // reboot
    lpsv.reset_req = 1;
    ulp_lp_core_halt();
}

static void tick(uint32_t now)
{
    // expected delay of the main loop: seq, length, seq again
    const uint32_t seq = lpsv.expect_seq;
    if (seq != expect_seen) {
        const uint32_t ms = lpsv.expect_ms;
        __sync_synchronize();
        if (lpsv.expect_seq == seq) {
            expect_seen = seq;
            expect_len = ms;
            expect_start = now;
        }
    }

    // statistics window closed by the HP
    const uint32_t cseq = lpsv.clear_seq;
    if (cseq != clear_seen) {
        clear_seen = cseq;
        for (uint32_t ch = 0; ch < LPSV_CH_MAX; ch++) {
            lpsv.ch[ch].wmax_prev_ms = lpsv.ch[ch].wmax_ms;
            lpsv.ch[ch].wmax_ms = 0;
        }
        __sync_synchronize();
        lpsv.clear_ack = cseq;
    }

    const uint32_t vital = lpsv.vital_mask;
    bool vital_stall = false;

    for (uint32_t ch = 0; ch < LPSV_CH_MAX; ch++) {
        const uint32_t warn = lpsv.warn_ms[ch];
        if (warn == 0) {
            continue;
        }
        const uint32_t v = lpsv.hb[ch];
        lpsv_ch_stats_t *st = &lpsv.ch[ch];
        if (!seen[ch]) {
            // a channel is supervised from its first heartbeat
            if (v != 0) {
                seen[ch] = 1;
                last_hb[ch] = v;
                last_change[ch] = now;
            }
            continue;
        }
        const uint32_t age = now - last_change[ch];
        if (v != last_hb[ch]) {
            last_hb[ch] = v;
            last_change[ch] = now;
            st->age_ms = 0;
            if (in_stall[ch]) {
                in_stall[ch] = 0;
                open_ev[ch].gap_ms = age;
                publish(&open_ev[ch]);
            }
        } else {
            st->age_ms = age;
            if (!in_stall[ch] && age >= warn) {
                in_stall[ch] = 1;
                st->stalls++;
                lpsv_event_t *e = &open_ev[ch];
                e->t_ms = now;
                e->ch = ch;
                e->flags = expecting(now) ? LPSV_EVF_EXPECTED : 0;
                e->gap_ms = age;
                e->task = (int16_t)lpsv.hp_task;
                e->mavcmd = (uint16_t)lpsv.hp_mavcmd;
                e->pc[0] = hp_pc(0);
                e->pc[1] = hp_pc(1);
            }
        }
        if (age > st->max_ms) {
            st->max_ms = age;
        }
        if (age > st->wmax_ms) {
            st->wmax_ms = age;
        }

        if (!in_stall[ch] || (vital & (1U << ch)) == 0) {
            continue;
        }
        vital_stall = true;
        const uint32_t wdt = lpsv.wdt_ms;
        if (wdt == 0) {
            continue;
        }
        // the timeout runs from the last heartbeat or from the end of the
        // expected delay, whichever is later
        uint32_t ref = last_change[ch];
        if (expect_len != 0) {
            const uint32_t end = expect_start + expect_len;
            if ((int32_t)(end - ref) > 0) {
                ref = end;
            }
        }
        if ((int32_t)(now - ref) >= (int32_t)wdt && !hp_debugging()) {
            fire(ch, now);
        }
    }

    // PC samples while a vital channel is stalled, kept for the post-mortem
    if (vital_stall) {
        if ((int32_t)(now - pm_next_ms) >= 0) {
            const uint32_t k = pm_n % LPSV_PM_PCS;
            pm_pc[0][k] = hp_pc(0);
            pm_pc[1][k] = hp_pc(1);
            pm_n++;
            pm_next_ms = now + LPSV_PM_PC_EVERY_MS;
        }
    } else {
        pm_n = 0;
        pm_next_ms = now;
    }
}

int main(void)
{
    if (lpsv.version != LPSV_VERSION || lpsv.token == 0 || lpsv.token_addr == 0) {
        lpsv.state = LPSV_STATE_BADCFG;
        ulp_lp_core_halt();
    }
    const volatile uint32_t *token = (const volatile uint32_t *)lpsv.token_addr;
    lpsv.state = LPSV_STATE_RUNNING;

    uint32_t cyc_ref = ulp_lp_core_get_cpu_cycles();
    uint32_t now = 0;
    for (;;) {
        // next millisecond
        uint32_t c;
        while ((uint32_t)((c = ulp_lp_core_get_cpu_cycles()) - cyc_ref) < TICK_CYCLES) {
        }
        const uint32_t ticks = (c - cyc_ref) / TICK_CYCLES;
        if (ticks > 1) {
            lpsv.late_ticks++;
        }
        cyc_ref += ticks * TICK_CYCLES;
        now += ticks;
        lpsv.uptime_ms = now;
        lpsv.alive++;

        // the HP rebooted (the bootloader reloaded its RTC data): stop until
        // reloaded
        if (*token != lpsv.token) {
            lpsv.state = LPSV_STATE_HP_GONE;
            ulp_lp_core_halt();
        }

        tick(now);

        const uint32_t work_us = (ulp_lp_core_get_cpu_cycles() - c) / CYCLES_PER_US;
        if (work_us > lpsv.tick_max_us) {
            lpsv.tick_max_us = work_us;
        }
    }
    return 0;
}
