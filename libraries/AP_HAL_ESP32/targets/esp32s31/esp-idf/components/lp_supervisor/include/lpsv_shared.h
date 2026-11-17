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
  Memory shared by the HP cores and the LP core supervisor (ESP32-S31).

  Everything is in LP RAM, so the LP core never reads HP memory or HP
  peripherals: the HP threads store their heartbeat counters and the main
  loop context there (lpsv_shared_t, the ULP variable "lpsv"), the LP core
  keeps the statistics and the event ring next to them, and writes the
  post-mortem of a watchdog reset to RTC no-init memory, which survives the
  reset.

  This header is compiled by the LP program (C), by the HP loader (C) and
  by the ArduPilot HAL (C++): plain C types only.
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LPSV_VERSION 2

// supervised HP activities: one heartbeat counter each
enum {
    LPSV_CH_MAIN = 0,   // vehicle main loop (APM_MAIN), once per loop
    LPSV_CH_TIMER,      // APM_TIMER, 1 kHz
    LPSV_CH_UART,       // APM_UART, 1 kHz
    LPSV_CH_RCIN,       // APM_RCIN
    LPSV_CH_RCOUT,      // APM_RCOUT
    LPSV_CH_IO,         // APM_IO: SD mount, IO processes
    LPSV_CH_STORAGE,    // APM_STORAGE: parameter flash writes
    LPSV_CH_NAV,        // navigation filter thread (Scheduler::heartbeat(NAV_FILTER))
    LPSV_CH_IDLE0,      // idle task of HP core 0: longest time without idle
    LPSV_CH_IDLE1,      // idle task of HP core 1
    LPSV_CH_BUS0,       // device bus threads, in the order they start
    LPSV_CH_BUS1,
    LPSV_CH_BUS2,
    LPSV_CH_BUS3,
    LPSV_CH_MAX
};

#define LPSV_NUM_BUS_CH (LPSV_CH_MAX - LPSV_CH_BUS0)

// LP program state
enum {
    LPSV_STATE_STOPPED = 0,
    LPSV_STATE_RUNNING = 1,
    LPSV_STATE_BADCFG = 2,      // started without a valid configuration
    LPSV_STATE_HP_GONE = 3,     // the HP rebooted: waiting to be reloaded
    LPSV_STATE_RESETTING = 4,   // watchdog fired, reset requested to the HP
};

// event flags
#define LPSV_EVF_EXPECTED  (1U << 0)   // during an HP expect_delay_ms()

// a gap of one channel longer than its threshold, published when the
// heartbeat resumes. The PCs and the HP context are taken when the gap
// passes the threshold
typedef struct {
    uint32_t t_ms;          // LP uptime when the gap passed the threshold
    uint16_t ch;
    uint16_t flags;
    uint32_t gap_ms;        // whole gap
    int16_t task;           // AP_Scheduler task of the main loop (-1 none, -3 waiting IMU)
    uint16_t mavcmd;        // last MAVLink command being handled
    uint32_t pc[2];         // PC of HP core 0 and 1 (0 without PC sampling)
} lpsv_event_t;

#define LPSV_EV_N 16        // ring size (power of two)

typedef struct {
    volatile uint32_t max_ms;       // longest gap since the LP start
    volatile uint32_t wmax_ms;      // longest gap of the current window
    volatile uint32_t wmax_prev_ms; // longest gap of the window closed by the last clear
    volatile uint32_t stalls;       // gaps longer than warn_ms
    volatile uint32_t age_ms;       // time since the last heartbeat
} lpsv_ch_stats_t;

// LP RAM, the ULP variable "lpsv"
typedef struct {
    // set by the HP before the start
    uint32_t version;
    uint32_t token;             // value of *token_addr for this HP boot
    uint32_t token_addr;        // HP RTC data (LP RAM), reloaded at every HP boot
    uint32_t pm_addr;           // lpsv_postmortem_t * in RTC no-init memory
    uint32_t pc_sampling;       // read the HP core PCs from the bus monitor
    uint16_t warn_ms[LPSV_CH_MAX]; // gap counted as a stall; 0 = not supervised

    // written by the HP threads: one heartbeat counter per channel, one
    // writer each, and the main loop context copied by the HP supervisor task
    volatile uint32_t hb[LPSV_CH_MAX];
    volatile int32_t hp_task;       // persistent_data.scheduler_task
    volatile uint32_t hp_mavcmd;    // persistent_data.last_mavlink_cmd
    volatile uint32_t hp_mavmsg;    // persistent_data.last_mavlink_msgid

    // changed by the HP while running
    volatile uint32_t vital_mask;   // channels whose stall resets the system
    volatile uint32_t wdt_ms;       // stall of a vital channel that resets; 0 = never
    volatile uint32_t expect_seq;   // incremented with each expect_delay_ms()
    volatile uint32_t expect_ms;    // length of the expected delay, 0 = cancelled
    volatile uint32_t clear_seq;    // incremented to close a statistics window

    // written by the LP core
    volatile uint32_t reset_req;    // the watchdog fired: the HP aborts
    volatile uint32_t state;
    volatile uint32_t alive;        // ticks run
    volatile uint32_t uptime_ms;
    volatile uint32_t tick_max_us;  // longest LP tick work
    volatile uint32_t late_ticks;   // ticks started late (LP core held up)
    volatile uint32_t clear_ack;    // clear_seq of the last window closed
    lpsv_ch_stats_t ch[LPSV_CH_MAX];
    volatile uint32_t ev_head;      // events published
    lpsv_event_t ev[LPSV_EV_N];
} lpsv_shared_t;

#define LPSV_PM_MAGIC       0x4C50574Du  // "LPWM"
#define LPSV_PM_PCS         16
#define LPSV_PM_PC_EVERY_MS 50

enum {
    LPSV_PM_REASON_STALL = 1,   // a vital channel stalled longer than wdt_ms
};

// RTC no-init memory: written by the LP core just before the reset
typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t reason;
    uint32_t ch;
    uint32_t gap_ms;
    uint32_t uptime_ms;         // LP uptime at the reset
    int32_t task;               // scheduler task when the stall began
    int32_t task_now;           // and at the reset
    uint32_t mavcmd;
    uint32_t mavmsg;
    uint32_t n_pc;              // PC samples in pc[][], oldest first
    uint32_t pc[2][LPSV_PM_PCS];
    uint32_t age_ms[LPSV_CH_MAX];   // every channel at the reset, UINT32_MAX = not running
    uint32_t sum;
} lpsv_postmortem_t;

static inline uint32_t lpsv_pm_sum(const lpsv_postmortem_t *pm)
{
    const uint32_t *w = (const uint32_t *)pm;
    uint32_t s = 0x13579BDFu;
    for (uint32_t i = 0; i < (uint32_t)(sizeof(*pm) / 4) - 1; i++) {
        s = ((s << 5) | (s >> 27)) ^ w[i];
    }
    return s;
}

#ifdef __cplusplus
}
#endif
