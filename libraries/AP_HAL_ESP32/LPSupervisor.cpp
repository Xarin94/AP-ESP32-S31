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

#include "LPSupervisor.h"

#if HAL_ESP32_LP_SUPERVISOR

#include "sdkconfig.h"
#if !CONFIG_IDF_TARGET_ESP32S31
#error "HAL_ESP32_LP_SUPERVISOR needs the ESP32-S31 LP core"
#endif

#include <string.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Common/ExpandingString.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <GCS_MAVLink/GCS.h>
#include <lpsv.h>
#include "Scheduler.h"

#include "esp_freertos_hooks.h"
#include "esp_system.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// main loop stall that resets the system, with BRD_OPTIONS bit 0 set
#ifndef HAL_ESP32_LP_WDT_MS
#define HAL_ESP32_LP_WDT_MS 2000
#endif

// the LP core reads the PC of the HP cores from the bus monitor when a
// thread stalls (an HP peripheral: off until verified on the board)
#ifndef HAL_ESP32_LP_PC_SAMPLING
#define HAL_ESP32_LP_PC_SAMPLING 0
#endif

// a flag in NVS around the start of the LP program: if a start ever left
// the chip unresponsive (the USB serial cannot reset it while the
// application runs), the next boot skips the LP core and the board stays
// reachable; the boot after that tries again
#ifndef HAL_ESP32_LP_START_GUARD
#define HAL_ESP32_LP_START_GUARD 1
#endif
#define LP_START_CONFIRM_MS 10000

// main loop stall reported to the GCS
#ifndef HAL_ESP32_LP_MAIN_STALL_TEXT_MS
#define HAL_ESP32_LP_MAIN_STALL_TEXT_MS 200
#endif

using namespace ESP32;

extern const AP_HAL::HAL& hal;

lpsv_shared_t *LPSupervisor::_lp;
uint32_t LPSupervisor::_hb_count[LPSV_CH_MAX];
bool LPSupervisor::_start_failed;
bool LPSupervisor::_start_skipped;
bool LPSupervisor::_guard_pending;
uint32_t LPSupervisor::_start_ms;
bool LPSupervisor::_main_started;
char LPSupervisor::_bus_names[LPSV_NUM_BUS_CH][5];
uint8_t LPSupervisor::_num_bus;
uint32_t LPSupervisor::_expect_seq;
uint32_t LPSupervisor::_clear_seq;
uint32_t LPSupervisor::_ev_tail;
uint32_t LPSupervisor::_ev_lost;
uint32_t LPSupervisor::_last_ev_ms;
uint32_t LPSupervisor::_last_report_ms;
uint32_t LPSupervisor::_last_alive;
bool LPSupervisor::_lp_stopped_reported;
uint32_t LPSupervisor::_last_stall_text_ms;
lpsv_event_t LPSupervisor::_recent[RECENT_N];
uint8_t LPSupervisor::_recent_n;
bool LPSupervisor::_have_pm;
lpsv_postmortem_t LPSupervisor::_pm;
int LPSupervisor::_reset_reason;
uint8_t LPSupervisor::_pm_texts;
uint32_t LPSupervisor::_last_pm_text_ms;

// threshold of a gap counted as a stall, per channel (ms). The bus threads
// sleep up to the shortest period of their callbacks, at most 50 ms; the
// idle tasks measure how long a core runs without any idle time
static const uint16_t warn_ms[LPSV_CH_MAX] = {
    100,    // MAIN
    20,     // TIMER
    20,     // UART
    50,     // RCIN
    50,     // RCOUT
    500,    // IO
    500,    // STORAGE
    100,    // NAV
    500,    // IDLE0
    500,    // IDLE1
    60,     // BUS0
    60,     // BUS1
    60,     // BUS2
    60,     // BUS3
};
static_assert(LPSV_CH_MAX == 14, "update warn_ms and fixed_names");

static const char *const fixed_names[LPSV_CH_BUS0] = {
    "MAIN", "TIMR", "UART", "RCIN", "RCOU", "IO", "STOR", "NAV", "IDL0", "IDL1",
};

static bool idle_hook0(void)
{
    LPSupervisor::heartbeat(LPSV_CH_IDLE0);
    return true;
}

static bool idle_hook1(void)
{
    LPSupervisor::heartbeat(LPSV_CH_IDLE1);
    return true;
}

static const char *reset_reason_name(int reason)
{
    switch (reason) {
    case ESP_RST_POWERON: return "power on";
    case ESP_RST_EXT: return "external pin";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "interrupt watchdog";
    case ESP_RST_TASK_WDT: return "task watchdog";
    case ESP_RST_WDT: return "watchdog";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_USB: return "USB";
    case ESP_RST_JTAG: return "JTAG";
    case ESP_RST_CPU_LOCKUP: return "CPU lockup";
    default: return "other";
    }
}

static bool reset_reason_abnormal(int reason)
{
    return reason == ESP_RST_PANIC || reason == ESP_RST_INT_WDT || reason == ESP_RST_TASK_WDT ||
           reason == ESP_RST_WDT || reason == ESP_RST_BROWNOUT || reason == ESP_RST_CPU_LOCKUP;
}

void LPSupervisor::init()
{
    _have_pm = lpsv_take_postmortem(&_pm);
    _reset_reason = esp_reset_reason();
}

/*
  HP side of the watchdog, on the core that does not run the main loop, above
  the other tasks there: copies the main loop context for the LP core, and
  reboots when the LP core asks for it (through the panic handler, the path
  of the task watchdog). Wakes 50 times a second and does a few stores
 */
void LPSupervisor::supervisor_task(void *arg)
{
    const AP_HAL::Util::PersistentData &pd = hal.util->persistent_data;
    while (true) {
        _lp->hp_task = pd.scheduler_task;
        _lp->hp_mavcmd = pd.last_mavlink_cmd;
        _lp->hp_mavmsg = pd.last_mavlink_msgid;
        if (_lp->reset_req != 0) {
            esp_system_abort("LP watchdog: main loop stalled");
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// true if the LP program may be started; marks the start as pending
bool LPSupervisor::start_guard_allows()
{
#if HAL_ESP32_LP_START_GUARD
    nvs_flash_init();
    nvs_handle_t h;
    if (nvs_open("lpsv", NVS_READWRITE, &h) != ESP_OK) {
        return true;
    }
    uint8_t pending = 0;
    nvs_get_u8(h, "pending", &pending);
    // a pending start was never confirmed: skip it this time only
    nvs_set_u8(h, "pending", pending ? 0 : 1);
    nvs_commit(h);
    nvs_close(h);
    _guard_pending = pending == 0;
    return pending == 0;
#else
    return true;
#endif
}

void LPSupervisor::start_guard_confirm()
{
#if HAL_ESP32_LP_START_GUARD
    nvs_handle_t h;
    if (nvs_open("lpsv", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, "pending", 0);
        nvs_commit(h);
        nvs_close(h);
    }
#endif
    _guard_pending = false;
}

void LPSupervisor::main_loop_started()
{
    if (!start_guard_allows()) {
        _start_skipped = true;
        return;
    }
    _start_ms = AP_HAL::millis();
    lpsv_config_t cfg {};
    cfg.pc_sampling = HAL_ESP32_LP_PC_SAMPLING;
    for (uint8_t i = 0; i < LPSV_CH_MAX; i++) {
        cfg.warn_ms[i] = warn_ms[i];
    }
    if (!lpsv_start(&cfg)) {
        _start_failed = true;
        return;
    }
    _lp = lpsv_shared();
    esp_register_freertos_idle_hook_for_cpu(idle_hook0, 0);
    esp_register_freertos_idle_hook_for_cpu(idle_hook1, 1);
    xTaskCreatePinnedToCore(supervisor_task, "APM_LPSV", 2048, nullptr, configMAX_PRIORITIES - 1,
                            nullptr, HAL_ESP32_SLOWCPU);
    _main_started = true;
    update_watchdog();
}

// the main loop is vital: with BRD_OPTIONS bit 0 its stall resets the system
void LPSupervisor::update_watchdog()
{
    if (_lp == nullptr || !_main_started) {
        return;
    }
    _lp->vital_mask = 1U << LPSV_CH_MAIN;
    _lp->wdt_ms = AP_BoardConfig::watchdog_enabled() ? HAL_ESP32_LP_WDT_MS : 0;
}

uint8_t LPSupervisor::add_bus(const char *name)
{
    const uint8_t n = __atomic_fetch_add(&_num_bus, 1, __ATOMIC_RELAXED);
    if (n >= LPSV_NUM_BUS_CH) {
        return LPSV_CH_MAX;
    }
    strncpy(_bus_names[n], name, sizeof(_bus_names[n]) - 1);
    return LPSV_CH_BUS0 + n;
}

void LPSupervisor::expect_delay_ms(uint32_t ms)
{
    if (_lp == nullptr) {
        return;
    }
    _lp->expect_ms = ms;
    __sync_synchronize();
    _lp->expect_seq = ++_expect_seq;
}

const char *LPSupervisor::channel_name(uint8_t ch)
{
    if (ch < LPSV_CH_BUS0) {
        return fixed_names[ch];
    }
    if (ch < LPSV_CH_MAX && ch - LPSV_CH_BUS0 < _num_bus) {
        return _bus_names[ch - LPSV_CH_BUS0];
    }
    return "?";
}

const char *LPSupervisor::task_name(int16_t task)
{
    if (task == -1) {
        return "none";
    }
    if (task == -3) {
        return "wait_ins";
    }
    const AP_Scheduler *sched = AP_Scheduler::get_singleton();
    const char *name = sched != nullptr ? sched->task_name(task) : nullptr;
    return name != nullptr ? name : "?";
}

void LPSupervisor::handle_event(const lpsv_event_t &e)
{
    // recent events for @SYS/threads.txt, newest last
    if (_recent_n == RECENT_N) {
        memmove(&_recent[0], &_recent[1], sizeof(_recent[0]) * (RECENT_N - 1));
        _recent_n--;
    }
    _recent[_recent_n++] = e;

#if HAL_LOGGING_ENABLED
    if (AP_Logger::get_singleton() != nullptr) {
        // @LoggerMessage: LPEV
        // @Description: HP thread stall measured by the LP core supervisor
        // @Field: TimeUS: Time since system startup
        // @Field: I: supervised channel
        // @Field: Name: channel name
        // @Field: Gap: time without progress
        // @Field: Task: main loop scheduler task when the gap passed the threshold
        // @Field: Cmd: MAVLink command being handled
        // @Field: Flg: 1 = during an expected delay
        // @Field: PC0: PC of HP core 0 when the gap passed the threshold
        // @Field: PC1: PC of HP core 1
        AP::logger().Write("LPEV", "TimeUS,I,Name,Gap,Task,Cmd,Flg,PC0,PC1",
                           "s#-s-----", "F--C-----", "QBnIhHBII",
                           AP_HAL::micros64(), (uint8_t)e.ch, channel_name(e.ch),
                           (uint32_t)e.gap_ms, (int16_t)e.task, (uint16_t)e.mavcmd,
                           (uint8_t)e.flags, (uint32_t)e.pc[0], (uint32_t)e.pc[1]);
    }
#endif

    const uint32_t now_ms = AP_HAL::millis();
    if (e.ch == LPSV_CH_MAIN && e.gap_ms >= HAL_ESP32_LP_MAIN_STALL_TEXT_MS &&
        (e.flags & LPSV_EVF_EXPECTED) == 0 && now_ms - _last_stall_text_ms >= 2000) {
        _last_stall_text_ms = now_ms;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LP: main stall %u ms in %s",
                      (unsigned)e.gap_ms, task_name(e.task));
    }
}

void LPSupervisor::drain_events()
{
    while (true) {
        const uint32_t head = _lp->ev_head;
        if (head - _ev_tail > LPSV_EV_N) {
            _ev_lost += head - _ev_tail - LPSV_EV_N;
            _ev_tail = head - LPSV_EV_N;
        }
        if (_ev_tail == head) {
            return;
        }
        const volatile lpsv_event_t *src = &_lp->ev[_ev_tail & (LPSV_EV_N - 1)];
        lpsv_event_t e;
        e.t_ms = src->t_ms;
        e.ch = src->ch;
        e.flags = src->flags;
        e.gap_ms = src->gap_ms;
        e.task = src->task;
        e.mavcmd = src->mavcmd;
        e.pc[0] = src->pc[0];
        e.pc[1] = src->pc[1];
        __sync_synchronize();
        if (_lp->ev_head - _ev_tail > LPSV_EV_N) {
            // overwritten while being copied
            continue;
        }
        _ev_tail++;
        if (e.ch < LPSV_CH_MAX) {
            handle_event(e);
        }
    }
}

// statistics of the window closed by the previous update, then close the
// current one
void LPSupervisor::report_window()
{
#if HAL_LOGGING_ENABLED
    if (_lp->clear_ack == _clear_seq && _clear_seq != 0 && AP_Logger::get_singleton() != nullptr) {
        const uint64_t now_us = AP_HAL::micros64();
        for (uint8_t ch = 0; ch < LPSV_CH_MAX; ch++) {
            if (_hb_count[ch] == 0) {
                continue;
            }
            const lpsv_ch_stats_t &st = _lp->ch[ch];
            // @LoggerMessage: LPSV
            // @Description: HP thread gaps measured by the LP core supervisor, each second
            // @Field: TimeUS: Time since system startup
            // @Field: I: supervised channel
            // @Field: Name: channel name
            // @Field: WMax: longest gap in the last second
            // @Field: Max: longest gap since boot
            // @Field: NSt: gaps longer than the channel threshold since boot
            // @Field: Age: time since the last heartbeat
            AP::logger().Write("LPSV", "TimeUS,I,Name,WMax,Max,NSt,Age",
                               "s#-ss-s", "F--CC-C", "QBnIIII",
                               now_us, ch, channel_name(ch),
                               (uint32_t)st.wmax_prev_ms, (uint32_t)st.max_ms,
                               (uint32_t)st.stalls, (uint32_t)st.age_ms);
        }
    }
#endif
    _lp->clear_seq = ++_clear_seq;
}

void LPSupervisor::report_postmortem(uint32_t now_ms)
{
#if HAL_LOGGING_ENABLED
    // once a second, as the log may start long after the boot
    if (AP_Logger::get_singleton() != nullptr) {
        const uint32_t last = _pm.n_pc > 0 ? _pm.n_pc - 1 : 0;
        // @LoggerMessage: LPPM
        // @Description: the LP core watchdog reset the system before this boot
        // @Field: TimeUS: Time since system startup
        // @Field: Name: stalled channel
        // @Field: Gap: stall at the reset
        // @Field: Task: main loop scheduler task when the stall began
        // @Field: TaskR: scheduler task at the reset
        // @Field: Cmd: last MAVLink command
        // @Field: Msg: last MAVLink message
        // @Field: PC0: last PC sample of HP core 0
        // @Field: PC1: last PC sample of HP core 1
        AP::logger().Write("LPPM", "TimeUS,Name,Gap,Task,TaskR,Cmd,Msg,PC0,PC1",
                           "s-s------", "F-C------", "QnIhhHHII",
                           AP_HAL::micros64(), channel_name(_pm.ch), (uint32_t)_pm.gap_ms,
                           (int16_t)_pm.task, (int16_t)_pm.task_now,
                           (uint16_t)_pm.mavcmd, (uint16_t)_pm.mavmsg,
                           (uint32_t)_pm.pc[0][last], (uint32_t)_pm.pc[1][last]);
    }
#endif
    // to the GCS 10 s after the boot, then every 30 s, 5 times
    if (_pm_texts < 5 && now_ms > 10000 && (_pm_texts == 0 || now_ms - _last_pm_text_ms >= 30000)) {
        _pm_texts++;
        _last_pm_text_ms = now_ms;
        const uint32_t last = _pm.n_pc > 0 ? _pm.n_pc - 1 : 0;
        // STATUSTEXT holds 50 characters
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "LP wdog reset: %s %u ms in %s",
                      channel_name(_pm.ch), (unsigned)_pm.gap_ms, task_name(_pm.task));
        if (_pm.pc[0][last] != 0 || _pm.pc[1][last] != 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "LP wdog: cmd %u PC 0x%08x 0x%08x",
                          (unsigned)_pm.mavcmd, (unsigned)_pm.pc[0][last], (unsigned)_pm.pc[1][last]);
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "LP wdog: MAVLink cmd %u msg %u",
                          (unsigned)_pm.mavcmd, (unsigned)_pm.mavmsg);
        }
    }
}

void LPSupervisor::update()
{
    const uint32_t now_ms = AP_HAL::millis();

    if (_lp == nullptr) {
        if ((_start_failed || _start_skipped) && !_lp_stopped_reported && now_ms > 10000) {
            _lp_stopped_reported = true;
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, _start_skipped ?
                          "LP supervisor skipped: last start incomplete" :
                          "LP supervisor failed to start");
        }
        return;
    }

    // the start did not leave the chip unresponsive
    if (_guard_pending && now_ms - _start_ms > LP_START_CONFIRM_MS && _lp->alive > 0) {
        start_guard_confirm();
    }

    if (now_ms - _last_ev_ms >= 100) {
        _last_ev_ms = now_ms;
        drain_events();
    }

    if (now_ms - _last_report_ms < 1000) {
        return;
    }
    _last_report_ms = now_ms;

    // the LP core itself
    const uint32_t alive = _lp->alive;
    if (alive == _last_alive) {
        if (!_lp_stopped_reported) {
            _lp_stopped_reported = true;
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "LP supervisor stopped (state %u)", (unsigned)_lp->state);
        }
    } else {
        _lp_stopped_reported = false;
    }
    _last_alive = alive;

    report_window();
    update_watchdog();

    if (_have_pm) {
        report_postmortem(now_ms);
    } else if (reset_reason_abnormal(_reset_reason) && _pm_texts == 0 && now_ms > 10000) {
        _pm_texts = 1;
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Last reset: %s", reset_reason_name(_reset_reason));
    }
}

void LPSupervisor::thread_info(ExpandingString &str)
{
    if (_lp == nullptr) {
        str.printf("LP supervisor: %s\n", _start_failed ? "failed to start" :
                   _start_skipped ? "skipped, its last start did not complete" : "not started");
        return;
    }
    static const char *const states[] = { "stopped", "running", "bad config", "HP gone", "resetting" };
    const uint32_t state = _lp->state;
    const uint32_t wdt = _lp->wdt_ms;
    str.printf("LP supervisor: %s, up %u s, tick max %u us, late ticks %u, watchdog %s",
               state < ARRAY_SIZE(states) ? states[state] : "?",
               (unsigned)(_lp->uptime_ms / 1000), (unsigned)_lp->tick_max_us,
               (unsigned)_lp->late_ticks, wdt != 0 ? "on" : "off");
    if (wdt != 0) {
        str.printf(" (main loop %u ms)", (unsigned)wdt);
    }
    str.printf("\n%-5s %7s %7s %7s %6s %5s\n", "ch", "age", "last1s", "max", "stalls", "warn");
    for (uint8_t ch = 0; ch < LPSV_CH_MAX; ch++) {
        if (_hb_count[ch] == 0) {
            continue;
        }
        const lpsv_ch_stats_t &st = _lp->ch[ch];
        str.printf("%-5s %7u %7u %7u %6u %5u\n", channel_name(ch),
                   (unsigned)st.age_ms, (unsigned)st.wmax_prev_ms, (unsigned)st.max_ms,
                   (unsigned)st.stalls, (unsigned)_lp->warn_ms[ch]);
    }
    if (_recent_n > 0 || _ev_lost > 0) {
        str.printf("recent stalls (ms; lost %u):\n", (unsigned)_ev_lost);
        for (uint8_t i = 0; i < _recent_n; i++) {
            const lpsv_event_t &e = _recent[i];
            str.printf(" t=%u %s %u task %s cmd %u%s", (unsigned)e.t_ms, channel_name(e.ch),
                       (unsigned)e.gap_ms, task_name(e.task), (unsigned)e.mavcmd,
                       (e.flags & LPSV_EVF_EXPECTED) ? " expected" : "");
            if (e.pc[0] != 0 || e.pc[1] != 0) {
                str.printf(" pc 0x%08x 0x%08x", (unsigned)e.pc[0], (unsigned)e.pc[1]);
            }
            str.printf("\n");
        }
    }
    str.printf("last reset: %s\n", reset_reason_name(_reset_reason));
    if (_have_pm) {
        str.printf("LP watchdog: %s stalled %u ms, task %s (%d), at reset %s (%d), cmd %u, msg %u\n",
                   channel_name(_pm.ch), (unsigned)_pm.gap_ms,
                   task_name(_pm.task), (int)_pm.task, task_name(_pm.task_now), (int)_pm.task_now,
                   (unsigned)_pm.mavcmd, (unsigned)_pm.mavmsg);
        for (uint8_t core = 0; core < 2 && _pm.n_pc > 0 && (_pm.pc[0][0] | _pm.pc[1][0]) != 0; core++) {
            str.printf(" PC%u:", core);
            for (uint32_t i = 0; i < _pm.n_pc && i < LPSV_PM_PCS; i++) {
                str.printf(" %08x", (unsigned)_pm.pc[core][i]);
            }
            str.printf("\n");
        }
        str.printf(" ages:");
        for (uint8_t ch = 0; ch < LPSV_CH_MAX; ch++) {
            if (_pm.age_ms[ch] != UINT32_MAX) {
                str.printf(" %s=%u", channel_name(ch), (unsigned)_pm.age_ms[ch]);
            }
        }
        str.printf("\n");
    }
}

#endif // HAL_ESP32_LP_SUPERVISOR
