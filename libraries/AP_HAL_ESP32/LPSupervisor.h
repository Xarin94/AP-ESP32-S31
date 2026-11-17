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
  Supervision, watchdog and diagnostics of the HP threads from the ESP32-S31
  LP core (hwdef: define HAL_ESP32_LP_SUPERVISOR 1).

  The HP threads only store a counter in LP RAM; the LP core measures the
  gaps, keeps the statistics and the events, and when the main loop stops
  (HAL_ESP32_LP_WDT_MS, with BRD_OPTIONS bit 0) saves a post-mortem and asks
  for a reboot, which a small HP task on the other core carries out. The HP
  side reads the results once a second from the IO thread and writes them to
  the log (LPSV, LPEV, LPPM), to the GCS and to @SYS/threads.txt.
 */
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <stdint.h>

#ifndef HAL_ESP32_LP_SUPERVISOR
#define HAL_ESP32_LP_SUPERVISOR 0
#endif

#if HAL_ESP32_LP_SUPERVISOR

#include <lpsv_shared.h>

class ExpandingString;

namespace ESP32 {

class LPSupervisor
{
public:
    // takes the post-mortem of the last reset. Called by Scheduler::init()
    static void init();

    // after setup(): load and start the LP program, supervise the main loop
    // and arm the watchdog
    static void main_loop_started();

    // one store in LP RAM; each channel has a single writer thread
    static inline void heartbeat(uint8_t ch)
    {
        if (_lp != nullptr && ch < LPSV_CH_MAX) {
            _lp->hb[ch] = ++_hb_count[ch];
        }
    }

    // channel of a device bus thread, LPSV_CH_MAX if none is left
    static uint8_t add_bus(const char *name);

    // expected delay of the main loop (Scheduler::expect_delay_ms), 0 = none
    static void expect_delay_ms(uint32_t ms);

    // IO thread: events at 10 Hz, statistics and reports at 1 Hz
    static void update();

    // text for @SYS/threads.txt
    static void thread_info(ExpandingString &str);

private:
    static const char *channel_name(uint8_t ch);
    static const char *task_name(int16_t task);
    static void update_watchdog();
    static void drain_events();
    static void handle_event(const lpsv_event_t &e);
    static void report_window();
    static void report_postmortem(uint32_t now_ms);
    static void supervisor_task(void *arg);
    static bool start_guard_allows();
    static void start_guard_confirm();

    static lpsv_shared_t *_lp;
    static uint32_t _hb_count[LPSV_CH_MAX];
    static bool _start_failed;
    static bool _start_skipped;
    static bool _guard_pending;
    static uint32_t _start_ms;
    static bool _main_started;

    static char _bus_names[LPSV_NUM_BUS_CH][5];
    static uint8_t _num_bus;

    static uint32_t _expect_seq;
    static uint32_t _clear_seq;
    static uint32_t _ev_tail;
    static uint32_t _ev_lost;
    static uint32_t _last_ev_ms;
    static uint32_t _last_report_ms;
    static uint32_t _last_alive;
    static bool _lp_stopped_reported;
    static uint32_t _last_stall_text_ms;

    // recent events for @SYS/threads.txt
    static const uint8_t RECENT_N = 6;
    static lpsv_event_t _recent[RECENT_N];
    static uint8_t _recent_n;

    // the last reset
    static bool _have_pm;
    static lpsv_postmortem_t _pm;
    static int _reset_reason;
    static uint8_t _pm_texts;
    static uint32_t _last_pm_text_ms;
};

} // namespace ESP32

#endif // HAL_ESP32_LP_SUPERVISOR
