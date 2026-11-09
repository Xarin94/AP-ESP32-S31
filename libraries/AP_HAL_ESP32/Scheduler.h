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

#pragma once
#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>
#include "HAL_ESP32_Namespace.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
  core assignment of the HAL threads, overridable from hwdef.dat:
  - FASTCPU: main loop, timer and UART threads
  - SLOWCPU: Wi-Fi, IO, storage and RC threads (no task watchdog)
  - DEVICE_BUS_CPU: SPI/I2C bus threads, which run the sensor drivers
  - THREAD_CPU: threads created by libraries (DroneCAN, logging, ...)
 */
#ifndef HAL_ESP32_FASTCPU
#define HAL_ESP32_FASTCPU 0
#endif
#ifndef HAL_ESP32_SLOWCPU
#define HAL_ESP32_SLOWCPU 1
#endif
#ifndef HAL_ESP32_DEVICE_BUS_CPU
#define HAL_ESP32_DEVICE_BUS_CPU tskNO_AFFINITY
#endif
#ifndef HAL_ESP32_THREAD_CPU
#define HAL_ESP32_THREAD_CPU tskNO_AFFINITY
#endif
// core of the CAN (DroneCAN, CAN sensors) and IO-class (logger, compass
// calibration, OSD...) library threads, by default with the other ones
#ifndef HAL_ESP32_CAN_THREAD_CPU
#define HAL_ESP32_CAN_THREAD_CPU HAL_ESP32_THREAD_CPU
#endif
#ifndef HAL_ESP32_IO_THREAD_CPU
#define HAL_ESP32_IO_THREAD_CPU HAL_ESP32_THREAD_CPU
#endif

#define ESP32_SCHEDULER_MAX_TIMER_PROCS 10
#define ESP32_SCHEDULER_MAX_IO_PROCS 10
#define TWDT_TIMEOUT_MS 3000

/* Scheduler implementation: */
class ESP32::Scheduler : public AP_HAL::Scheduler
{

public:
    Scheduler();
    ~Scheduler();
    /* AP_HAL::Scheduler methods */
    void     init() override;
    void     set_callbacks(AP_HAL::HAL::Callbacks *cb)
    {
        callbacks = cb;
    };
    void     delay(uint16_t ms) override;
    void     delay_microseconds(uint16_t us) override;
    void     register_timer_process(AP_HAL::MemberProc) override;
    void     register_io_process(AP_HAL::MemberProc) override;
    void     register_timer_failsafe(AP_HAL::Proc, uint32_t period_us) override;
    void     reboot(bool hold_in_bootloader) override;
    bool     in_main_thread() const override;
    // check and set the startup state
    void     set_system_initialized() override;
    bool     is_system_initialized() override;

    void     print_stats(void) ;
    void     print_main_loop_rate(void);

    uint16_t get_loop_rate_hz(void);
    AP_Int16 _active_loop_rate_hz;
    AP_Int16 _loop_rate_hz;

    static void thread_create_trampoline(void *ctx);
    bool thread_create(AP_HAL::MemberProc, const char *name, uint32_t stack_size, priority_base base, int8_t priority) override;

    /*static const int SPI_PRIORITY = 40; // if your primary imu is spi, this should be above the i2c value, spi is better.
    static const int MAIN_PRIO = 15;
    static const int I2C_PRIORITY = 8; // if your primary imu is i2c, this should be above the spi value, i2c is not preferred.
    static const int TIMER_PRIO = 15;
    static const int RCIN_PRIO = 15;
    static const int RCOUT_PRIO = 15;
    static const int WIFI_PRIO = 10;
    static const int UART_PRIO = 8;
    static const int IO_PRIO = 6;
    static const int STORAGE_PRIO = 6; */

      // configMAX_PRIORITIES=25


    static const int SPI_PRIORITY = 24; //      if your primary imu is spi, this should be above the i2c value, spi is better.
    static const int MAIN_PRIO    = 24; //	cpu0: we want scheduler running at full tilt.
    static const int I2C_PRIORITY = 5;  //      if your primary imu is i2c, this should be above the spi value, i2c is not preferred.
    static const int TIMER_PRIO   = 23; //dont make 24. a low priority mere might cause wifi thruput to suffer
    static const int RCIN_PRIO    = 5;
    static const int RCOUT_PRIO   = 10;
    static const int WIFI_PRIO1   = 20; //cpu1:
    static const int WIFI_PRIO2   = 12; //cpu1:
    static const int UART_PRIO    = 23; //dont make 24, scheduler suffers a bit. cpu1: a low priority mere might cause wifi thruput to suffer, as wifi gets passed its data frim the uart subsustem in _writebuf/_readbuf
    static const int IO_PRIO      = 5;
    static const int STORAGE_PRIO = 4;

    static const int TIMER_SS     = 1024*3;
    static const int MAIN_SS      = 1024*5;
    static const int RCIN_SS      = 1024*3;
    static const int RCOUT_SS     = 1024*1.5;
    static const int WIFI_SS1     = 1024*2.25;
    static const int WIFI_SS2     = 1024*2.25;
    static const int UART_SS      = 1024*2.25;
    static const int DEVICE_SS    = 1024*4;     // DEVICEBUS/s
    static const int IO_SS        = 1024*3.5;   // APM_IO
    static const int STORAGE_SS   = 1024*2;     // APM_STORAGE

private:
    AP_HAL::HAL::Callbacks *callbacks;
    AP_HAL::Proc _failsafe;

    AP_HAL::MemberProc _timer_proc[ESP32_SCHEDULER_MAX_TIMER_PROCS];
    uint8_t _num_timer_procs;

    AP_HAL::MemberProc _io_proc[ESP32_SCHEDULER_MAX_IO_PROCS];
    uint8_t _num_io_procs;

    static bool _initialized;

    tskTaskControlBlock* _main_task_handle;
    tskTaskControlBlock* _timer_task_handle;
    tskTaskControlBlock* _rcin_task_handle;
    tskTaskControlBlock* _rcout_task_handle;
    tskTaskControlBlock* _uart_task_handle;
    tskTaskControlBlock* _io_task_handle;
    tskTaskControlBlock* test_task_handle;
    tskTaskControlBlock* _storage_task_handle;

    static void _main_thread(void *arg);
    static void _timer_thread(void *arg);
    static void _rcout_thread(void *arg);
    static void _rcin_thread(void *arg);
    static void _uart_thread(void *arg);
    static void _io_thread(void *arg);
    static void _storage_thread(void *arg);

    static void set_position(void* arg);

    static void _print_profile(void* arg);

    static void test_esc(void* arg);

    static void wdt_init(uint32_t timeout, uint32_t core_mask);

    bool _in_timer_proc;
    void _run_timers();
    Semaphore _timer_sem;

    bool _in_io_proc;
    void _run_io();
    Semaphore _io_sem;
};
