/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_Notify_config.h"

#if AP_NOTIFY_GPIO_LED_INIT_GPS_ENABLED

#include "AP_BoardLED_InitGPS.h"

#include <AP_HAL/AP_HAL.h>

#include "AP_Notify.h"

static_assert((HAL_GPIO_A_LED_PIN != HAL_GPIO_B_LED_PIN), "Duplicate LED assignments detected");

extern const AP_HAL::HAL& hal;

// full blink periods, multiples of the 20 ms update so each half is even
static constexpr uint32_t LED_FAST_PERIOD_MS = 480;
static constexpr uint32_t LED_GPS_PERIOD_MS = 1000;

static uint8_t blink_level(uint32_t now_ms, uint32_t period_ms)
{
    return (now_ms % period_ms) < (period_ms / 2) ? HAL_GPIO_LED_ON : HAL_GPIO_LED_OFF;
}

bool AP_BoardLED_InitGPS::init(void)
{
    hal.gpio->pinMode(HAL_GPIO_A_LED_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(HAL_GPIO_B_LED_PIN, HAL_GPIO_OUTPUT);

    hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);
    hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_OFF);
    return true;
}

/*
  main update function called at 50Hz
 */
void AP_BoardLED_InitGPS::update(void)
{
    const uint32_t now_ms = AP_HAL::millis();
    const bool gps_lock = AP_Notify::flags.gps_status >= AP_GPS_FixType::FIX_3D;

    // the system flag covers the vehicle setup, the notify flag the IMU calibration
    const bool init_done = hal.scheduler->is_system_initialized() && !AP_Notify::flags.initialising;
    const uint8_t led_a = init_done ? HAL_GPIO_LED_ON : blink_level(now_ms, LED_FAST_PERIOD_MS);
    hal.gpio->write(HAL_GPIO_A_LED_PIN, led_a);

    // blinking with a 3D fix, steady once the navigation uses it (absolute position)
    uint8_t led_b = HAL_GPIO_LED_OFF;
    if (gps_lock) {
        led_b = AP_Notify::flags.have_pos_abs ? HAL_GPIO_LED_ON : blink_level(now_ms, LED_GPS_PERIOD_MS);
    }
    hal.gpio->write(HAL_GPIO_B_LED_PIN, led_b);
}

#endif  // AP_NOTIFY_GPIO_LED_INIT_GPS_ENABLED
