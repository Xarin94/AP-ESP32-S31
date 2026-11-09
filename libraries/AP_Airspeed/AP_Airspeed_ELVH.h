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
#pragma once

/*
  backend driver for the All Sensors (Amphenol) ELVH differential pressure
  sensors with SPI output and transfer function A (10%-90%), e.g.
  ELVH-L02D-HRRD-C-NSA4. The full scale (+-) comes from ARSPD_ELVH_RNG, or
  from ARSPD_PSI_RANGE when that is 0; both are read at boot.
 */

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_ELVH_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>

#include "AP_Airspeed_Backend.h"

#ifndef HAL_AIRSPEED_ELVH_SPI_NAME
#define HAL_AIRSPEED_ELVH_SPI_NAME "elvh"
#endif

class AP_Airspeed_ELVH : public AP_Airspeed_Backend
{
public:
    using AP_Airspeed_Backend::AP_Airspeed_Backend;

    ~AP_Airspeed_ELVH(void) {
        delete dev;
    }

    // probe and initialise the sensor
    bool init() override;

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure) override;

    // return the current temperature in degrees C, if available
    bool get_temperature(float &temperature) override;

private:
    enum class Status : uint8_t {
        NORMAL = 0,
        RESERVED = 1,
        STALE = 2,
        FAULT = 3,
    };

    // one SPI read: status, pressure and temperature counts
    bool read_counts(Status &status, uint16_t &press_counts, uint16_t &temp_counts);
    void timer();

    float range_pa;
    float pressure;
    float temperature;
    float pressure_sum;
    float temperature_sum;
    uint32_t press_count;
    uint32_t temp_count;
    uint32_t last_sample_time_ms;

    AP_HAL::SPIDevice *dev;
};

#endif  // AP_AIRSPEED_ELVH_ENABLED
