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
/*
  driver for the ELVH differential pressure sensors with SPI output
  https://www.allsensors.com/products/elvh-l02d-hrrd-c-nsa4
  datasheet DS-0376: after power-up the sensor converts continuously and the
  only bus operation is a read (SS, SCLK, MISO, 50-800 kHz) of
  2 status bits, 14 pressure bits, 11 temperature bits and 5 padding bits
 */

#include "AP_Airspeed_ELVH.h"

#if AP_AIRSPEED_ELVH_ENABLED

#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

// transfer function A (10%-90% of 2^14 counts): a differential range sits
// on 8192 counts, -range at 10% and +range at 90%
#define ELVH_ZERO_COUNTS 8192.0f
#define ELVH_SPAN_COUNTS 13107.2f
#define ELVH_MIN_COUNTS 1638U
#define ELVH_MAX_COUNTS 14746U

// temperature counts at -40 and +85 degC (operating range)
#define ELVH_TEMP_MIN_COUNTS 102U
#define ELVH_TEMP_MAX_COUNTS 1382U

#define ELVH_SAMPLE_HZ 100U
#define PSI_TO_PASCAL 6894.757f

// range (+-) of the differential parts by ARSPD_ELVH_RNG, Pa (DS-0376 Rev B)
static const float elvh_range_pa[] {
    0,                              // ARSPD_PSI_RANGE
    0.5f * INCH_OF_H2O_TO_PASCAL,   // F50D
    1 * INCH_OF_H2O_TO_PASCAL,      // L01D
    2 * INCH_OF_H2O_TO_PASCAL,      // L02D
    4 * INCH_OF_H2O_TO_PASCAL,      // L04D
    5 * INCH_OF_H2O_TO_PASCAL,      // L05D
    10 * INCH_OF_H2O_TO_PASCAL,     // L10D
    20 * INCH_OF_H2O_TO_PASCAL,     // L20D
    30 * INCH_OF_H2O_TO_PASCAL,     // L30D
    60 * INCH_OF_H2O_TO_PASCAL,     // L60D
    250,                            // MF25D, 2.5 mbar
    1250,                           // MF12D
    2500,                           // M025D
    5000,                           // M050D
    7500,                           // M075D
    10000,                          // M100D
    16000,                          // M160D
    25000,                          // M250D
};

bool AP_Airspeed_ELVH::init()
{
    const uint8_t rng = get_elvh_range();
    if (rng >= ARRAY_SIZE(elvh_range_pa)) {
        return false;
    }
    range_pa = (rng == 0) ? get_psi_range() * PSI_TO_PASCAL : elvh_range_pa[rng];
    if (!is_positive(range_pa)) {
        return false;
    }

    dev = hal.spi->get_device_ptr(HAL_AIRSPEED_ELVH_SPI_NAME);
    if (dev == nullptr) {
        return false;
    }

    {
        WITH_SEMAPHORE(dev->get_semaphore());
        dev->set_speed(AP_HAL::Device::SPEED_LOW);
        dev->set_device_type(uint8_t(DevType::ELVH));
    }

    /*
      SPI has no acknowledge: a missing sensor reads all zeros or all ones.
      Require three reads with a valid status, the pressure inside the
      calibrated band and consistent temperatures (startup is 3 ms). The
      bus is shared with the IMU: it is only held for each read
     */
    uint8_t good = 0;
    uint16_t last_temp = 0;
    for (uint8_t i = 0; i < 10 && good < 3; i++) {
        hal.scheduler->delay(5);
        Status status = Status::FAULT;
        uint16_t press_counts = 0, temp_counts = 0;
        bool ok;
        {
            WITH_SEMAPHORE(dev->get_semaphore());
            ok = read_counts(status, press_counts, temp_counts);
        }
        if (!ok ||
            (status != Status::NORMAL && status != Status::STALE) ||
            press_counts < ELVH_MIN_COUNTS || press_counts > ELVH_MAX_COUNTS ||
            temp_counts < ELVH_TEMP_MIN_COUNTS || temp_counts > ELVH_TEMP_MAX_COUNTS ||
            (good > 0 && abs(int32_t(temp_counts) - int32_t(last_temp)) > 50)) {
            good = 0;
            continue;
        }
        last_temp = temp_counts;
        good++;
    }
    if (good < 3) {
        return false;
    }

    set_bus_id(dev->get_bus_id());
    dev->register_periodic_callback(1000000UL / ELVH_SAMPLE_HZ,
                                    FUNCTOR_BIND_MEMBER(&AP_Airspeed_ELVH::timer, void));
    return true;
}

bool AP_Airspeed_ELVH::read_counts(Status &status, uint16_t &press_counts, uint16_t &temp_counts)
{
    // read only: the sensor has no MOSI
    uint8_t data[4];
    if (!dev->transfer(nullptr, 0, data, sizeof(data))) {
        return false;
    }
    status = Status(data[0] >> 6);
    press_counts = (uint16_t(data[0] & 0x3FU) << 8) | data[1];
    temp_counts = (uint16_t(data[2]) << 3) | (data[3] >> 5);
    return true;
}

// 100 Hz timer
void AP_Airspeed_ELVH::timer()
{
    Status status = Status::FAULT;
    uint16_t press_counts = 0, temp_counts = 0;
    // a stale sample has already been read; a fault or a pressure beyond
    // the calibrated range is not a measurement
    if (!read_counts(status, press_counts, temp_counts) ||
        status != Status::NORMAL ||
        press_counts < ELVH_MIN_COUNTS || press_counts > ELVH_MAX_COUNTS) {
        return;
    }

    // the span covers -range..+range
    const float press = (press_counts - ELVH_ZERO_COUNTS) * (2.0f * range_pa / ELVH_SPAN_COUNTS);
    const float temp = temp_counts * (200.0f / 2047.0f) - 50.0f;

    WITH_SEMAPHORE(sem);
    pressure_sum += press;
    temperature_sum += temp;
    press_count++;
    temp_count++;
    last_sample_time_ms = AP_HAL::millis();
}

// return the current differential_pressure in Pascal
bool AP_Airspeed_ELVH::get_differential_pressure(float &_pressure)
{
    WITH_SEMAPHORE(sem);

    if ((AP_HAL::millis() - last_sample_time_ms) > 100) {
        return false;
    }

    if (press_count > 0) {
        pressure = pressure_sum / press_count;
        press_count = 0;
        pressure_sum = 0;
    }

    _pressure = pressure;
    return true;
}

// return the current temperature in degrees C, if available
bool AP_Airspeed_ELVH::get_temperature(float &_temperature)
{
    WITH_SEMAPHORE(sem);

    if ((AP_HAL::millis() - last_sample_time_ms) > 100) {
        return false;
    }

    if (temp_count > 0) {
        temperature = temperature_sum / temp_count;
        temp_count = 0;
        temperature_sum = 0;
    }

    _temperature = temperature;
    return true;
}

#endif  // AP_AIRSPEED_ELVH_ENABLED
