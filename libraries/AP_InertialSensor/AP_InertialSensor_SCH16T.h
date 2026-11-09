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
  Murata SCH16T-K10 / SCH16T-K01 6-axis IMU, direct SPI (48-bit off-frame
  protocol); the variant is selected with HAL_SCH16T_VARIANT in hwdef.dat
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

#ifndef AP_INERTIALSENSOR_SCH16T_ENABLED
#define AP_INERTIALSENSOR_SCH16T_ENABLED 1
#endif

#if AP_INERTIALSENSOR_SCH16T_ENABLED

class AP_InertialSensor_SCH16T : public AP_InertialSensor_Backend
{
public:
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                            enum Rotation rotation);

    void start() override;
    bool update() override;

private:
    AP_InertialSensor_SCH16T(AP_InertialSensor &imu,
                             AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                             enum Rotation rotation);

    bool init();
    void hardware_reset();
    void read_status_registers();
    void timer_read();
    void read_sample(uint64_t sample_us);

    // one 48-bit frame with the chip select held around it; returns the
    // MISO frame, which carries the response to the previous request
    uint64_t transfer_frame(uint64_t command);
    // request a register and clock its response out with a second frame
    uint8_t read_register(uint64_t command, int32_t &value);
    void write_command(uint64_t command);

    static uint64_t with_crc(uint64_t command);
    static uint8_t frame_flags(uint64_t response, uint64_t command);
    static int32_t frame_value(uint64_t response);

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev;
    enum Rotation rotation;

    // command of the last frame of the previous read cycle: its response
    // arrives in the first frame of the next cycle
    uint64_t pending_command;
    uint8_t temperature_counter;
};

#endif // AP_INERTIALSENSOR_SCH16T_ENABLED
