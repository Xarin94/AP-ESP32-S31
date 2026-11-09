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
  Murata SCH16T-K10 / SCH16T-K01 driver, direct SPI.

  The SPI protocol is off-frame: every 48-bit frame carries the response to
  the request of the previous frame. A sample is read with seven pipelined
  frames (rate X/Y/Z, accel X/Y/Z, temperature); the temperature response
  arrives in the first frame of the next cycle.

  Start-up sequence (datasheet §5.2): soft reset, range/filter
  configuration, EN_SENSOR, >= 215 ms, read all status registers, EOI, read
  status again, check STAT_SUM.
 */

#include <utility>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "AP_InertialSensor_SCH16T.h"

#if AP_INERTIALSENSOR_SCH16T_ENABLED

extern const AP_HAL::HAL& hal;

// read rate of the sensor registers. The SCH16T filters internally
// (HAL_SCH16T_LPF_CODE); a lower rate than its output rate is a plain
// decimation of the filtered signal and reduces the SPI load
#ifndef HAL_SCH16T_SAMPLE_RATE
#define HAL_SCH16T_SAMPLE_RATE 1000
#endif
#define BACKEND_SAMPLE_RATE HAL_SCH16T_SAMPLE_RATE

// 48-bit MOSI frame: [47:38] TA+SA address, [37] RW, [35:32] FT, [27:8] data, [7:0] CRC
static constexpr uint64_t CMD_READ_RATE_X   = 0x004800000000ULL;
static constexpr uint64_t CMD_READ_RATE_Y   = 0x008800000000ULL;
static constexpr uint64_t CMD_READ_RATE_Z   = 0x00C800000000ULL;
static constexpr uint64_t CMD_READ_ACC_X    = 0x010800000000ULL;
static constexpr uint64_t CMD_READ_ACC_Y    = 0x014800000000ULL;
static constexpr uint64_t CMD_READ_ACC_Z    = 0x018800000000ULL;
static constexpr uint64_t CMD_READ_TEMP     = 0x040800000000ULL;
static constexpr uint64_t CMD_READ_STAT_SUM = 0x050800000000ULL;

static constexpr uint64_t CMD_SOFT_RESET    = 0x0DA800000A00ULL;  // CTRL_RESET = 0xA
static constexpr uint64_t CMD_ENABLE_SENSOR = 0x0D6800000100ULL;  // CTRL_MODE EN_SENSOR
static constexpr uint64_t CMD_SET_EOI       = 0x0D6800000300ULL;  // CTRL_MODE EOI | EN_SENSOR
static constexpr uint64_t CMD_SET_DYN3_RATE = 0x0A2800360000ULL;  // CTRL_RATE: DYN3 on XYZ1/XYZ2
static constexpr uint64_t CMD_SET_DYN2_ACC  = 0x0A6800240000ULL;  // CTRL_ACC12: DYN2 on XYZ1/XYZ2
static constexpr uint64_t CMD_SET_FILT_RATE = 0x096800000000ULL;  // CTRL_FILT_RATE (code in data)
static constexpr uint64_t CMD_SET_FILT_ACC  = 0x09A800000000ULL;  // CTRL_FILT_ACC12 (code in data)

// LPF1 code per axis, 3 bits x 3 axes: 000 68 Hz, 001 30 Hz, 010 13 Hz,
// 011 280 Hz, 100 370 Hz, 101 235 Hz. Default 68 Hz on every axis, the
// part default.
#ifndef HAL_SCH16T_LPF_CODE
#define HAL_SCH16T_LPF_CODE 0x000
#endif

/*
  part variant, from hwdef.dat (HAL_SCH16T_VARIANT):
  - SCH16T-K10: +-2000 deg/s, +-16 g. Rate DYN3 and accel DYN2 are written
    at start-up: 200 LSB/(deg/s),
    6400 LSB/(m/s/s), 20-bit
  - SCH16T-K01: +-300 deg/s, +-8 g, high precision (0.3 deg/h bias
    instability). The factory dynamic ranges are kept: 1600 LSB/(deg/s),
    3200 LSB/(m/s/s), 20-bit (Murata product data)
 */
#define SCH16T_VARIANT_K01 1
#define SCH16T_VARIANT_K10 10
#ifndef HAL_SCH16T_VARIANT
#define HAL_SCH16T_VARIANT SCH16T_VARIANT_K10
#endif

#if HAL_SCH16T_VARIANT == SCH16T_VARIANT_K10
static constexpr float GYRO_LSB_PER_DPS  = 200.0f;
static constexpr float ACCEL_LSB_PER_MSS = 6400.0f;
static constexpr bool SET_DYNAMIC_RANGE  = true;
#elif HAL_SCH16T_VARIANT == SCH16T_VARIANT_K01
static constexpr float GYRO_LSB_PER_DPS  = 1600.0f;
static constexpr float ACCEL_LSB_PER_MSS = 3200.0f;
static constexpr bool SET_DYNAMIC_RANGE  = false;
#else
#error "HAL_SCH16T_VARIANT must be 1 (SCH16T-K01) or 10 (SCH16T-K10)"
#endif

// frame check results
static constexpr uint8_t FRAME_ERR_CRC    = 1U << 0;
static constexpr uint8_t FRAME_ERR_SA     = 1U << 1;
static constexpr uint8_t FRAME_ERR_CE     = 1U << 2;
static constexpr uint8_t FRAME_ERR_IDS    = 1U << 3;
static constexpr uint8_t FRAME_ERR_STATUS = 1U << 4;
static constexpr uint8_t FRAME_SATURATED  = 1U << 5;
static constexpr uint8_t FRAME_ERRORS     = FRAME_ERR_CRC | FRAME_ERR_SA | FRAME_ERR_CE |
                                            FRAME_ERR_IDS | FRAME_ERR_STATUS;

AP_InertialSensor_SCH16T::AP_InertialSensor_SCH16T(AP_InertialSensor &imu,
                                                   AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev,
                                                   enum Rotation _rotation)
    : AP_InertialSensor_Backend(imu)
    , dev(std::move(_dev))
    , rotation(_rotation)
    , pending_command(0)
    , temperature_counter(0)
{
}

AP_InertialSensor_Backend *AP_InertialSensor_SCH16T::probe(AP_InertialSensor &imu,
                                                           AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                                           enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    auto sensor = NEW_NOTHROW AP_InertialSensor_SCH16T(imu, std::move(dev), rotation);
    if (!sensor) {
        return nullptr;
    }
    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

// CRC-8, polynomial 0x2F, seed 0xFF, over bits 47..8
uint64_t AP_InertialSensor_SCH16T::with_crc(uint64_t command)
{
    const uint64_t data = command & 0xFFFFFFFFFF00ULL;
    uint8_t crc = 0xFF;
    for (int8_t i = 47; i >= 0; i--) {
        const uint8_t bit = (data >> i) & 0x01;
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x2F) ^ bit : (uint8_t)(crc << 1) | bit;
    }
    return data | crc;
}

/*
  MISO frame: [47] D, [46:37] SA, [36] CE, [35] IDS, [34:33] S1:S0,
  [27:8] data, [7:0] CRC. A missing part answers all-zeros or all-ones,
  both of which fail the CRC.
 */
uint8_t AP_InertialSensor_SCH16T::frame_flags(uint64_t response, uint64_t command)
{
    if (with_crc(response) != response) {
        return FRAME_ERR_CRC;
    }
    uint8_t flags = 0;
    if (((response >> 37) & 0x3FF) != ((command >> 38) & 0x3FF)) {
        flags |= FRAME_ERR_SA;
    }
    if (response & (1ULL << 36)) {
        flags |= FRAME_ERR_CE;
    }
    if (response & (1ULL << 35)) {
        flags |= FRAME_ERR_IDS;
    }
    const uint8_t status = (response >> 33) & 0x3;
    if (status == 0x1 || status == 0x3) {
        flags |= FRAME_ERR_STATUS;
    } else if (status == 0x2) {
        flags |= FRAME_SATURATED;
    }
    return flags;
}

int32_t AP_InertialSensor_SCH16T::frame_value(uint64_t response)
{
    int32_t raw = (int32_t)((response >> 8) & 0xFFFFF);
    if (raw & 0x80000) {
        raw -= 0x100000;
    }
    return raw;
}

uint64_t AP_InertialSensor_SCH16T::transfer_frame(uint64_t command)
{
    uint8_t buf[6];
    for (uint8_t i = 0; i < 6; i++) {
        buf[i] = (uint8_t)(command >> (40 - 8 * i));
    }
    if (!dev->transfer_fullduplex(buf, sizeof(buf))) {
        return 0;
    }
    uint64_t response = 0;
    for (uint8_t i = 0; i < 6; i++) {
        response = (response << 8) | buf[i];
    }
    return response;
}

void AP_InertialSensor_SCH16T::write_command(uint64_t command)
{
    transfer_frame(with_crc(command));
}

uint8_t AP_InertialSensor_SCH16T::read_register(uint64_t command, int32_t &value)
{
    command = with_crc(command);
    transfer_frame(command);
    // a valid register read clocks the response out
    const uint64_t response = transfer_frame(with_crc(CMD_READ_STAT_SUM));
    value = frame_value(response);
    return frame_flags(response, command);
}

void AP_InertialSensor_SCH16T::hardware_reset()
{
#ifdef HAL_SCH16T_RESET_PIN
    // EXTRESN is active low
    hal.gpio->pinMode(HAL_SCH16T_RESET_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->write(HAL_SCH16T_RESET_PIN, 0);
    hal.scheduler->delay(2);
    hal.gpio->write(HAL_SCH16T_RESET_PIN, 1);
    hal.scheduler->delay(50);
#endif
}

// reading every status register clears the flags latched by the internal
// start-up tests
void AP_InertialSensor_SCH16T::read_status_registers()
{
    static const uint64_t status_regs[] = {
        0x050800000000ULL, 0x054800000000ULL, 0x058800000000ULL, 0x05C800000000ULL, // SUM, SUM_SAT, COM, RATE_COM
        0x060800000000ULL, 0x064800000000ULL, 0x068800000000ULL,                    // RATE_X/Y/Z
        0x06C800000000ULL, 0x070800000000ULL, 0x074800000000ULL,                    // ACC_X/Y/Z
    };
    for (const uint64_t reg : status_regs) {
        int32_t v;
        read_register(reg, v);
    }
}

bool AP_InertialSensor_SCH16T::init()
{
    hardware_reset();

    WITH_SEMAPHORE(dev->get_semaphore());
    dev->set_speed(AP_HAL::Device::SPEED_LOW);

    // the part must answer a register read with a valid frame
    int32_t sum;
    if (read_register(CMD_READ_STAT_SUM, sum) & FRAME_ERR_CRC) {
        return false;
    }

    for (uint8_t attempt = 0; attempt < 2; attempt++) {
        // the soft reset is accepted with EOI already set, so a reboot
        // without power cycle starts from a clean state
        write_command(CMD_SOFT_RESET);
        hal.scheduler->delay(50);
        if (SET_DYNAMIC_RANGE) {
            write_command(CMD_SET_DYN3_RATE);
            write_command(CMD_SET_DYN2_ACC);
        }
        write_command(CMD_SET_FILT_RATE | ((uint64_t)HAL_SCH16T_LPF_CODE << 8));
        write_command(CMD_SET_FILT_ACC | ((uint64_t)HAL_SCH16T_LPF_CODE << 8));
        write_command(CMD_ENABLE_SENSOR);
        // status is defined 215 ms after EN_SENSOR; EOI written earlier
        // freezes the "initialization running" status
        hal.scheduler->delay(250);
        read_status_registers();
        write_command(CMD_SET_EOI);
        hal.scheduler->delay(5);
        read_status_registers();

        int32_t rate;
        const uint8_t sum_flags = read_register(CMD_READ_STAT_SUM, sum);
        const uint8_t rate_flags = read_register(CMD_READ_RATE_X, rate);
        if ((sum_flags & FRAME_ERR_CRC) == 0 && (sum & 0xFFFF) == 0xFFFF && rate_flags == 0) {
            dev->set_speed(AP_HAL::Device::SPEED_HIGH);
            return true;
        }
    }
    return false;
}

void AP_InertialSensor_SCH16T::start()
{
    if (!_imu.register_accel(accel_instance, BACKEND_SAMPLE_RATE, dev->get_bus_id_devtype(DEVTYPE_INS_SCH16T)) ||
        !_imu.register_gyro(gyro_instance, BACKEND_SAMPLE_RATE, dev->get_bus_id_devtype(DEVTYPE_INS_SCH16T))) {
        return;
    }

    set_gyro_orientation(gyro_instance, rotation);
    set_accel_orientation(accel_instance, rotation);

    dev->register_periodic_callback(1000000UL / BACKEND_SAMPLE_RATE,
                                    FUNCTOR_BIND_MEMBER(&AP_InertialSensor_SCH16T::timer_read, void));
}

// the part has no FIFO: the backend takes the integration time from the
// sample time, so a late or missed callback does not skew it
void AP_InertialSensor_SCH16T::timer_read()
{
    read_sample(AP_HAL::micros64());
}

void AP_InertialSensor_SCH16T::read_sample(uint64_t sample_us)
{
    static const uint64_t requests[] = {
        CMD_READ_RATE_X, CMD_READ_RATE_Y, CMD_READ_RATE_Z,
        CMD_READ_ACC_X, CMD_READ_ACC_Y, CMD_READ_ACC_Z,
        CMD_READ_TEMP,
    };
    const uint8_t n = ARRAY_SIZE(requests);
    int32_t raw[n - 1];
    uint8_t flags = 0;

    // frame i carries the response to request i-1; frame 0 the response
    // to the temperature request of the previous cycle
    uint64_t previous = pending_command;
    for (uint8_t i = 0; i < n; i++) {
        const uint64_t command = with_crc(requests[i]);
        const uint64_t response = transfer_frame(command);
        if (i == 0) {
            if (previous != 0 && (frame_flags(response, previous) & FRAME_ERRORS) == 0 &&
                ++temperature_counter >= 100) {
                temperature_counter = 0;
                // 16-bit value left-aligned in the data field, 100 LSB/degC
                _publish_temperature(accel_instance, (frame_value(response) >> 4) * 0.01f);
            }
        } else {
            raw[i - 1] = frame_value(response);
            flags |= frame_flags(response, previous);
        }
        previous = command;
    }
    pending_command = previous;

    if (flags & FRAME_ERRORS) {
        _inc_gyro_error_count(gyro_instance);
        _inc_accel_error_count(accel_instance);
        return;
    }

    Vector3f gyro(raw[0], raw[1], raw[2]);
    gyro *= radians(1.0f / GYRO_LSB_PER_DPS);
    Vector3f accel(raw[3], raw[4], raw[5]);
    accel *= 1.0f / ACCEL_LSB_PER_MSS;

    _rotate_and_correct_gyro(gyro_instance, gyro);
    _notify_new_gyro_raw_sample(gyro_instance, gyro, sample_us);
    _rotate_and_correct_accel(accel_instance, accel);
    _notify_new_accel_raw_sample(accel_instance, accel, sample_us);
}

bool AP_InertialSensor_SCH16T::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}

#endif // AP_INERTIALSENSOR_SCH16T_ENABLED
