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
  Live vibration spectrum of the first usable IMU, sent to the GCS as
  DEBUG_FLOAT_ARRAY (INS_FFT_LIVE). The message format is described in
  FFT_LIVE.md.

  The samples come at the driver's read times, not evenly spaced: every
  PERIOD_MS the last N/RATE_HZ seconds are resampled on an even RATE_HZ grid
  (linear interpolation on the sample times), the mean of each axis is
  removed, a Hann window applied and the power spectral density of the three
  axes summed: one spectrum for the gyro, one for the accelerometer, in dB of
  (rad/s)^2/Hz and (m/s^2)^2/Hz. The work runs in a thread at the lowest
  priority, so it only takes time the other threads leave.

  Each spectrum is two messages per sensor: name "FFT_GYRO" or "FFT_ACC",
  array_id = 2 * sensor + part (sensor 0 gyro, 1 accel), time_usec the time
  of the newest sample, data[i] the level of bin 1 + 58 * part + i, at
  (1 + 58 * part + i) * RATE_HZ / N Hz.
 */
#pragma once

#include "AP_InertialSensor_config.h"

#if AP_INERTIALSENSOR_SPECTRUM_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

class AP_InertialSensor_Spectrum {
public:
    static constexpr uint16_t N = 256;
    static constexpr float RATE_HZ = 800.0f;
    static constexpr uint8_t PART_BINS = 58;            // DEBUG_FLOAT_ARRAY data length
    static constexpr uint8_t PARTS = 2;
    static constexpr uint8_t BINS = PART_BINS * PARTS;  // 3.125 to 362.5 Hz
    static constexpr uint32_t PERIOD_MS = 250;

    ~AP_InertialSensor_Spectrum();

    // allocates the buffers and starts the thread
    bool init();

    // IMU backend thread, every sample
    void push(const Vector3f &gyro, const Vector3f &accel, uint64_t sample_us);

    // main thread, every loop: sends the newest spectrum one message a call
    void send();

private:
    struct Sample {
        uint32_t t_us;
        float v[6];     // gyro xyz, accel xyz
    };
    static constexpr uint16_t RING = 512;   // 0.5 s at a 1 kHz read

    Sample *_ring;
    uint32_t _head;     // samples written, read with acquire by the thread
    float *_grid;       // 6 x N, resampled
    float *_re;
    float *_im;
    float *_window;
    float *_cos;
    float *_sin;
    float _window_power;

    HAL_Semaphore _sem;
    float _db[2][BINS];
    uint64_t _time_us;
    uint32_t _seq;
    uint32_t _sent_seq;

    // spectrum being sent, message _tx_next of 2 * PARTS
    float _tx_db[2][BINS];
    uint64_t _tx_time_us;
    uint8_t _tx_next;
    uint32_t _tx_last_ms;
    bool links_have_space() const;

    void thread();
    void compute();
    bool resample(uint32_t &t_end_us);
    void fft() const;
};

#endif  // AP_INERTIALSENSOR_SPECTRUM_ENABLED
