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

#include "AP_InertialSensor_Spectrum.h"

#if AP_INERTIALSENSOR_SPECTRUM_ENABLED

#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// grid step, exact in microseconds
static constexpr uint32_t GRID_US = 1250;
static_assert(uint32_t(GRID_US * AP_InertialSensor_Spectrum::RATE_HZ) == 1000000U, "grid step");
static_assert(AP_InertialSensor_Spectrum::BINS < AP_InertialSensor_Spectrum::N / 2, "bins under Nyquist");
// samples further apart are a gap in the read, not something to interpolate
static constexpr int32_t MAX_GAP_US = 20000;

AP_InertialSensor_Spectrum::~AP_InertialSensor_Spectrum()
{
    delete[] _ring;
    delete[] _grid;
    delete[] _re;
    delete[] _im;
    delete[] _window;
    delete[] _cos;
    delete[] _sin;
}

bool AP_InertialSensor_Spectrum::init()
{
    _ring = NEW_NOTHROW Sample[RING];
    _grid = NEW_NOTHROW float[6 * N];
    _re = NEW_NOTHROW float[N];
    _im = NEW_NOTHROW float[N];
    _window = NEW_NOTHROW float[N];
    _cos = NEW_NOTHROW float[N / 2];
    _sin = NEW_NOTHROW float[N / 2];
    if (_ring == nullptr || _grid == nullptr || _re == nullptr || _im == nullptr ||
        _window == nullptr || _cos == nullptr || _sin == nullptr) {
        return false;
    }
    _tx_next = 2 * PARTS;   // nothing to send yet
    _window_power = 0;
    for (uint16_t n = 0; n < N; n++) {
        _window[n] = 0.5f * (1.0f - cosf(M_2PI * n / (N - 1)));
        _window_power += sq(_window[n]);
    }
    for (uint16_t k = 0; k < N / 2; k++) {
        _cos[k] = cosf(M_2PI * k / N);
        _sin[k] = sinf(M_2PI * k / N);
    }
    // lowest priority: it runs only in the time the other threads leave
    return hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_Spectrum::thread, void),
                                        "INSfft", 4096, AP_HAL::Scheduler::PRIORITY_STORAGE, -2);
}

void AP_InertialSensor_Spectrum::push(const Vector3f &gyro, const Vector3f &accel, uint64_t sample_us)
{
    // single writer: the IMU backend thread
    const uint32_t h = __atomic_load_n(&_head, __ATOMIC_RELAXED);
    Sample &s = _ring[h % RING];
    s.t_us = uint32_t(sample_us);
    s.v[0] = gyro.x; s.v[1] = gyro.y; s.v[2] = gyro.z;
    s.v[3] = accel.x; s.v[4] = accel.y; s.v[5] = accel.z;
    __atomic_store_n(&_head, h + 1, __ATOMIC_RELEASE);
}

void AP_InertialSensor_Spectrum::thread()
{
    while (true) {
        hal.scheduler->delay(PERIOD_MS);
        compute();
    }
}

/*
  the last (N-1) grid steps up to the newest sample, linear interpolation
  on the sample times. The ring is read in place: the read is valid if the
  writer has not wrapped onto the oldest sample used meanwhile
 */
bool AP_InertialSensor_Spectrum::resample(uint32_t &t_end_us)
{
    const uint32_t h = __atomic_load_n(&_head, __ATOMIC_ACQUIRE);
    if (h < 2) {
        return false;
    }
    t_end_us = _ring[(h - 1) % RING].t_us;
    const uint32_t t0 = t_end_us - (N - 1) * GRID_US;

    // newest sample at or before the first grid point
    const uint32_t oldest = h > RING ? h - RING + 1 : 0;
    uint32_t k = h - 1;
    while (k > oldest && int32_t(_ring[k % RING].t_us - t0) > 0) {
        k--;
    }
    if (int32_t(_ring[k % RING].t_us - t0) > 0) {
        return false;   // less than N/RATE_HZ of samples yet
    }

    uint32_t j = k;
    for (uint16_t n = 0; n < N; n++) {
        const uint32_t tg = t0 + n * GRID_US;
        while (j + 1 < h && int32_t(_ring[(j + 1) % RING].t_us - tg) <= 0) {
            j++;
        }
        const Sample &a = _ring[j % RING];
        const Sample &b = _ring[(j + 1 < h ? j + 1 : j) % RING];
        const int32_t dt = int32_t(b.t_us - a.t_us);
        if (dt > MAX_GAP_US) {
            return false;
        }
        const float f = dt > 0 ? constrain_float(float(int32_t(tg - a.t_us)) / dt, 0.0f, 1.0f) : 0.0f;
        for (uint8_t c = 0; c < 6; c++) {
            _grid[c * N + n] = a.v[c] + f * (b.v[c] - a.v[c]);
        }
    }
    return __atomic_load_n(&_head, __ATOMIC_ACQUIRE) - k < RING;
}

// in place, radix 2, on _re/_im
void AP_InertialSensor_Spectrum::fft() const
{
    for (uint16_t i = 1, j = 0; i < N; i++) {
        uint16_t bit = N >> 1;
        for (; j & bit; bit >>= 1) {
            j ^= bit;
        }
        j ^= bit;
        if (i < j) {
            const float tr = _re[i]; _re[i] = _re[j]; _re[j] = tr;
            const float ti = _im[i]; _im[i] = _im[j]; _im[j] = ti;
        }
    }
    for (uint16_t len = 2; len <= N; len <<= 1) {
        const uint16_t half = len >> 1;
        const uint16_t step = N / len;
        for (uint16_t i = 0; i < N; i += len) {
            for (uint16_t k = 0; k < half; k++) {
                const float wr = _cos[k * step];
                const float wi = -_sin[k * step];
                const uint16_t a = i + k;
                const uint16_t b = a + half;
                const float tr = _re[b] * wr - _im[b] * wi;
                const float ti = _re[b] * wi + _im[b] * wr;
                _re[b] = _re[a] - tr;
                _im[b] = _im[a] - ti;
                _re[a] += tr;
                _im[a] += ti;
            }
        }
    }
}

void AP_InertialSensor_Spectrum::compute()
{
    uint32_t t_end_us;
    if (!resample(t_end_us)) {
        return;
    }
    float power[2][BINS] {};
    for (uint8_t c = 0; c < 6; c++) {
        const float *x = &_grid[c * N];
        float mean = 0;
        for (uint16_t n = 0; n < N; n++) {
            mean += x[n];
        }
        mean /= N;
        for (uint16_t n = 0; n < N; n++) {
            _re[n] = (x[n] - mean) * _window[n];
            _im[n] = 0;
        }
        fft();
        for (uint8_t b = 0; b < BINS; b++) {
            power[c / 3][b] += sq(_re[b + 1]) + sq(_im[b + 1]);
        }
    }
    // one-sided power spectral density of the windowed signal
    const float scale = 2.0f / (RATE_HZ * _window_power);
    float db[2][BINS];
    for (uint8_t s = 0; s < 2; s++) {
        for (uint8_t b = 0; b < BINS; b++) {
            db[s][b] = 10.0f * log10f(MAX(power[s][b] * scale, 1e-20f));
        }
    }
    // the newest sample time on the 64-bit clock
    const uint64_t now = AP_HAL::micros64();
    const uint64_t t_us = now - uint32_t(uint32_t(now) - t_end_us);

    WITH_SEMAPHORE(_sem);
    memcpy(_db, db, sizeof(_db));
    _time_us = t_us;
    _seq++;
}

// every active link has room for one more message
bool AP_InertialSensor_Spectrum::links_have_space() const
{
    const uint16_t needed = MAVLINK_NUM_NON_PAYLOAD_BYTES + MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN;
    for (uint8_t i = 0; i < gcs().num_gcs(); i++) {
        const GCS_MAVLINK *c = gcs().chan(i);
        if (c == nullptr || !c->is_active()) {
            continue;
        }
        if (c->txspace() < needed) {
            return false;
        }
    }
    return true;
}

/*
  one message a call, once every active link has room for it: the four
  messages of a spectrum sent together can overflow a USB link and lose the
  accelerometer ones. A link that stays full for MAX_WAIT_MS no longer holds
  the others back
 */
void AP_InertialSensor_Spectrum::send()
{
    static constexpr uint32_t MAX_WAIT_MS = 100;
    const uint32_t now_ms = AP_HAL::millis();
    if (_tx_next >= 2 * PARTS) {
        WITH_SEMAPHORE(_sem);
        if (_seq == _sent_seq) {
            return;
        }
        _sent_seq = _seq;
        memcpy(_tx_db, _db, sizeof(_tx_db));
        _tx_time_us = _time_us;
        _tx_next = 0;
        _tx_last_ms = now_ms;
    }
    if (now_ms - _tx_last_ms < MAX_WAIT_MS && !links_have_space()) {
        return;
    }
    static const char *const names[2] { "FFT_GYRO", "FFT_ACC" };
    const uint8_t s = _tx_next / PARTS;
    const uint8_t part = _tx_next % PARTS;
    mavlink_debug_float_array_t pkt {};
    pkt.time_usec = _tx_time_us;
    pkt.array_id = 2 * s + part;
    memcpy(pkt.name, names[s], MIN(strlen(names[s]), sizeof(pkt.name)));
    memcpy(pkt.data, &_tx_db[s][part * PART_BINS], PART_BINS * sizeof(float));
    gcs().send_to_active_channels(MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY, (const char *)&pkt);
    _tx_next++;
    _tx_last_ms = now_ms;
}

#endif  // AP_INERTIALSENSOR_SPECTRUM_ENABLED
