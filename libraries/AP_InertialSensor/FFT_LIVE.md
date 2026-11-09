# FFT live

How the autopilot computes the vibration spectrum of the IMU and sends it over
telemetry, so that any ground station or tool can decode it, and any
autopilot can produce a compatible stream.

The spectrum is enabled with the parameter `INS_FFT_LIVE` (see
[Enabling it](#enabling-it)). Four times a second the autopilot sends one
spectrum of the gyroscope and one of the accelerometer, from 3.125 to
362.5 Hz, as standard MAVLink messages.

## Message

The spectrum travels in the standard MAVLink message
[`DEBUG_FLOAT_ARRAY`](https://mavlink.io/en/messages/common.html#DEBUG_FLOAT_ARRAY)
(id 350, `common.xml`). No dialect change is needed.

| Field | Type | Content |
|---|---|---|
| `time_usec` | `uint64_t` | Time of the newest IMU sample in the spectrum, µs since boot. The same for every message of one spectrum. |
| `name` | `char[10]` | `FFT_GYRO` or `FFT_ACC` (MAVLink does not null-terminate a 10-character name; these are shorter). |
| `array_id` | `uint16_t` | `2 * sensor + part`: sensor 0 = gyroscope, 1 = accelerometer; part 0 or 1. |
| `data` | `float[58]` | 58 levels in dB (see below). MAVLink 2 extension field: always present in this stream. |

One spectrum of one sensor is **two messages** (part 0 and part 1) with the
same `time_usec`. A full update (gyroscope and accelerometer) is four
messages:

| `array_id` | `name` | bins |
|---|---|---|
| 0 | `FFT_GYRO` | 0 – 57 |
| 1 | `FFT_GYRO` | 58 – 115 |
| 2 | `FFT_ACC` | 0 – 57 |
| 3 | `FFT_ACC` | 58 – 115 |

## Bins and frequencies

- Sample rate of the analysed signal: `fs = 800 Hz`
- FFT length: `N = 256`
- Bin spacing: `df = fs / N = 3.125 Hz`
- Bins sent: FFT bins 1 to 116 (DC is not sent)

Bin `i` of the assembled 116-value array (`i = 58 * part + k`, `k` the index
in `data`) is at

```
f(i) = (i + 1) * 3.125 Hz        i = 0 … 115   →   3.125 … 362.5 Hz
```

## Values

Each value is the one-sided power spectral density summed over the three axes
of the sensor, in decibels:

```
value = 10 * log10( PSD_x + PSD_y + PSD_z )
```

- Gyroscope: dB re 1 (rad/s)²/Hz
- Accelerometer: dB re 1 (m/s²)²/Hz
- Floor: values are clipped at −200 dB (power 1e−20)

To get the amplitude spectral density of the three axes together:
`ASD = 10^(value / 20)` in rad/s/√Hz or m/s²/√Hz.

## How the autopilot computes it

The samples are those of the first usable IMU, as the driver delivers them:
calibrated and rotated to the body frame, before the `INS_GYRO_FILTER` and
`INS_ACCEL_FILTER` low-pass filters and the harmonic notches. A sensor with
an internal low-pass filter shows it: on the MINI AHRS board the SCH16T
filters at 68 Hz (4th order Butterworth, about −24 dB per octave above the
cut-off), and strong vibrations stay visible above it.

The IMU is read at the driver rate, not necessarily evenly (about 1 kHz on
the MINI AHRS board, 880–950 Hz effective; 1 kHz in SITL). Every 250 ms:

1. **Resample.** Take the last `(N − 1) / fs = 318.75 ms` of samples and
   interpolate them linearly, by their timestamps, on an even 800 Hz grid of
   256 points ending at the newest sample. If two consecutive samples are more
   than 20 ms apart inside the window, skip this spectrum.
2. **Detrend.** Remove the mean of each axis (gravity on the accelerometer,
   bias and rotation rate on the gyroscope).
3. **Window.** Multiply by a Hann window `w[n] = 0.5 (1 − cos(2πn / (N − 1)))`.
4. **FFT.** 256-point complex FFT of each axis (imaginary part zero).
5. **PSD.** For bins `k = 1 … 116`:
   `PSD_axis[k] = 2 |X[k]|² / (fs · Σ w[n]²)`
6. **Sum and convert.** Add the three axes, take `10 log10`.

The computation runs in its own thread at the lowest priority
(`AP_HAL::Scheduler::PRIORITY_STORAGE`), so it only uses the time the other
threads leave. On the ESP32-S31 board that thread runs on core 1, away from
the main loop.

## Timing and bandwidth

- One full update (4 messages) every ~250 ms.
- The autopilot sends **one message per main-loop cycle**, and only when every
  active link has room for it (it waits at most 100 ms for a full link), so
  the four messages of an update are spread over a few loop cycles. Do not
  expect them back to back.
- The messages go to every active MAVLink link.
- `DEBUG_FLOAT_ARRAY` is 252 bytes of payload, 264 with the MAVLink 2 header:
  16 messages a second, about 4.2 kB/s on each link.

## Receiver: assembling a spectrum

```
on DEBUG_FLOAT_ARRAY msg:
    name = msg.name up to the first NUL
    sensor = msg.array_id >> 1          # 0 gyro, 1 accel
    part   = msg.array_id & 1
    if sensor > 1 or name != ["FFT_GYRO", "FFT_ACC"][sensor]: ignore
    pending = pending_for[sensor]
    if pending.time_usec != msg.time_usec: pending = new(time_usec), discard the old one
    pending.parts[part] = msg.data[0:58]
    if both parts present:
        spectrum = parts[0] + parts[1]   # 116 values, bin i at (i + 1) * 3.125 Hz
        publish(sensor, msg.time_usec, spectrum)
```

Rules:

- Join parts by `(sensor, time_usec)`. A part with a new `time_usec` starts a
  new spectrum: the previous one, incomplete, is dropped (one of its messages
  was lost).
- Parts can arrive in either order.
- A gap of more than about 1 s means the stream stopped (disabled, link lost,
  or the autopilot skipped spectra because of a gap in the IMU samples).
- `time_usec` is the autopilot clock, not the GCS clock: use it to order and
  join messages, and the arrival time for display windows.

Python (pymavlink):

```python
pending = {}

def on_debug_float_array(m):
    name = m.name.split("\0")[0]
    sensor, part = m.array_id >> 1, m.array_id & 1
    if sensor > 1 or name != ("FFT_GYRO", "FFT_ACC")[sensor]:
        return None
    p = pending.get(sensor)
    if p is None or p["t"] != m.time_usec:
        p = pending[sensor] = {"t": m.time_usec, "parts": [None, None]}
    p["parts"][part] = list(m.data[:58])
    if None in p["parts"]:
        return None
    del pending[sensor]
    db = p["parts"][0] + p["parts"][1]
    freqs = [(i + 1) * 800 / 256 for i in range(116)]
    return sensor, m.time_usec, freqs, db
```

## Enabling it

- Firmware: built with `AP_INERTIALSENSOR_SPECTRUM_ENABLED`. It is on in SITL
  and on the `esp32s31mini` board. For another board add
  `define AP_INERTIALSENSOR_SPECTRUM_ENABLED 1` to its hwdef, or select the
  build option `IMU_FFT_LIVE`.
- Vehicle: parameter `INS_FFT_LIVE = 1` (0 by default). At the first enable
  the autopilot allocates the buffers and starts the thread, which then runs
  until reboot; setting 0 stops the messages only. If the memory is not
  available, the GCS gets `INS: no memory for the FFT` and the stream stays
  off until reboot.
- Memory: about 26 kB of heap and a 4 kB thread stack, only once enabled.

## Checking a receiver

With the board still on a bench, both spectra are flat up to the IMU filter
cut-off, at the sensor noise. With the SCH16T-K10 (gyroscope 6 m°/s/√Hz,
accelerometer 0.8 mm/s²/√Hz per axis, three axes summed):

- gyroscope about −75 dB re 1 (rad/s)²/Hz (measured on the MINI AHRS V1.2
  board with the same driver and spectrum code: −77 dB from 5 to 60 Hz),
- accelerometer about −57 dB re 1 (m/s²)²/Hz (measured: −56 dB from 5 to
  40 Hz),

then falling above 68 Hz. A motor or propeller shows as a line at its rotation
frequency and harmonics.

In SITL, `SIM_VIB_FREQ_X/Y/Z` add a vibration at a known frequency while the
throttle is above `SIM_INS_THR_MIN`, with an amplitude set by `SIM_ACC1_RND`
and `SIM_GYR1_RND`. The autotest `Tools/autotest/autotest.py test.Plane.FFTLive`
flies with an 80 Hz vibration, assembles the spectra as above and checks that
both peak at 80 Hz.

## Source

- `libraries/AP_InertialSensor/AP_InertialSensor_Spectrum.{h,cpp}`: resampling,
  FFT and sending.
- `libraries/AP_InertialSensor/AP_InertialSensor.cpp`: the `INS_FFT_LIVE`
  parameter and the per-loop send; `AP_InertialSensor_Backend.cpp`: the IMU
  samples.
