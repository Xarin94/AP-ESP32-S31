# ArduPilot on the MINI AHRS board (ESP32-S31)

Standard ArduPilot, with EKF3, ported to the ESP32-S31 (ESP-IDF 6.1) for the MINI AHRS V1.2 board, module ESP32-S31-WROOM-1-N16R16V.

- Board: `esp32s31mini`, hwdef in this directory.
- Base: ArduPilot master `dc78bc5` (October 2026).
- Vehicles: ArduPlane, ArduRover, ArduCopter (the board has no PWM outputs and no CAN, see below).
- Navigation: ArduPilot AHRS/EKF3, unmodified (`AHRS_EKF_TYPE 3`, the default).

## Build

ESP-IDF v6.1 is required (branch `release/v6.1`; the S31 is a *preview* target in ESP-IDF), on Linux or WSL. ESP-IDF does not accept spaces in the path: clone the repository into a directory without spaces.

```sh
. ~/esp/esp-idf-v6.1/export.sh
git submodule update --init --recursive
./waf configure --board esp32s31mini
./waf plane        # or: ./waf rover, ./waf copter
```

Run `./waf configure` again after every change to `hwdef.dat`. The images are in `build/esp32s31mini/esp-idf_build/`: `bootloader/bootloader.bin`, `partition_table/partition-table.bin`, `ardupilot.bin`.

## Prebuilt firmware

[`firmware/plane`](firmware/plane) and [`firmware/rover`](firmware/rover) hold ready-to-flash images, built from commit `bc48764` (ArduPlane / ArduRover V4.8.0-dev) with ESP-IDF v6.1: `bootloader.bin` at 0x2000, `partition-table.bin` at 0x10000, `ardupilot.bin` at 0x20000, and their `SHA256SUMS`. Flash them as below, from inside the directory and with the file names without their build subdirectories. They are not a release and are not rebuilt automatically: a later commit may change the code without changing them.

## Flashing

Over USB (USB-C connector, USB Serial/JTAG of the ESP32-S31). On the board's USB link only `--no-stub` at 115200 baud works:

```sh
cd build/esp32s31mini/esp-idf_build
python -m esptool --chip esp32s31 -p <PORT> -b 115200 --no-stub write-flash \
    --flash-mode dio --flash-freq 80m --flash-size 16MB \
    0x2000 bootloader/bootloader.bin 0x10000 partition_table/partition-table.bin 0x20000 ardupilot.bin
```

To update only the firmware, `0x20000 ardupilot.bin` is enough: the bootloader, the partition table and the parameters are kept. If the board ran another ArduPilot firmware before, reset the parameters at the first boot (`FORMAT_VERSION 0`, or "Reset to defaults" from the GCS).

After flashing, the chip stays in download mode: the new firmware starts only after unplugging and replugging the USB cable.

If Windows reports "USB device not recognized": with the USB unplugged, connect pad 27 of the module (IO61) to GND, plug the USB in, remove the wire and, with the USB still connected, flash adding `--before no-reset`.

## Board map

| Function | GPIO | ArduPilot |
|---|---|---|
| SPI SCK / MOSI / MISO | 20 / 21 / 22 | `ESP32_SPIBUS SPI2_HOST` |
| SCH16T CS / RESET / DRY | 25 / 17 / 19 | `IMU SCH16T`, `ROTATION_ROLL_180`, `HAL_SCH16T_RESET_PIN 17`; DRY not used |
| BMP581 #1 / #2 CS | 23 / 24 | `BARO BMP581` x2 |
| RM3100 CS | 38 | `COMPASS RM3100`, `ROTATION_ROLL_180` (module upside down under the board) |
| ELVH CS | 18 | airspeed `SPI:elvh`, `ARSPD_TYPE 21` |
| UART1 RX / TX | 8 / 9 (connector S1) | SERIAL3, GPS |
| UART2 RX / TX | 10 / 11 (connector S2) | SERIAL2, telemetry or serial RC (CRSF/ELRS: `SERIAL2_PROTOCOL 23`) |
| USB D- / D+ | 33 / 34 | SERIAL0 (USB Serial/JTAG, MAVLink) |
| LED1 / LED2 | 13 / 12 | status LEDs, see below |
| Wi-Fi | — | SERIAL1, access point `ardupilot-s31` (password `ardupilot1`), UDP 192.168.4.1:14550 |

Notes on the V1.2 board, measured on the hardware:
- Connectors S1 and S2 are not on the schematic pins (10/11 and 12/13) but on IO8/IO9 and IO10/IO11. The pin marked `R` is the board RX: connect it to the TX of the device.
- The LEDs are on IO13 and IO12, not on IO14 and IO15 as in the schematic. LED3 (IO16) is not used.
- The `CSmag` line (IO18) reads 0 V at the module and kept the RM3100 always selected: the RM3100 CS is bridged to IO38. IO18 stays as the ELVH CS, which is not connected on the V1.2 test board.
- CAN (MCP2542FD): TXD and RXD reach no readable GPIO, so CAN is not configured (no `ESP32_CAN` lines, no DroneCAN). The HAL already has the CAN (TWAI) driver: on a board with CAN wired, `ESP32_CAN <TX> <RX>` in the hwdef is enough.
- No PWM outputs and no dedicated RC input: RC comes from a serial protocol.
- No SD card: logging, scripting and the filesystem are disabled.

The SCH16T and RM3100 rotations should be checked by rotating the board by hand (see below).

## Configuration

The hardware configuration lives in two files in this directory:

| File | Contents | When it takes effect |
|---|---|---|
| `hwdef.dat` | pins, buses, sensors, IMU variant, use of the cores | at build time (`./waf configure` + build) |
| `defaults.parm` | parameter defaults | at the first boot or after a parameter reset |

### SCH16T IMU

```
define HAL_SCH16T_VARIANT 10    # 1 = SCH16T-K01, 10 = SCH16T-K10
```

| | SCH16T-K01 | SCH16T-K10 |
|---|---|---|
| Gyro range | ±300 °/s | ±2000 °/s |
| Accel range | ±8 g | ±16 g |
| Sensitivity used by the driver (20 bit) | 1600 LSB/(°/s), 3200 LSB/(m/s²), factory ranges | 200 LSB/(°/s) DYN3, 6400 LSB/(m/s²) DYN2, written at start-up |

The value must match the part on the board: with the wrong variant the scales are off by a factor of 8 (gyro) or 2 (accel). The default is K10; the V1.2 schematic lists SCH16T-K01-10. To check on the assembled board:
- accelerometer: still and level, `RAW_IMU` / `SCALED_IMU` must read about −9.8 m/s² on Z (−4.9 or −19.6 mean the wrong variant);
- gyro: rotating the board by 90° about the vertical axis must change the GCS yaw by 90°.

Internal filter and read rate:

```
define HAL_SCH16T_LPF_CODE 0x000     # 68 Hz on every axis
define HAL_SCH16T_SAMPLE_RATE 1000   # reads per second
```

The code repeats the filter three times (3 bits per axis): `000` 68 Hz, `001` 30 Hz, `010` 13 Hz, `011` 280 Hz, `100` 370 Hz, `101` 235 Hz (for example 30 Hz on every axis is `0x049`). The datasheet asks for at least 500 Hz reading with 68 Hz, 1000 Hz with 235 and 280 Hz, 2000 Hz with 370 Hz. The driver reads the interpolated outputs with 7 pipelined SPI frames, checks the CRC and status of every frame and publishes the temperature at 10 Hz.

### EKF3 noise (SCH16T-K10, RM3100)

EKF3 takes its process noises as the standard deviation per prediction step divided by the step: variance per step = (dt · NSE)². A white noise of density N gives a variance of N² · dt, so NSE = N / √dt. An offset drifting as a random walk of σ over a time T gives NSE = σ / (√T · √dt). The values below are computed with dt = 12 ms (`EKF_TARGET_DT`); Plane and Rover, with a 50 Hz loop, predict every 20 ms and with the same parameters have a 1.67 times larger variance per second, that is more conservative.

| Parameter | Sensor data | Sensor only | EKF3 default | In `defaults.parm` |
|---|---|---|---|---|
| `EK3_GYRO_P_NSE` (rad/s) | K10: 6 m°/s/√Hz | 9.6·10⁻⁴ | 1.5·10⁻² | default |
| `EK3_ACC_P_NSE` (m/s²) | K10: 0.8 mm/s²/√Hz | 7.3·10⁻³ | 0.35 | default |
| `EK3_GBIAS_P_NSE` (rad/s²) | K10: gyro offset drift 0.015 °/s/min max, 1σ = max/3 | 1.0·10⁻⁴ | 1·10⁻³ | 1·10⁻⁴ |
| `EK3_ABIAS_P_NSE` (m/s³) | K10: accel offset drift 0.002 m/s²/min max, 1σ = max/3 | 7.9·10⁻⁴ | 2·10⁻² | 8·10⁻⁴ |
| `EK3_MAG_M_NSE` (Gauss) | RM3100 at cycle count 200 (the driver setting): 15 nT | 1.5·10⁻⁴ | 0.05 | default |

- **Offsets (GBIAS, ABIAS).** The strength of the K10 is the stability of its offsets: the random walk from the datasheet is 10 times (gyro) and 25 times (accel) below the EKF3 defaults, which are the top of their range. With these values EKF3 learns the offsets more slowly and with less noise. An offset changing faster than expected is still covered by `EK3_GYRO_P_NSE` and `EK3_ACC_P_NSE`, left at their defaults.
- **White noise (GYRO, ACC).** The white noise of the K10 is in the same class as the common MEMS parts the EKF3 defaults are tuned for. The defaults are about 15 times (gyro) and 50 times (accel) the sensor-only value: that is the margin for vibration, clipping, scale factor and timing errors, which depend on the installation and not on the datasheet. Values with a 5× and 4× margin would be 4.8·10⁻³ and 2.9·10⁻². Try them only while watching the innovations in telemetry (`EKF_STATUS_REPORT`, ratios below 1, first on the ground and then in flight): the board has no log.
- **Compass (MAG_M_NSE).** The 15 nT of the RM3100 are 70 times below the minimum EKF3 accepts in the fusion (0.01 Gauss). The measurement error comes from the calibration residual (`COMPASS_CAL_FIT` accepts up to 16 mGauss) and from currents near the compass, that is from the installation. With a good calibration (fit below 5 mGauss) and no current interference, 0.02 can be tried.

These values are for the K10. The full K01 datasheet was not available.

### SPI bus

- ESP-IDF takes at most 6 devices per SPI host, and an `ESP32_SPIDEV` line with `LoSpeed` different from `HiSpeed` takes two of them. Today: SCH16T 2, BMP581 1 + 1, RM3100 1, ELVH 1 = 6. One more device stops the boot with `SPI: no room for <name> on its bus`.
- Every device on the bus must be listed, even without a driver, so that its chip select stays high. The HAL drives every CS high before the buses are set up.
- Part limits: SCH16T 10 MHz, BMP581 12 MHz, RM3100 1 MHz, ELVH 800 kHz.
- To read only one barometer, remove the second `BARO` line and keep its `ESP32_SPIDEV` line.

### Sensor orientation

The last word of the `IMU` and `COMPASS` lines is the rotation from the sensor frame to the board frame (X forward, Y right, Z down). If the board is mounted rotated on the vehicle, leave the hwdef alone and use `AHRS_ORIENTATION`; for the compass there is also `COMPASS_ORIENT`.

### ELVH airspeed

New driver `AP_Airspeed_ELVH`: All Sensors ELVH differential pressure sensors with SPI output and transfer function A (10–90 %), for example ELVH-L02D-HRRD-C-NSA4.

- `ARSPD_TYPE 21` (SPI-ELVH). It is the board default on ArduPlane; on the other vehicles the default is 0.
- `ARSPD_ELVH_RNG` selects the range by the code in the part number: 1 F50D (±0.5 inH2O), 2 L01D, 3 L02D (±2 inH2O, board default), 4 L04D, 5 L05D, 6 L10D, 7 L20D, 8 L30D, 9 L60D, 10 MF25D (±2.5 mbar), 11 MF12D, 12 M025D, 13 M050D, 14 M075D, 15 M100D, 16 M160D, 17 M250D (±250 mbar). 0 takes the full scale from `ARSPD_PSI_RANGE`. It is read at boot.
- The driver reads at 100 Hz; at start-up it needs three valid reads (status, pressure inside the calibrated band, consistent temperature). Pressures outside the calibrated band are discarded: the L02D saturates at ±498 Pa, about 28 m/s indicated airspeed.
- SITL has a simulated ELVH-L04D on `SPI:elvh`; the test `Tools/autotest/autotest.py test.Plane.AirspeedDrivers` flies it both with `ARSPD_ELVH_RNG 4` and with `ARSPD_PSI_RANGE`.

### GPS

`defaults.parm` sets a u-blox on SERIAL3 at 460800 (`SERIAL3_BAUD 460`) with `GPS_AUTO_CONFIG 0`: with auto-configuration the driver accepts the u-blox only at 230400, so the module must already be configured to send UBX (NAV-PVT and DOP) at 460800 and saved to its flash. The GPS is powered by the board: after a USB replug it does a cold start.

### Serial ports and telemetry

- SERIAL0: USB Serial/JTAG. Open the port with DTR and RTS low: closing it after opening it with DTR/RTS high (the pyserial and pymavlink default) silences the board USB until a replug. `HAL_ESP32_USB_NO_CHIP_RESET` stops the USB lines from resetting the chip.
- SERIAL1: Wi-Fi UDP.
- SERIAL2, SERIAL3: UARTs on S2 and S1. The ESP32 HAL supports `SERIALn_OPTIONS` bit 3 (RX/TX swap).
- Stream rates are per MAVLink channel (`MAVn_*`): SERIAL0 is MAV1, Wi-Fi MAV2, SERIAL2 MAV3.

## Live FFT

With `INS_FFT_LIVE 1` the board sends the vibration spectrum of the SCH16T (gyro and accelerometer, 3.125–362.5 Hz, four times a second) to the GCS as `DEBUG_FLOAT_ARRAY` messages, about 4.2 kB/s on each link. The FFT runs in a low priority thread on core 1. Message format, computation and a receiver example: [FFT_LIVE.md](../../../AP_InertialSensor/FFT_LIVE.md).

## Status LEDs

| LED | State | Meaning |
|---|---|---|
| LED1 | fast flash (about 2 Hz) | starting up |
| LED1 | solid | initialisation finished (vehicle setup and IMU calibration) |
| LED2 | off | no GPS or no 3D fix |
| LED2 | 1 Hz blink | 3D fix, AHRS without an absolute position yet |
| LED2 | solid | 3D fix and absolute position from the AHRS |

Arming, pre-arm and calibration states are shown by the GCS.

## Use of the two cores

| Core | Threads |
|---|---|
| HP CPU0 | main loop (vehicle, AHRS/EKF3), timer, UART, SPI bus thread (SCH16T at 1 kHz, baros, compass) |
| HP CPU1 | Wi-Fi/lwIP, IO, storage, library threads |

Set from the hwdef: `HAL_ESP32_FASTCPU`, `HAL_ESP32_SLOWCPU`, `HAL_ESP32_DEVICE_BUS_CPU`, `HAL_ESP32_THREAD_CPU` (and `HAL_ESP32_CAN_THREAD_CPU`, `HAL_ESP32_IO_THREAD_CPU`). The load is sent in telemetry: `SYS_STATUS.load` is the busiest core, measured from the FreeRTOS idle time; `NAMED_VALUE_FLOAT` `CPU0` and `CPU1` give each core in %.

## Changes to ArduPilot

Build:
- `Tools/ardupilotwaf/boards.py`, `esp32.py`: `esp32s31` board class (toolchain `riscv32-esp-elf`, `-march=rv32imafcb_zicsr_zifencei_zaamo_zalrsc_zcb_zcmp_zcmt -mabi=ilp32f`, without the PIE/Xesploop extensions, because ESP-IDF moves the tasks that use them to core 1); DroneCAN enabled when the hwdef has `ESP32_CAN` lines.
- `libraries/AP_HAL_ESP32/targets/esp32s31/`: ESP-IDF project, `sdkconfig.defaults` (320 MHz, newlib, PSRAM disabled), partitions (6 MB app). The CMake file links every component, because in ESP-IDF 6 the HAL is split into `esp_hal_*` components with their own ROM linker scripts.

ESP32 HAL on ESP-IDF 6.1:
- `CANIface` (new): CAN on TWAI with the `esp_twai` node driver; RX through a FreeRTOS queue from the ISR, TX slots, loopback for the DroneCAN time sync, bus-off recovery at 1 Hz.
- `UARTDriver`: `USB_SERIAL_JTAG` port; RX/TX swap with `SERIALn_OPTIONS`.
- `Scheduler`, `DeviceBus`, `WiFi*`: cores set from the hwdef, bus and library threads with a fixed affinity.
- `SPIDevice`: short transfers (≤ 64 bytes) polled; all CS driven high before any bus setup; stop at boot when an SPI host has no room for a device.
- `Util`: load of each core from the FreeRTOS idle time.
- `Semaphores`: waits rounded up to the next tick.
- Legacy RMT (PWM/PPM RC input) and software I2C (Xtensa only) made conditional; MCPWM capabilities from `mcpwm_ll.h`; empty I2C table allowed; `ffconf.h` with `MIN`/`MAX`; `RCOutput` without outputs.
- `esp32_hwdef.py`: `ESP32_CAN TX RX` keyword.

Drivers and libraries:
- `AP_InertialSensor_SCH16T` (new): Murata SCH16T-K01/K10 directly on SPI.
- `AP_InertialSensor_Spectrum` (new): live vibration spectrum of the IMU over MAVLink (`INS_FFT_LIVE`, `AP_INERTIALSENSOR_SPECTRUM_ENABLED`), with the `FFTLive` autotest.
- `AP_Airspeed_ELVH` (new, `ARSPD_TYPE 21`, `ARSPD_ELVH_RNG`), with the simulated sensor in SITL and the case in `AirspeedDrivers`.
- `AP_Notify`: `AP_BoardLED_InitGPS`, the two status LEDs.
- `GCS_MAVLink`: `SYS_STATUS.load` from the core load when the HAL measures it, `NAMED_VALUE_FLOAT` `CPU0`/`CPU1`.
- ArduPlane: `PLANE_GCS_SEND_TIME_US`, the per-loop time of `update_send` set by the board (3 ms here).

## Status

The HAL, the drivers and the pinout are the same that already ran on the V1.2 board (USB, Wi-Fi, SCH16T, baros, RM3100, GPS, LEDs). This EKF3 build has not been started on the hardware yet. To check at the first boot:
1. boot, USB console (SERIAL0) and Wi-Fi;
2. IMU variant and rotation, compass rotation, calibrations;
3. EKF3: alignment, use of GPS and compass, load of core 0 (`CPU0`) with EKF3 in the main loop;
4. ELVH on a board where it is connected: `DiffPress` close to 0 with the pitot covered and a plausible `Temp`.
