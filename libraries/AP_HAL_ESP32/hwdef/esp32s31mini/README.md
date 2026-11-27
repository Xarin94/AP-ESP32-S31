# ArduPilot on the MINI AHRS board (ESP32-S31)

Standard ArduPilot, with EKF3, ported to the ESP32-S31 (ESP-IDF 6.1) for the MINI AHRS V1.5 board, module ESP32-S31-WROOM-1-N16R16V. Up to release 0.0.2 the board was the V1.2: its pinout is in the tag `v0.0.2`. The V1.4, with a u-blox DAN-F10N on the board, is `esp32s31mini14`: its [README](../esp32s31mini14/README.md) compares the V1.4 and V1.5 pinouts.

- Board: `esp32s31mini`, hwdef in this directory.
- Base: ArduPilot master `dc78bc5` (October 2026).
- Vehicles: ArduPlane, ArduRover, ArduCopter (8 PWM outputs, DroneCAN, see below).
- Navigation: ArduPilot AHRS/EKF3, unmodified (`AHRS_EKF_TYPE 3`, the default).
- The LP core, the third CPU of the S31, supervises the threads of the two main cores and resets the board when the main loop stops: see [LP core](#lp-core-supervisor-watchdog-and-diagnostics).

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

[`firmware/plane`](firmware/plane), [`firmware/copter`](firmware/copter) and [`firmware/rover`](firmware/rover) hold the ready-to-flash images of release 0.0.3 (tag `v0.0.3`, [GitHub release](https://github.com/Xarin94/AP-ESP32-S31/releases/tag/v0.0.3), ArduPlane / ArduCopter / ArduRover V4.8.0-dev, ESP-IDF v6.1, flash in QIO), built for the V1.5 pinout: `bootloader.bin` at 0x2000, `partition-table.bin` at 0x10000, `ardupilot.bin` at 0x20000, and their `SHA256SUMS`. Flash them as below, from inside the directory and with the file names without their build subdirectories. ArduPlane was tested on the board, ArduCopter and ArduRover are built from the same code but were not started on the board. They are not rebuilt automatically: a later commit may change the code without changing them. They do not match the V1.2 pinout (IMU bus, compass chip select, GPS serial, CAN, SERVO7-8): for a V1.2 board use the images of release 0.0.2 (tag `v0.0.2`); the V1.4 has its own board, `esp32s31mini14`, built from the sources. The images of release 0.0.1 (tag `v0.0.1`, [GitHub release](https://github.com/Xarin94/AP-ESP32-S31/releases/tag/v0.0.1)) read the flash in DIO: with a GPS fix and EKF3 using it they hold the main loop at about 177 Hz instead of 200.

## Flashing

Over USB (USB-C connector, USB Serial/JTAG of the ESP32-S31). On the board's USB link only `--no-stub` at 115200 baud works:

```sh
cd build/esp32s31mini/esp-idf_build
python -m esptool --chip esp32s31 -p <PORT> -b 115200 --no-stub write-flash \
    --flash-mode keep --flash-freq keep --flash-size 16MB \
    0x2000 bootloader/bootloader.bin 0x10000 partition_table/partition-table.bin 0x20000 ardupilot.bin
```

The images carry their flash mode, QIO (DIO in release 0.0.1): `--flash-mode keep` leaves it as built, `--flash-mode dio` would slow the board down (see [Use of the two cores](#use-of-the-two-cores)). The bootloader sets the mode: a board that has the 0.0.1 bootloader needs the new `bootloader.bin` written once. To update only the firmware, `0x20000 ardupilot.bin` is enough: the bootloader, the partition table and the parameters are kept. If the board ran another ArduPilot firmware before, reset the parameters at the first boot (`FORMAT_VERSION 0`, or "Reset to defaults" from the GCS).

After flashing, the chip stays in download mode (esptool resets it with DTR high, which selects download mode again). The new firmware starts after unplugging and replugging the USB cable, or after a reset over USB with DTR low and RTS high:

```sh
python -c "import serial, time; s = serial.Serial('<PORT>'); s.rts = False; s.dtr = False; s.rts = True; time.sleep(0.1); s.rts = False"
```

If Windows reports "USB device not recognized": with the USB unplugged, connect pad 27 of the module (IO61) to GND, plug the USB in, remove the wire and, with the USB still connected, flash adding `--before no-reset`.

## Board map

| Function | GPIO | ArduPilot |
|---|---|---|
| SPI bus 1 (IMU only) SCK / MOSI / MISO | 20 / 21 / 22 | `ESP32_SPIBUS SPI2_HOST` |
| SCH16T CS | 25 | `IMU SCH16T`, `ROTATION_ROLL_180`; no RESET or DRY line on the V1.5 |
| SPI bus 2 (sensors and SD) SCK / MOSI / MISO | 35 / 36 / 37 | `ESP32_SPIBUS SPI3_HOST` |
| BMP581 #1 / #2 CS | 23 / 24 | `BARO BMP581` x2 |
| RM3100 CS (`CSmag`) | 2 | `COMPASS RM3100`, `ROTATION_ROLL_180` (module upside down under the board) |
| ELVH CS (`TOT`) | 38 | airspeed `SPI:elvh`, `ARSPD_TYPE 21` |
| SD memory CS (`cssd`) | 39 | `ESP32_SDSPI SPI3_HOST`, logs in `/SDCARD/APM/LOGS` |
| GPS serial RX / TX (`rxgps` / `txgps`) | 3 / 4 (connector S3) | SERIAL3, GPS |
| UART2 RX / TX | 10 / 11 (connector S2) | SERIAL2, telemetry or serial RC (CRSF/ELRS: `SERIAL2_PROTOCOL 23`) |
| UART1 RX / TX | 8 / 9 (connector S1) | **SERIAL4** (SERIAL3 on the V1.2) |
| USB D- / D+ | 33 / 34 | SERIAL0 (USB Serial/JTAG, MAVLink) and ESP-IDF console |
| CAN TX / RX (MCP2542FD) | 57 / 56 | `ESP32_CAN`, DroneCAN |
| SERVO1-8 | 42, 43, 44, 45, 46, 47, 54, 55 | `ESP32_RCOUT`, MCPWM; outputs 1-2, 3-4, 5-6, 7-8 share a timer (one frequency) |
| LED1 / LED2 | 13 / 12 | status LEDs, see below |
| Wi-Fi | — | SERIAL1, access point `ardupilot-s31` (password `ardupilot1`), UDP 192.168.4.1:14550 |

Notes on the V1.5 board:
- On the V1.2 the module pads 42-53 were not on the GPIO of the schematic. The V1.5 keeps UART1, UART2 and the LEDs on the same pads, so they stay on the GPIOs measured on the V1.2: connectors S1 and S2 on IO8/IO9 and IO10/IO11 (schematic 10/11 and 12/13), LEDs on IO13 and IO12 (schematic IO14 and IO15). The pin marked `R` is the board RX: connect it to the TX of the device. LED3 is not used.
- CAN, `CSmag`, the GPS serial, the SD card and SERVO7-8 are on new pins, taken from the schematic.
- The IMU has its own SPI bus: the SCH16T read at 1 kHz no longer waits for the baros, the compass, the airspeed sensor or the SD card.
- IO36 and IO37 (MOSI and MISO of SPI bus 2) are strapping pins. IO36 has a pull-up in the module; IO37 has none and, according to the datasheet, must not float at reset (with the eFuses it selects the JTAG source).
- No dedicated RC input: RC comes from a serial protocol or DroneCAN.
- Scripting is disabled.

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

EKF3 takes its process noises as the standard deviation per prediction step divided by the step: variance per step = (dt · NSE)². A white noise of density N gives a variance of N² · dt, so NSE = N / √dt. An offset drifting as a random walk of σ over a time T gives NSE = σ / (√T · √dt). The values below are computed with dt = 12 ms (`EKF_TARGET_DT`); with the 400 Hz loop of the board EKF3 predicts about every 12.5 ms (five IMU frames), so the variance per second is close to the computed one (it was 0.83 times it with the 200 Hz loop, a prediction every 10 ms).

| Parameter | Sensor data | Sensor only | EKF3 default | In `defaults.parm` |
|---|---|---|---|---|
| `EK3_GYRO_P_NSE` (rad/s) | K10: 6 m°/s/√Hz | 9.6·10⁻⁴ | 1.5·10⁻² | default |
| `EK3_ACC_P_NSE` (m/s²) | K10: 0.8 mm/s²/√Hz | 7.3·10⁻³ | 0.35 | default |
| `EK3_GBIAS_P_NSE` (rad/s²) | K10: gyro offset drift 0.015 °/s/min max, 1σ = max/3 | 1.0·10⁻⁴ | 1·10⁻³ | 1·10⁻⁴ |
| `EK3_ABIAS_P_NSE` (m/s³) | K10: accel offset drift 0.002 m/s²/min max, 1σ = max/3 | 7.9·10⁻⁴ | 2·10⁻² | 8·10⁻⁴ |
| `EK3_MAG_M_NSE` (Gauss) | RM3100 at cycle count 200 (the driver setting): 15 nT | 1.5·10⁻⁴ | 0.05 | default |

- **Offsets (GBIAS, ABIAS).** The strength of the K10 is the stability of its offsets: the random walk from the datasheet is 10 times (gyro) and 25 times (accel) below the EKF3 defaults, which are the top of their range. With these values EKF3 learns the offsets more slowly and with less noise. An offset changing faster than expected is still covered by `EK3_GYRO_P_NSE` and `EK3_ACC_P_NSE`, left at their defaults.
- **White noise (GYRO, ACC).** The white noise of the K10 is in the same class as the common MEMS parts the EKF3 defaults are tuned for. The defaults are about 15 times (gyro) and 50 times (accel) the sensor-only value: that is the margin for vibration, clipping, scale factor and timing errors, which depend on the installation and not on the datasheet. Values with a 5× and 4× margin would be 4.8·10⁻³ and 2.9·10⁻². Try them only while watching the innovations (`EKF_STATUS_REPORT` in telemetry or `XKF3`/`XKF4` in the log, ratios below 1, first on the ground and then in flight).
- **Compass (MAG_M_NSE).** The 15 nT of the RM3100 are 70 times below the minimum EKF3 accepts in the fusion (0.01 Gauss). The measurement error comes from the calibration residual (`COMPASS_CAL_FIT` accepts up to 16 mGauss) and from currents near the compass, that is from the installation. With a good calibration (fit below 5 mGauss) and no current interference, 0.02 can be tried.

These values are for the K10. The full K01 datasheet was not available.

### SPI bus

- Two buses: SPI2 (`ESP32_SPIBUS` line 0) has only the SCH16T, SPI3 (line 1) the two BMP581, the RM3100, the ELVH and the SD card.
- ESP-IDF takes at most 6 devices per SPI host, and an `ESP32_SPIDEV` line with `LoSpeed` different from `HiSpeed` takes two of them. Today: SPI2 SCH16T 2; SPI3 BMP581 1 + 1, RM3100 1, ELVH 1, SD 1 = 5. One more device stops the boot with `SPI: no room for <name> on its bus`.
- Every device on the bus must be listed, even without a driver, so that its chip select stays high. The HAL drives every CS high before the buses are set up: the `sdcs` line, opened by no driver, keeps the SD card deselected until the SD driver takes it.
- Part limits: SCH16T 10 MHz, BMP581 12 MHz, RM3100 1 MHz, ELVH 800 kHz. The SD card is identified at 400 kHz, then runs at `HAL_ESP32_SDSPI_MAX_FREQ_KHZ` (20 MHz).
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

`defaults.parm` sets a u-blox on the GPS serial (IO3/IO4, SERIAL3) at 460800 (`SERIAL3_BAUD 460`) with `GPS_AUTO_CONFIG 0`: with auto-configuration the driver accepts the u-blox only at 230400, so the module must already be configured to send UBX (NAV-PVT and DOP) at 460800 and saved to its flash. The GPS is powered by the board: after a USB replug it does a cold start. On ArduPlane EKF3 starts only with a 3D fix: until then the AHRS runs on DCM and the PreArm check reports "AHRS: not using configured AHRS type". A GPS on connector S1 is now on SERIAL4: to use it as GPS1 set `SERIAL3_PROTOCOL -1`, `SERIAL4_PROTOCOL 5` and `SERIAL4_BAUD 460`. If the GPS sends nothing, try `SERIAL3_OPTIONS 8` (RX/TX swap).

### SD card and logs

The SD memory is soldered on the board and works in SPI mode on SPI bus 2, with the sensors; on the bench board it is an SD NAND that identifies itself as `APPSD`, SDSC, 480 MB, and runs at 20 MHz. A microSD card in a socket adapter on the same pads works as well (checked with a 64 GB card). The card needs a FAT filesystem: the ESP-IDF FatFs is built without exFAT, so a card sold as exFAT (SDXC, 64 GB and more) does not mount (`vfs_fat_sdmmc: failed to mount card (13)` on the USB console). `MAV_CMD_STORAGE_FORMAT` (the SD card format of the GCS, disarmed only) partitions it with one MBR partition and formats it FAT32 with 32 kB clusters, in about 21 s for 64 GB; the logger and the other filesystem users get `EBUSY` meanwhile. The IO thread mounts it at boot and, while disarmed, retries every 3 s; the logs go to `/SDCARD/APM/LOGS` (`LOG_BACKEND_TYPE 1`, the default) and are written from arming (`LOG_DISARMED 0`). Over MAVFTP the paths are the same, starting from `/SDCARD`. The SD pins stay the same on the later board versions.

The log writer runs in a low priority thread and writes from a RAM buffer of 48 kB (`LOG_FILE_BUFSIZE`, default from `HAL_LOGGING_FILE_BUFSIZE`), in blocks of 4 kB with an fsync after each (the ESP32 defaults of `HAL_ESP32_FS_IO_SIZE` and `HAL_ESP32_FS_FSYNC_SIZE`): a power loss costs the block being written and what is still in the buffer, about 0.1 s of log at `LOG_BITMASK 65535` when the card keeps up. Each fsync rewrites the FAT and the directory entry. Up to release 0.0.3 the board wrote 32 kB blocks, about 1 s of log. The SD driver is a copy of the ESP-IDF one (`targets/esp32s31/esp-idf/components/esp_driver_sdspi`): while the card is busy writing it deselects the card, releases the bus to the baros, the compass and the airspeed sensor and sleeps, instead of holding the bus and polling. MISO does not go high with the card deselected, so the ESP-IDF wait for MISO high before every command (a 40 ms busy loop holding the bus) is off (`HAL_ESP32_SDSPI_WAIT_FOR_MISO -1`). Old logs are deleted at boot to keep `LOG_FILE_MB_FREE` free: the ArduPilot default, 500 MB, is more than the card, and deleted every log at each boot; the board default is 128 MB (`HAL_LOGGER_MIN_MB_FREE`), about an hour of log at `LOG_BITMASK 65535`. The LP core reports the gaps of the bus thread (channel `SPI1`, see [LP core](#lp-core-supervisor-watchdog-and-diagnostics)). If the card does not mount or the log has write errors, lower `HAL_ESP32_SDSPI_MAX_FREQ_KHZ`.

Measured on the bench on 2026-11-27 with ArduPlane, disarmed, `LOG_DISARMED 1` and `LOG_BITMASK 65535`, for 300 s, with the mosaic-X5 on S1 sending SBF at 50 Hz (460800) to load the GPS driver and the log: at the 400 Hz loop 49.6 kB/s of log (38.6 kB/s at 200 Hz), every message written (`DSF` drops 0, also at the start of the log), at least 26.7 kB of the 48 kB buffer free after the first second (11.5 kB in the first, at the start of the log), core 1 at 39% as without the log, no thread stalls, longest `SPI1` gap 11 ms, IMU messages at 397 Hz with no gap over 3.8 ms. The first log after a format dropped 202 messages in its first 0.2 s, probably while `/APM/LOGS` was created; three later logs dropped none. With 32 kB blocks a write took about 40 ms and an fsync about 7 ms. Before the changes to the SD driver, with the ESP-IDF and ArduPilot defaults, the same log took core 1 to 100%, dropped about 370 messages per second, held the sensors off the bus for up to 100 ms and the RC input, IO and storage threads for up to a second.

The SD erase command was accepted by the memory without erasing every sector: a full erase of the bench board was done by writing 0xFF to every sector and reading them back, then a new FAT. A reset of the chip in the middle of a write leaves the card inside it, deaf to CMD0, until a power cycle. Before the mount the HAL clocks it out of the write (`SdCard.cpp`): it waits while the card is busy, completes a data block left half sent (rejected by the card, the CRC is wrong) and ends a multi-block write with the stop token; a card that is not in a write ignores this. Checked with four resets forced in the middle of multi-block writes, with the card busy between two blocks and after the start token of a block: the card mounted after each one.

### CAN

TWAI controller on the MCP2542FD transceiver, TX IO57 and RX IO56. `defaults.parm` enables DroneCAN at 1 Mbit/s (`CAN_P1_DRIVER 1`, `CAN_D1_PROTOCOL 1`); the servos stay on PWM (`CAN_D1_UC_SRV_BM 0`), a DroneCAN servo or ESC is added with `CAN_D1_UC_SRV_BM` / `CAN_D1_UC_ESC_BM`. DroneCAN compasses are enabled.

### Serial ports and telemetry

- SERIAL0: USB Serial/JTAG, MAVLink and the ESP-IDF console (start-up log, HAL messages, panics) on the same port; a GCS skips the text. With DTR low and RTS high the USB Serial/JTAG resets the chip, and on the S31 (chip revision v0.0) `HAL_ESP32_USB_NO_CHIP_RESET` does not stop it, although the disable bit reads back set. Linux raises DTR and RTS when the port is opened and pyserial (pymavlink, MAVProxy) then lowers DTR first, which resets the board: it reboots in a few seconds. To connect without a reset, open the port and lower RTS before DTR (pyserial: `s = serial.Serial(port); s.rts = False; s.dtr = False`).
- SERIAL1: Wi-Fi UDP.
- SERIAL2: UART on connector S2. SERIAL3: UART on connector S3 (IO3/IO4), GPS by default. SERIAL4: UART on connector S1. All three take any serial protocol (`SERIALn_PROTOCOL`). The ESP32 HAL sets parity, stop bits, RX and TX inversion (`SERIALn_OPTIONS` bits 0 and 1) and the RX/TX swap (bit 3), not half-duplex, flow control and the pull options.
- SERIAL4 is on the UART0 of the S31, because ESP-IDF 6.1 has no GPIO matrix signals for the UART3 of the S31 and never connects it to the pins. Checked on the V1.4, only compiled for the V1.5.
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
| HP CPU0 | main loop (vehicle, AHRS/EKF3), timer, UART |
| HP CPU1 | SPI bus threads (SCH16T at 1 kHz on its bus; baros, compass and airspeed on the other; priority 24, above Wi-Fi), Wi-Fi/lwIP, IO (SD mount), storage, RC, library threads (DroneCAN, logger, live FFT), `APM_LPSV` (LP core supervisor) |
| LP core | supervisor of the HP threads and watchdog of the main loop, see below |

The main loop runs at 400 Hz (`SCHED_LOOP_RATE` in `defaults.parm`), as on the V1.4. The code and read-only data are copied at start-up from the flash to the 16 MB octal PSRAM of the module and run from there at 200 MHz (`CONFIG_SPIRAM_XIP_FROM_PSRAM` in `targets/esp32s31/esp-idf/sdkconfig.defaults`, the same for both boards); the PSRAM is not added to the heap, so the ArduPilot heap and the DMA buffers stay in internal SRAM. The start-up of the PSRAM takes about 0.8 s, and flash writes (parameters) no longer stop the cache of both cores. Up to release 0.0.3 the code ran from the QIO flash through the 16 kB instruction cache, a large part of core 0 time went into cache refills, and the loop ran at 200 Hz.

Measured on the V1.5 on 2026-11-27 with ArduPlane, at rest and disarmed, the mosaic-X5 on S1 sending SBF at 50 Hz (460800) with a 3D fix, EKF3 started in constant position mode, the log on (`LOG_DISARMED 1`, `LOG_BITMASK 65535`), live FFT off, from `NAMED_VALUE_FLOAT` `CPU0`/`CPU1` and the `PM` log messages:

| Code | `SCHED_LOOP_RATE` | Core 0 | Core 1 | Loop rate (`PM.LR`) |
|---|---|---|---|---|
| flash (release 0.0.3), 300 s | 200 | 70–75% (mean 73%) | 38–43% | 192–200 Hz |
| PSRAM, 90 s | 200 | 29–34% (mean 30.5%) | 37–42% | 200 Hz |
| PSRAM, 300 s | 400 | 44–48% (mean 45.6%) | 37–42% | 395–400 Hz |

At 400 Hz the MAVLink streams keep the requested rate up to 10 Hz with `PLANE_GCS_SEND_TIME_US` at 750 µs. The SCH16T is read at 1 kHz and the main loop waits for an IMU sample, so at 400 Hz the loops last mostly 2 or 3 ms, 2.5 ms on average: the 3 ms loops are just over the long-loop threshold (1.2 times the period), and `PM.NLon` counts about 7% of the loops with `PM.MaxT` at most 3.03 ms.

The measurements below are with the code in the flash and the 200 Hz loop. On the V1.2 (one SPI bus for all the sensors, no log, no CAN) with ArduPlane, a GPS fix with EKF3 using it and the live FFT on (`INS_FFT_LIVE 1`): core 0 at 61%, core 1 at 35%, scheduler load 61%. The flash mode counted: in DIO, the same load took core 0 to 88% and held the loop at 177 Hz ("Main loop slow"), while without the GPS fix it stayed at 200 Hz; compiling for speed (`-O2`) instead of size made no difference. Without a GPS fix the main loop alone reaches about 300 Hz; at 400 Hz it does not keep up and the PreArm check reports "Main loop slow". The SPI bus thread takes about a third of a core: on core 0, at the main loop priority, it held the loop at 190 Hz.

The V1.5 adds the second SPI bus thread, the log writer and DroneCAN on core 1, and the log messages on the main loop. On the bench, with ArduPlane at rest, disarmed, without a GPS fix and without a log running, the main loop holds 200 Hz and `SYS_STATUS.load` reads 46–47%; with the log on (`LOG_DISARMED 1`, `LOG_BITMASK 65535`) core 0 is at 49% and core 1 at 40%, as without it.

With a GPS (Septentrio mosaic-X5 on S1, SBF at 10 Hz), measured on 2026-11-27 with the ArduPlane images of release 0.0.3, at rest and disarmed, live FFT off (`INS_FFT_LIVE 0`), from `NAMED_VALUE_FLOAT` `CPU0`/`CPU1` over 60–90 s; the first two rows are the measurements above, without GPS:

| State | Core 0 | Core 1 |
|---|---|---|
| no GPS fix, log off (2026-11-17, `SYS_STATUS.load`) | 46–47% | – |
| no GPS fix, log on (2026-11-26) | 49% | 40% |
| 3D fix, EKF3 started, not yet using the GPS (constant position mode), log on | 70–72% | 39–41% |
| 3D fix, EKF3 using the GPS, log off | 67–70% (mean 68.5%) | 39–40% |

With the fix EKF3 starts and core 0 goes up by about 20 points; core 1 does not change. No "Main loop slow" in any of these states. With EKF3 using the GPS and the log on it has not been measured.

Set from the hwdef: `HAL_ESP32_FASTCPU`, `HAL_ESP32_SLOWCPU`, `HAL_ESP32_DEVICE_BUS_CPU`, `HAL_ESP32_THREAD_CPU` (and `HAL_ESP32_CAN_THREAD_CPU`, `HAL_ESP32_IO_THREAD_CPU`). The load is sent in telemetry: `SYS_STATUS.load` is the busiest core, measured from the FreeRTOS idle time; `NAMED_VALUE_FLOAT` `CPU0` and `CPU1` give each core in %.

## LP core: supervisor, watchdog and diagnostics

The S31 has a third core, the LP core: RISC-V at 40 MHz (from the crystal, `CONFIG_RTC_FAST_CLK_SRC_XTAL`) with 32 KB of LP RAM. On this board it supervises the threads of the two HP cores (`define HAL_ESP32_LP_SUPERVISOR 1` in the hwdef). It runs from LP RAM and works only on LP RAM, so it keeps running while the HP cores are stopped: flash erase, deadlock, crash. It starts at the end of the vehicle setup.

How it works:
- Every HAL thread stores a counter in LP RAM at each cycle: main loop, timer, UART, RCIN, RCOUT, IO, storage, the SPI bus threads and the idle tasks of the two cores. On the HP cores it costs one store, without waits or locks. A library thread can do the same with `hal.scheduler->heartbeat()` (channel `NAV`, for a navigation filter in its own thread; EKF3 runs in the main loop, so it is not used here).
- The LP core reads the counters every millisecond and measures for each the time without progress: maximum since the start, maximum of the last second, number of stalls (gaps over the threshold of the channel). For the idle tasks it is the longest time the core was never idle.
- When a gap passes its threshold it records an event with the scheduler task the main loop was in and the last MAVLink command. A task on HP core 1 (`APM_LPSV`, 50 Hz, top priority, a few microseconds) copies these two values to LP RAM.
- Watchdog: if the main loop makes no progress for `HAL_ESP32_LP_WDT_MS` (2000 ms) outside an `expect_delay_ms()`, the LP core writes a post-mortem to RTC memory (task, MAVLink command and message, age of every channel) and asks for a reboot; `APM_LPSV` does it with `esp_system_abort()`, the panic path of the task watchdog. It is switched off with bit 0 of `BRD_OPTIONS` (default 1 on this board; with 0 the LP core only measures). The ESP-IDF task watchdog (3 s on core 0) stays as backstop for a stop of both cores.
- `expect_delay_ms()` is now implemented by the ESP32 HAL: the operations that announce a long delay of the main loop (parameter erase, calibrations) do not trip the watchdog, which restarts its timeout at the end of the delay.
- The HP side collects the results in the IO thread, once a second (the events at 10 Hz), and writes them to the log and to the GCS: nothing blocking on the threads that matter.
- The PC of the two HP cores (from the bus monitor, an HP peripheral) is in the events and in the post-mortem only with `define HAL_ESP32_LP_PC_SAMPLING 1`. It is off because it is not verified on the board.
- Start guard (`HAL_ESP32_LP_START_GUARD`, on): a flag in NVS marks the start of the LP core and is cleared after 10 s of operation. If a start never confirms it, the next boot skips the LP core (`LP supervisor skipped` to the GCS) and the boot after that tries again, so that a start that hung the chip cannot leave the board unreachable. A reboot within 10 s of the LP start counts as an incomplete start.

Where to read the results:

| Where | What |
|---|---|
| `@SYS/threads.txt` (MAVFTP: Mission Planner, or MAVProxy `ftp get @SYS/threads.txt`) | per channel: age, maximum of the last second, maximum, stalls, threshold; last stalls with task and MAVLink command; reason of the last reset and post-mortem |
| log `LPSV` | each second, per channel: `WMax` (last second), `Max`, `NSt` (stalls), `Age` |
| log `LPEV` | each stall: channel, length, task, MAVLink command, `PC0`/`PC1` (0 without PC sampling) |
| log `LPPM` | after a reset of the LP watchdog, each second for the whole session |
| GCS | `LP: main stall N ms in <task>` (main loop stalls of 200 ms or more), `LP wdog reset: MAIN N ms in <task>` and `LP wdog: MAVLink cmd … msg …` after the reboot, `LP supervisor stopped` if the LP core stops, `LP supervisor skipped: last start incomplete` (start guard), `Last reset: panic` (or task watchdog, brownout…) after an abnormal reset |

The post-mortem lives in RTC memory: it survives the reboot but not a power cycle. If the USB serial does not come back after the reboot, read it over Wi-Fi before unplugging the USB.

With PC sampling on, a PC is translated to function and line with `riscv32-esp-elf-addr2line -pfiaC -e build/esp32s31mini/esp-idf_build/ardupilot.elf 0x…` (with the ELF of the same build).

Stall thresholds in ms (`LPSupervisor.cpp`): `MAIN` 100, `TIMR`/`UART` 20, `RCIN`/`RCOU` 50, `IO`/`STOR` 500, `NAV` 100, `IDL0`/`IDL1` 500, bus threads 60 (`SPI0` IMU bus, `SPI1` sensor and SD bus).

Code:
- `targets/esp32s31/esp-idf/components/lp_supervisor/`: the LP program (`lp/lpsv_lp.c`), the HP loader (`lpsv.c`) and the shared memory (`include/lpsv_shared.h`). An ESP-IDF component built with the ULP rules and embedded in the application; `sdkconfig.defaults` enables the LP core and reserves 8 KB of LP RAM for it.
- `AP_HAL_ESP32/LPSupervisor.{h,cpp}`: start and start guard, heartbeats, `APM_LPSV` task, statistics, events, log and messages; hooks in `Scheduler.cpp` (HAL threads, `expect_delay_ms()`), `DeviceBus.cpp` (bus threads), `Util.cpp` (`@SYS/threads.txt`).

## Changes to ArduPilot

Build:
- `Tools/ardupilotwaf/boards.py`, `esp32.py`: `esp32s31` board class (toolchain `riscv32-esp-elf`, `-march=rv32imafcb_zicsr_zifencei_zaamo_zalrsc_zcb_zcmp_zcmt -mabi=ilp32f`, without the PIE/Xesploop extensions, because ESP-IDF moves the tasks that use them to core 1); DroneCAN enabled when the hwdef has `ESP32_CAN` lines.
- `libraries/AP_HAL_ESP32/targets/esp32s31/`: ESP-IDF project, `sdkconfig.defaults` (320 MHz, flash in QIO, code run from the octal PSRAM, newlib), partitions (6 MB app). The CMake file links every component, because in ESP-IDF 6 the HAL is split into `esp_hal_*` components with their own ROM linker scripts.

ESP32 HAL on ESP-IDF 6.1:
- `CANIface` (new): CAN on TWAI with the `esp_twai` node driver; RX through a FreeRTOS queue from the ISR, TX slots, loopback for the DroneCAN time sync, bus-off recovery at 1 Hz.
- `AnalogIn`: on the S31 (differential SAR ADC), one input range, 17-bit codes and the nominal code-to-voltage scale, inverted on an N-side input.
- `UARTDriver`: `USB_SERIAL_JTAG` port; RX/TX swap and inversion with `SERIALn_OPTIONS`, parity and stop bits; the console moves to the USB Serial/JTAG driver when SERIAL0 opens.
- Console on the USB Serial/JTAG (`sdkconfig.defaults`): UART0 is SERIAL4.
- `Scheduler`, `DeviceBus`, `WiFi*`: cores set from the hwdef, bus and library threads with a fixed affinity.
- `SPIDevice`: short transfers (≤ 64 bytes) polled; all CS driven high before any bus setup; stop at boot when an SPI host has no room for a device.
- `Util`: load of each core from the FreeRTOS idle time; LP core results in `@SYS/threads.txt`.
- `LPSupervisor` (new) and the `lp_supervisor` ESP-IDF component (new): supervisor, watchdog and diagnostics of the HP threads on the LP core; `Scheduler`: heartbeats of the HAL threads, `expect_delay_ms()` and `in_expected_delay()`; `DeviceBus`: heartbeat of the bus threads.
- `SdCard`: SD card in SPI mode on a host shared with the sensors, host and DMA channel from the `ESP32_SDSPI` line, clock from `HAL_ESP32_SDSPI_MAX_FREQ_KHZ`, wait for MISO high before each command from `HAL_ESP32_SDSPI_WAIT_FOR_MISO`; before the mount, recovery of a card left in a write by a reset; `format_sdcard()`: FAT format of the card, mounted or not (FatFs `f_fdisk`/`f_mkfs` on a drive of its own when it has no FAT volume).
- `esp_driver_sdspi` ESP-IDF component, copied in `targets/esp32s31/esp-idf/components/` and registered after the ESP-IDF ones, so it replaces theirs: while the card is busy after a write it is deselected, the bus released and the task sleeps.
- `HAL_ESP32_Class.cpp`: a fourth `ESP32_SERIAL` line is SERIAL4.
- `Semaphores`: waits rounded up to the next tick.
- Legacy RMT (PWM/PPM RC input) and software I2C (Xtensa only) made conditional; MCPWM capabilities from `mcpwm_ll.h`; empty I2C table allowed; `ffconf.h` with `MIN`/`MAX`; `RCOutput` without outputs.
- `esp32_hwdef.py`: `ESP32_CAN TX RX` keyword; `HAL_ESP32_UART_COUNT`; no double comma in the SPI bus table, so a board with two buses compiles.

Drivers and libraries:
- `AP_InertialSensor_SCH16T` (new): Murata SCH16T-K01/K10 directly on SPI.
- `AP_InertialSensor_Spectrum` (new): live vibration spectrum of the IMU over MAVLink (`INS_FFT_LIVE`, `AP_INERTIALSENSOR_SPECTRUM_ENABLED`), with the `FFTLive` autotest.
- `AP_Airspeed_ELVH` (new, `ARSPD_TYPE 21`, `ARSPD_ELVH_RNG`), with the simulated sensor in SITL and the case in `AirspeedDrivers`.
- `AP_Baro_BMP581`: soft reset before the power-on check, so the baro starts after a reboot without a power cycle.
- `AP_Filesystem_ESP32`, `AP_Logger_File`: log write size and fsync interval from the hwdef (`HAL_ESP32_FS_IO_SIZE`, `HAL_ESP32_FS_FSYNC_SIZE`, default 4 kB as before).
- `AP_Filesystem_ESP32`: SD card format (`MAV_CMD_STORAGE_FORMAT`, `format()`), run in the IO thread; every call holds a semaphore that the format takes, and fails with `EBUSY` while a format is pending or running (`close` and `closedir` wait).
- `AP_Notify`: `AP_BoardLED_InitGPS`, the two status LEDs.
- `GCS_MAVLink`: `SYS_STATUS.load` from the core load when the HAL measures it, `NAMED_VALUE_FLOAT` `CPU0`/`CPU1`.
- `AP_HAL::Scheduler::heartbeat()`: heartbeat of long-running library threads, for a HAL that watches its threads from outside; does nothing elsewhere.
- `AP_Scheduler::task_name()`: name of a task from the index in `persistent_data.scheduler_task`, for the stall reports.
- ArduPlane: `PLANE_GCS_SEND_TIME_US`, the per-loop time of `update_send` set by the board (750 µs here for the 400 Hz loop; at 50 Hz the board needs 3 ms).

## Status

Release 0.0.2, checked on the V1.2 board with ArduPlane, at rest with a GPS 3D fix:
- MAVLink on SERIAL0 (USB);
- SCH16T, both BMP581 (calibrated at boot) and RM3100 running;
- u-blox detected at 460800 on S1, 3D fix;
- EKF3: tilt and yaw alignment (MAG0), origin set, using GPS, AHRS on EKF3;
- main loop at 200 Hz with EKF3 using GPS and the live FFT on: core 0 at 61%, core 1 at 35%, scheduler load 61%;
- live FFT: four spectra per second, gyro floor −77 dB and accelerometer −58 dB below the 68 Hz IMU filter.

Release 0.0.1 also checked the Wi-Fi access point, the BMP581 after a reset over USB and the 8 PWM channels set up (`RCOutput::init()` reports 8).

V1.5 board, checked on 2026-11-17 with ArduPlane, at rest and disarmed, without a GPS connected:
- boot, MAVLink on SERIAL0 (USB), Wi-Fi access point started;
- SCH16T on its own bus, both BMP581 calibrated at boot, ELVH airspeed calibrated;
- main loop at 200 Hz, `SYS_STATUS.load` 46–47%;
- SD memory erased and formatted, mounted after a power-on: `/SDCARD/APM/LOGS` created, no log yet (logs start at arming);
- on 2026-11-26, the log while disarmed (`LOG_DISARMED 1`, `LOG_BITMASK 65535`): see [SD card and logs](#sd-card-and-logs); the SD mounted after software resets, also in the middle of a write;
- LP core running, watchdog armed at 2000 ms, all 11 channels reporting (`MAIN`, `TIMR`, `UART`, `RCIN`, `RCOU`, `IO`, `STOR`, `IDL0`, `IDL1`, `SPI0`, `SPI1`), LP tick at most 29 µs out of 1000, no late ticks, no stalls 16 s after the start (longest gaps: 11 ms `SPI1`, 8 ms `MAIN`). The first boot after flashing reported `LP supervisor skipped: last start incomplete`: the start guard found in NVS the flag of an earlier start that did not reach 10 s, and the next boot started the LP core;
- RM3100 not detected: `No Compass backends available` and `PreArm: Compass 1 not healthy`.

Release 0.0.3, checked on the V1.5 board with ArduPlane on 2026-11-27, at rest and disarmed, with the log running (`LOG_DISARMED 1`, `LOG_BITMASK 65535`):
- the ArduPlane images of the release flashed as in [Flashing](#flashing): SCH16T, both BMP581, ELVH and RM3100 (`COMPASS_DEV_ID` 0x11020A) healthy;
- a 64 GB microSD in a socket adapter, sold exFAT, did not mount; `MAV_CMD_STORAGE_FORMAT` formatted it FAT32 in 21 s without a reset; after a reboot it mounted and the logs were written. The release image formatted it again in 22 s (`Format: OK`, no logs left);
- Septentrio mosaic-X5 on S1 (`SERIAL4_BAUD 115`, `GPS2_TYPE 10`, `GPS2_COM_PORT 2`, SBF output of its COM2 saved on the receiver, `GPS_AUTO_CONFIG 0`): detected at boot, 3D fix with 16 satellites. After a restart of the receiver EKF3 stayed in constant position mode for the first minutes of the fix (9–12 satellites, horizontal accuracy 14 m), then used the GPS for horizontal and vertical position (16 satellites);
- no "Main loop slow" at `SCHED_LOOP_RATE` 200, with the GPS fix and EKF3, with or without the log: core 0 at 67–72%, core 1 at 39–41% (see [Use of the two cores](#use-of-the-two-cores)).

Still to check:
1. EKF3 with GPS and compass in motion and in flight;
2. IMU variant and rotation, compass rotation, calibrations;
3. PWM signals on SERVO1-8 (IO42-IO47, IO54, IO55), telemetry on S2;
4. ELVH on a board where it is connected: `DiffPress` close to 0 with the pitot covered and a plausible `Temp`;
5. ArduCopter and ArduRover on the board.
6. V1.5: GPS on SERIAL3 (IO3/IO4); a DroneCAN node seen by the GCS; telemetry over Wi-Fi; LEDs.
7. SD memory: a log written when armed; `BARO` and `MAG` sample intervals while logging, with GPS and EKF3.
8. LP core: a main loop stall reported to the GCS (`MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` 42/24/71/97, a 250 ms loop); the reboot of the LP watchdog (42/24/71/93, a main loop lockup) and its post-mortem in `@SYS/threads.txt`, on the GCS and in `LPPM`.
9. `HAL_ESP32_LP_PC_SAMPLING 1` alone, with the board at hand.
