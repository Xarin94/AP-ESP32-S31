# MINI AHRS V1.4 and V1.5: pinout

Both boards carry the ESP32-S31-WROOM-1-N16R16V module (16 MB flash, 16 MB octal PSRAM) and run the same ArduPilot port:

| Board | hwdef | Notes |
|---|---|---|
| V1.2 | `esp32s31mini` up to tag `v0.0.2` | one SPI bus for every sensor, no PWM on the schematic pins |
| V1.4 | `esp32s31mini14` (this directory) | u-blox DAN-F10N GPS on the board, code run from the PSRAM |
| V1.5 | [`esp32s31mini`](../esp32s31mini/README.md) | SD memory, RM3100 chip select on IO2 |

## Module pads 42-53

On the V1.2 the module pads 42-53 were found not to be on the GPIO named by the schematic symbol: UART1, UART2 and the LEDs were measured on other GPIOs, and the CAN lines and `CSmag` reached no readable GPIO. The V1.4 and V1.5 keep UART1, UART2 and the LEDs on the same pads, so they use the GPIOs measured on the V1.2. Every other function in the tables below is taken from the schematic, except where marked as measured.

| Pad | Schematic GPIO | Net (V1.4, V1.5) | GPIO used |
|---|---|---|---|
| 44 / 45 | IO10 / IO11 | UART1 RX / TX (connector S1, U4) | IO8 / IO9 (measured) |
| 46 / 47 | IO12 / IO13 | UART2 RX / TX (connector S2, U7) | IO10 / IO11 (measured) |
| 48 / 49 | IO14 / IO15 | LED1 / LED2 | IO13 / IO12 (measured) |
| 42 / 43 | IO8 / IO9 | CANRX / CANTX on the V1.4 schematic | none: the V1.4 CAN is on IO56 / IO57, as on the V1.5 |
| 52 | IO18 | `CSmag` on the V1.4 | unknown: on the V1.2 this pad is not IO18 |

## Pinout

| Function | Net | V1.4 GPIO | V1.5 GPIO |
|---|---|---|---|
| SPI bus 1 (IMU only) SCK / MOSI / MISO | `SCK` / `MOSI` / `MISO` | 20 / 21 / 22 | 20 / 21 / 22 |
| SCH16T CS | `CSGyro2` | 25 | 25 |
| SCH16T EXTRESN | | tied to VCC | no line |
| SCH16T DRY_SYNC | `DR2` | pad 53, not used | no line |
| SPI bus 2 SCK / MOSI / MISO | `SCK2` / `MOSI2` / `MISO2` | 35 / 36 / 37 | 35 / 36 / 37 |
| BMP581 #1 / #2 CS | `CSbaro1` / `CSbaro2` | 23 / 24 | 23 / 24 |
| RM3100 CS | `CSmag` | pad 52 (see above) | 2 |
| ELVH CS | `TOT` | 38 | 38 |
| SD card CS | `cssd` | 39 | 39 |
| GPS serial RX / TX (S3) | `rxgps` / `txgps` | 3 / 4, DAN-F10N on the board | 3 / 4 (connector S3) |
| GPS RESET_N | `GPSrst` | 2 | (IO2 is `CSmag`) |
| GPS TIMEPULSE | `TP` | 5 | no line |
| UART1 RX / TX (connector S1) | `UART1RX` / `UART1TX` | 8 / 9 | 8 / 9 |
| UART2 RX / TX (connector S2) | `UART2RX` / `UART2TX` | 10 / 11 | 10 / 11 |
| USB D- / D+ | `D-` / `D+` | 33 / 34 | 33 / 34 |
| CAN TX / RX (MCP2542FD) | `CANTX` / `CANRX` | 57 / 56 | 57 / 56 |
| SERVO1-6 | `s1`-`s6` | 42-47 | 42-47 |
| SERVO7 / SERVO8 | `s7` / `s8` | 48 / 49 | 54 / 55 |
| LED1 / LED2 | `led1` / `led2` | 13 / 12 | 13 / 12 |

Common to both boards:
- The pin marked `R` on S1 and S2 is the board RX: it goes to the TX of the device.
- IO36 and IO37 (MOSI2, MISO2) are strapping pins of the S31.
- SERVO outputs share one timer per pair (1-2, 3-4, 5-6, 7-8), so one frequency per pair.
- No dedicated RC input pin: RC comes from a serial protocol or over MAVLink.

## ArduPilot mapping of the V1.4 (`esp32s31mini14`)

| Port / device | Hardware | Default |
|---|---|---|
| SERIAL0 | USB Serial/JTAG | MAVLink, ESP-IDF console |
| SERIAL1 | Wi-Fi access point `ardupilot-s31` | MAVLink, UDP 192.168.4.1:14550 |
| SERIAL2 | connector S2 (net UART2), ESP32 UART2 | MAVLink2 at 230400, LTE telemetry |
| SERIAL3 | S3: DAN-F10N (IO3/IO4), ESP32 UART1 | GPS1, auto-configured |
| SERIAL4 | connector S1 (net UART1), ESP32 UART0 | rangefinder at 115200 (`SERIAL4_PROTOCOL 9`) |
| IMU | SCH16T on SPI bus 1 | `ROTATION_ROLL_180` |
| Baros | 2x BMP581 on SPI bus 2 | |
| CAN1 | TWAI on the MCP2542FD (TX IO57, RX IO56) | DroneCAN, 1 Mbit/s |

S1, S2 and S3 take any serial protocol of ArduPilot (`SERIALn_PROTOCOL`); on the V1.4 S3 is wired to the DAN-F10N. The HAL sets parity, stop bits, RX and TX inversion (`SERIALn_OPTIONS` bits 0 and 1) and the RX/TX swap (bit 3); half-duplex, flow control and the pull options are not supported. SERIAL4 is on the UART0 of the S31, free because the console is on the USB: ESP-IDF 6.1 has no GPIO matrix signals for the UART3 of the S31 (`-1` in `uart_periph.c`), so UART3 is never connected to its pins. A TFmini on S1 needs `RNGFND1_TYPE 20`.

The hwdef is for a V1.4 without RM3100, SD card and ELVH mounted: none of them is configured, and there is no log. IO2 is listed as an SPI chip select that no driver opens, so the HAL drives it high at boot and keeps the GPS out of reset.

`defaults.parm` sets the board up for an ArduCopter tricopter: `FRAME_CLASS 7` (motors on SERVO1, SERVO2 and SERVO4, tail servo on SERVO7), a 400 Hz main loop, and EKF3 yaw from the GSF (`EK3_SRC1_YAW 8`, `COMPASS_ENABLE 0`), since there is no compass. Without a compass the yaw aligns only after moving with a GPS fix, so the first take-off must be in a mode that does not need a position.

CAN: DroneCAN on CAN1 at 1 Mbit/s (`CAN_P1_DRIVER 1`, `CAN_D1_PROTOCOL 1`); the servos stay on PWM. SLCAN works as on the STM32 boards with one USB port: with `CAN_SLCAN_CPORT 1` (read at boot) and `CAN_SLCAN_SERNUM 0` the USB (SERIAL0) becomes an SLCAN adapter on CAN1 until `CAN_SLCAN_TIMOUT` or a reboot, and Mission Planner sets this by itself. The S31 cannot add a second USB CDC for a dedicated SLCAN port: the USB connector (IO33/IO34) is the USB Serial/JTAG, which has one fixed CDC, and the USB OTG of the S31 has only the HS (UTMI) PHY, on other pads. Over MAVLink (USB or Wi-Fi) the CAN bus is also reachable with `MAV_CMD_CAN_FORWARD` ("MAVLink-CAN" in Mission Planner, `mavcan:` in the DroneCAN GUI tool).

Checked on the V1.4 with ArduCopter, at rest and indoors: SCH16T (about -1000 mG on Z at rest, so the K10 variant is right), both BMP581, the DAN-F10N on SERIAL3 in both directions at 10 Hz, the 400 Hz main loop, a TFmini on S1 (SERIAL4), with the RX inversion and the swap. Not checked yet: CAN, the PWM signals, a device on S2, a serial device with parity, the LEDs, a GPS fix and flight.

## DAN-F10N on the V1.4

- RESET_N (IO2) is active low, with an internal pull-up. Driving it low resets the receiver and clears its configuration.
- On the DAN-F10N, TIMEPULSE and SAFEBOOT_N are the same internal function, with SAFEBOOT_N tied to TIMEPULSE through 1 kΩ (Integration manual, 3.2.2): if the pin is low when the receiver starts, it enters safeboot mode. IO5 must not be driven low at start-up. To force safeboot for a firmware recovery, hold IO5 low and pulse IO2 low.
- The ESP32 HAL has no PPS input: the time pulse is not used.
- V_BCKP is not connected: every power-up is a cold start.
- The antenna is the patch antenna of the module; EXT_RF_IN is not used.
- `defaults.parm`: GPS + SBAS + Galileo at 10 Hz (`GPS1_GNSS_MODE 7`, `GPS1_RATE_MS 100`) and the GPS L5 signals used before they are declared healthy (`GPS_DRV_OPTIONS 32`; u-blox advises against it for safety-critical uses).
- Firmware: the receiver came with EXT SPGL1L5 6.00 and was updated to EXT SPGL1L5 7.01 (protocol 52.00) with u-center, through a plain USB-UART bridge firmware on the ESP32. ArduPilot detects and configures both. The 7.01 firmware has Galileo OSNMA built in, with no configuration keys; ArduPilot does not read its status yet.

## Code run from the PSRAM

`sdkconfig.defaults` in this directory adds ESP-IDF settings after those of the esp32s31 target (the waf build passes it when a board directory has one). On the V1.4 the code and read-only data are copied from the flash to the octal PSRAM at start-up and run from there (`CONFIG_SPIRAM_XIP_FROM_PSRAM`), at 200 MHz. The PSRAM is not added to the heap, so the ArduPilot heap and the DMA buffers stay in internal SRAM.

The instruction cache of the S31 is 16 kB; with the code in the QIO flash, a large part of core 0 time was spent waiting for cache refills. Measured with ArduCopter on the V1.4, disarmed, indoors without a GPS fix:

| | Code in flash (QIO, 80 MHz) | Code in PSRAM |
|---|---|---|
| Core 0 at 200 Hz | 66% | 26% |
| Core 0 at 400 Hz | does not keep up | 41%, loop at 399.8-400.6 Hz |
| Core 1 | 36% | 34% |

The PSRAM start-up (detection, timing tuning, copy of the code, memory test) takes about 0.8 s. Flash writes, such as saving parameters, no longer stop the cache of both cores. The V1.5 has the same module but has not been tested with the code in the PSRAM.

## Build and flash

```sh
./waf configure --board esp32s31mini14
./waf copter
```

Flash as described in the [V1.5 README](../esp32s31mini/README.md#flashing): `bootloader.bin` at 0x2000, `partition-table.bin` at 0x10000, `ardupilot.bin` at 0x20000.
