# ArduPilot su scheda MINI AHRS (ESP32-S31)

ArduPilot standard, con EKF3, portato su ESP32-S31 (ESP-IDF 6.1) per la scheda MINI AHRS V1.2, modulo ESP32-S31-WROOM-1-N16R16V.

- Board: `esp32s31mini`, hwdef in questa cartella.
- Base: ArduPilot master `dc78bc5` (ottobre 2026).
- Veicoli: ArduPlane, ArduRover, ArduCopter (la scheda non ha uscite PWM né CAN, vedi sotto).
- Navigazione: AHRS/EKF3 di ArduPilot, nessuna modifica (`AHRS_EKF_TYPE 3`, default).

## Build

Serve ESP-IDF v6.1 (branch `release/v6.1`; per ESP-IDF l'S31 è un target *preview*) su Linux o WSL. ESP-IDF non accetta spazi nel percorso: clona il repository in una cartella senza spazi.

```sh
. ~/esp/esp-idf-v6.1/export.sh
git submodule update --init --recursive
./waf configure --board esp32s31mini
./waf plane        # oppure: ./waf rover, ./waf copter
```

Dopo ogni modifica a `hwdef.dat` va ripetuto `./waf configure`. Le immagini sono in `build/esp32s31mini/esp-idf_build/`: `bootloader/bootloader.bin`, `partition_table/partition-table.bin`, `ardupilot.bin`.

## Flash

Via USB (connettore USB-C, USB Serial/JTAG dell'ESP32-S31). Sul collegamento USB della scheda funziona solo `--no-stub` a 115200 baud:

```sh
cd build/esp32s31mini/esp-idf_build
python -m esptool --chip esp32s31 -p <PORTA> -b 115200 --no-stub write-flash \
    --flash-mode dio --flash-freq 80m --flash-size 16MB \
    0x2000 bootloader/bootloader.bin 0x10000 partition_table/partition-table.bin 0x20000 ardupilot.bin
```

Per aggiornare solo il firmware basta `0x20000 ardupilot.bin`: bootloader, tabella delle partizioni e parametri restano. Se sulla scheda c'era un altro firmware ArduPilot, al primo avvio conviene azzerare i parametri (`FORMAT_VERSION 0`, oppure "Reset to defaults" dalla GCS).

Dopo il flash il chip resta in modalità download: il nuovo firmware parte solo dopo aver scollegato e ricollegato l'USB.

Se Windows segnala "dispositivo non riconosciuto": USB scollegato, collega il pad 27 del modulo (IO61) a GND, collega l'USB, togli il filo e, con l'USB sempre collegato, flasha aggiungendo `--before no-reset`.

## Mappa della scheda

| Funzione | GPIO | ArduPilot |
|---|---|---|
| SPI SCK / MOSI / MISO | 20 / 21 / 22 | `ESP32_SPIBUS SPI2_HOST` |
| SCH16T CS / RESET / DRY | 25 / 17 / 19 | `IMU SCH16T`, `ROTATION_ROLL_180`, `HAL_SCH16T_RESET_PIN 17`; DRY non usato |
| BMP581 #1 / #2 CS | 23 / 24 | `BARO BMP581` x2 |
| RM3100 CS | 38 | `COMPASS RM3100`, `ROTATION_ROLL_180` (modulo capovolto sotto la scheda) |
| ELVH CS | 18 | airspeed `SPI:elvh`, `ARSPD_TYPE 21` |
| UART1 RX / TX | 8 / 9 (connettore S1) | SERIAL3, GPS |
| UART2 RX / TX | 10 / 11 (connettore S2) | SERIAL2, telemetria o RC seriale (CRSF/ELRS: `SERIAL2_PROTOCOL 23`) |
| USB D- / D+ | 33 / 34 | SERIAL0 (USB Serial/JTAG, MAVLink) |
| LED1 / LED2 | 13 / 12 | LED di stato, vedi sotto |
| Wi-Fi | — | SERIAL1, access point `ardupilot-s31` (password `ardupilot1`), UDP 192.168.4.1:14550 |

Note sulla V1.2, misurate sulla scheda:
- I connettori S1 e S2 non sono sui pin dello schema (10/11 e 12/13), ma su IO8/IO9 e IO10/IO11. Il pin marcato `R` è l'RX della scheda: va collegato al TX del dispositivo.
- I LED sono su IO13 e IO12, non su IO14 e IO15 come da schema. LED3 (IO16) non è usato.
- La linea `CSmag` (IO18) è a 0 V al modulo e teneva l'RM3100 sempre selezionato: il CS dell'RM3100 è ponticellato su IO38. IO18 resta come CS dell'ELVH, che sulla V1.2 di prova non è collegato.
- CAN (MCP2542FD): TXD e RXD non arrivano a nessun GPIO leggibile, quindi il CAN non è configurato (niente righe `ESP32_CAN`, niente DroneCAN). L'HAL ha già il driver CAN (TWAI): su una scheda con il CAN cablato basta `ESP32_CAN <TX> <RX>` nell'hwdef.
- Nessuna uscita PWM e nessun ingresso RC dedicato: l'RC arriva da un protocollo seriale.
- Nessuna SD: log, scripting e filesystem sono disattivati.

Le rotazioni di SCH16T e RM3100 vanno ricontrollate ruotando la scheda a mano (vedi sotto).

## Configurazione

La configurazione hardware sta in due file di questa cartella:

| File | Contenuto | Quando ha effetto |
|---|---|---|
| `hwdef.dat` | pin, bus, sensori, variante IMU, uso dei core | alla compilazione (`./waf configure` + build) |
| `defaults.parm` | default dei parametri | al primo avvio o dopo un reset dei parametri |

### IMU SCH16T

```
define HAL_SCH16T_VARIANT 10    # 1 = SCH16T-K01, 10 = SCH16T-K10
```

| | SCH16T-K01 | SCH16T-K10 |
|---|---|---|
| Range gyro | ±300 °/s | ±2000 °/s |
| Range accel | ±8 g | ±16 g |
| Sensibilità usata dal driver (20 bit) | 1600 LSB/(°/s), 3200 LSB/(m/s²), range di fabbrica | 200 LSB/(°/s) DYN3, 6400 LSB/(m/s²) DYN2, scritti all'avvio |

Il valore deve corrispondere al componente montato: con la variante sbagliata le scale sono errate di un fattore 8 (gyro) o 2 (accel). Il default è K10; lo schema V1.2 riporta SCH16T-K01-10. Verifica a scheda montata:
- accelerometro: ferma e in piano, `RAW_IMU` / `SCALED_IMU` deve dare circa −9,8 m/s² su Z (−4,9 o −19,6 indicano la variante sbagliata);
- gyro: ruotando la scheda di 90° attorno all'asse verticale lo yaw della GCS deve cambiare di 90°.

Filtro interno e frequenza di lettura:

```
define HAL_SCH16T_LPF_CODE 0x000     # 68 Hz su tutti gli assi
define HAL_SCH16T_SAMPLE_RATE 1000   # letture al secondo
```

Il codice ripete tre volte (3 bit per asse) il filtro: `000` 68 Hz, `001` 30 Hz, `010` 13 Hz, `011` 280 Hz, `100` 370 Hz, `101` 235 Hz (per esempio 30 Hz su tutti gli assi è `0x049`). Il datasheet chiede almeno 500 Hz di lettura con 68 Hz, 1000 Hz con 235 e 280 Hz, 2000 Hz con 370 Hz. Il driver legge le uscite interpolate con 7 frame SPI in pipeline, controlla CRC e stato di ogni frame e pubblica la temperatura a 10 Hz.

### Bus SPI

- ESP-IDF accetta al massimo 6 dispositivi per host SPI, e una riga `ESP32_SPIDEV` con `LoSpeed` diverso da `HiSpeed` ne occupa due. Oggi: SCH16T 2, BMP581 1 + 1, RM3100 1, ELVH 1 = 6. Un dispositivo in più ferma l'avvio con `SPI: no room for <nome> on its bus`.
- Ogni dispositivo presente sul bus va elencato, anche senza driver, così il suo chip select resta alto. L'HAL porta alti tutti i CS prima di inizializzare i bus.
- Limiti dei componenti: SCH16T 10 MHz, BMP581 12 MHz, RM3100 1 MHz, ELVH 800 kHz.
- Per leggere un solo barometro si toglie la seconda riga `BARO` e si lascia la sua riga `ESP32_SPIDEV`.

### Orientamento dei sensori

L'ultima parola delle righe `IMU` e `COMPASS` è la rotazione dal frame del sensore al frame della scheda (X avanti, Y destra, Z giù). Se è la scheda a essere montata ruotata sul veicolo, non toccare l'hwdef e usa `AHRS_ORIENTATION`; per la bussola c'è anche `COMPASS_ORIENT`.

### Airspeed ELVH

Driver nuovo `AP_Airspeed_ELVH`: sensori di pressione differenziale All Sensors ELVH con uscita SPI e funzione di trasferimento A (10–90 %), per esempio ELVH-L02D-HRRD-C-NSA4.

- `ARSPD_TYPE 21` (SPI-ELVH). Su ArduPlane è il default della scheda; sugli altri veicoli il default è 0.
- `ARSPD_ELVH_RNG`: codice del range come nell'ordine del componente (3 = L02D, ±2 inH2O, default della scheda); 0 prende il fondoscala da `ARSPD_PSI_RANGE`.
- Il driver legge a 100 Hz; all'avvio richiede tre letture valide (stato, pressione nella banda calibrata, temperatura coerente). Pressioni fuori dalla banda calibrata vengono scartate: l'L02D satura a ±498 Pa, circa 28 m/s di velocità indicata.
- In SITL c'è un ELVH-L04D simulato su `SPI:elvh`; il test `Tools/autotest/autotest.py test.Plane.AirspeedDrivers` lo prova sia con `ARSPD_ELVH_RNG 4` sia con `ARSPD_PSI_RANGE`.

### GPS

`defaults.parm` imposta un u-blox su SERIAL3 a 460800 (`SERIAL3_BAUD 460`) con `GPS_AUTO_CONFIG 0`: con l'autoconfigurazione il driver accetta l'u-blox solo a 230400, quindi il modulo deve essere già configurato per mandare UBX (NAV-PVT e DOP) a 460800 e salvato in flash. Il GPS è alimentato dalla scheda: dopo un ricollegamento dell'USB riparte a freddo.

### Seriali e telemetria

- SERIAL0: USB Serial/JTAG. Aprire la porta con DTR e RTS bassi: chiuderla dopo averla aperta con DTR/RTS alti (default di pyserial e pymavlink) zittisce l'USB della scheda fino a un ricollegamento. `HAL_ESP32_USB_NO_CHIP_RESET` impedisce che le linee USB resettino il chip.
- SERIAL1: Wi-Fi UDP.
- SERIAL2, SERIAL3: UART su S2 e S1. L'HAL ESP32 supporta `SERIALn_OPTIONS` bit 3 (swap RX/TX).
- Le stream rate sono per canale MAVLink (`MAVn_*`): SERIAL0 è MAV1, il Wi-Fi MAV2, SERIAL2 MAV3.

## LED di stato

| LED | Stato | Significato |
|---|---|---|
| LED1 | lampeggio veloce (circa 2 Hz) | avvio in corso |
| LED1 | fisso | inizializzazione finita (setup del veicolo e calibrazione IMU) |
| LED2 | spento | GPS assente o senza fix 3D |
| LED2 | lampeggio 1 Hz | fix 3D, AHRS ancora senza posizione assoluta |
| LED2 | fisso | fix 3D e posizione assoluta dall'AHRS |

Arming, pre-arm e calibrazioni si leggono dalla GCS.

## Uso dei due core

| Core | Thread |
|---|---|
| HP CPU0 | main loop (veicolo, AHRS/EKF3), timer, UART, thread del bus SPI (SCH16T a 1 kHz, baro, bussola) |
| HP CPU1 | Wi-Fi/lwIP, IO, storage, thread delle librerie |

Si configura dall'hwdef: `HAL_ESP32_FASTCPU`, `HAL_ESP32_SLOWCPU`, `HAL_ESP32_DEVICE_BUS_CPU`, `HAL_ESP32_THREAD_CPU` (e `HAL_ESP32_CAN_THREAD_CPU`, `HAL_ESP32_IO_THREAD_CPU`). Il carico arriva in telemetria: `SYS_STATUS.load` è il core più carico, misurato dal tempo idle di FreeRTOS; `NAMED_VALUE_FLOAT` `CPU0` e `CPU1` danno i due core in %.

## Modifiche ad ArduPilot

Build:
- `Tools/ardupilotwaf/boards.py`, `esp32.py`: classe `esp32s31` (toolchain `riscv32-esp-elf`, `-march=rv32imafcb_zicsr_zifencei_zaamo_zalrsc_zcb_zcmp_zcmt -mabi=ilp32f`, senza le estensioni PIE/Xesploop, perché ESP-IDF sposta sul core 1 i task che le usano); DroneCAN abilitato se l'hwdef contiene `ESP32_CAN`.
- `libraries/AP_HAL_ESP32/targets/esp32s31/`: progetto ESP-IDF, `sdkconfig.defaults` (320 MHz, newlib, PSRAM disattivata), partizioni (app 6 MB). Il CMake linka tutti i componenti, perché in ESP-IDF 6 l'HAL è diviso in `esp_hal_*` con script linker ROM propri.

HAL ESP32 su ESP-IDF 6.1:
- `CANIface` (nuovo): CAN su TWAI con il driver a nodi `esp_twai`; RX in coda FreeRTOS da ISR, TX a slot, loopback per il time sync DroneCAN, recovery dal bus-off a 1 Hz.
- `UARTDriver`: porta `USB_SERIAL_JTAG`; swap RX/TX con `SERIALn_OPTIONS`.
- `Scheduler`, `DeviceBus`, `WiFi*`: core configurabili dall'hwdef, thread dei bus e delle librerie con affinità fissa.
- `SPIDevice`: trasferimenti brevi (≤ 64 byte) in polling; CS portati alti prima di ogni setup dei bus; stop all'avvio se un host SPI non ha posto per un dispositivo.
- `Util`: carico di ogni core dal tempo idle di FreeRTOS.
- `Semaphores`: attese arrotondate al tick superiore.
- RMT legacy (ingresso RC PWM/PPM) e I2C software (solo Xtensa) resi condizionali; capability MCPWM da `mcpwm_ll.h`; tabella I2C vuota ammessa; `ffconf.h` con `MIN`/`MAX`; `RCOutput` senza uscite.
- `esp32_hwdef.py`: keyword `ESP32_CAN TX RX`.

Driver e librerie:
- `AP_InertialSensor_SCH16T` (nuovo): Murata SCH16T-K01/K10 diretto su SPI.
- `AP_Airspeed_ELVH` (nuovo, `ARSPD_TYPE 21`, `ARSPD_ELVH_RNG`), con il sensore simulato in SITL e il test in `AirspeedDrivers`.
- `AP_Notify`: `AP_BoardLED_InitGPS`, i due LED di stato.
- `GCS_MAVLink`: `SYS_STATUS.load` dal carico dei core quando l'HAL lo misura, `NAMED_VALUE_FLOAT` `CPU0`/`CPU1`.
- ArduPlane: `PLANE_GCS_SEND_TIME_US`, tempo per loop di `update_send` impostabile dalla board (3 ms qui).

## Stato

HAL, driver e pinout sono gli stessi già fatti girare sulla scheda V1.2 (USB, Wi-Fi, SCH16T, baro, RM3100, GPS, LED). Questa build con EKF3 non è ancora stata avviata sull'hardware. Da verificare al primo avvio:
1. boot, console USB (SERIAL0) e Wi-Fi;
2. variante e rotazione dell'IMU, rotazione della bussola, calibrazioni;
3. EKF3: allineamento, uso di GPS e bussola, carico del core 0 (`CPU0`) con EKF3 nel main loop;
4. ELVH sulla scheda dove è collegato: `DiffPress` vicino a 0 a pitot coperto e `Temp` plausibile.
