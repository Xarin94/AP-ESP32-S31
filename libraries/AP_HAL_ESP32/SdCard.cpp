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
//#define HAL_ESP32_SDCARD 1


#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "SdCard.h"

// ESP-IDF >= 6 ffconf.h evaluates MIN()/MAX() in #if expressions, while
// AP_Math replaces those macros with templates
#pragma push_macro("MIN")
#pragma push_macro("MAX")
#undef MIN
#undef MAX
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#include "esp_vfs_fat.h"
#include "diskio_impl.h"
#include "diskio_sdmmc.h"
#pragma pop_macro("MAX")
#pragma pop_macro("MIN")
#include "esp_ota_ops.h"

#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include <sys/stat.h>
#include <sys/unistd.h>
#include <sys/types.h>
#include "SPIDevice.h"

#ifdef HAL_ESP32_SDCARD

sdmmc_card_t* card = nullptr;

const size_t buffer_size = 8*1024;
const char* fw_name = "/SDCARD/APM/ardupilot.bin";

static bool sdcard_running;
static HAL_Semaphore sem;

void update_fw()
{
    FILE *f = fopen(fw_name, "r");
    void *buffer = calloc(1, buffer_size);
    esp_ota_handle_t update_handle = 0 ;
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    size_t nread = 0;
    if (f == nullptr || buffer == nullptr || update_partition == nullptr) {
        goto done;
    }
    printf("updating firmware...\n");
    if (esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle) != ESP_OK) {
        goto done;
    }
    do {
        nread = fread(buffer, 1, buffer_size, f);
        if (nread > 0) {
            if (esp_ota_write(update_handle, buffer, nread) != ESP_OK) {
                goto done;
            }
        }
    } while (nread > 0);
done:
    if (update_handle != 0) {
        if (esp_ota_end(update_handle) == ESP_OK &&
            esp_ota_set_boot_partition(update_partition) == ESP_OK) {
            printf("firmware updated\n");
        }
    }
    if (f != nullptr) {
        fclose(f);
    }
    if (buffer != nullptr) {
        free(buffer);
    }
    unlink(fw_name);
}

#ifdef HAL_ESP32_SDMMC

void mount_sdcard_mmc()
{
    printf("Mounting sd \n");
    WITH_SEMAPHORE(sem);

    /*
    https://docs.espressif.com/projects/esp-idf/en/v4.1/api-reference/peripherals/sdmmc_host.html

    // we take the MMC parts from this example:
    https://github.com/espressif/esp-idf/blob/release/v4.1/examples/storage/sd_card/main/sd_card_example_main.c

    hardcoded SDMMC gpio options....

    Slot 0 (SDMMC_HOST_SLOT_0) is an 8-bit slot. It uses HS1_* signals in the PIN MUX.- dont use slot0, is used for SPI-flash chip.

    Slot 1 (SDMMC_HOST_SLOT_1) is a 4-bit slot. It uses HS2_* signals in the PIN MUX.

    this is the full list, but u can get away without some (2 or 3) of these in some cases:
    Signal	Slot 1
      CMD	GPIO15
      CLK	GPIO14
      D0	GPIO2
      D1	GPIO4
      D2	GPIO12
      D3	GPIO13

    */

    //  the dedicated SDMMC host peripheral on the esp32 is about twice as fast as SD card SPI interface in '1-line' mode and somewht faster again in '4-line' mode.  we've using 1-line mode in this driver, so we need less gpio's

    static const char *TAG = "SD...";
    ESP_LOGI(TAG, "Initializing SD card as SDMMC");

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

    // To use 1-line SD mode (this driver does), uncomment the following line:
    slot_config.width = 1;

    // GPIOs 15, 2, 4, 12, 13 should have external 10k pull-ups.
    // Internal pull-ups are not sufficient. However, enabling internal pull-ups
    // does make a difference some boards, so we do that here.
    gpio_set_pull_mode(GPIO_NUM_15, GPIO_PULLUP_ONLY);   // CMD, needed in 4- and 1- line modes
    gpio_set_pull_mode(GPIO_NUM_2, GPIO_PULLUP_ONLY);    // D0, needed in 4- and 1-line modes
    //gpio_set_pull_mode(GPIO_NUM_4, GPIO_PULLUP_ONLY);    // D1, needed in 4-line mode only
    //gpio_set_pull_mode(GPIO_NUM_12, GPIO_PULLUP_ONLY);   // D2, needed in 4-line mode only
    //
    // Pin 13 / chip-select  - is an interesting one, because if its the only thing on this
    //   spi bus(it is), then NOT connecting the SD to this pin, and instead directly to a pull-up
    //   also asserts the CS pin 'permanently high' to the SD card, without the micro being involved..
    //   which means pin 13 on micro can be re-used elsewhere. If one of these isn't true for u,
    //   then uncomment this line and connect it electrically to the CS pin on the SDcard.
    //gpio_set_pull_mode(GPIO_NUM_13, GPIO_PULLUP_ONLY);   // D3, needed in 4- and 1-line modes


    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 10,
        .allocation_unit_size = 16 * 1024
    };

    //https://docs.espressif.com/projects/esp-idf/en/v4.1/api-reference/peripherals/sdmmc_host.html


    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc_mount is an all-in-one convenience function.
    // Please check its source code and implement error recovery when developing
    // production applications.
    sdmmc_card_t* card;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/SDCARD", &host, &slot_config, &mount_config, &card);

    if (ret == ESP_OK) {
        mkdir("/SDCARD/APM", 0777);
        mkdir("/SDCARD/APM/LOGS", 0777);
        printf("sdcard is mounted\n");
        //update_fw();
        sdcard_running = true;
    } else {
        printf("sdcard is not mounted\n");
        sdcard_running = false;
    }
}

void mount_sdcard()
{
    mount_sdcard_mmc();
}

bool format_sdcard()
{
    return false;
}

#endif // emd mmc


#ifdef HAL_ESP32_SDSPI
ESP32::SPIBusDesc bus_ = HAL_ESP32_SDSPI;

// SD clock after the card is identified (the identification runs at
// SDMMC_FREQ_PROBING, 400 kHz)
#ifndef HAL_ESP32_SDSPI_MAX_FREQ_KHZ
#define HAL_ESP32_SDSPI_MAX_FREQ_KHZ SDMMC_FREQ_PROBING
#endif

// wait for MISO high before each SD command (ESP-IDF wait_for_miso): 0 up to
// 40 ms, -1 no wait, 1-127 up to that many ms. The wait is a busy loop that
// holds the bus: on a bus where MISO does not go high with the card
// deselected (no pull-up) it lasts the whole timeout before every command
#ifndef HAL_ESP32_SDSPI_WAIT_FOR_MISO
#define HAL_ESP32_SDSPI_WAIT_FOR_MISO 0
#endif

// cluster size of a format: 32 kB, one log write on the boards with
// HAL_ESP32_FS_IO_SIZE 32768 (a 64 GB card gets 2M clusters, two 8 MB FATs)
static const esp_vfs_fat_mount_config_t sdspi_mount_config = {
    .format_if_mount_failed = false,
    .max_files = 5,
    .allocation_unit_size = 32 * 1024
};

static uint8_t sdspi_xfer(spi_device_handle_t dev, uint8_t tx)
{
    spi_transaction_t t {};
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.length = 8;
    t.tx_data[0] = tx;
    if (spi_device_polling_transmit(dev, &t) != ESP_OK) {
        return 0;
    }
    return t.rx_data[0];
}

// clock the card until it releases the busy signal (two consecutive non-zero
// bytes) or for timeout_ms
static void sdspi_wait_ready(spi_device_handle_t dev, uint32_t timeout_ms)
{
    const uint64_t t0 = AP_HAL::micros64();
    uint8_t ready = 0;
    while (ready < 2 && AP_HAL::micros64() - t0 < timeout_ms * 1000ULL) {
        ready = sdspi_xfer(dev, 0xFF) != 0 ? ready + 1 : 0;
    }
}

/*
  A reset of the chip in the middle of a write leaves the card inside it:
  waiting for the rest of a data block, or for the next block of a
  multi-block write. It ignores CMD0 and stays unusable until a power cycle,
  which a soldered card gets only with the whole board. Before the card is
  identified, clock it out of the write: wait while it is busy, complete a
  data block left half sent (the card rejects it: its CRC is wrong, and CRC
  checks are on in SPI mode), end a multi-block write with the stop token.
  A card that is not in a write ignores all of this
 */
static void sdspi_recover_card()
{
    spi_device_interface_config_t cfg {};
    cfg.mode = 0;
    cfg.clock_speed_hz = 400 * 1000;
    cfg.spics_io_num = -1;
    cfg.queue_size = 1;
    spi_device_handle_t dev;
    if (spi_bus_add_device(bus_.host, &cfg, &dev) != ESP_OK) {
        return;
    }
    if (spi_device_acquire_bus(dev, portMAX_DELAY) != ESP_OK) {
        spi_bus_remove_device(dev);
        return;
    }
    gpio_set_direction(bus_.cs, GPIO_MODE_OUTPUT);
    gpio_set_level(bus_.cs, 0);
    // a card in SPI mode busy programming; one not in SPI mode (after power
    // on) does not drive MISO, which then reads as busy: a short timeout
    sdspi_wait_ready(dev, 250);
    // rest of a data block: up to 512 bytes, its CRC and the data response
    for (uint16_t i = 0; i < 512 + 2 + 8; i++) {
        sdspi_xfer(dev, 0xFF);
    }
    sdspi_wait_ready(dev, 250);
    // stop token of a multi-block write, then the card is busy while it
    // programs what it received
    sdspi_xfer(dev, 0xFD);
    sdspi_xfer(dev, 0xFF);
    sdspi_wait_ready(dev, 250);
    gpio_set_level(bus_.cs, 1);
    sdspi_xfer(dev, 0xFF);
    spi_device_release_bus(dev);
    spi_bus_remove_device(dev);
}

static sdspi_device_config_t sdspi_slot_config()
{
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = bus_.cs;
    slot_config.host_id = bus_.host;
    slot_config.wait_for_miso = HAL_ESP32_SDSPI_WAIT_FOR_MISO;
    return slot_config;
}

void mount_sdcard_spi()
{

    esp_err_t ret;
    printf("Mounting sd \n");
    WITH_SEMAPHORE(sem);

    //  In SPI mode, pins can be customized...

    // and 'SPI bus sharing with SDSPI has been added in 067f3d2 — please see the new sdspi_host_init_device, sdspi_host_remove_device functions'. https://github.com/espressif/esp-idf/issues/1597 buzz..: thats in idf circa 4.3-dev tho.

    // readme shows esp32 pinouts and pullups needed for different modes and 1 vs 4 line gotchass...
    //https://github.com/espressif/esp-idf/blob/master/examples/storage/sd_card/README.md
    //https://github.com/espressif/esp-idf/blob/master/examples/storage/sd_card/main/sd_card_example_main.c

    static const char *TAG = "SD...";
    ESP_LOGI(TAG, "Initializing SD card as SDSPI");
    esp_vfs_fat_mount_config_t mount_config = sdspi_mount_config;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = bus_.host;
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = bus_.mosi,
        .miso_io_num = bus_.miso,
        .sclk_io_num = bus_.sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    // the card can share the host with the sensors of an ESP32_SPIBUS line on
    // the same pins: whichever starts second finds the bus already set up
    ret = spi_bus_initialize(bus_.host, &bus_cfg, bus_.dma_ch);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }

    sdspi_recover_card();

    const sdspi_device_config_t slot_config = sdspi_slot_config();

    //host.flags = SDMMC_HOST_FLAG_1BIT | SDMMC_HOST_FLAG_DDR;
    host.max_freq_khz = HAL_ESP32_SDSPI_MAX_FREQ_KHZ;
    ret = esp_vfs_fat_sdspi_mount("/SDCARD", &host, &slot_config, &mount_config, &card);

    if (ret == ESP_OK) {
        // Card has been initialized, print its properties

        mkdir("/SDCARD/APM", 0777);
        mkdir("/SDCARD/APM/LOGS", 0777);
        printf("sdcard is mounted\n");
        //update_fw();
        sdcard_running = true;
    } else {
        printf("sdcard is not mounted\n");
        sdcard_running = false;
    }
}
void mount_sdcard()
{
    mount_sdcard_spi();
}

/*
  partition and format a card that is not mounted: one MBR partition over
  the whole card, then a FAT volume. The card gets a FatFs drive of its own
  and no /SDCARD path meanwhile. The ESP-IDF format_if_mount_failed keeps
  the path on the unmounted volume during the format: a logger open went
  into it, and the mount after the format reset the board
 */
static bool sdspi_format_unmounted()
{
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = HAL_ESP32_SDSPI_MAX_FREQ_KHZ;
    const sdspi_device_config_t slot_config = sdspi_slot_config();
    sdspi_dev_handle_t handle;
    if (host.init() != ESP_OK ||
        sdspi_host_init_device(&slot_config, &handle) != ESP_OK) {
        return false;
    }
    host.slot = handle;
    const UINT work_size = 4096;
    void *work = ff_memalloc(work_size);
    sdmmc_card_t *c = (sdmmc_card_t *)calloc(1, sizeof(sdmmc_card_t));
    BYTE pdrv;
    bool ok = false;
    if (work != nullptr && c != nullptr && sdmmc_card_init(&host, c) == ESP_OK) {
        if (ff_diskio_get_drive(&pdrv) == ESP_OK) {
            ff_diskio_register_sdmmc(pdrv, c);
            const LBA_t ptbl[4] = {100, 0, 0, 0};
            const MKFS_PARM opt = {FM_ANY, 2, 0, 0, (DWORD)sdspi_mount_config.allocation_unit_size};
            const char drv[3] = {char('0' + pdrv), ':', 0};
            ok = f_fdisk(pdrv, ptbl, work) == FR_OK &&
                 f_mkfs(drv, &opt, work, work_size) == FR_OK;
            ff_diskio_unregister(pdrv);
        }
        sdmmc_card_deinit(c);
    }
    free(c);
    free(work);
    sdspi_host_remove_device(handle);
    return ok;
}

/*
  format the card as FAT (FAT32 on a card larger than a few hundred MB).
  The ESP-IDF FatFs is built without exFAT (FF_FS_EXFAT 0): a card sold as
  exFAT (SDXC, 64 GB and more) mounts only after this. Blocking, tens of
  seconds on a large card: run it from the IO thread, with no other thread
  in the filesystem
 */
bool format_sdcard()
{
    WITH_SEMAPHORE(sem);
    if (!sdcard_running) {
        const bool ok = sdspi_format_unmounted();
        mount_sdcard_spi();
        return ok && sdcard_running;
    }
    // the open files stay open on the old volume, their calls fail
    esp_vfs_fat_mount_config_t cfg = sdspi_mount_config;
    if (esp_vfs_fat_sdcard_format_cfg("/SDCARD", card, &cfg) != ESP_OK) {
        // the volume is left unmounted, the card still registered
        unmount_sdcard();
        return false;
    }
    uint64_t total_bytes, free_bytes;
    if (esp_vfs_fat_info("/SDCARD", &total_bytes, &free_bytes) != ESP_OK) {
        // the mount after the format failed: ESP-IDF has released the card
        card = nullptr;
        sdcard_running = false;
        return false;
    }
    mkdir("/SDCARD/APM", 0777);
    mkdir("/SDCARD/APM/LOGS", 0777);
    return true;
}

#endif // end spi

bool sdcard_retry(void)
{
    if (!sdcard_running) {
        mount_sdcard();
    }
    return sdcard_running;
}

void unmount_sdcard()
{
    if (card != nullptr) {
        esp_vfs_fat_sdcard_unmount( "/SDCARD", card);
        card = nullptr;
    }
    sdcard_running = false;
}

#else
// empty impl's
void mount_sdcard()
{
    printf("No sdcard setup.\n");
}
void unmount_sdcard()
{
}
bool sdcard_retry(void)
{
    return true;
}
bool format_sdcard()
{
    return false;
}
#endif




