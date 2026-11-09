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

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>

#include "HAL_ESP32_Class.h"
#ifdef HAL_ESP32_USB_NO_CHIP_RESET
#include "soc/soc.h"
#include "soc/usb_serial_jtag_reg.h"
#endif
#include "Scheduler.h"
#include "I2CDevice.h"
#include "SPIDevice.h"
#include "UARTDriver.h"
#include "WiFiDriver.h"
#include "WiFiUdpDriver.h"
#include "RCInput.h"
#include "RCOutput.h"
#include "GPIO.h"
#include "Storage.h"
#include "AnalogIn.h"
#include "Util.h"
#if AP_SIM_ENABLED
#include <AP_HAL/SIMState.h>
#endif

static ESP32::UARTDriver cons(0);
#ifdef HAL_ESP32_WIFI
#if HAL_ESP32_WIFI == 1
static ESP32::WiFiDriver serial1Driver; //tcp, client should connect to 192.168.4.1 port 5760
#elif HAL_ESP32_WIFI == 2
static ESP32::WiFiUdpDriver serial1Driver; //udp
#else
static Empty::UARTDriver serial1Driver;
#endif
#else
static Empty::UARTDriver serial1Driver;
#endif
static ESP32::UARTDriver serial2Driver(2);
static ESP32::UARTDriver serial3Driver(1);
static Empty::UARTDriver serial4Driver;
static Empty::UARTDriver serial5Driver;
static Empty::UARTDriver serial6Driver;
static Empty::UARTDriver serial7Driver;
static Empty::UARTDriver serial8Driver;
static Empty::UARTDriver serial9Driver;

#if HAL_WITH_DSP
static Empty::DSP dspDriver;
#endif

static ESP32::I2CDeviceManager i2cDeviceManager;
#if defined(HAL_ESP32_SPI_BUSES)
static ESP32::SPIDeviceManager spiDeviceManager;
#else
static Empty::SPIDeviceManager spiDeviceManager;
#endif
#if AP_HAL_ANALOGIN_ENABLED
static ESP32::AnalogIn analogIn;
#else
static Empty::AnalogIn analogIn;
#endif
#ifdef HAL_USE_EMPTY_STORAGE
static Empty::Storage storageDriver;
#else
static ESP32::Storage storageDriver;
#endif
static ESP32::GPIO gpioDriver;
#if AP_SIM_ENABLED
static Empty::RCOutput rcoutDriver;
#else
static ESP32::RCOutput rcoutDriver;
#endif
static ESP32::RCInput rcinDriver;
static ESP32::Scheduler schedulerInstance;
static ESP32::Util utilInstance;
static Empty::OpticalFlow opticalFlowDriver;
static Empty::Flash flashDriver;

#if AP_SIM_ENABLED
static AP_HAL::SIMState xsimstate;
#endif

#if HAL_NUM_CAN_IFACES > 0
// created by AP_CANManager
static ESP32::CANIface* canDrivers[HAL_NUM_CAN_IFACES];
#endif

extern const AP_HAL::HAL& hal;

HAL_ESP32::HAL_ESP32() :
    AP_HAL::HAL(
        &cons, //Console/mavlink
        &serial1Driver, //Telem 1
        &serial2Driver, //Telem 2
        &serial3Driver, //GPS 1
        &serial4Driver, //GPS 2
        &serial5Driver, //Extra 1
        &serial6Driver, //Extra 2
        &serial7Driver, //Extra 3
        &serial8Driver, //Extra 4
        &serial9Driver, //Extra 5
        &i2cDeviceManager,
        &spiDeviceManager,
        nullptr,
        &analogIn,
        &storageDriver,
        &cons,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance,
        &opticalFlowDriver,
        &flashDriver,
#if AP_SIM_ENABLED
        &xsimstate,
#endif
#if HAL_WITH_DSP
        &dspDriver,
#endif
#if HAL_NUM_CAN_IFACES > 0
        (AP_HAL::CANIface**)canDrivers
#else
        nullptr
#endif
    )
{}

void HAL_ESP32::run(int argc, char * const argv[], Callbacks* callbacks) const
{
#ifdef HAL_ESP32_USB_NO_CHIP_RESET
    // the host must not reset the chip through the lines of the USB serial port
    SET_PERI_REG_MASK(USB_SERIAL_JTAG_CHIP_RST_REG, USB_SERIAL_JTAG_USB_UART_CHIP_RST_DIS);
#endif

#ifdef HAL_ESP32_SPI_DEVICES
    ESP32::SPIDeviceManager::deassert_all_cs();
#endif

#if AP_SIM_ENABLED
    AP::sitl()->init();
#endif  // AP_SIM_ENABLED

    ((ESP32::Scheduler *)hal.scheduler)->set_callbacks(callbacks);
    hal.scheduler->init();
}

void AP_HAL::init()
{
}

