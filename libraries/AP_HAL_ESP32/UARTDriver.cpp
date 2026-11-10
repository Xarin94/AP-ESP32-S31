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

#include <AP_HAL_ESP32/UARTDriver.h>
#include <AP_Math/AP_Math.h>

#include "driver/gpio.h"
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#include "driver/usb_serial_jtag_vfs.h"
#endif
#include "esp_log.h"

extern const AP_HAL::HAL& hal;

namespace ESP32
{

UARTDesc uart_desc[] = {HAL_ESP32_UART_DEVICES};

static inline bool is_usb_serial_jtag(uart_port_t p)
{
#if SOC_USB_SERIAL_JTAG_SUPPORTED
    return p == USB_SERIAL_JTAG;
#else
    return false;
#endif
}

void UARTDriver::vprintf(const char *fmt, va_list ap)
{

    uart_port_t p = uart_desc[uart_num].port;
    if (p == 0) {
        esp_log_writev(ESP_LOG_INFO, "", fmt, ap);
    } else {
        AP_HAL::UARTDriver::vprintf(fmt, ap);
    }
}

// route the UART to its pins, the other way round with OPTION_SWAP
void UARTDriver::apply_pins()
{
    const UARTDesc &d = uart_desc[uart_num];
    const bool swap = option_is_set(OPTION_SWAP);
    uart_set_pin(d.port,
                 swap ? d.rx : d.tx,
                 swap ? d.tx : d.rx,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

bool UARTDriver::set_options(uint16_t options)
{
    if (uart_num >= ARRAY_SIZE(uart_desc) || is_usb_serial_jtag(uart_desc[uart_num].port)) {
        return AP_HAL::UARTDriver::set_options(options);
    }
    const bool swap_changed = ((options ^ _last_options) & OPTION_SWAP) != 0;
    _last_options = options;
    if (_initialized && swap_changed) {
        // the GPIO matrix keeps a pin on its old signal until it is reset
        const UARTDesc &d = uart_desc[uart_num];
        if (d.rx != GPIO_NUM_NC) {
            gpio_reset_pin(d.rx);
        }
        if (d.tx != GPIO_NUM_NC) {
            gpio_reset_pin(d.tx);
        }
        apply_pins();
    }
    return (options & ~OPTION_SWAP) == 0;
}

void UARTDriver::_begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    if (b == 0 && txS == 0 && rxS == 0 && _initialized) {
        // the thread owning this port has changed
        _uart_owner_thd = xTaskGetCurrentTaskHandle();
        return;
    }

    if (uart_num < ARRAY_SIZE(uart_desc)) {
        uart_port_t p = uart_desc[uart_num].port;
#if SOC_USB_SERIAL_JTAG_SUPPORTED
        if (is_usb_serial_jtag(p)) {
            if (!_initialized) {
                usb_serial_jtag_driver_config_t config = {
                    .tx_buffer_size = TX_BUF_SIZE,
                    .rx_buffer_size = RX_BUF_SIZE,
                };
                if (!usb_serial_jtag_is_driver_installed()) {
                    usb_serial_jtag_driver_install(&config);
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
                    // the console writes through the driver too, not
                    // straight to the FIFO the driver is filling
                    usb_serial_jtag_vfs_use_driver();
#endif
                }
                _readbuf.set_size(RX_BUF_SIZE);
                _writebuf.set_size(TX_BUF_SIZE);
                _uart_owner_thd = xTaskGetCurrentTaskHandle();
                _initialized = true;
            }
            // no baudrate on USB
            _baudrate = b;
            return;
        }
#endif
        if (!_initialized) {

            uart_config_t config = {
                .baud_rate = (int)b,
                .data_bits = UART_DATA_8_BITS,
                .parity = UART_PARITY_DISABLE,
                .stop_bits = UART_STOP_BITS_1,
                .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            };
            uart_param_config(p, &config);
            apply_pins();
            //uart_driver_install(p, 2*UART_FIFO_LEN, 0, 0, nullptr, 0);
            uart_driver_install(p, 2*UART_HW_FIFO_LEN(p), 0, 0, nullptr, 0);
            _readbuf.set_size(RX_BUF_SIZE);
            _writebuf.set_size(TX_BUF_SIZE);
            _uart_owner_thd = xTaskGetCurrentTaskHandle();

            _initialized = true;
        } else {
            flush();
            uart_set_baudrate(p, b);

        }
    }
    _baudrate = b;
}

void UARTDriver::_end()
{
    if (_initialized) {
        if (is_usb_serial_jtag(uart_desc[uart_num].port)) {
#if SOC_USB_SERIAL_JTAG_SUPPORTED
            usb_serial_jtag_driver_uninstall();
#endif
        } else {
            uart_driver_delete(uart_desc[uart_num].port);
        }
        _readbuf.set_size(0);
        _writebuf.set_size(0);
    }
    _initialized = false;
}

void UARTDriver::_flush()
{
    uart_port_t p = uart_desc[uart_num].port;
    if (is_usb_serial_jtag(p)) {
        return;
    }
    uart_flush(p);
}

bool UARTDriver::is_initialized()
{
    return _initialized;
}

bool UARTDriver::tx_pending()
{
    return (_writebuf.available() > 0);
}


uint32_t UARTDriver::_available()
{
    if (!_initialized || _uart_owner_thd != xTaskGetCurrentTaskHandle()) {
        return 0;
    }
    return _readbuf.available();
}

uint32_t UARTDriver::txspace()
{
    if (!_initialized) {
        return 0;
    }
    int result =  _writebuf.space();
    result -= TX_BUF_SIZE / 4;
    return MAX(result, 0);

}

ssize_t IRAM_ATTR UARTDriver::_read(uint8_t *buffer, uint16_t count)
{
    if (_uart_owner_thd != xTaskGetCurrentTaskHandle()) {
        return -1;
    }

    if (!_initialized) {
        return -1;
    }

    const uint32_t ret = _readbuf.read(buffer, count);
    if (ret == 0) {
        return 0;
    }


    _receive_timestamp_update();

    return ret;
}

void IRAM_ATTR UARTDriver::_timer_tick(void)
{
    if (!_initialized) {
        return;
    }
    read_data();
    write_data();
}

void IRAM_ATTR UARTDriver::read_data()
{
    uart_port_t p = uart_desc[uart_num].port;
    int count = 0;
#if SOC_USB_SERIAL_JTAG_SUPPORTED
    if (is_usb_serial_jtag(p)) {
        do {
            count = usb_serial_jtag_read_bytes(_buffer, sizeof(_buffer), 0);
            if (count > 0) {
                _readbuf.write(_buffer, count);
            }
        } while (count > 0);
        return;
    }
#endif
    do {
        count = uart_read_bytes(p, _buffer, sizeof(_buffer), 0);
        if (count > 0) {
            _readbuf.write(_buffer, count);
        }
    } while (count > 0);
}

void IRAM_ATTR UARTDriver::write_data()
{
    uart_port_t p = uart_desc[uart_num].port;
    int count = 0;
    _write_mutex.take_blocking();
    do {
        count = _writebuf.peekbytes(_buffer, sizeof(_buffer));
        if (count > 0) {
#if SOC_USB_SERIAL_JTAG_SUPPORTED
            if (is_usb_serial_jtag(p)) {
                count = usb_serial_jtag_write_bytes(_buffer, count, 0);
            } else
#endif
            count = uart_tx_chars(p, (const char*) _buffer, count);
            _writebuf.advance(count);
        }
    } while (count > 0);
    _write_mutex.give();
}

size_t IRAM_ATTR UARTDriver::_write(const uint8_t *buffer, size_t size)
{
    if (!_initialized) {
        return 0;
    }

    _write_mutex.take_blocking();


    size_t ret = _writebuf.write(buffer, size);
    _write_mutex.give();
    return ret;
}

bool UARTDriver::_discard_input()
{
    if (_uart_owner_thd != xTaskGetCurrentTaskHandle()) {
        return false;
    }
    if (!_initialized) {
        return false;
    }

    _readbuf.clear();

    return true;
}

// record timestamp of new incoming data
void IRAM_ATTR UARTDriver::_receive_timestamp_update(void)
{
    _receive_timestamp[_receive_timestamp_idx^1] = AP_HAL::micros64();
    _receive_timestamp_idx ^= 1;
}


/*
  return timestamp estimate in microseconds for when the start of
  a nbytes packet arrived on the uart. This should be treated as a
  time constraint, not an exact time. It is guaranteed that the
  packet did not start being received after this time, but it
  could have been in a system buffer before the returned time.
  This takes account of the baudrate of the link. For transports
  that have no baudrate (such as USB) the time estimate may be
  less accurate.
  A return value of zero means the HAL does not support this API
*/
uint64_t UARTDriver::receive_time_constraint_us(uint16_t nbytes)
{
    uint64_t last_receive_us = _receive_timestamp[_receive_timestamp_idx];
    if (_baudrate > 0 && !is_usb_serial_jtag(uart_desc[uart_num].port)) {
        // assume 10 bits per byte. For USB we assume zero transport delay
        uint32_t transport_time_us = (1000000UL * 10UL / _baudrate) * (nbytes + available());
        last_receive_us -= transport_time_us;
    }
    return last_receive_us;
}

}
