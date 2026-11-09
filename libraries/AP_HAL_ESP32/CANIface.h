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
  CAN interface on the ESP32 TWAI controller, through the ESP-IDF >= 5.5
  node driver (esp_twai.h). Classic CAN frames only (DroneCAN at 1 Mbit/s).
 */
#pragma once

#include "AP_HAL_ESP32.h"

#if HAL_NUM_CAN_IFACES > 0

#include <AP_HAL/CANIface.h>
#include "Semaphores.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"

namespace ESP32
{

struct CANDesc {
    gpio_num_t tx;
    gpio_num_t rx;
};

class CANIface : public AP_HAL::CANIface
{
public:
    CANIface(uint8_t index);

    bool init(const uint32_t bitrate) override;

    int16_t send(const AP_HAL::CANFrame& frame, uint64_t tx_deadline, CanIOFlags flags) override;
    int16_t receive(AP_HAL::CANFrame& out_frame, uint64_t& out_timestamp_us, CanIOFlags& out_flags) override;

    bool select(bool &read_select, bool &write_select,
                const AP_HAL::CANFrame* const pending_tx, uint64_t blocking_deadline) override;
    bool set_event_handle(AP_HAL::BinarySemaphore *handle) override;

    bool is_initialized() const override { return node != nullptr; }
    bool is_busoff() const override { return busoff; }
    uint32_t getErrorCount() const override { return error_count; }

    void flush_tx() override;
    void clear_rx() override;

    void get_stats(ExpandingString &str) override;
    const bus_stats_t *get_statistics(void) const override { return &stats; }

protected:
    int8_t get_iface_num() const override { return index; }
    bool add_to_rx_queue(const CanRxItem &rx_item) override;

private:
    static constexpr uint8_t TX_QUEUE_DEPTH = 8;
    // on an empty bus every frame ends in bus-off: retry at this period
    static constexpr uint32_t BUSOFF_RECOVER_MS = 1000;
    // TX frames are referenced by the driver until sent, so each one owns a
    // slot that is released by the TX-done interrupt
    static constexpr uint8_t TX_SLOTS = 2 * TX_QUEUE_DEPTH;
    static constexpr uint16_t RX_QUEUE_DEPTH = 128;

    struct TxSlot {
        twai_frame_t frame;  // first member: the TX-done event points here
        uint8_t data[AP_HAL::CANFrame::NonFDCANMaxDataLen];
        bool loopback;
        volatile bool busy;
    };

    // ISR context
    static bool on_rx_done(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *ctx);
    static bool on_tx_done(twai_node_handle_t handle, const twai_tx_done_event_data_t *edata, void *ctx);
    static bool on_state_change(twai_node_handle_t handle, const twai_state_change_event_data_t *edata, void *ctx);
    static bool on_error(twai_node_handle_t handle, const twai_error_event_data_t *edata, void *ctx);

    bool tx_space() const;
    void recover_busoff();

    const uint8_t index;
    twai_node_handle_t node;
    QueueHandle_t rx_queue;
    AP_HAL::BinarySemaphore *sem_handle;
    HAL_Semaphore tx_sem;

    TxSlot tx_slots[TX_SLOTS];

    volatile bool busoff;

    uint32_t last_recover_ms;
    volatile uint32_t error_count;
    bus_stats_t stats;
};

}

#endif // HAL_NUM_CAN_IFACES > 0
