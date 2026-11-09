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

#include "CANIface.h"

#if HAL_NUM_CAN_IFACES > 0

#include <AP_Common/ExpandingString.h>
#include <AP_Math/AP_Math.h>
#include "esp_timer.h"

extern const AP_HAL::HAL& hal;

using namespace ESP32;

static const CANDesc can_desc[] = { HAL_ESP32_CAN_DEVICES };
static_assert(ARRAY_SIZE(can_desc) == HAL_NUM_CAN_IFACES, "ESP32_CAN lines must match HAL_NUM_CAN_IFACES");

CANIface::CANIface(uint8_t _index) :
    index(_index),
    node(nullptr),
    rx_queue(nullptr),
    sem_handle(nullptr),
    busoff(false),
    error_count(0),
    stats{}
{
    memset(tx_slots, 0, sizeof(tx_slots));
}

bool CANIface::init(const uint32_t bitrate)
{
    if (node != nullptr) {
        return true;
    }
    if (index >= ARRAY_SIZE(can_desc)) {
        return false;
    }
    if (rx_queue == nullptr) {
        rx_queue = xQueueCreate(RX_QUEUE_DEPTH, sizeof(CanRxItem));
        if (rx_queue == nullptr) {
            return false;
        }
    }

    twai_onchip_node_config_t cfg {};
    cfg.io_cfg.tx = can_desc[index].tx;
    cfg.io_cfg.rx = can_desc[index].rx;
    cfg.io_cfg.quanta_clk_out = GPIO_NUM_NC;
    cfg.io_cfg.bus_off_indicator = GPIO_NUM_NC;
    cfg.bit_timing.bitrate = bitrate;
    cfg.tx_queue_depth = TX_QUEUE_DEPTH;
    // retransmit until acknowledged, as a standard CAN controller does
    cfg.fail_retry_cnt = -1;
    cfg.flags.no_receive_rtr = 1;

    if (twai_new_node_onchip(&cfg, &node) != ESP_OK) {
        node = nullptr;
        return false;
    }

    twai_event_callbacks_t cbs {};
    cbs.on_rx_done = on_rx_done;
    cbs.on_tx_done = on_tx_done;
    cbs.on_state_change = on_state_change;
    cbs.on_error = on_error;
    if (twai_node_register_event_callbacks(node, &cbs, this) != ESP_OK ||
        twai_node_enable(node) != ESP_OK) {
        twai_node_delete(node);
        node = nullptr;
        return false;
    }

    bitrate_ = bitrate;
    return true;
}

/*
  interrupt handlers
 */
bool IRAM_ATTR CANIface::on_rx_done(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *ctx)
{
    CANIface *iface = (CANIface *)ctx;
    uint8_t data[AP_HAL::CANFrame::NonFDCANMaxDataLen];
    twai_frame_t f {};
    f.buffer = data;
    f.buffer_len = sizeof(data);
    if (twai_node_receive_from_isr(handle, &f) != ESP_OK) {
        iface->stats.rx_errors++;
        return false;
    }

    CanRxItem item;
    item.timestamp_us = esp_timer_get_time();
    item.flags = 0;
    item.frame.id = f.header.id & (f.header.ide ? AP_HAL::CANFrame::MaskExtID : AP_HAL::CANFrame::MaskStdID);
    if (f.header.ide) {
        item.frame.id |= AP_HAL::CANFrame::FlagEFF;
    }
    if (f.header.rtr) {
        item.frame.id |= AP_HAL::CANFrame::FlagRTR;
    }
    const uint16_t len = MIN(twaifd_dlc2len(f.header.dlc), (uint16_t)sizeof(data));
    item.frame.dlc = len;
    memcpy(item.frame.data, data, len);

    BaseType_t woken = pdFALSE;
    if (xQueueSendFromISR(iface->rx_queue, &item, &woken) == pdTRUE) {
        iface->stats.rx_received++;
    } else {
        iface->stats.rx_overflow++;
    }
    if (iface->sem_handle != nullptr) {
        iface->sem_handle->signal_ISR();
    }
    return woken == pdTRUE;
}

bool IRAM_ATTR CANIface::on_tx_done(twai_node_handle_t handle, const twai_tx_done_event_data_t *edata, void *ctx)
{
    CANIface *iface = (CANIface *)ctx;
    TxSlot *slot = (TxSlot *)edata->done_tx_frame;
    BaseType_t woken = pdFALSE;

    if (edata->is_tx_success) {
        const uint64_t now_us = esp_timer_get_time();
        iface->stats.tx_success++;
        iface->stats.last_transmit_us = now_us;
        if (slot->loopback) {
            // transmitted frames requested back, with their TX time (DroneCAN time sync)
            CanRxItem item;
            item.timestamp_us = now_us;
            item.flags = Loopback;
            item.frame.id = slot->frame.header.id | (slot->frame.header.ide ? AP_HAL::CANFrame::FlagEFF : 0);
            item.frame.dlc = slot->frame.buffer_len;
            memcpy(item.frame.data, slot->data, slot->frame.buffer_len);
            xQueueSendFromISR(iface->rx_queue, &item, &woken);
        }
    } else {
        iface->stats.tx_abort++;
    }
    slot->busy = false;

    if (iface->sem_handle != nullptr) {
        iface->sem_handle->signal_ISR();
    }
    return woken == pdTRUE;
}

bool IRAM_ATTR CANIface::on_state_change(twai_node_handle_t handle, const twai_state_change_event_data_t *edata, void *ctx)
{
    CANIface *iface = (CANIface *)ctx;
    if (edata->new_sta == TWAI_ERROR_BUS_OFF) {
        iface->busoff = true;
        iface->stats.num_busoff_err++;
    } else if (edata->new_sta == TWAI_ERROR_ACTIVE) {
        iface->busoff = false;
    }
    return false;
}

bool IRAM_ATTR CANIface::on_error(twai_node_handle_t handle, const twai_error_event_data_t *edata, void *ctx)
{
    CANIface *iface = (CANIface *)ctx;
    iface->error_count++;
    return false;
}

/*
  task context
 */
void CANIface::recover_busoff()
{
    const uint32_t now_ms = AP_HAL::millis();
    if (busoff && node != nullptr && now_ms - last_recover_ms >= BUSOFF_RECOVER_MS) {
        last_recover_ms = now_ms;
        // starts the bus-off recovery sequence (128 x 11 recessive bits);
        // the state change interrupt clears busoff once it completes
        twai_node_recover(node);
    }
}

bool CANIface::tx_space() const
{
    if (busoff) {
        return false;
    }
    for (const auto &slot : tx_slots) {
        if (!slot.busy) {
            return true;
        }
    }
    return false;
}

int16_t CANIface::send(const AP_HAL::CANFrame& frame, uint64_t tx_deadline, CanIOFlags flags)
{
    // frame forwarding to registered callbacks
    AP_HAL::CANIface::send(frame, tx_deadline, flags);

    if (node == nullptr) {
        return -1;
    }
    stats.tx_requests++;
    if (frame.isErrorFrame() || frame.dlc > AP_HAL::CANFrame::NonFDCANMaxDataLen || frame.isCanFDFrame()) {
        stats.tx_rejected++;
        return -1;
    }
    if (tx_deadline != 0 && AP_HAL::micros64() > tx_deadline) {
        stats.tx_timedout++;
        return -1;
    }
    recover_busoff();
    if (busoff) {
        // the driver would refuse the frame and print the error through
        // the ROM, straight into the USB console stream
        return 0;
    }

    WITH_SEMAPHORE(tx_sem);

    TxSlot *slot = nullptr;
    for (auto &s : tx_slots) {
        if (!s.busy) {
            slot = &s;
            break;
        }
    }
    if (slot == nullptr) {
        stats.tx_overflow++;
        return 0;
    }

    slot->frame = twai_frame_t {};
    slot->frame.header.ide = frame.isExtended();
    slot->frame.header.rtr = frame.isRemoteTransmissionRequest();
    slot->frame.header.id = frame.id & (frame.isExtended() ? AP_HAL::CANFrame::MaskExtID : AP_HAL::CANFrame::MaskStdID);
    slot->frame.header.dlc = frame.dlc;
    memcpy(slot->data, frame.data, frame.dlc);
    slot->frame.buffer = slot->data;
    slot->frame.buffer_len = frame.dlc;
    slot->loopback = (flags & Loopback) != 0;
    slot->busy = true;

    const esp_err_t ret = twai_node_transmit(node, &slot->frame, 0);
    if (ret == ESP_OK) {
        return 1;
    }
    slot->busy = false;
    if (ret == ESP_ERR_TIMEOUT) {
        // driver queue full
        stats.tx_overflow++;
        return 0;
    }
    stats.tx_rejected++;
    return -1;
}

int16_t CANIface::receive(AP_HAL::CANFrame& out_frame, uint64_t& out_timestamp_us, CanIOFlags& out_flags)
{
    if (rx_queue == nullptr) {
        return -1;
    }
    CanRxItem item;
    if (xQueueReceive(rx_queue, &item, 0) != pdTRUE) {
        return 0;
    }
    out_frame = item.frame;
    out_timestamp_us = item.timestamp_us;
    out_flags = item.flags;

    // frame forwarding to registered callbacks
    AP_HAL::CANIface::receive(out_frame, out_timestamp_us, out_flags);
    return 1;
}

bool CANIface::add_to_rx_queue(const CanRxItem &rx_item)
{
    if (rx_queue == nullptr) {
        return false;
    }
    if (xQueueSend(rx_queue, &rx_item, 0) != pdTRUE) {
        stats.rx_overflow++;
        return false;
    }
    if (sem_handle != nullptr) {
        sem_handle->signal();
    }
    return true;
}

bool CANIface::select(bool &read_select, bool &write_select,
                      const AP_HAL::CANFrame* const pending_tx, uint64_t blocking_deadline)
{
    if (node == nullptr) {
        return false;
    }
    recover_busoff();

    const bool want_read = read_select;
    const bool want_write = write_select;
    read_select = want_read && uxQueueMessagesWaiting(rx_queue) > 0;
    write_select = want_write && tx_space();
    if (read_select || write_select) {
        return true;
    }

    const uint64_t now_us = AP_HAL::micros64();
    if (sem_handle != nullptr && blocking_deadline > now_us) {
        IGNORE_RETURN(sem_handle->wait(blocking_deadline - now_us));
    }

    read_select = want_read && uxQueueMessagesWaiting(rx_queue) > 0;
    write_select = want_write && tx_space();
    return read_select || write_select;
}

bool CANIface::set_event_handle(AP_HAL::BinarySemaphore *handle)
{
    sem_handle = handle;
    return true;
}

void CANIface::flush_tx()
{
    // frames already handed to the driver cannot be withdrawn; wait for them
    if (node != nullptr) {
        twai_node_transmit_wait_all_done(node, 10);
    }
}

void CANIface::clear_rx()
{
    if (rx_queue != nullptr) {
        xQueueReset(rx_queue);
    }
}

void CANIface::get_stats(ExpandingString &str)
{
    twai_node_status_t status {};
    twai_node_record_t record {};
    if (node != nullptr) {
        twai_node_get_info(node, &status, &record);
    }
    str.printf("tx_requests:    %lu\n"
               "tx_rejected:    %lu\n"
               "tx_overflow:    %lu\n"
               "tx_success:     %lu\n"
               "tx_timedout:    %lu\n"
               "tx_abort:       %lu\n"
               "rx_received:    %lu\n"
               "rx_overflow:    %lu\n"
               "rx_errors:      %lu\n"
               "num_busoff_err: %lu\n"
               "bus_errors:     %lu\n"
               "tec/rec:        %u/%u\n",
               (unsigned long)stats.tx_requests,
               (unsigned long)stats.tx_rejected,
               (unsigned long)stats.tx_overflow,
               (unsigned long)stats.tx_success,
               (unsigned long)stats.tx_timedout,
               (unsigned long)stats.tx_abort,
               (unsigned long)stats.rx_received,
               (unsigned long)stats.rx_overflow,
               (unsigned long)stats.rx_errors,
               (unsigned long)stats.num_busoff_err,
               (unsigned long)record.bus_err_num,
               (unsigned)status.tx_error_count,
               (unsigned)status.rx_error_count);
}

#endif // HAL_NUM_CAN_IFACES > 0
