#include "SIM_config.h"

#if AP_SIM_AIRSPEED_ELVH_ENABLED

#include "SIM_Airspeed_ELVH.h"

#include <AP_Baro/AP_Baro.h>

#include "SITL.h"

int SITL::Airspeed_ELVH::rdwr(uint8_t count, SPI::spi_ioc_transfer *&tfrs)
{
    // read-only sensor: one receive of 2, 3 or 4 bytes
    if (count != 1 || tfrs[0].tx_buf != 0 || tfrs[0].rx_buf == 0 ||
        tfrs[0].len < 2 || tfrs[0].len > 4) {
        AP_HAL::panic("ELVH: unexpected SPI transfer");
    }

    // conversions are about 0.5 ms apart: a faster read gets the stale status
    const uint64_t now_us = AP_HAL::micros64();
    const uint8_t status = (now_us - last_read_us < 500) ? 0b10 : 0;
    last_read_us = now_us;

    const auto *sitl = AP::sitl();

    // +-4 inH2O over 10%-90% of the 2^14 counts, zero at 8192
    const float range_pa = 4 * INCH_OF_H2O_TO_PASCAL;
    const float counts = 8192.0f + sitl->state.airspeed_raw_pressure[0] * (13107.2f / (2 * range_pa));
    const uint32_t press_raw = constrain_float(counts, 0, 16383);

    const float temperature = AP_Baro::get_temperatureC_for_alt_amsl(sitl->state.altitude);
    const uint32_t temp_raw = constrain_float((temperature + 50.0f) * (2047.0f / 200.0f), 0, 2047);

    const uint32_t packed = (uint32_t(status) << 30) | (press_raw << 16) | (temp_raw << 5);
    uint8_t *buf = (uint8_t *)(tfrs[0].rx_buf);
    for (uint8_t i = 0; i < tfrs[0].len; i++) {
        buf[i] = (packed >> (24 - 8 * i)) & 0xff;
    }
    return 0;
}

#endif  // AP_SIM_AIRSPEED_ELVH_ENABLED
