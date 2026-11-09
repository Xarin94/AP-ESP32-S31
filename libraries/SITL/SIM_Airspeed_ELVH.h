/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  simulated ELVH-L04D differential pressure sensor with SPI output
  (+-4 inH2O, transfer function A), on SPI:elvh; set ARSPD_TYPE 21 and
  ARSPD_ELVH_RNG 4
 */
#pragma once

#include "SIM_config.h"

#if AP_SIM_AIRSPEED_ELVH_ENABLED

#include "SIM_SPIDevice.h"

namespace SITL {

class Airspeed_ELVH : public SPIDevice
{
public:

    int rdwr(uint8_t count, SPI::spi_ioc_transfer *&data) override;

private:

    // time of the last conversion read, for the stale status
    uint64_t last_read_us;
};

} // namespace SITL

#endif  // AP_SIM_AIRSPEED_ELVH_ENABLED
