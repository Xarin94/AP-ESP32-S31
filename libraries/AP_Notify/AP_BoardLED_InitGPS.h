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
#pragma once

#include "AP_Notify_config.h"

#if AP_NOTIFY_GPIO_LED_INIT_GPS_ENABLED

#include "NotifyDevice.h"

/*
  two status LEDs:
  LED A: fast flash = starting up, solid = initialisation finished
  LED B: off = no GPS lock, 1 Hz blink = GPS lock, solid = GPS lock and
         absolute position from the AHRS
 */
class AP_BoardLED_InitGPS: public NotifyDevice
{
public:
    bool init(void) override;

    // should be called at 50Hz
    void update(void) override;
};

#endif  // AP_NOTIFY_GPIO_LED_INIT_GPS_ENABLED
