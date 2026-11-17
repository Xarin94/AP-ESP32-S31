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
  HP side of the LP core supervisor: loads and starts the LP program.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "lpsv_shared.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool pc_sampling;       // the LP core reads the HP core PCs (bus monitor)
    uint16_t warn_ms[LPSV_CH_MAX];
} lpsv_config_t;

// copy out and invalidate the post-mortem left by the watchdog before the
// last reset; false if there is none. Call before lpsv_start()
bool lpsv_take_postmortem(lpsv_postmortem_t *out);

// load the LP program, configure it and start it. Returns false if the
// program could not be loaded or started
bool lpsv_start(const lpsv_config_t *cfg);

// stop the LP program (also run on esp_restart())
void lpsv_stop(void);

// configuration, heartbeats, statistics and events in LP RAM
lpsv_shared_t *lpsv_shared(void);

#ifdef __cplusplus
}
#endif
