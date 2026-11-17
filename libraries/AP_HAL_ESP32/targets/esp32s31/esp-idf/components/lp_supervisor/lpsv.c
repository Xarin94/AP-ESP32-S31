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

#include <string.h>
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_system.h"
#include "ulp_lp_core.h"
#include "ulp_lpsv.h"
#include "lpsv.h"

static const char *TAG = "lpsv";

extern const uint8_t lpsv_bin_start[] asm("_binary_ulp_lpsv_bin_start");
extern const uint8_t lpsv_bin_end[] asm("_binary_ulp_lpsv_bin_end");

// survives the reboot requested by the LP core
static RTC_NOINIT_ATTR lpsv_postmortem_t s_postmortem;

// LP RAM, reloaded by the bootloader at every HP boot: the LP core stops
// when it no longer holds the token of the boot that started it
static RTC_DATA_ATTR uint32_t s_token;

static bool s_started;

lpsv_shared_t *lpsv_shared(void)
{
    return (lpsv_shared_t *)&ulp_lpsv;
}

bool lpsv_take_postmortem(lpsv_postmortem_t *out)
{
    lpsv_postmortem_t *pm = &s_postmortem;
    const bool valid = pm->magic == LPSV_PM_MAGIC &&
                       pm->version == LPSV_VERSION &&
                       pm->sum == lpsv_pm_sum(pm);
    if (valid && out != NULL) {
        memcpy(out, pm, sizeof(*out));
    }
    pm->magic = 0;
    return valid;
}

void lpsv_stop(void)
{
    if (!s_started) {
        return;
    }
    s_started = false;
    s_token = 0;
    ulp_lp_core_stop();
}

bool lpsv_start(const lpsv_config_t *cfg)
{
    if (s_started) {
        return true;
    }
    esp_err_t err = ulp_lp_core_load_binary(lpsv_bin_start, lpsv_bin_end - lpsv_bin_start);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "load failed: %s", esp_err_to_name(err));
        return false;
    }

    lpsv_shared_t *s = lpsv_shared();
    uint32_t token;
    do {
        token = esp_random();
    } while (token == 0);
    s_token = token;

    s->token = token;
    s->token_addr = (uint32_t)&s_token;
    s->pm_addr = (uint32_t)&s_postmortem;
    s->pc_sampling = cfg->pc_sampling ? 1 : 0;
    s->hp_task = -1;
    for (int i = 0; i < LPSV_CH_MAX; i++) {
        s->warn_ms[i] = cfg->warn_ms[i];
    }
    s->vital_mask = 0;
    s->wdt_ms = 0;
    __sync_synchronize();
    s->version = LPSV_VERSION;

    ulp_lp_core_cfg_t run_cfg = {
        .wakeup_source = ULP_LP_CORE_WAKEUP_SOURCE_HP_CPU,
    };
    err = ulp_lp_core_run(&run_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "start failed: %s", esp_err_to_name(err));
        return false;
    }
    s_started = true;

    // an orderly restart must not look like a stall to the LP core
    esp_register_shutdown_handler(lpsv_stop);
    return true;
}
