/*
 * ph_sensor.c
 *
 * pH Sensor Driver (PH-4502C) — Implementation
 *
 * Uses the ESP-IDF v6.0 ADC One-Shot Driver to read ADC1 Channel 0 (GPIO 36).
 * Signal conditioning: trimmed mean of ADC samples, ADC line-fitting voltage
 * conversion, and user-measured two-buffer linear pH calibration.
 */

#include "ph_sensor.h"

#include <stdlib.h>
#include <math.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* -------------------------------------------------------------------------- */
/* Constants & Defaults                                                       */
/* -------------------------------------------------------------------------- */

static const char *TAG = "ph_sensor";

#define PH_ADC_UNIT          ADC_UNIT_1
#define PH_ADC_CHANNEL       ADC_CHANNEL_0       /* GPIO 36 (VP) */
#define PH_ADC_ATTEN         ADC_ATTEN_DB_12
#define PH_ADC_BITWIDTH      ADC_BITWIDTH_12      /* 0 – 4095 */

/* Trimmed moving-average — filter constants */
#define PH_FILTER_SAMPLES     30
#define PH_FILTER_DISCARD      5    /* discard lowest 5 and highest 5 */
#define PH_FILTER_INTERVAL_MS 30

/* Legacy normal-mode estimate retained until measured calibration is ready.
 * A fixed offset is not a physical Nernst slope correction. */
#define PH_DEFAULT_TEMP_COEFF 0.003f

/* Legacy estimates, used only until a real two-buffer calibration is entered. */
#define PH_DEFAULT_SLOPE      (-5.70f)   /* pH per volt */
#define PH_DEFAULT_OFFSET      21.25f    /* pH at 0 V  (7 - slope * 2.5) */

/* -------------------------------------------------------------------------- */
/* Global Calibration Variables                                               */
/* -------------------------------------------------------------------------- */

float g_ph_slope      = PH_DEFAULT_SLOPE;
float g_ph_offset     = PH_DEFAULT_OFFSET;
float g_ph_temp_coeff = PH_DEFAULT_TEMP_COEFF;

/* -------------------------------------------------------------------------- */
/* Static State                                                               */
/* -------------------------------------------------------------------------- */

static adc_oneshot_unit_handle_t s_adc1_handle = NULL;
static adc_cali_handle_t s_cali_handle = NULL;
static bool s_initialized = false;
static bool s_calibrated = false;

/* -------------------------------------------------------------------------- */
/* Comparator for qsort (ascending)                                           */
/* -------------------------------------------------------------------------- */

static int cmp_asc(const void *a, const void *b)
{
    int ia = *(const int *)a;
    int ib = *(const int *)b;
    return (ia > ib) - (ia < ib);
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

int ph_sensor_init(void)
{
    if (s_initialized) {
        return 0;   /* ESP_OK */
    }

    /* ---------- ADC1 unit configuration ---------- */
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = PH_ADC_UNIT,
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    esp_err_t ret = adc_oneshot_new_unit(&unit_cfg, &s_adc1_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ADC1 handle: %s", esp_err_to_name(ret));
        return ret;
    }

    /* ---------- Channel 0 configuration ---------- */
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = PH_ADC_ATTEN,
        .bitwidth = PH_ADC_BITWIDTH,
    };

    ret = adc_oneshot_config_channel(s_adc1_handle, PH_ADC_CHANNEL, &chan_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC1 channel 0: %s", esp_err_to_name(ret));
        adc_oneshot_del_unit(s_adc1_handle);
        s_adc1_handle = NULL;
        return ret;
    }

    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = PH_ADC_UNIT,
        .atten = PH_ADC_ATTEN,
        .bitwidth = PH_ADC_BITWIDTH,
        .default_vref = 1100,
    };
    ret = adc_cali_create_scheme_line_fitting(&cali_cfg, &s_cali_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC voltage calibration init failed: %s", esp_err_to_name(ret));
        adc_oneshot_del_unit(s_adc1_handle);
        s_adc1_handle = NULL;
        return ret;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "ADC1_CH0/GPIO36 ready; calibrate with same external divider used for readings");
    return 0;   /* ESP_OK */
}

/* -------------------------------------------------------------------------- */

adc_oneshot_unit_handle_t ph_sensor_get_adc_handle(void)
{
    return s_adc1_handle;
}

/* -------------------------------------------------------------------------- */

int ph_sensor_read_sample(ph_sensor_sample_t *sample)
{
    if (!s_initialized || !sample) {
        ESP_LOGE(TAG, "Sensor not initialized or sample pointer missing");
        return ESP_ERR_INVALID_STATE;
    }

    int samples[PH_FILTER_SAMPLES];

    /* Bench and normal mode use the same trimmed-mean filter. An ADC failure
     * invalidates the whole batch; substituting zero would bias calibration. */
    for (int i = 0; i < PH_FILTER_SAMPLES; i++) {
        int raw = 0;
        esp_err_t ret = adc_oneshot_read(s_adc1_handle, PH_ADC_CHANNEL, &raw);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ADC read failed at sample %d: %s", i, esp_err_to_name(ret));
            return ret;
        }
        samples[i] = raw;
        if (i + 1 < PH_FILTER_SAMPLES) {
            vTaskDelay(pdMS_TO_TICKS(PH_FILTER_INTERVAL_MS));
        }
    }

    /* Sort ascending and trim extremes */
    qsort(samples, PH_FILTER_SAMPLES, sizeof(samples[0]), cmp_asc);

    int sum = 0;
    for (int i = PH_FILTER_DISCARD; i < PH_FILTER_SAMPLES - PH_FILTER_DISCARD; i++) {
        sum += samples[i];
    }
    const int kept = PH_FILTER_SAMPLES - 2 * PH_FILTER_DISCARD;
    const int raw_avg = (sum + kept / 2) / kept;
    int millivolts = 0;
    esp_err_t ret = adc_cali_raw_to_voltage(s_cali_handle, raw_avg, &millivolts);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC voltage conversion failed: %s", esp_err_to_name(ret));
        return ret;
    }

    sample->raw_adc = raw_avg;
    sample->pin_voltage_v = (float)millivolts / 1000.0f;

    ESP_LOGD(TAG, "ADC=%d, GPIO36=%.3f V",
             raw_avg, (double)sample->pin_voltage_v);
    return ESP_OK;
}

float ph_sensor_read_voltage(void)
{
    ph_sensor_sample_t sample;
    return ph_sensor_read_sample(&sample) == ESP_OK ? sample.pin_voltage_v : -1.0f;
}

bool ph_sensor_calibrate_two_point(float voltage_686_v, float voltage_401_v)
{
    const float delta_v = voltage_401_v - voltage_686_v;
    if (!isfinite(voltage_686_v) || !isfinite(voltage_401_v) ||
        voltage_686_v <= 0.0f || voltage_401_v <= 0.0f ||
        voltage_686_v > 2.45f || voltage_401_v > 2.45f ||
        fabsf(delta_v) < 0.03f) {
        ESP_LOGE(TAG, "Invalid two-point calibration voltages");
        return false;
    }

    const float slope = (4.01f - 6.86f) / delta_v;
    const float offset = 6.86f - slope * voltage_686_v;
    if (!isfinite(slope) || !isfinite(offset)) return false;
    g_ph_slope = slope;
    g_ph_offset = offset;
    g_ph_temp_coeff = 0.0f; /* 25 C buffer fit; no unmeasured correction. */
    s_calibrated = true;
    ESP_LOGI(TAG, "Two-point calibration set: slope=%.4f pH/V, offset=%.4f",
             (double)slope, (double)offset);
    return true;
}

bool ph_sensor_is_calibrated(void)
{
    return s_calibrated;
}

/* -------------------------------------------------------------------------- */

float ph_sensor_read_raw(void)
{
    ph_sensor_sample_t sample;
    if (ph_sensor_read_sample(&sample) != ESP_OK) return -1.0f;

    /* Until two real buffer points have been supplied, retain the original
     * normal-mode pH estimate. The bench deliberately hides that estimate.
     * This prevents the new ADC calibration from silently changing
     * automation decisions before pH calibration is validated. */
    const float voltage = s_calibrated ? sample.pin_voltage_v :
        (float)sample.raw_adc * (3.3f / 4095.0f);

    /* Linear calibration: pH_raw = slope * voltage + offset */
    float ph_raw = (g_ph_slope * voltage) + g_ph_offset;

    ESP_LOGI(TAG, "Raw pH: %.3f (V = %.4f)", (double)ph_raw, (double)voltage);

    return ph_raw;
}

/* -------------------------------------------------------------------------- */

float ph_sensor_read_compensated(float temperature_c)
{
    float ph_raw = ph_sensor_read_raw();
    if (ph_raw < 0.0f) {
        return ph_raw;
    }

    /* The old 0.003 pH/C estimate remains only in uncalibrated normal mode.
     * Once two-point calibrated at 25 C, its coefficient is set to zero. */
    float ph_comp = ph_raw + (temperature_c - 25.0f) * g_ph_temp_coeff;

    ESP_LOGI(TAG, "pH compensated: %.3f (raw: %.3f, T: %.1f °C)",
             (double)ph_comp, (double)ph_raw, (double)temperature_c);

    return ph_comp;
}
