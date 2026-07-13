/*
 * ph_sensor.c
 *
 * pH Sensor Driver (PH-4502C) — Implementation
 *
 * Uses the ESP-IDF v6.0 ADC One-Shot Driver to read ADC1 Channel 0 (GPIO 36).
 * Signal conditioning: trimmed moving-average filter, linear calibration,
 * and Nernst-based temperature compensation.
 */

#include "ph_sensor.h"

#include <stdlib.h>

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
#define PH_ADC_ATTEN         ADC_ATTEN_DB_12      /* 0 – 3.1 V range */
#define PH_ADC_BITWIDTH      ADC_BITWIDTH_12      /* 0 – 4095 */

#define PH_ADC_MAX_READING    4095.0f
#define PH_ADC_REF_VOLTAGE    3.3f

/* Trimmed moving-average — filter constants */
#define PH_FILTER_SAMPLES     30
#define PH_FILTER_DISCARD      5    /* discard lowest 5 and highest 5 */
#define PH_FILTER_INTERVAL_MS 30

/* Temperature compensation — default coefficient (pH / °C) */
#define PH_DEFAULT_TEMP_COEFF 0.003f

/* Default linear calibration (ideal sensor: pH = 7 at V = 2.5 V) */
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
static bool s_initialized = false;

/* -------------------------------------------------------------------------- */
/* Comparator for qsort (ascending)                                           */
/* -------------------------------------------------------------------------- */

static int cmp_asc(const void *a, const void *b)
{
    float fa = *(const float *)a;
    float fb = *(const float *)b;
    return (fa > fb) - (fa < fb);
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

    s_initialized = true;
    ESP_LOGI(TAG, "Initialized ADC1_CH0 (GPIO 36), slope=%.2f, offset=%.2f",
             (double)g_ph_slope, (double)g_ph_offset);
    return 0;   /* ESP_OK */
}

/* -------------------------------------------------------------------------- */

adc_oneshot_unit_handle_t ph_sensor_get_adc_handle(void)
{
    return s_adc1_handle;
}

/* -------------------------------------------------------------------------- */

float ph_sensor_read_voltage(void)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Sensor not initialized — call ph_sensor_init() first");
        return -1.0f;
    }

    float samples[PH_FILTER_SAMPLES];

    /* Collect 30 samples at 30 ms intervals */
    for (int i = 0; i < PH_FILTER_SAMPLES; i++) {
        int raw = 0;
        esp_err_t ret = adc_oneshot_read(s_adc1_handle, PH_ADC_CHANNEL, &raw);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "ADC read failed at sample %d: %s", i, esp_err_to_name(ret));
            samples[i] = 0.0f;
        } else {
            samples[i] = (float)raw;
        }
        vTaskDelay(pdMS_TO_TICKS(PH_FILTER_INTERVAL_MS));
    }

    /* Sort ascending and trim extremes */
    qsort(samples, PH_FILTER_SAMPLES, sizeof(float), cmp_asc);

    float sum = 0.0f;
    int   count = 0;
    for (int i = PH_FILTER_DISCARD; i < PH_FILTER_SAMPLES - PH_FILTER_DISCARD; i++) {
        sum += samples[i];
        count++;
    }

    float adc_avg = sum / (float)count;

    /* Convert ADC reading to voltage */
    float voltage = adc_avg * (PH_ADC_REF_VOLTAGE / PH_ADC_MAX_READING);

    ESP_LOGD(TAG, "Filtered ADC avg: %.1f → Voltage: %.4f V",
             (double)adc_avg, (double)voltage);

    return voltage;
}

/* -------------------------------------------------------------------------- */

float ph_sensor_read_raw(void)
{
    float voltage = ph_sensor_read_voltage();
    if (voltage < 0.0f) {
        return voltage;   /* propagate error */
    }

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

    /*
     * Temperature compensation:
     *   pH_comp = pH_raw + (T - 25) * coeff
     *
     * The coefficient 0.003 pH/°C accounts for the Nernstian shift of
     * ~0.03 pH per 10 °C, scaled to a per-degree linear approximation.
     */
    float ph_comp = ph_raw + (temperature_c - 25.0f) * g_ph_temp_coeff;

    ESP_LOGI(TAG, "pH compensated: %.3f (raw: %.3f, T: %.1f °C)",
             (double)ph_comp, (double)ph_raw, (double)temperature_c);

    return ph_comp;
}
