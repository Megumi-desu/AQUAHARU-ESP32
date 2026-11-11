/*
 * tds_sensor.c
 *
 * TDS / EC Sensor Driver (DFRobot Gravity TDS / generic EC transmitter)
 * — Implementation
 *
 * Uses the ESP-IDF v6.0 ADC One-Shot Driver to read ADC1 Channel 3 (GPIO 39).
 * Applies the DFRobot temperature-compensation model and cubic polynomial
 * to derive TDS (ppm), then derives EC (mS/cm).
 */

#include "tds_sensor.h"

#include <stdint.h>

#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* -------------------------------------------------------------------------- */
/* Constants & Defaults                                                       */
/* -------------------------------------------------------------------------- */

static const char *TAG = "tds_sensor";

#define TDS_ADC_UNIT          ADC_UNIT_1
#define TDS_ADC_CHANNEL       ADC_CHANNEL_3       /* GPIO 39 (VN) */
#define TDS_ADC_ATTEN         ADC_ATTEN_DB_12      /* 0 – 3.1 V range */
#define TDS_ADC_BITWIDTH      ADC_BITWIDTH_12      /* 0 – 4095 */

#define TDS_ADC_MAX_READING    4095.0f
#define TDS_ADC_REF_VOLTAGE    3.3f

/* Voltage divider correction factor: V_sensor = V_adc * R_div_correction
 * R1 = 10kΩ, R2 = 20kΩ => ratio = (R1+R2)/R2 = 1.5 */
#define TDS_DIVIDER_FACTOR     1.5f

/* DFRobot temperature compensation coefficient (per °C) */
#define TDS_TEMP_COMP_COEFF    0.02f

/* 30 samples at 40 ms apart (29 waits) reject short ADC/noise spikes. */
#define TDS_MEDIAN_SAMPLE_COUNT 30
#define TDS_SAMPLE_INTERVAL_MS  40

/* Cubic polynomial coefficients (DFRobot Gravity TDS) */
#define TDS_CUBIC_C3  133.42f
#define TDS_CUBIC_C2  (-255.86f)
#define TDS_CUBIC_C1  857.39f
#define TDS_CUBIC_SCALE  0.5f

/* Default calibration and conversion factors */
#define TDS_DEFAULT_CAL_FACTOR  1.0f
#define TDS_DEFAULT_EC_FACTOR   500.0f

/* -------------------------------------------------------------------------- */
/* Global Calibration Variables                                               */
/* -------------------------------------------------------------------------- */

float g_tds_calibration_factor = TDS_DEFAULT_CAL_FACTOR;
float g_tds_to_ec_factor       = TDS_DEFAULT_EC_FACTOR;

/* -------------------------------------------------------------------------- */
/* Static State                                                               */
/* -------------------------------------------------------------------------- */

static adc_oneshot_unit_handle_t s_adc1_handle = NULL;
static bool s_initialized = false;

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

int tds_sensor_init(adc_oneshot_unit_handle_t existing_handle)
{
    if (s_initialized) {
        return 0;   /* ESP_OK */
    }

    if (existing_handle != NULL) {
        /* Use the ADC1 handle shared from another component (e.g. ph_sensor) */
        s_adc1_handle = existing_handle;
        ESP_LOGI(TAG, "Using shared ADC1 handle from another component");
    } else {
        /* Create our own ADC1 handle (standalone mode) */
        adc_oneshot_unit_init_cfg_t unit_cfg = {
            .unit_id  = TDS_ADC_UNIT,
            .clk_src  = ADC_DIGI_CLK_SRC_DEFAULT,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };

        esp_err_t ret = adc_oneshot_new_unit(&unit_cfg, &s_adc1_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create ADC1 handle: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    /* ---------- Channel 3 configuration ---------- */
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = TDS_ADC_ATTEN,
        .bitwidth = TDS_ADC_BITWIDTH,
    };

    esp_err_t ret = adc_oneshot_config_channel(s_adc1_handle, TDS_ADC_CHANNEL, &chan_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC1 channel 3: %s", esp_err_to_name(ret));
        s_adc1_handle = NULL;
        return ret;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Initialized ADC1_CH3 (GPIO 39), cal_factor=%.3f, ec_factor=%.1f",
             (double)g_tds_calibration_factor, (double)g_tds_to_ec_factor);
    return 0;   /* ESP_OK */
}

/* -------------------------------------------------------------------------- */

float tds_sensor_read_voltage(void)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Sensor not initialized — call tds_sensor_init() first");
        return -1.0f;
    }

    int raw = 0;
    esp_err_t ret = adc_oneshot_read(s_adc1_handle, TDS_ADC_CHANNEL, &raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC read failed: %s", esp_err_to_name(ret));
        return -1.0f;
    }

    /* Convert ADC reading to voltage at the ESP32 pin */
    float v_adc = (float)raw * (TDS_ADC_REF_VOLTAGE / TDS_ADC_MAX_READING);

#ifdef TDS_USE_VOLTAGE_DIVIDER
    /* External resistor divider correction: V_sensor = V_adc * (R1+R2)/R2 */
    float v_sensor = v_adc * TDS_DIVIDER_FACTOR;
    ESP_LOGD(TAG, "V_adc=%.4f V, V_sensor (with divider)=%.4f V",
             (double)v_adc, (double)v_sensor);
    return v_sensor;
#else
    ESP_LOGD(TAG, "V_sensor=%.4f V (direct)", (double)v_adc);
    return v_adc;
#endif
}

/* -------------------------------------------------------------------------- */

static float voltage_to_tds(float voltage, float temperature_c)
{
    /*
     * Temperature compensation of voltage (DFRobot model):
     *   V_comp = V_sensor / (1.0 + k * (T - 25.0))
     *
     * The DFRobot Gravity TDS sensor output drifts with temperature;
     * this compensation normalizes to the 25 °C reference.
     */
    float v_comp = voltage / (1.0f + TDS_TEMP_COMP_COEFF * (temperature_c - 25.0f));

    /*
     * Cubic polynomial TDS calculation (DFRobot Gravity TDS):
     *   TDS_raw = (C3 * V^3 + C2 * V^2 + C1 * V) * scale
     */
    float v2 = v_comp * v_comp;
    float v3 = v2 * v_comp;

    float tds_raw = (TDS_CUBIC_C3 * v3 + TDS_CUBIC_C2 * v2 + TDS_CUBIC_C1 * v_comp)
                    * TDS_CUBIC_SCALE;

    /* Apply user calibration factor */
    float tds_cal = tds_raw * g_tds_calibration_factor;

    ESP_LOGI(TAG, "V_sensor=%.4f V, V_comp=%.4f V, TDS_raw=%.1f ppm, "
             "TDS_cal=%.1f ppm (T=%.1f °C)",
             (double)voltage, (double)v_comp,
             (double)tds_raw, (double)tds_cal, (double)temperature_c);

    return tds_cal;
}

float tds_sensor_read_tds_filtered(float temperature_c, float *filtered_voltage_out)
{
    int samples[TDS_MEDIAN_SAMPLE_COUNT];
    if (filtered_voltage_out) *filtered_voltage_out = -1.0f;
    if (!s_initialized || !s_adc1_handle) {
        ESP_LOGE(TAG, "Sensor not initialized — call tds_sensor_init() first");
        return -1.0f;
    }

    /* Collect a fixed window without heap allocation. Each wait is 40 ms,
     * well above this project's 10 ms FreeRTOS tick. */
    for (int index = 0; index < TDS_MEDIAN_SAMPLE_COUNT; ++index) {
        int raw = 0;
        esp_err_t err = adc_oneshot_read(s_adc1_handle, TDS_ADC_CHANNEL, &raw);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ADC sample %d/%d failed: %s", index + 1,
                     TDS_MEDIAN_SAMPLE_COUNT, esp_err_to_name(err));
            return -1.0f;
        }
        samples[index] = raw;
        if (index + 1 < TDS_MEDIAN_SAMPLE_COUNT) {
            vTaskDelay(pdMS_TO_TICKS(TDS_SAMPLE_INTERVAL_MS));
        }
    }

    /* In-place insertion sort; 30 elements keep work and stack bounded. */
    for (int index = 1; index < TDS_MEDIAN_SAMPLE_COUNT; ++index) {
        const int value = samples[index];
        int position = index;
        while (position > 0 && samples[position - 1] > value) {
            samples[position] = samples[position - 1];
            --position;
        }
        samples[position] = value;
    }

    /* Even sample count: average the two central ADC counts. */
    const float median_raw =
        ((float)samples[TDS_MEDIAN_SAMPLE_COUNT / 2 - 1] +
         (float)samples[TDS_MEDIAN_SAMPLE_COUNT / 2]) * 0.5f;
    float voltage = median_raw * (TDS_ADC_REF_VOLTAGE / TDS_ADC_MAX_READING);
#ifdef TDS_USE_VOLTAGE_DIVIDER
    voltage *= TDS_DIVIDER_FACTOR;
#endif
    if (filtered_voltage_out) *filtered_voltage_out = voltage;

    return voltage_to_tds(voltage, temperature_c);
}

float tds_sensor_read_tds(float temperature_c)
{
    return tds_sensor_read_tds_filtered(temperature_c, NULL);
}

float tds_sensor_ec_from_tds(float tds_ppm)
{
    if (tds_ppm < 0.0f) return tds_ppm;
    return tds_ppm / g_tds_to_ec_factor;
}

/* -------------------------------------------------------------------------- */

float tds_sensor_read_ec(float temperature_c)
{
    float tds = tds_sensor_read_tds(temperature_c);
    if (tds < 0.0f) {
        return tds;   /* propagate error */
    }

    float ec = tds_sensor_ec_from_tds(tds);

    ESP_LOGI(TAG, "EC: %.3f mS/cm (TDS: %.1f ppm, factor: %.1f)",
             (double)ec, (double)tds, (double)g_tds_to_ec_factor);

    return ec;
}
