/* BENCH ONLY: real pH, TDS/EC, DS18B20, LDR DO and float switch with LCD + Firebase.
 * No control or actuator task is started; digital polarity was verified on device_01.
 * Six connected relay inputs are active LOW and held HIGH/OFF. */
#include "bench_all_sensors.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "firebase_client.h"
#include "lcd_display.h"
#include "ph_sensor.h"
#include "tds_sensor.h"
#include "temp_sensor.h"

static const char *TAG = "bench_all";

/* 30 s includes ~0.8 s temperature, ~0.9 s pH, ~1.2 s TDS and HTTPS.
 * History is one POST per 5 min (first successful live upload posts once). */
#define BENCH_LIVE_INTERVAL_MS    30000
#define BENCH_HISTORY_INTERVAL_MS 300000
#define BENCH_TASK_STACK         12288

/* Provisional pH mapping from the latest bench. 6.86 point is filtered
 * GPIO36 voltage; 4.01 point was measured with a multimeter, not yet
 * reconfirmed as a filtered ADC reading. This is an estimate only. */
#define BENCH_PH_V_686 2.736f
#define BENCH_PH_V_401 3.130f

/* BENCH ONLY: set after measuring a known TDS standard at its stated temperature.
 * New factor = current factor * standard_ppm / logged_TDS_cal_ppm. */
#define BENCH_TDS_CAL_FACTOR 1.0f

#define BENCH_LDR_GPIO   GPIO_NUM_34
#define BENCH_FLOAT_GPIO GPIO_NUM_35
#define BENCH_DIGITAL_SAMPLES 5
#define BENCH_DIGITAL_GAP_MS 20

typedef struct {
    float ph, ec_ms_cm, tds_ppm, temperature_c;
    bool ph_ok, tds_ok, temp_ok;
    bool light_do_high, float_switch_high;
    int ph_raw;
} readings_t;

static bool s_lcd_ok;
static bool s_temp_ok;

static void lcd_line(unsigned row, const char *value)
{
    if (!s_lcd_ok) return;
    char padded[LCD_COLS + 1];
    snprintf(padded, sizeof(padded), "%-*.*s", LCD_COLS, LCD_COLS, value);
    lcd_write_at(row, 0, padded);
}

static void format_value(char *out, size_t size, float value, bool valid,
                         unsigned decimals)
{
    if (!valid || !isfinite(value)) {
        snprintf(out, size, "null");
    } else {
        snprintf(out, size, "%.*f", (int)decimals, (double)value);
    }
}

static bool read_digital_high(gpio_num_t pin)
{
    /* Majority vote suppresses short DO noise and float-switch contact bounce. */
    unsigned high_count = 0;
    for (unsigned i = 0; i < BENCH_DIGITAL_SAMPLES; ++i) {
        if (gpio_get_level(pin) == 1) ++high_count;
        if (i + 1 < BENCH_DIGITAL_SAMPLES) {
            vTaskDelay(pdMS_TO_TICKS(BENCH_DIGITAL_GAP_MS));
        }
    }
    return high_count > BENCH_DIGITAL_SAMPLES / 2;
}

static readings_t read_all(void)
{
    readings_t result = {0};
    if (s_temp_ok) {
        result.temperature_c = temp_sensor_sample();
        result.temp_ok = isfinite(result.temperature_c) &&
                         result.temperature_c > -40.0f &&
                         result.temperature_c < 85.0f;
    }

    ph_sensor_sample_t ph_sample;
    if (ph_sensor_read_sample(&ph_sample) == ESP_OK) {
        result.ph_raw = ph_sample.raw_adc;
        /* At the ADC rail the voltage no longer carries pH information. */
        result.ph_ok = ph_sample.raw_adc > 5 && ph_sample.raw_adc < 4090;
        if (result.ph_ok) {
            result.ph = 6.86f + (ph_sample.pin_voltage_v - BENCH_PH_V_686) *
                        (4.01f - 6.86f) / (BENCH_PH_V_401 - BENCH_PH_V_686);
            result.ph_ok = isfinite(result.ph) && result.ph >= 0.0f &&
                           result.ph <= 14.0f;
        }
    }

    if (result.temp_ok) {
        result.tds_ppm = tds_sensor_read_tds(result.temperature_c);
        result.tds_ok = isfinite(result.tds_ppm) && result.tds_ppm >= 0.0f;
        if (result.tds_ok) {
            result.ec_ms_cm = tds_sensor_ec_from_tds(result.tds_ppm);
        }
    }
    result.light_do_high = read_digital_high(BENCH_LDR_GPIO);
    result.float_switch_high = read_digital_high(BENCH_FLOAT_GPIO);
    return result;
}

static bool make_payload(char *buffer, size_t size, const readings_t *r,
                         bool live)
{
    char ph[20], ec[20], tds[20], temp[20];
    format_value(ph, sizeof(ph), r->ph, r->ph_ok, 2);
    format_value(ec, sizeof(ec), r->ec_ms_cm, r->tds_ok, 3);
    format_value(tds, sizeof(tds), r->tds_ppm, r->tds_ok, 1);
    format_value(temp, sizeof(temp), r->temperature_c, r->temp_ok, 2);
    const char *pumps = live ?
        ",\"pump_status\":{\"ph_up\":false,\"ph_down\":false,"
        "\"nutrient_a\":false,\"nutrient_b\":false,\"peltier\":false,"
        "\"circulation_pump\":false}" : "";
    const int written = snprintf(buffer, size,
        "{\"ph\":%s,\"ec_ms_cm\":%s,\"tds_ppm\":%s,"
        "\"temperature_c\":%s,\"light_do_high\":%s,"
        "\"float_switch_high\":%s,\"water_low\":%s,"
        "\"ph_estimated\":true,\"measurement_mode\":\"bench\","
        "\"timestamp_ms\":{\".sv\":\"timestamp\"}%s}",
        ph, ec, tds, temp,
        r->light_do_high ? "true" : "false",
        r->float_switch_high ? "true" : "false",
        r->float_switch_high ? "true" : "false", pumps);
    return written > 0 && (size_t)written < size;
}

static void bench_task(void *arg)
{
    s_lcd_ok = lcd_init() == ESP_OK;
    lcd_line(0, "AQUAHARU SENS BENCH");
    lcd_line(1, "Relay inputs HIGH");
    lcd_line(2, "Sensors initializing");
    lcd_line(3, "Firebase waiting");

    s_temp_ok = temp_sensor_init();
    g_tds_calibration_factor = BENCH_TDS_CAL_FACTOR;
    if (ph_sensor_init() != ESP_OK ||
        tds_sensor_init(ph_sensor_get_adc_handle()) != ESP_OK) {
        ESP_LOGE(TAG, "ADC sensor init failed; no upload");
        lcd_line(3, "ADC init failed");
        vTaskDelete(NULL);
        return;
    }
    const gpio_config_t digital_inputs = {
        .pin_bit_mask = (1ULL << BENCH_LDR_GPIO) | (1ULL << BENCH_FLOAT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&digital_inputs));
    if (!s_temp_ok) ESP_LOGE(TAG, "DS18B20 init failed; temp/TDS/EC sent as null");

    const int init_result = firebase_client_init();
    if (init_result != ESP_OK) {
        ESP_LOGE(TAG, "Firebase init failed: %s; sensor bench remains local",
                 esp_err_to_name(init_result));
    }
    ESP_LOGI(TAG, "Bench: live/heartbeat every 30 s, history every 5 min; LDR HIGH=bright, FLOAT HIGH=water low");

    TickType_t wake = xTaskGetTickCount();
    TickType_t last_history = wake;
    bool first_history = true;
    const TickType_t live_period = pdMS_TO_TICKS(BENCH_LIVE_INTERVAL_MS);
    const TickType_t history_period = pdMS_TO_TICKS(BENCH_HISTORY_INTERVAL_MS);
    unsigned sample_number = 0;
    while (true) {
        const readings_t r = read_all();
        char ph[20], ec[20], tds[20], temp[20];
        format_value(ph, sizeof(ph), r.ph, r.ph_ok, 2);
        format_value(ec, sizeof(ec), r.ec_ms_cm, r.tds_ok, 2);
        format_value(tds, sizeof(tds), r.tds_ppm, r.tds_ok, 0);
        format_value(temp, sizeof(temp), r.temperature_c, r.temp_ok, 1);
        ESP_LOGI(TAG, "Sample %u: pH~%s ADC=%d, EC=%s mS/cm, TDS=%s ppm, T=%s C, LDR_DO=%s, FLOAT=%s",
                 sample_number++, ph, r.ph_raw, ec, tds, temp,
                 r.light_do_high ? "HIGH" : "LOW",
                 r.float_switch_high ? "HIGH" : "LOW");

        char line[LCD_COLS + 1];
        snprintf(line, sizeof(line), "pH~%.5s T:%.5sC", r.ph_ok ? ph : "--",
                 r.temp_ok ? temp : "--");
        lcd_line(0, line);
        snprintf(line, sizeof(line), "EC:%.5s TDS:%.5s", r.tds_ok ? ec : "--",
                 r.tds_ok ? tds : "--");
        lcd_line(1, line);
        snprintf(line, sizeof(line), "L:%c F:%c WiFi:%s",
                 r.light_do_high ? 'H' : 'L',
                 r.float_switch_high ? 'H' : 'L',
                 firebase_wifi_is_connected() ? "UP" : "DOWN");
        lcd_line(2, line);

        int live_status = 0, heartbeat_status = 0;
        int live_result = ESP_ERR_INVALID_STATE;
        int heartbeat_result = ESP_ERR_INVALID_STATE;
        int history_result = ESP_ERR_INVALID_STATE;
        bool history_attempted = false;
        char payload[640];
        if (firebase_is_ready() && make_payload(payload, sizeof(payload), &r, true)) {
            live_result = firebase_put_telemetry_with_status(payload, &live_status);
            heartbeat_result = firebase_put_heartbeat_with_status(&heartbeat_status);
            const TickType_t now = xTaskGetTickCount();
            if (live_result == ESP_OK &&
                (first_history || (TickType_t)(now - last_history) >= history_period) &&
                make_payload(payload, sizeof(payload), &r, false)) {
                history_attempted = true;
                first_history = false;
                last_history = now; /* No burst or duplicate retry on uncertain response. */
                history_result = firebase_post_history(payload);
            }
        }
        ESP_LOGI(TAG, "Firebase live HTTP %d %s; heartbeat HTTP %d %s; history %s%s",
                 live_status, esp_err_to_name(live_result),
                 heartbeat_status, esp_err_to_name(heartbeat_result),
                 history_attempted ? esp_err_to_name(history_result) : "not due",
                 history_attempted ? " (POST)" : "");
        snprintf(line, sizeof(line), "FB:%d HB:%d H:%s", live_status,
                 heartbeat_status, history_attempted ?
                 (history_result == ESP_OK ? "OK" : "ERR") : "--");
        lcd_line(3, line);

        if ((TickType_t)(xTaskGetTickCount() - wake) >= live_period) {
            ESP_LOGW(TAG, "Cycle exceeded 30 s; rescheduling without catch-up burst");
            wake = xTaskGetTickCount();
        }
        vTaskDelayUntil(&wake, live_period);
    }
}

void bench_all_sensors_start(void)
{
    /* Hardware observation shows active-LOW input polarity. Hold the six
     * original relay inputs and reported GPIO2 input HIGH/OFF throughout.
     * Set latches before changing direction; never call actuators_init. */
    const gpio_num_t pins[] = {GPIO_NUM_19, GPIO_NUM_18, GPIO_NUM_5,
                              GPIO_NUM_17, GPIO_NUM_16, GPIO_NUM_4, GPIO_NUM_2};
    uint64_t mask = 0;
    for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
        ESP_ERROR_CHECK(gpio_set_level(pins[i], 1));
        mask |= 1ULL << pins[i];
    }
    const gpio_config_t config = {
        .pin_bit_mask = mask, .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&config));
    if (xTaskCreatePinnedToCore(bench_task, "benchAll", BENCH_TASK_STACK,
                                NULL, 1, NULL, 0) != pdPASS) {
        ESP_LOGE(TAG, "Cannot start bench task; relay pins remain HIGH/OFF");
        configASSERT(0);
    }
}
