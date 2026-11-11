/* BENCH ONLY: synthetic telemetry, no sensors/control/actuator driver or SSE.
 * The six connected relay inputs are active LOW; this file holds them HIGH/OFF.
 * Enabled by RUN_BENCH_FIREBASE_SIM in main.c. */
#include "bench_firebase_sim.h"

#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "firebase_client.h"
#include "lcd_display.h"

#define BENCH_INTERVAL_MS 3000
/* Opt in only when testing the dashboard HISTORY chart. No POST by default.
 * Limit attempts (including uncertain/failed responses) to avoid unbounded data.
 * Reboot resets this budget. Record the serial history keys for cleanup. */
#define BENCH_SIM_HISTORY_ENABLED 0
#define BENCH_SIM_HISTORY_MAX_ATTEMPTS 30

typedef struct {
    double ph, ec_ms_cm, tds_ppm, temperature_c, light_v;
    bool water_low;
} sim_sample_t;

/* Edit this array to change the repeating sequence. All values are synthetic. */
static const sim_sample_t SIM_SAMPLES[] = {
    {6.2, 1.4, 700, 24.0, 1.8, false},
    {6.7, 1.8, 900, 25.5, 2.0, false},
    {7.1, 2.6, 1300, 27.0, 2.2, true},
};

static const char *TAG = "bench_firebase";
static bool s_lcd_ok;

__attribute__((format(printf, 2, 3)))
static void lcd_line(unsigned row, const char *format, ...)
{
    if (!s_lcd_ok) return;
    char text[80], padded[LCD_COLS + 1];
    va_list args;
    va_start(args, format);
    vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    snprintf(padded, sizeof(padded), "%-*.*s", LCD_COLS, LCD_COLS, text);
    lcd_write_at(row, 0, padded);
}

/* Fixed-size serialization avoids per-cycle heap allocations. Field names and
 * types mirror task_upload() exactly. RTDB resolves .sv to numeric Unix ms.
 * The pump_status object is ONLY a report of our forced-OFF bench outputs. */
static bool make_payload(char *buffer, size_t size, const sim_sample_t *sample,
                         bool live)
{
    const char *pumps = live ?
        ",\"pump_status\":{\"ph_up\":false,\"ph_down\":false,"
        "\"nutrient_a\":false,\"nutrient_b\":false,\"peltier\":false,"
        "\"circulation_pump\":false}" : "";
    int length = snprintf(buffer, size,
        "{\"ph\":%.2f,\"ec_ms_cm\":%.2f,\"tds_ppm\":%.0f,"
        "\"temperature_c\":%.1f,\"light_v\":%.2f,\"water_low\":%s,"
        "\"timestamp_ms\":{\".sv\":\"timestamp\"}%s}",
        sample->ph, sample->ec_ms_cm, sample->tds_ppm,
        sample->temperature_c, sample->light_v,
        sample->water_low ? "true" : "false", pumps);
    return length > 0 && (size_t)length < size;
}

static void bench_task(void *arg)
{
    s_lcd_ok = lcd_init() == ESP_OK;
    if (!s_lcd_ok) ESP_LOGE(TAG, "LCD init failed; serial remains available");
    lcd_line(0, "AQUAHARU SIM ONLY");
    lcd_line(1, "All relays OFF");
    lcd_line(2, "WiFi: connecting");
    lcd_line(3, "Firebase: init/auth");
    ESP_LOGW(TAG, "SIMULATION: no sensor/control/actuator tasks, history=%s",
             BENCH_SIM_HISTORY_ENABLED ? "ON (bounded)" : "OFF");

    /* init owns Wi-Fi and Auth; do not run standalone bench Wi-Fi first.
     * Call once: repeating init after failure would duplicate SDK resources. */
    int init_result = firebase_client_init();
    if (init_result != ESP_OK) {
        ESP_LOGE(TAG, "Firebase init failed: %s; fix connection/Auth and reset",
                 esp_err_to_name(init_result));
        lcd_line(3, "FB init FAIL: reset");
        while (true) {
            lcd_line(2, "WiFi:%s NO UPLOAD", firebase_wifi_is_connected() ? "UP" : "DOWN");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_LOGI(TAG, "Target: /%s/%s/live (3-second cadence)",
             FIREBASE_DEVICE_ROOT, firebase_get_device_id());
    TickType_t wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(BENCH_INTERVAL_MS);
    size_t sample_index = 0;
#if BENCH_SIM_HISTORY_ENABLED
    unsigned history_attempts = 0;
#endif
    while (true) {
        const sim_sample_t *sample = &SIM_SAMPLES[sample_index];
        lcd_line(0, "SIM pH:%.1f EC:%.2f", sample->ph, sample->ec_ms_cm);
        lcd_line(1, "T:%.1fC H2O:%s", sample->temperature_c, sample->water_low ? "LOW" : "OK");
        lcd_line(2, "WiFi:%s HB:pending", firebase_wifi_is_connected() ? "UP" : "DOWN");
        lcd_line(3, "FB PUT: sending");
        ESP_LOGI(TAG, "SIM[%u] ph=%.1f ec=%.2f tds=%.0f temp=%.1f water_low=%s",
                 (unsigned)sample_index, sample->ph, sample->ec_ms_cm, sample->tds_ppm,
                 sample->temperature_c, sample->water_low ? "true" : "false");

        char payload[640];
        int live_status = 0, heartbeat_status = 0;
        int live_result = ESP_ERR_INVALID_STATE, heartbeat_result = ESP_ERR_INVALID_STATE;
        if (firebase_is_ready() && make_payload(payload, sizeof(payload), sample, true)) {
            live_result = firebase_put_telemetry_with_status(payload, &live_status);
            heartbeat_result = firebase_put_heartbeat_with_status(&heartbeat_status);
#if BENCH_SIM_HISTORY_ENABLED
            if (live_result == ESP_OK && history_attempts < BENCH_SIM_HISTORY_MAX_ATTEMPTS &&
                make_payload(payload, sizeof(payload), sample, false)) {
                ++history_attempts; /* Count attempts even if response is lost. */
                int result = firebase_post_history(payload);
                ESP_LOGI(TAG, "SIM history attempt %u/%u: %s", history_attempts,
                         (unsigned)BENCH_SIM_HISTORY_MAX_ATTEMPTS, esp_err_to_name(result));
            }
#endif
        }
        ESP_LOGI(TAG, "live PUT HTTP %d result=%s; heartbeat PUT HTTP %d result=%s",
                 live_status, esp_err_to_name(live_result),
                 heartbeat_status, esp_err_to_name(heartbeat_result));
        lcd_line(2, "WiFi:%s HB:%d%s", firebase_wifi_is_connected() ? "UP" : "DOWN",
                 heartbeat_status, heartbeat_result == ESP_OK ? "" : "!");
        lcd_line(3, "FB PUT:%d %s", live_status, live_result == ESP_OK ? "OK" : "FAIL");
        sample_index = (sample_index + 1) % (sizeof(SIM_SAMPLES) / sizeof(SIM_SAMPLES[0]));

        /* Normal cadence is start-to-start 3 s. Slow HTTPS cannot meet it:
         * log the overrun and wait a fresh period, never burst catch-up writes. */
        if ((TickType_t)(xTaskGetTickCount() - wake) >= period) {
            ESP_LOGW(TAG, "Upload exceeded 3 s; delaying next sample (no catch-up burst)");
            wake = xTaskGetTickCount();
        }
        vTaskDelayUntil(&wake, period);
    }
}

void bench_firebase_sim_start(void)
{
    /* No actuator init/timers; explicitly hold active-LOW inputs HIGH/OFF
     * throughout bench execution. Set output latch before enabling output. */
    const gpio_num_t pins[] = {GPIO_NUM_19, GPIO_NUM_18, GPIO_NUM_5,
                              GPIO_NUM_17, GPIO_NUM_16, GPIO_NUM_4};
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
    /* HTTPS/Auth needs more stack than app_main's configured 3584 bytes. */
    BaseType_t created = xTaskCreatePinnedToCore(bench_task, "benchFirebase", 12288,
                                               NULL, 1, NULL, 0);
    if (created != pdPASS) {
        ESP_LOGE(TAG, "Cannot create bench task; relay outputs remain OFF");
        configASSERT(0);
    }
}
