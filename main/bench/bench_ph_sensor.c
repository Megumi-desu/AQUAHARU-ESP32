/* BENCH ONLY: PH-4502C voltage acquisition and provisional pH estimate.
 * No Wi-Fi, Firebase, control tasks, or actuator driver. Disconnect relays.
 * This module's PO is <=3.3 V. A verified PO offset trimmer or an external
 * divider can keep GPIO36 in the characterized ADC range. Calibration
 * voltages are always measured at GPIO36 with the final hardware setup. */
#include "bench_ph_sensor.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lcd_display.h"
#include "ph_sensor.h"

static const char *TAG = "bench_ph";

#define PH_BENCH_INTERVAL_MS 2000
#define PH_BENCH_TASK_STACK  4096

/* Provisional bench-only mapping with the trimpot at its latest position.
 * 6.86 V is the mean of recent ESP32 readings (2.732-2.740 V).
 * 4.01 V is the owner's multimeter reading, not yet a same-session ESP32
 * reading. Replace both with stable ESP32 values from the final setup.
 * These points exceed the ESP32 characterized ADC range; never use this
 * estimate for dosing or transfer it to normal operation. */
#define PH_BENCH_VOLTAGE_686_V 2.736f
#define PH_BENCH_VOLTAGE_401_V 3.130f

static float bench_ph_estimate(float voltage_v)
{
    return 6.86f + (voltage_v - PH_BENCH_VOLTAGE_686_V) *
           (4.01f - 6.86f) /
           (PH_BENCH_VOLTAGE_401_V - PH_BENCH_VOLTAGE_686_V);
}

static void write_line(uint8_t row, const char *text)
{
    char line[LCD_COLS + 1];
    snprintf(line, sizeof(line), "%-*.*s", LCD_COLS, LCD_COLS, text);
    lcd_write_at(row, 0, line);
}

static void ph_bench_task(void *arg)
{
    const bool lcd_ready = lcd_init() == ESP_OK;
    if (lcd_ready) {
        lcd_clear();
        write_line(0, "AQUAHARU pH BENCH");
        write_line(1, "pH input GPIO36");
        write_line(2, "Initializing ADC...");
        write_line(3, "pH estimate: bench");
    } else {
        ESP_LOGE(TAG, "LCD init failed; serial readings remain available");
    }

    const int init_result = ph_sensor_init();
    if (init_result != ESP_OK) {
        ESP_LOGE(TAG, "pH ADC init failed: %s", esp_err_to_name(init_result));
        if (lcd_ready) write_line(2, "ADC init failed");
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "pH bench ready: provisional estimate from %.3f V / pH 6.86 and %.3f V / pH 4.01",
             (double)PH_BENCH_VOLTAGE_686_V,
             (double)PH_BENCH_VOLTAGE_401_V);

    TickType_t wake = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(PH_BENCH_INTERVAL_MS);
    unsigned sample_number = 0;
    while (true) {
        ph_sensor_sample_t sample;
        const int read_result = ph_sensor_read_sample(&sample);
        if (read_result != ESP_OK) {
            ESP_LOGE(TAG, "Sample %u failed: %s", sample_number,
                     esp_err_to_name(read_result));
            if (lcd_ready) {
                write_line(2, "ADC read failed");
                write_line(3, "Check serial/wiring");
            }
        } else {
            const float ph_est = bench_ph_estimate(sample.pin_voltage_v);
            ESP_LOGI(TAG, "Sample %u: ADC=%d, GPIO36=%.3f V, pH~=%.2f (bench estimate)",
                     sample_number, sample.raw_adc,
                     (double)sample.pin_voltage_v, (double)ph_est);
            if (lcd_ready) {
                char line[LCD_COLS + 1];
                snprintf(line, sizeof(line), "ADC raw %4d", sample.raw_adc);
                write_line(1, line);
                snprintf(line, sizeof(line), "GPIO36 %.3f V",
                         (double)sample.pin_voltage_v);
                write_line(2, line);
                snprintf(line, sizeof(line), "pH~%.2f bench", (double)ph_est);
                write_line(3, line);
            }
        }
        ++sample_number;
        vTaskDelayUntil(&wake, interval);
    }
}

void bench_ph_sensor_start(void)
{
    const BaseType_t created = xTaskCreatePinnedToCore(
        ph_bench_task, "benchPh", PH_BENCH_TASK_STACK, NULL, 1, NULL, 0);
    if (created != pdPASS) {
        ESP_LOGE(TAG, "Cannot create pH bench task");
        configASSERT(0);
    }
}
