/* BENCH ONLY: Gravity analog TDS voltage and calculated ppm on the LCD.
 * No temperature sensor, pH, control tasks, Firebase, or actuator driver.
 * Relay pins are not initialized or changed; disconnect the relay harness. */
#include "bench_tds_sensor.h"

#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lcd_display.h"
#include "tds_sensor.h"

static const char *TAG = "bench_tds";

#define TDS_BENCH_INTERVAL_MS 1500
#define TDS_BENCH_TEMP_REF_C  25.0f
#define TDS_BENCH_TASK_STACK  4096

static void write_line(uint8_t row, const char *text)
{
    char line[LCD_COLS + 1];
    snprintf(line, sizeof(line), "%-*.*s", LCD_COLS, LCD_COLS, text);
    lcd_write_at(row, 0, line);
}

static void tds_bench_task(void *arg)
{
    const bool lcd_ready = lcd_init() == ESP_OK;
    if (!lcd_ready) {
        ESP_LOGE(TAG, "LCD init failed; readings remain available in serial log");
    } else {
        lcd_clear();
        write_line(0, "AQUAHARU TDS TEST");
        write_line(1, "ADC1 GPIO39");
        write_line(2, "Initializing...");
        write_line(3, "Ref temperature 25C");
    }

    /* Standalone ADC1 unit: do not start ph_sensor_init(), which would also
     * initialize/read the separate pH channel. The app normal path can share
     * the ADC handle later; this isolated bench deliberately does not. */
    const int init_result = tds_sensor_init(NULL);
    if (init_result != ESP_OK) {
        ESP_LOGE(TAG, "TDS ADC init failed: %s", esp_err_to_name(init_result));
        if (lcd_ready) {
            write_line(2, "ADC init failed");
            write_line(3, "Check serial log");
        }
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_LOGI(TAG, "TDS bench ready: ADC1_CH3/GPIO39, temperature reference=25.0 C, factor=%.3f",
             (double)g_tds_calibration_factor);
    TickType_t wake = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(TDS_BENCH_INTERVAL_MS);
    unsigned sample = 0;

    while (true) {
        /* This first test isolates the analog TDS board. Use its documented
         * 25 C reference until TDS + DS18B20 are deliberately integrated. */
        float voltage = -1.0f;
        const float tds_ppm = tds_sensor_read_tds_filtered(
            TDS_BENCH_TEMP_REF_C, &voltage);
        if (voltage < 0.0f || tds_ppm < 0.0f) {
            ESP_LOGE(TAG, "Sample %u failed (voltage=%.3f V, TDS=%.1f ppm)",
                     sample, (double)voltage, (double)tds_ppm);
            if (lcd_ready) {
                write_line(2, "ADC read failed");
                write_line(3, "Check wiring/log");
            }
        } else {
            ESP_LOGI(TAG, "Sample %u: %.3f V, %.1f ppm at 25.0 C ref (cal factor %.3f)",
                     sample, (double)voltage, (double)tds_ppm,
                     (double)g_tds_calibration_factor);
            if (lcd_ready) {
                char line[LCD_COLS + 1];
                snprintf(line, sizeof(line), "ADC: %.3f V", (double)voltage);
                write_line(2, line);
                snprintf(line, sizeof(line), "TDS: %.0f ppm", (double)tds_ppm);
                write_line(3, line);
            }
        }
        ++sample;
        vTaskDelayUntil(&wake, interval);
    }
}

void bench_tds_sensor_start(void)
{
    /* The relay board's polarity is unverified; keep its GPIOs untouched. */
    const BaseType_t created = xTaskCreatePinnedToCore(
        tds_bench_task, "benchTds", TDS_BENCH_TASK_STACK, NULL, 1, NULL, 0);
    if (created != pdPASS) {
        ESP_LOGE(TAG, "Cannot create TDS bench task");
        configASSERT(0);
    }
}
