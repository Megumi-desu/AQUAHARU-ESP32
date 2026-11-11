/* BENCH ONLY: DS18B20 temperature reading displayed on the 20x4 LCD.
 * No Wi-Fi, Firebase, other sensors, control tasks, or actuator driver.
 * Relay pins are not initialized or changed; disconnect the relay harness. */
#include "bench_temp_sensor.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lcd_display.h"
#include "temp_sensor.h"

static const char *TAG = "bench_temp";

#define TEMP_BENCH_INTERVAL_MS 2000
#define TEMP_BENCH_TASK_STACK  4096

static void write_line(uint8_t row, const char *text)
{
    char line[LCD_COLS + 1];
    snprintf(line, sizeof(line), "%-*.*s", LCD_COLS, LCD_COLS, text);
    lcd_write_at(row, 0, line);
}

static void temp_bench_task(void *arg)
{
    const bool lcd_ready = lcd_init() == ESP_OK;
    if (!lcd_ready) {
        ESP_LOGE(TAG, "LCD init failed");
    } else {
        lcd_clear();
        write_line(0, "AQUAHARU TEMP TEST");
        write_line(1, "DS18B20 GPIO32");
        write_line(2, "Checking sensor...");
        write_line(3, "Bench: temp only");
    }

    ESP_LOGI(TAG, "DS18B20 bench: GPIO32, LCD 20x4; no other subsystems started");
    const bool sensor_ready = temp_sensor_init();
    if (!sensor_ready) {
        ESP_LOGE(TAG, "No DS18B20 presence pulse; check GPIO32, GND and external 4.7k pull-up");
        if (lcd_ready) {
            write_line(2, "DS18B20 not found");
            write_line(3, "GPIO32 / 4.7k? Reset");
        }
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_LOGI(TAG, "DS18B20 detected; sampling at 12-bit resolution");
    TickType_t wake = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(TEMP_BENCH_INTERVAL_MS);
    unsigned sample = 0;

    while (true) {
        const float temperature_c = temp_sensor_sample();
        if (temperature_c == TEMP_SENSOR_NO_DEVICE) {
            ESP_LOGE(TAG, "Sample %u failed; check the sensor bus", sample);
            if (lcd_ready) {
                write_line(2, "Read failed");
                write_line(3, "Check GPIO32 / wire");
            }
        } else {
            ESP_LOGI(TAG, "Sample %u: %.2f C", sample, (double)temperature_c);
            if (lcd_ready) {
                char value[LCD_COLS + 1];
                snprintf(value, sizeof(value), "Temperature: %.2f C", (double)temperature_c);
                write_line(2, value);
                write_line(3, "DS18B20 reading OK");
            }
        }
        ++sample;
        vTaskDelayUntil(&wake, interval);
    }
}

void bench_temp_sensor_start(void)
{
    /* The relay board's polarity is not verified. Leave its GPIOs untouched. */
    const BaseType_t created = xTaskCreatePinnedToCore(
        temp_bench_task, "benchTemp", TEMP_BENCH_TASK_STACK, NULL, 1, NULL, 0);
    if (created != pdPASS) {
        ESP_LOGE(TAG, "Cannot create DS18B20 bench task");
        configASSERT(0);
    }
}
