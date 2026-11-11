/* BENCH ONLY: drive GPIO16 and GPIO4 relay inputs together for 3 minutes.
 * Observed relay polarity is active-LOW: HIGH is OFF, LOW is ON. This bench
 * bypasses normal actuator interlocks; verify loads before powering them. */
#include "bench_relay.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lcd_display.h"

static const char *TAG = "bench_relay";

#define RELAY_OFF_LEVEL 1
#define RELAY_ON_LEVEL  0
#define TEST_DURATION_MS (3 * 60 * 1000)

/* Keep all connected relay inputs OFF except GPIO16/GPIO4 during the test.
 * The user reported GPIO2 is also wired to an IN pin; hold it OFF only. */
static const gpio_num_t s_relay_pins[] = {
    GPIO_NUM_19, GPIO_NUM_18, GPIO_NUM_5, GPIO_NUM_17,
    GPIO_NUM_16, GPIO_NUM_4, GPIO_NUM_2,
};

static bool s_lcd_ok;

static void lcd_line(uint8_t row, const char *message)
{
    if (!s_lcd_ok) return;
    char padded[LCD_COLS + 1];
    snprintf(padded, sizeof(padded), "%-*.*s", LCD_COLS, LCD_COLS, message);
    lcd_write_at(row, 0, padded);
}

static void all_off(void)
{
    for (size_t i = 0; i < sizeof(s_relay_pins) / sizeof(s_relay_pins[0]); ++i) {
        ESP_ERROR_CHECK(gpio_set_level(s_relay_pins[i], RELAY_OFF_LEVEL));
    }
}

void bench_relay_start(void)
{
    /* Preload HIGH before enabling outputs. GPIO2, GPIO4 and GPIO5 are
     * strapping pins; firmware cannot control their level during reset. */
    uint64_t pin_mask = 0;
    for (size_t i = 0; i < sizeof(s_relay_pins) / sizeof(s_relay_pins[0]); ++i) {
        ESP_ERROR_CHECK(gpio_set_level(s_relay_pins[i], RELAY_OFF_LEVEL));
        pin_mask |= 1ULL << s_relay_pins[i];
    }
    const gpio_config_t output_config = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&output_config));
    all_off();

    s_lcd_ok = lcd_init() == ESP_OK;
    lcd_line(0, "AQUAHARU RELAY BENCH");
    lcd_line(1, "GPIO16 + GPIO4");
    lcd_line(2, "ON TOGETHER: 3 MIN");
    lcd_line(3, "ALL OFF - STARTING");
    ESP_LOGW(TAG, "BENCH ONLY: GPIO16 and GPIO4 will turn ON together for 3 minutes; normal safety interlocks are inactive");

    ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_16, RELAY_ON_LEVEL));
    ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_4, RELAY_ON_LEVEL));
    lcd_line(3, "GPIO16+4 ON: 3 MIN");
    ESP_LOGI(TAG, "GPIO16 and GPIO4 ON (LOW); other connected relay inputs HIGH/OFF");

    vTaskDelay(pdMS_TO_TICKS(TEST_DURATION_MS));

    all_off();
    lcd_line(3, "ALL RELAYS OFF");
    ESP_LOGI(TAG, "Three-minute relay bench finished; all seven connected inputs HIGH/OFF");
}
