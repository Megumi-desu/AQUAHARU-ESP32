/*
 * lcd_display.c
 *
 * I2C 20×4 LCD Driver (HD44780 + PCF8574) — Implementation
 *
 * Uses the ESP-IDF v6.0 I2C Master Driver (driver/i2c_master.h).
 * 4-bit interface with backlight control via PCF8574 bit 3.
 *
 * References:
 *   - HD44780U datasheet (Hitachi)
 *   - PCF8574 datasheet (TI / NXP)
 *   - UncleRus esp-idf-lib hd44780 driver
 */

#include "lcd_display.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* -------------------------------------------------------------------------- */
/* PCF8574 Pin Mapping (bits in the I/O expander byte)                        */
/* -------------------------------------------------------------------------- */

#define PCF_RS     0x01    /* Register Select: 0 = command, 1 = data */
#define PCF_RW     0x02    /* R/W: 0 = write (tied to GND in most backpacks) */
#define PCF_E      0x04    /* Enable strobe */
#define PCF_BL     0x08    /* Backlight */
/* Data bits D4–D7 map to PCF bits P4–P7 (upper nibble) */

/* -------------------------------------------------------------------------- */
/* HD44780 Command Set                                                        */
/* -------------------------------------------------------------------------- */

#define LCD_CMD_CLEAR       0x01
#define LCD_CMD_HOME        0x02
#define LCD_CMD_ENTRY_MODE  0x06    /* increment cursor, no display shift */
#define LCD_CMD_DISP_OFF    0x08
#define LCD_CMD_DISP_ON     0x0C    /* display ON, cursor OFF, blink OFF */
#define LCD_CMD_FUNCTION    0x28    /* 4-bit, 2-line, 5×8 dots */
#define LCD_CMD_SET_DDRAM   0x80    /* + address bits */

/* -------------------------------------------------------------------------- */
/* DDRAM Row Start Addresses (20×4)                                           */
/* -------------------------------------------------------------------------- */

static const uint8_t s_row_addr[4] = { 0x00, 0x40, 0x14, 0x54 };

/* -------------------------------------------------------------------------- */
/* Timing Constants (µs)                                                      */
/* -------------------------------------------------------------------------- */

#define PULSE_US       1
#define CMD_DELAY_US   50
#define CLEAR_DELAY_MS 3
#define INIT_DELAY_MS  50

/* -------------------------------------------------------------------------- */
/* Static State                                                               */
/* -------------------------------------------------------------------------- */

static const char *TAG = "lcd";

static i2c_master_bus_handle_t s_bus   = NULL;
static i2c_master_dev_handle_t s_dev   = NULL;
static bool                     s_init = false;
static bool                     s_bl   = true;   /* backlight on by default */

/* -------------------------------------------------------------------------- */
/* I2C Low-Level Helpers                                                       */
/* -------------------------------------------------------------------------- */

/**
 * @brief Write a raw byte to the PCF8574 expander.
 *
 * The byte encodes: D7–D4 (upper nibble), BL, E, RW, RS (lower nibble).
 */
static esp_err_t pcf_write(uint8_t byte)
{
    /* FreeRTOS tick is 10 ms (CONFIG_FREERTOS_HZ=100): a 10 ms timeout is only
     * 1 tick and can expire mid-transfer, so allow several ticks. */
    return i2c_master_transmit(s_dev, &byte, 1, 100);
}

/**
 * @brief Pulse the Enable pin: E=1 → delay → E=0.
 *
 * The data and RS/RW/BL signals are already set in `byte`; this function
 * toggles only the E bit.
 */
static esp_err_t pcf_pulse(uint8_t byte)
{
    esp_err_t ret;

    ret = pcf_write(byte | PCF_E);
    if (ret != ESP_OK) return ret;
    esp_rom_delay_us(PULSE_US);

    ret = pcf_write(byte & ~PCF_E);
    if (ret != ESP_OK) return ret;
    esp_rom_delay_us(CMD_DELAY_US);

    return ret;
}

/**
 * @brief Send a 4-bit nibble to the LCD (upper 4 bits of `data`).
 */
static esp_err_t lcd_send_nibble(uint8_t data, bool rs)
{
    uint8_t high = (data & 0xF0)                    /* D7–D4 */
                 | (rs     ? PCF_RS : 0)
                 | (s_bl   ? PCF_BL : 0);
    /* RW is always 0 (write-only, grounded on most backpacks) */

    return pcf_pulse(high);
}

/**
 * @brief Send a full 8-bit byte as two 4-bit nibbles (high first).
 */
static esp_err_t lcd_send_byte(uint8_t data, bool rs)
{
    esp_err_t ret;

    /* High nibble */
    ret = lcd_send_nibble(data, rs);
    if (ret != ESP_OK) return ret;

    /* Low nibble */
    ret = lcd_send_nibble(data << 4, rs);
    return ret;
}

/**
 * @brief Send a command byte (RS = 0).
 */
static esp_err_t lcd_command(uint8_t cmd)
{
    return lcd_send_byte(cmd, false);
}

/* -------------------------------------------------------------------------- */
/* Initialisation                                                             */
/* -------------------------------------------------------------------------- */

int lcd_init(void)
{
    if (s_init) return 0;

    /* ---- I2C bus configuration ---- */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port                 = I2C_NUM_0,
        .sda_io_num               = GPIO_NUM_21,
        .scl_io_num               = GPIO_NUM_22,
        .clk_source               = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt        = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* ---- PCF8574 device ---- */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = LCD_I2C_ADDR,
        .scl_speed_hz    = 100000,   /* PCF8574 is rated 100 kHz; 400 kHz corrupted data on bench */
    };

    ret = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C device add failed (addr 0x%02X): %s",
                 LCD_I2C_ADDR, esp_err_to_name(ret));
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
        return ret;
    }

    ESP_LOGI(TAG, "I2C initialised — SDA=21, SCL=22, addr=0x%02X, 100 kHz",
             LCD_I2C_ADDR);

    /* ---- HD44780 4-bit initialisation sequence ---- */

    /* Data sheet: wait >40 ms after Vcc rises to 2.7 V */
    vTaskDelay(pdMS_TO_TICKS(INIT_DELAY_MS));

    /* Three wake-up pulses in 8-bit mode.  Sub-tick waits use busy delays:
     * pdMS_TO_TICKS(5) is 0 ticks at 100 Hz. */
    lcd_send_nibble(0x30, false);
    esp_rom_delay_us(5000);

    lcd_send_nibble(0x30, false);
    esp_rom_delay_us(150);

    lcd_send_nibble(0x30, false);
    esp_rom_delay_us(150);

    /* Switch to 4-bit mode */
    lcd_send_nibble(0x20, false);
    esp_rom_delay_us(150);

    /* Now in 4-bit mode — send actual configuration commands */

    lcd_command(LCD_CMD_FUNCTION);          /* 4-bit, 2-line, 5×8 */
    lcd_command(LCD_CMD_DISP_OFF);          /* display OFF */
    lcd_command(LCD_CMD_CLEAR);             /* clear */
    esp_rom_delay_us(CLEAR_DELAY_MS * 1000);

    lcd_command(LCD_CMD_ENTRY_MODE);        /* increment, no shift */
    lcd_command(LCD_CMD_DISP_ON);           /* display ON, cursor OFF */

    s_init = true;
    ESP_LOGI(TAG, "LCD 20×4 initialised");
    return 0;
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

void lcd_clear(void)
{
    if (!s_init) return;

    lcd_command(LCD_CMD_CLEAR);
    esp_rom_delay_us(CLEAR_DELAY_MS * 1000);
}

/* -------------------------------------------------------------------------- */

void lcd_set_cursor(uint8_t row, uint8_t col)
{
    if (!s_init) return;
    if (row >= LCD_ROWS) row = LCD_ROWS - 1;
    if (col >= LCD_COLS) col = LCD_COLS - 1;

    lcd_command(LCD_CMD_SET_DDRAM | (s_row_addr[row] + col));
}

/* -------------------------------------------------------------------------- */

void lcd_write_str(const char *str)
{
    if (!s_init || !str) return;

    while (*str) {
        lcd_send_byte((uint8_t)*str, true);   /* RS = 1 = data */
        str++;
    }
}

/* -------------------------------------------------------------------------- */

void lcd_write_at(uint8_t row, uint8_t col, const char *str)
{
    lcd_set_cursor(row, col);
    lcd_write_str(str);
}

/* -------------------------------------------------------------------------- */

void lcd_set_backlight(bool on)
{
    s_bl = on;

    /* The backlight bit is embedded in every PCF write.  The simplest way
     * to apply a backlight change without sending a new command is to write
     * the current state with the new BL bit.  We write 0x00 with the new
     * BL + E low → this doesn't trigger any LCD operation but updates BL. */
    uint8_t byte = (s_bl ? PCF_BL : 0);
    pcf_write(byte);
}
