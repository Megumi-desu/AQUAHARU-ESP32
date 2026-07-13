/*
 * lcd_display.h
 *
 * I2C 20×4 LCD Driver for AQUAHARU
 *
 * Hardware: HD44780 controller + PCF8574 I2C expander
 *           SDA = GPIO 21, SCL = GPIO 22, 400 kHz
 *           I²C address configurable (default 0x3C)
 *
 * 4-bit interface with backlight control.
 */

#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

/** PCF8574 I2C address (0x3C or 0x27 are common). */
#ifndef LCD_I2C_ADDR
#define LCD_I2C_ADDR  0x3C
#endif

/** Number of character columns. */
#define LCD_COLS  20

/** Number of character rows. */
#define LCD_ROWS  4

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialise the I2C bus, PCF8574 expander, and HD44780 controller.
 *
 * Performs the standard 4-bit initialisation sequence:
 *   1. Wait >40 ms post-power
 *   2. Send 0x03 × 3 (8-bit wake-up), then 0x02 (4-bit switch)
 *   3. Function set: 4-bit, 2-line, 5×8 font
 *   4. Display OFF, Clear, Entry mode, Display ON (no cursor)
 *
 * @return 0 (ESP_OK) on success, or an ESP_ERR_* code.
 */
int lcd_init(void);

/**
 * @brief Clear the display and home the cursor.
 */
void lcd_clear(void);

/**
 * @brief Set the cursor position (0-indexed).
 *
 * @param row  0–3.
 * @param col  0–19.
 */
void lcd_set_cursor(uint8_t row, uint8_t col);

/**
 * @brief Write a null-terminated string at the current cursor position.
 *
 * Characters beyond column 19 wrap to the next line automatically.
 *
 * @param str  Null-terminated ASCII string.
 */
void lcd_write_str(const char *str);

/**
 * @brief Position the cursor and write a string (convenience).
 *
 * Equivalent to lcd_set_cursor() followed by lcd_write_str().
 *
 * @param row  0–3.
 * @param col  0–19.
 * @param str  Null-terminated ASCII string.
 */
void lcd_write_at(uint8_t row, uint8_t col, const char *str);

/**
 * @brief Turn the LCD backlight on or off.
 *
 * @param on  true = backlight ON, false = OFF.
 */
void lcd_set_backlight(bool on);

#ifdef __cplusplus
}
#endif

#endif /* LCD_DISPLAY_H */
