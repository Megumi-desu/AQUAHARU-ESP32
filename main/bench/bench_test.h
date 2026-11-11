/*
 * bench_test.h
 *
 * Bench test for the I2C 20x4 LCD, Wi-Fi scan and station link.
 */

#ifndef BENCH_TEST_H
#define BENCH_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief I2C scan + LCD pattern test + standalone Wi-Fi STA test.
 *
 * Holds the six connected active-HIGH relay inputs LOW (OFF), then uses only
 * the I2C bus, LCD and Wi-Fi radio. Scans nearby 2.4 GHz APs before trying
 * the configured network. No sensors or Firebase. Never returns.
 */
void bench_test_lcd_wifi(void);

#ifdef __cplusplus
}
#endif

#endif /* BENCH_TEST_H */
