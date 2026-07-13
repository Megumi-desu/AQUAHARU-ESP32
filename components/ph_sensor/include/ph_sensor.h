/*
 * ph_sensor.h
 *
 * pH Sensor Driver for AQUAHARU (PH-4502C analog probe)
 * Hardware: GPIO 36 / ADC1_CH0, 11dB attenuation, 12-bit resolution
 *
 * Features:
 *   - Trimmed moving-average filter (30 samples, discard 5 extremes each side)
 *   - Linear calibration with configurable slope/offset
 *   - Temperature compensation via Nernst-derived formula
 */

#ifndef PH_SENSOR_H
#define PH_SENSOR_H

#include <stdint.h>

#include "esp_adc/adc_oneshot.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Calibration Parameters (extern — defined in ph_sensor.c, settable at runtime) */
/* -------------------------------------------------------------------------- */

/** pH calibration slope (pH units per volt). Negative for Nernst response. */
extern float g_ph_slope;

/** pH calibration offset (pH at 0 V). */
extern float g_ph_offset;

/** Temperature coefficient for compensation (pH units per °C deviation). */
extern float g_ph_temp_coeff;

/**
 * @brief Initialize the ADC1 unit and configure Channel 0 for pH reading.
 *
 * Sets up ADC1 with 12-bit width, 11dB attenuation (0–3.1 V range).
 * Must be called once before ph_sensor_read().
 *
 * @return ESP_OK on success, or an ESP_ERR_* code on failure.
 */
int ph_sensor_init(void);

/**
 * @brief Read raw (uncompensated) pH from the sensor.
 *
 * Performs a trimmed moving-average filter:
 *   1. Collect 30 ADC samples at ~30 ms intervals
 *   2. Sort ascending, discard the 5 lowest and 5 highest
 *   3. Average the remaining 20 samples
 *   4. Convert to voltage: V = adc_avg * (3.3 / 4095)
 *   5. Apply linear calibration: pH_raw = g_ph_slope * V + g_ph_offset
 *
 * @return Raw pH value (before temperature compensation).
 */
float ph_sensor_read_raw(void);

/**
 * @brief Read temperature-compensated pH.
 *
 * Applies the formula:
 *   pH_comp = pH_raw + (temperature_c - 25.0) * g_ph_temp_coeff
 *
 * @param temperature_c  Water temperature in degrees Celsius.
 * @return Temperature-compensated pH value.
 */
float ph_sensor_read_compensated(float temperature_c);

/**
 * @brief Convenience: read raw ADC, filtered, and convert to voltage.
 *
 * Bypasses the pH calibration step — useful for calibration routines.
 *
 * @return Voltage at the ADC pin (0–3.3 V).
 */
float ph_sensor_read_voltage(void);

/**
 * @brief Return the ADC1 unit handle owned by this component.
 *
 * Other components sharing ADC1 (e.g. tds_sensor) can use this handle
 * to configure their own channels without creating a duplicate unit.
 *
 * @return Valid ADC1 handle, or NULL if ph_sensor_init() has not been called.
 */
adc_oneshot_unit_handle_t ph_sensor_get_adc_handle(void);

#ifdef __cplusplus
}
#endif

#endif /* PH_SENSOR_H */
