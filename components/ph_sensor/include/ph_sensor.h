/*
 * ph_sensor.h
 *
 * pH Sensor Driver for AQUAHARU (PH-4502C analog probe)
 * Hardware: GPIO 36 / ADC1_CH0, 12 dB attenuation, 12-bit resolution.
 * The green module is powered from 5 V; its PO output was verified by the
 * owner to remain <= 3.3 V. Keep the GPIO36 signal inside the ESP32 ADC's
 * characterized range, by adjusting a verified PO offset trimmer or using an
 * external divider. Both boards share GND. Calibrate with final wiring.
 *
 * Features:
 *   - Trimmed moving-average filter (30 samples, discard 5 extremes each side)
 *   - Linear calibration with configurable slope/offset
 *   - ADC line-fitting calibration and two-buffer pH calibration
 */

#ifndef PH_SENSOR_H
#define PH_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

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

/** Legacy correction until calibrated; set to zero by two-point calibration. */
extern float g_ph_temp_coeff;

typedef struct {
    int raw_adc;               /**< Trimmed mean of ADC counts. */
    float pin_voltage_v;       /**< Voltage at GPIO36 after any external divider. */
} ph_sensor_sample_t;

/**
 * @brief Initialize the ADC1 unit and configure Channel 0 for pH reading.
 *
 * Sets up ADC1 and the ESP32 line-fitting voltage calibration.
 * Must be called once before ph_sensor_read().
 *
 * @return ESP_OK on success, or an ESP_ERR_* code on failure.
 */
int ph_sensor_init(void);

/** Take one filtered 30-sample batch; all readings come from that batch. */
int ph_sensor_read_sample(ph_sensor_sample_t *sample);

/** Set two-point calibration from filtered GPIO36 voltages at 25 C. RAM only.
 * Both measurements and later readings must use the same divider/wiring. */
bool ph_sensor_calibrate_two_point(float voltage_686_v, float voltage_401_v);

/** True only after two valid buffer readings have been supplied this boot. */
bool ph_sensor_is_calibrated(void);

/**
 * @brief Read raw (uncompensated) pH from the sensor.
 *
 * Performs a trimmed moving-average filter:
 *   1. Collect 30 ADC samples at ~30 ms intervals
 *   2. Sort ascending, discard the 5 lowest and 5 highest
 *   3. Average the remaining 20 samples
 *   4. Convert trimmed ADC mean to mV with ESP-IDF ADC calibration
 *   5. After two-point calibration, apply fitted pH
 *      slope/offset. Before then, retain the legacy normal-mode estimate.
 *
 * @return Raw pH value (before temperature compensation).
 */
float ph_sensor_read_raw(void);

/**
 * @brief Read temperature-compensated pH.
 *
 * The legacy normal-mode estimate uses its original fixed coefficient until
 * two-point calibration is entered. Calibration disables that coefficient;
 * a fixed pH/degree offset is not a Nernst slope correction.
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
 * @return Voltage measured at GPIO36 (after any external divider).
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
