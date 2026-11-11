/*
 * tds_sensor.h
 *
 * TDS / EC Sensor Driver for AQUAHARU (DFRobot Gravity TDS / generic EC probe)
 * Hardware: GPIO 39 / ADC1_CH3, 11dB attenuation, 12-bit resolution
 *
 * Features:
 *   - Optional voltage-divider correction (macro-selectable)
 *   - Temperature compensation of measured voltage (DFRobot model)
 *   - Cubic polynomial TDS calculation
 *   - Calibration factor for sensor-to-sensor variance
 *   - EC derivation (TDS / 500)
 */

#ifndef TDS_SENSOR_H
#define TDS_SENSOR_H

#include <stdint.h>

#include "esp_adc/adc_oneshot.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Uncomment the following define if using an external resistor divider
 * (R1 = 10 kΩ, R2 = 20 kΩ) between a 0–3.2 V transmitter and the ADC pin.
 * Leave commented for direct connection (e.g. DFRobot Gravity TDS).
 */
/* #define TDS_USE_VOLTAGE_DIVIDER */

/* -------------------------------------------------------------------------- */
/* Calibration Parameters (extern — defined in tds_sensor.c) */
/* -------------------------------------------------------------------------- */

/** Multiplicative calibration factor applied to the raw TDS value. */
extern float g_tds_calibration_factor;

/** Conversion factor from TDS (ppm) to EC (mS/cm). Default 500. */
extern float g_tds_to_ec_factor;

/**
 * @brief Initialize the ADC1 unit and configure Channel 3 for TDS reading.
 *
 * Sets up ADC1 with 12-bit width, 11dB attenuation (0–3.1 V range).
 * Must be called once before any tds_sensor_read_*() function.
 *
 * Since ADC1 is shared with the pH sensor, the caller should:
 *   1. Call ph_sensor_init() first
 *   2. Call ph_sensor_get_adc_handle() to obtain the shared ADC1 handle
 *   3. Pass that handle to tds_sensor_init()
 *
 * If @p existing_handle is NULL, this function will attempt to create its
 * own ADC1 handle (use this only when tds_sensor is the sole ADC1 user).
 *
 * @param existing_handle  ADC1 handle from ph_sensor_get_adc_handle(), or NULL.
 * @return ESP_OK on success, or an ESP_ERR_* code on failure.
 */
int tds_sensor_init(adc_oneshot_unit_handle_t existing_handle);

/**
 * @brief Read the raw (uncompensated) voltage from the TDS sensor.
 *
 * Reads the ADC channel, converts to voltage, and optionally corrects
 * for an external resistor divider (if TDS_USE_VOLTAGE_DIVIDER is defined).
 *
 * @return Sensor output voltage in volts (0–3.3 V range after correction).
 */
float tds_sensor_read_voltage(void);

/**
 * @brief Read 30-sample median-filtered, temperature-compensated TDS in ppm.
 *
 * Flow:
 *   1. Collect 30 ADC readings 40 ms apart and use their median voltage
 *   2. If TDS_USE_VOLTAGE_DIVIDER: V_sensor = V_adc * 1.5
 *   3. Temperature-compensate voltage (DFRobot model):
 *        V_comp = V_sensor / (1.0 + 0.02 * (T - 25.0))
 *   4. Cubic polynomial:
 *        TDS_raw = (133.42*V^3 - 255.86*V^2 + 857.39*V) * 0.5
 *   5. Apply calibration factor:
 *        TDS_cal = TDS_raw * g_tds_calibration_factor
 *
 * @param temperature_c  Water temperature in degrees Celsius.
 * @return Calibrated TDS in ppm.
 */
float tds_sensor_read_tds(float temperature_c);

/** Same filtered reading, optionally returning the voltage used for its ppm.
 * The output voltage is -1.0f on error. One call takes about 1.16 seconds. */
float tds_sensor_read_tds_filtered(float temperature_c, float *filtered_voltage_out);

/** Convert an already-filtered TDS value to EC without taking new ADC samples. */
float tds_sensor_ec_from_tds(float tds_ppm);

/**
 * @brief Read temperature-compensated EC (Electrical Conductivity).
 *
 * Derived from the TDS reading:
 *   EC (mS/cm) = TDS_calibrated / g_tds_to_ec_factor
 *
 * @param temperature_c  Water temperature in degrees Celsius.
 * @return EC in mS/cm.
 */
float tds_sensor_read_ec(float temperature_c);

#ifdef __cplusplus
}
#endif

#endif /* TDS_SENSOR_H */
