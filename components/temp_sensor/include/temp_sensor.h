/*
 * temp_sensor.h
 *
 * DS18B20 1-Wire Temperature Sensor Driver for AQUAHARU
 * Hardware: GPIO 32, external 4.7kΩ pull-up required
 *
 * Provides:
 *   - Bit-bang 1-Wire protocol (reset, read/write bit/byte)
 *   - DS18B20-specific commands (start conversion, read temperature)
 */

#ifndef TEMP_SENSOR_H
#define TEMP_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Error value returned when no DS18B20 is detected on the bus.
 */
#define TEMP_SENSOR_NO_DEVICE  (-999.0f)

/**
 * @brief Initialize the 1-Wire bus on GPIO 32 and verify sensor presence.
 *
 * Configures the GPIO, performs a bus reset, and checks for the DS18B20
 * presence pulse. Must be called once before any other function.
 *
 * @return true if a device was detected, false otherwise.
 */
bool temp_sensor_init(void);

/**
 * @brief Issue a Convert T command (0x44) to the DS18B20.
 *
 * Uses Skip ROM (0xCC) to address a single device on the bus.
 * The caller must wait at least 750 ms before reading the result
 * for 12-bit resolution. The convenience sampler waits 800 ms for margin.
 *
 * @return true if the command was acknowledged, false on bus error.
 */
bool temp_sensor_start_conversion(void);

/**
 * @brief Read the most recent temperature conversion result.
 *
 * Issues Skip ROM (0xCC) + Read Scratchpad (0xBE), reads the 2-byte
 * temperature register, and converts to degrees Celsius.
 *
 * @return Temperature in °C, or TEMP_SENSOR_NO_DEVICE (-999.0f) on error.
 */
float temp_sensor_read_temp(void);

/**
 * @brief Convenience: trigger conversion, wait, and read temperature.
 *
 * Blocks for 800 ms during conversion.  Equivalent to calling
 * start_conversion() followed by a delay and read_temp().
 *
 * @return Temperature in °C, or TEMP_SENSOR_NO_DEVICE (-999.0f) on error.
 */
float temp_sensor_sample(void);

#ifdef __cplusplus
}
#endif

#endif /* TEMP_SENSOR_H */
