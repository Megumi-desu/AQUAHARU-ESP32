/*
 * actuators.h
 *
 * Relay Actuator Driver for AQUAHARU
 * Controls 6 relays + float switch safety monitoring.
 *
 * Hardware:
 *   GPIO 19 — Pump pH Up       (Active High)
 *   GPIO 18 — Pump pH Down     (Active High)
 *   GPIO  5 — Pump Nutrient A  (Active High)
 *   GPIO 17 — Pump Nutrient B  (Active High)
 *   GPIO 16 — Peltier + Fan    (Active High)
 *   GPIO  4 — Circulation Pump (Active High)
 *   GPIO 35 — Float Switch     (Active Low: 0 = water empty)
 */

#ifndef ACTUATORS_H
#define ACTUATORS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Hard-Limit Constants (ms)                                                   */
/* -------------------------------------------------------------------------- */

/** Maximum allowed runtime for pH pumps per cycle. */
#define ACTUATORS_PH_MAX_MS       5000

/** Maximum allowed runtime for nutrient pumps per cycle. */
#define ACTUATORS_NUTRIENT_MAX_MS 20000

/** Minimum lockout between dosing cycles (ms). */
#define ACTUATORS_MIXING_LOCKOUT_MS  (5 * 60 * 1000)

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialise all actuator GPIOs and the float switch input.
 *
 * Configures GPIOs 4,5,16,17,18,19 as push-pull outputs (default LOW).
 * Configures GPIO 35 as input (no internal pull-up; external pull-up required).
 * All relays start OFF.
 *
 * @return 0 (ESP_OK) on success, or an ESP_ERR_* code.
 */
int actuators_init(void);

/**
 * @brief Execute a complete staggered dosing cycle.
 *
 * Sequence (non-blocking at the RTOS level — uses vTaskDelay within the
 * calling task so other tasks are not starved):
 *   1. Pump Nutrient A ON for @p nut_a_ms  → OFF
 *   2. Mixing delay 10 s
 *   3. Pump Nutrient B ON for @p nut_b_ms  → OFF
 *   4. Mixing delay 10 s
 *   5. Pump pH (Up or Down) ON for @p ph_ms → OFF
 *
 * The float switch is checked before every step; if water goes low the
 * cycle is aborted immediately.
 *
 * If the 5-minute mixing lockout is active, the call returns false
 * without running any pump.
 *
 * @param nut_a_ms   Duration for Nutrient A pump (0–20000 ms).
 * @param nut_b_ms   Duration for Nutrient B pump (0–20000 ms).
 * @param ph_ms      Duration for pH pump (0–5000 ms).
 * @param ph_is_up   true = pH Up pump, false = pH Down pump.
 * @return true if the cycle completed, false if aborted or locked out.
 */
bool actuators_execute_cycle(uint32_t nut_a_ms, uint32_t nut_b_ms,
                             uint32_t ph_ms, bool ph_is_up);

/**
 * @brief Peltier cooling hysteresis controller.
 *
 * Call periodically (e.g. every 5 s) with the current water temperature.
 *
 *   T > 26.0 °C  →  Peltier/Fan ON  + Circulation Pump ON
 *   T ≤ 24.0 °C  →  Peltier/Fan OFF + Circulation Pump runs 60 s more, then OFF
 *
 * If the float switch indicates low water, cooling is force-disabled.
 *
 * @param temperature_c  Water temperature in degrees Celsius.
 */
void actuators_control_cooling(float temperature_c);

/**
 * @brief Emergency-stop: instantly turn OFF every relay.
 *
 * Called automatically when the float switch detects low water, but may
 * also be invoked manually.
 */
void actuators_emergency_stop(void);

/**
 * @brief Check whether a dosing cycle is currently allowed.
 *
 * @return false if mixing lockout is active or float switch is low.
 */
bool actuators_can_dose(void);

/**
 * @brief Read the float switch state.
 *
 * @return true if water level is OK (GPIO 35 == 1), false if low.
 */
bool actuators_water_ok(void);

/**
 * @brief Query whether the Peltier cooler is currently ON.
 *
 * @return true if Peltier relay is energised.
 */
bool actuators_is_peltier_on(void);

/**
 * @brief Query whether any dosing pump is currently running.
 *
 * @return true during an execute_cycle() call, false otherwise.
 */
bool actuators_is_pumping(void);

#ifdef __cplusplus
}
#endif

#endif /* ACTUATORS_H */
