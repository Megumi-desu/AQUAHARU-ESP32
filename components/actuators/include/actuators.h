/*
 * actuators.h
 *
 * Relay Actuator Driver for AQUAHARU
 * Controls 6 relays and reads the float switch for dashboard status.
 *
 * Hardware:
 *   GPIO 19/18/5/17/16/4 — relay inputs (LOW = ON, HIGH = OFF)
 *   GPIO 35 — float switch (LOW = water OK, HIGH = water low)
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

/** Snapshot of relay output states for telemetry. */
typedef struct {
    bool ph_up;
    bool ph_down;
    bool nutrient_a;
    bool nutrient_b;
    bool peltier;
    bool circulation_pump;
} actuators_status_t;

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialise all actuator GPIOs and the float switch input.
 *
 * Configures GPIOs 4,5,16,17,18,19 as push-pull outputs (HIGH = OFF).
 * Holds unused GPIO2 relay input HIGH/OFF.
 * Configures GPIO 35 as input (external bias required).
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
 * The float switch is status-only and does not stop this cycle.
 * An automation stop cancels an active relay within about 100 ms.
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
 *   T > setpoint + 2 °C  →  Peltier/Fan ON + Circulation Pump ON
 *   T ≤ setpoint          →  Peltier/Fan OFF + Circulation Pump runs 60 s more
 *
 * The float switch is status-only; cooling follows the configured setpoint.
 *
 * @param temperature_c  Water temperature in degrees Celsius.
 * @param setpoint_c     Cooling-off temperature from Web App configuration.
 */
void actuators_control_cooling(float temperature_c, float setpoint_c);

/**
 * @brief Emergency-stop: instantly turn OFF every relay.
 *
 * Called when automation stops or a sensor error occurs, and may
 * also be invoked manually.
 */
void actuators_emergency_stop(void);

/**
 * @brief Check whether a dosing cycle is currently allowed.
 *
 * @return false if mixing lockout is active.
 */
bool actuators_can_dose(void);

/**
 * @brief Read the float switch state for telemetry/dashboard only.
 *
 * @return true if water level is OK (GPIO 35 == 0), false if low.
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

/** Read the current GPIO output state for each relay. */
void actuators_get_status(actuators_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* ACTUATORS_H */
