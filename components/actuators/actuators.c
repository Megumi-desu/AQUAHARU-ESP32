/*
 * actuators.c
 *
 * Relay Actuator Driver — Implementation
 *
 * Manages 6 relays via GPIO, enforces staggered pump sequencing with
 * non-blocking RTOS delays, Peltier cooling hysteresis with a 60 s flush
 * timer, 5-minute mixing lockout, hard-limit over-dosing protection, and
 * float-switch dry-run protection.
 */

#include "actuators.h"

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* -------------------------------------------------------------------------- */
/* Constants                                                                  */
/* -------------------------------------------------------------------------- */

static const char *TAG = "actuators";

/* Relay GPIOs (active-high) */
#define GPIO_PH_UP          GPIO_NUM_19
#define GPIO_PH_DOWN        GPIO_NUM_18
#define GPIO_NUT_A          GPIO_NUM_5
#define GPIO_NUT_B          GPIO_NUM_17
#define GPIO_PELTIER        GPIO_NUM_16
#define GPIO_CIRC_PUMP      GPIO_NUM_4

/* Float switch */
#define GPIO_FLOAT_SW       GPIO_NUM_35

/* Stagger delays between pump stages */
#define STAGGER_DELAY_MS    10000   /* 10 s between nutrient A → B → pH */

/* Cooling hysteresis thresholds */
#define COOLING_ON_TEMP     26.0f
#define COOLING_OFF_TEMP    24.0f

/* Circulation pump flush time after Peltier off */
#define FLUSH_DURATION_US   (60ULL * 1000 * 1000)   /* 60 s in µs */

/* -------------------------------------------------------------------------- */
/* Static State                                                               */
/* -------------------------------------------------------------------------- */

static bool s_initialized = false;

/* Tracks whether we are inside a mixing-lockout window */
static int64_t s_last_cycle_end_us = 0;

/* Cooling state machine */
typedef enum {
    COOL_IDLE,
    COOL_ACTIVE,
    COOL_FLUSHING
} cooling_state_t;

static cooling_state_t   s_cooling_state = COOL_IDLE;
static esp_timer_handle_t s_flush_timer   = NULL;
static volatile bool      s_pumping       = false;   /* true during execute_cycle */

/* -------------------------------------------------------------------------- */
/* Forward Declarations                                                       */
/* -------------------------------------------------------------------------- */

static void flush_timer_cb(void *arg);
static bool read_float_switch(void);

/* -------------------------------------------------------------------------- */
/* Internal Helpers                                                           */
/* -------------------------------------------------------------------------- */

/** Turn a single relay ON (active-high). */
static inline void relay_on(gpio_num_t gpio)
{
    gpio_set_level(gpio, 1);
}

/** Turn a single relay OFF. */
static inline void relay_off(gpio_num_t gpio)
{
    gpio_set_level(gpio, 0);
}

/** Turn ALL relay outputs OFF. */
static void all_relays_off(void)
{
    relay_off(GPIO_PH_UP);
    relay_off(GPIO_PH_DOWN);
    relay_off(GPIO_NUT_A);
    relay_off(GPIO_NUT_B);
    relay_off(GPIO_PELTIER);
    relay_off(GPIO_CIRC_PUMP);
}

/** Read float switch: true = water OK (GPIO high), false = low (GPIO low). */
static bool read_float_switch(void)
{
    return (gpio_get_level(GPIO_FLOAT_SW) == 1);
}

/* -------------------------------------------------------------------------- */
/* Flush Timer Callback (ISR context — keep it minimal)                       */
/* -------------------------------------------------------------------------- */

static void flush_timer_cb(void *arg)
{
    /* If we are still in FLUSHING state when the timer fires, turn off the
     * circulation pump and go to IDLE. */
    if (s_cooling_state == COOL_FLUSHING) {
        relay_off(GPIO_CIRC_PUMP);
        s_cooling_state = COOL_IDLE;
        ESP_LOGI(TAG, "Flush complete — circulation pump OFF");
    }
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

int actuators_init(void)
{
    if (s_initialized) {
        return 0;
    }

    /* ---- Output relay GPIOs ---- */
    gpio_config_t out_conf = {
        .pin_bit_mask = (1ULL << GPIO_PH_UP)
                      | (1ULL << GPIO_PH_DOWN)
                      | (1ULL << GPIO_NUT_A)
                      | (1ULL << GPIO_NUT_B)
                      | (1ULL << GPIO_PELTIER)
                      | (1ULL << GPIO_CIRC_PUMP),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&out_conf);

    /* Ensure all relays start OFF */
    all_relays_off();

    /* ---- Float switch input (external pull-up) ---- */
    gpio_config_t in_conf = {
        .pin_bit_mask = (1ULL << GPIO_FLOAT_SW),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&in_conf);

    /* ---- Flush timer (one-shot, not yet started) ---- */
    const esp_timer_create_args_t timer_args = {
        .callback = &flush_timer_cb,
        .arg      = NULL,
        .name     = "flush_timer",
    };
    esp_err_t ret = esp_timer_create(&timer_args, &s_flush_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create flush timer: %s", esp_err_to_name(ret));
        return ret;
    }

    s_cooling_state   = COOL_IDLE;
    s_initialized     = true;

    ESP_LOGI(TAG, "Actuators initialised — all relays OFF");
    return 0;
}

/* -------------------------------------------------------------------------- */

bool actuators_water_ok(void)
{
    return read_float_switch();
}

/* -------------------------------------------------------------------------- */

bool actuators_is_peltier_on(void)
{
    return (s_cooling_state == COOL_ACTIVE);
}

bool actuators_is_pumping(void)
{
    return s_pumping;
}

/* -------------------------------------------------------------------------- */

bool actuators_can_dose(void)
{
    /* Check mixing lockout */
    int64_t now_us = esp_timer_get_time();
    int64_t elapsed_us = now_us - s_last_cycle_end_us;

    if (s_last_cycle_end_us > 0 && elapsed_us < (int64_t)ACTUATORS_MIXING_LOCKOUT_MS * 1000) {
        int remaining_s = (int)((ACTUATORS_MIXING_LOCKOUT_MS * 1000 - elapsed_us) / 1000000);
        ESP_LOGW(TAG, "Mixing lockout active — %d s remaining", remaining_s);
        return false;
    }

    /* Check water level */
    if (!read_float_switch()) {
        ESP_LOGE(TAG, "Water level LOW — dosing blocked");
        return false;
    }

    return true;
}

/* -------------------------------------------------------------------------- */

void actuators_emergency_stop(void)
{
    all_relays_off();

    /* Cancel any pending flush */
    if (s_flush_timer) {
        esp_timer_stop(s_flush_timer);
    }
    s_cooling_state = COOL_IDLE;

    ESP_LOGE(TAG, "EMERGENCY STOP — all relays OFF");
}

/* -------------------------------------------------------------------------- */

bool actuators_execute_cycle(uint32_t nut_a_ms, uint32_t nut_b_ms,
                             uint32_t ph_ms, bool ph_is_up)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Actuators not initialised");
        return false;
    }

    /* ---- Pre-checks ---- */
    if (!actuators_can_dose()) {
        return false;
    }

    /* Hard-limit enforcement */
    if (nut_a_ms > ACTUATORS_NUTRIENT_MAX_MS) {
        ESP_LOGW(TAG, "Nutrient A duration %lu ms exceeds limit %d ms — clamped",
                 (unsigned long)nut_a_ms, ACTUATORS_NUTRIENT_MAX_MS);
        nut_a_ms = ACTUATORS_NUTRIENT_MAX_MS;
    }
    if (nut_b_ms > ACTUATORS_NUTRIENT_MAX_MS) {
        ESP_LOGW(TAG, "Nutrient B duration %lu ms exceeds limit %d ms — clamped",
                 (unsigned long)nut_b_ms, ACTUATORS_NUTRIENT_MAX_MS);
        nut_b_ms = ACTUATORS_NUTRIENT_MAX_MS;
    }
    if (ph_ms > ACTUATORS_PH_MAX_MS) {
        ESP_LOGW(TAG, "pH pump duration %lu ms exceeds limit %d ms — clamped",
                 (unsigned long)ph_ms, ACTUATORS_PH_MAX_MS);
        ph_ms = ACTUATORS_PH_MAX_MS;
    }

    gpio_num_t ph_gpio = ph_is_up ? GPIO_PH_UP : GPIO_PH_DOWN;

    s_pumping = true;

    ESP_LOGI(TAG, "Starting dosing cycle: NutA=%lums, NutB=%lums, pH%s=%lums",
             (unsigned long)nut_a_ms, (unsigned long)nut_b_ms,
             ph_is_up ? "Up" : "Down", (unsigned long)ph_ms);

    /* ---- Stage 1: Nutrient A ---- */
    if (!read_float_switch()) goto water_low;

    if (nut_a_ms > 0) {
        ESP_LOGI(TAG, "Nutrient A ON (%lu ms)", (unsigned long)nut_a_ms);
        relay_on(GPIO_NUT_A);
        vTaskDelay(pdMS_TO_TICKS(nut_a_ms));
        relay_off(GPIO_NUT_A);
        ESP_LOGI(TAG, "Nutrient A OFF");
    }

    /* ---- Stagger delay ---- */
    vTaskDelay(pdMS_TO_TICKS(STAGGER_DELAY_MS));

    /* ---- Stage 2: Nutrient B ---- */
    if (!read_float_switch()) goto water_low;

    if (nut_b_ms > 0) {
        ESP_LOGI(TAG, "Nutrient B ON (%lu ms)", (unsigned long)nut_b_ms);
        relay_on(GPIO_NUT_B);
        vTaskDelay(pdMS_TO_TICKS(nut_b_ms));
        relay_off(GPIO_NUT_B);
        ESP_LOGI(TAG, "Nutrient B OFF");
    }

    /* ---- Stagger delay ---- */
    vTaskDelay(pdMS_TO_TICKS(STAGGER_DELAY_MS));

    /* ---- Stage 3: pH pump ---- */
    if (!read_float_switch()) goto water_low;

    if (ph_ms > 0) {
        ESP_LOGI(TAG, "pH %s ON (%lu ms)", ph_is_up ? "Up" : "Down", (unsigned long)ph_ms);
        relay_on(ph_gpio);
        vTaskDelay(pdMS_TO_TICKS(ph_ms));
        relay_off(ph_gpio);
        ESP_LOGI(TAG, "pH %s OFF", ph_is_up ? "Up" : "Down");
    }

    /* Record cycle completion for mixing lockout */
    s_last_cycle_end_us = esp_timer_get_time();
    s_pumping = false;
    ESP_LOGI(TAG, "Dosing cycle complete — %d min mixing lockout active",
             (int)(ACTUATORS_MIXING_LOCKOUT_MS / 60000));
    return true;

water_low:
    ESP_LOGE(TAG, "Float switch tripped during cycle — aborting");
    s_pumping = false;
    actuators_emergency_stop();
    return false;
}

/* -------------------------------------------------------------------------- */

void actuators_control_cooling(float temperature_c)
{
    if (!s_initialized) {
        return;
    }

    /* Dry-run protection overrides cooling */
    if (!read_float_switch()) {
        if (s_cooling_state != COOL_IDLE) {
            ESP_LOGW(TAG, "Water low — forcing cooling OFF");
            actuators_emergency_stop();
        }
        return;
    }

    switch (s_cooling_state) {

    case COOL_IDLE:
        if (temperature_c > COOLING_ON_TEMP) {
            ESP_LOGI(TAG, "T=%.1f °C > %.1f — Peltier + Circulation Pump ON",
                     (double)temperature_c, (double)COOLING_ON_TEMP);
            relay_on(GPIO_PELTIER);
            relay_on(GPIO_CIRC_PUMP);
            s_cooling_state = COOL_ACTIVE;
        }
        break;

    case COOL_ACTIVE:
        if (temperature_c <= COOLING_OFF_TEMP) {
            ESP_LOGI(TAG, "T=%.1f °C ≤ %.1f — Peltier OFF, flush 60 s",
                     (double)temperature_c, (double)COOLING_OFF_TEMP);
            relay_off(GPIO_PELTIER);
            /* Circulation pump stays ON for 60 s to flush residual cold */
            s_cooling_state = COOL_FLUSHING;

            /* Start / restart the one-shot flush timer */
            esp_timer_stop(s_flush_timer);          /* safe even if not running */
            esp_timer_start_once(s_flush_timer, FLUSH_DURATION_US);
        }
        break;

    case COOL_FLUSHING:
        if (temperature_c > COOLING_ON_TEMP) {
            /* Temperature rose again during flush — cancel flush, resume cooling */
            ESP_LOGI(TAG, "T=%.1f °C > %.1f during flush — resuming cooling",
                     (double)temperature_c, (double)COOLING_ON_TEMP);
            esp_timer_stop(s_flush_timer);
            relay_on(GPIO_PELTIER);
            s_cooling_state = COOL_ACTIVE;
        }
        /* If temperature stays ≤ 24: timer will fire and transition to IDLE */
        break;
    }
}
