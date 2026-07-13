/*
 * main.c
 *
 * AQUAHARU — Automated Water Quality Monitoring & Control for Hydroponics
 * ESP32 DevKit V1  |  ESP-IDF v6.0.2  |  FreeRTOS
 *
 * Core 1 (Control):  taskSensor → taskControl → taskActuator
 * Core 0 (Comms):    taskUpload, taskDisplay (+ firebase_client internals)
 *
 * See CLAUDE.md for the full system blueprint.
 */

/* =========================================================================
 * Optional: uncomment to run full system diagnostics on boot.
 * When defined, the diagnostic sequence runs BEFORE any FreeRTOS task is
 * spawned, blocking until all hardware checks pass.
 * ========================================================================= */
/* #define RUN_DIAGNOSTIC_TESTS */

/* -------------------------------------------------------------------------- */
/* Includes                                                                   */
/* -------------------------------------------------------------------------- */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cJSON.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "actuators.h"
#include "firebase_client.h"
#include "fuzzy_control.h"
#include "lcd_display.h"
#include "ph_sensor.h"
#include "tds_sensor.h"
#include "temp_sensor.h"

static const char *TAG = "aquaharu";

/* -------------------------------------------------------------------------- */
/* Global Shared Data                                                         */
/* -------------------------------------------------------------------------- */

struct SensorData {
    float pH;
    float ec;
    float tds;
    float temperature;
    float light;
    bool  waterLow;
};

struct SystemConfig {
    float sp_pH;
    float sp_ec;
    float sp_temp;
    bool  auto_enabled;
    int   auto_off_hour;
    int   auto_on_hour;
};

struct ActCmd {
    uint32_t nut_a_ms;
    uint32_t nut_b_ms;
    uint32_t ph_ms;
    bool     ph_is_up;
};

/* Global instances */
static struct SensorData   g_sensor = {0};
static struct SystemConfig g_config = {
    .sp_pH        = 6.5f,
    .sp_ec        = 1.5f,
    .sp_temp      = 25.0f,
    .auto_enabled = true,
};

/* Synchronisation primitives */
static SemaphoreHandle_t s_sensor_mutex = NULL;
static SemaphoreHandle_t s_config_mutex = NULL;
static QueueHandle_t     s_q_act        = NULL;

/* -------------------------------------------------------------------------- */
/* Forward Declarations (FreeRTOS tasks)                                      */
/* -------------------------------------------------------------------------- */

static void task_sensor(void *pv);
static void task_control(void *pv);
static void task_actuator(void *pv);
static void task_upload(void *pv);
static void task_display(void *pv);

/* -------------------------------------------------------------------------- */
/* LDR Light Sensor Helper (GPIO 34 / ADC1_CH6 — no dedicated component)      */
/* -------------------------------------------------------------------------- */

#define LDR_ADC_UNIT    ADC_UNIT_1
#define LDR_ADC_CHANNEL ADC_CHANNEL_6
#define LDR_ADC_ATTEN   ADC_ATTEN_DB_12
#define LDR_ADC_WIDTH   ADC_BITWIDTH_12

/**
 * @brief Read raw LDR voltage.  Must be called after ph_sensor_init() so
 *        that the ADC1 handle exists and channel 6 can be configured.
 */
static float ldr_read_voltage(void)
{
    adc_oneshot_unit_handle_t handle = ph_sensor_get_adc_handle();
    if (!handle) {
        ESP_LOGE(TAG, "LDR: ADC1 handle not available — call ph_sensor_init() first");
        return -1.0f;
    }

    /* Configure channel 6 on the existing ADC1 handle (idempotent) */
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = LDR_ADC_ATTEN,
        .bitwidth = LDR_ADC_WIDTH,
    };
    esp_err_t ret = adc_oneshot_config_channel(handle, LDR_ADC_CHANNEL, &chan_cfg);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "LDR: channel config failed: %s", esp_err_to_name(ret));
        return -1.0f;
    }

    int raw = 0;
    ret = adc_oneshot_read(handle, LDR_ADC_CHANNEL, &raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LDR: read failed: %s", esp_err_to_name(ret));
        return -1.0f;
    }

    return (float)raw * (3.3f / 4095.0f);
}

/* ========================================================================= */
/*  SECTION 1 — Individual Component Test Functions (Bench / Development)    */
/* ========================================================================= */
/*
 * Each function below runs an INFINITE LOOP with internal vTaskDelay().
 * To test a single component on a workbench, open app_main() and replace
 * the FreeRTOS task creation + diagnostic block with a call such as:
 *
 *     test_sensor_ph();
 *     return;   // or while(1) { vTaskDelay(1000); }
 *
 * This lets you validate one sensor or relay at a time without spawning
 * the full multi-tasking system.
 */

/* ---- Temperature Sensor (DS18B20, GPIO 32) ---- */

__attribute__((unused))
static void test_sensor_temp(void)
{
    ESP_LOGI(TAG, "=== TEST: DS18B20 Temperature Sensor ===");

    if (!temp_sensor_init()) {
        ESP_LOGE(TAG, "DS18B20 init FAILED");
        return;
    }

    while (1) {
        float t = temp_sensor_sample();
        if (t <= TEMP_SENSOR_NO_DEVICE) {
            ESP_LOGE(TAG, "Temperature: READ ERROR");
        } else {
            ESP_LOGI(TAG, "Temperature: %.2f °C", (double)t);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ---- pH Sensor (PH-4502C, GPIO 36 / ADC1_CH0) ---- */

__attribute__((unused))
static void test_sensor_ph(void)
{
    ESP_LOGI(TAG, "=== TEST: pH Sensor ===");

    if (ph_sensor_init() != 0) {
        ESP_LOGE(TAG, "pH sensor init FAILED");
        return;
    }

    while (1) {
        float voltage = ph_sensor_read_voltage();
        float ph_raw  = ph_sensor_read_raw();
        /* Use 25 °C as a fixed reference for bench testing */
        float ph_comp = ph_sensor_read_compensated(25.0f);

        ESP_LOGI(TAG, "pH — V:%.4f  raw:%.3f  comp@25°C:%.3f",
                 (double)voltage, (double)ph_raw, (double)ph_comp);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ---- TDS / EC Sensor (GPIO 39 / ADC1_CH3) ---- */

__attribute__((unused))
static void test_sensor_tds(void)
{
    ESP_LOGI(TAG, "=== TEST: TDS / EC Sensor ===");

    if (tds_sensor_init(ph_sensor_get_adc_handle()) != 0) {
        ESP_LOGE(TAG, "TDS sensor init FAILED");
        return;
    }

    while (1) {
        /* Fixed 25 °C for bench testing */
        float tds = tds_sensor_read_tds(25.0f);
        float ec  = tds_sensor_read_ec(25.0f);

        ESP_LOGI(TAG, "TDS: %.1f ppm  EC: %.3f mS/cm", (double)tds, (double)ec);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ---- Light Sensor (LDR, GPIO 34 / ADC1_CH6) ---- */

__attribute__((unused))
static void test_sensor_light(void)
{
    ESP_LOGI(TAG, "=== TEST: LDR Light Sensor ===");

    if (ph_sensor_init() != 0) {
        ESP_LOGE(TAG, "ADC1 init FAILED (required for LDR)");
        return;
    }

    while (1) {
        float v = ldr_read_voltage();
        if (v < 0.0f) {
            ESP_LOGE(TAG, "LDR: READ ERROR");
        } else {
            /* Higher voltage = more light (typical LDR divider circuit) */
            float pct = (v / 3.3f) * 100.0f;
            ESP_LOGI(TAG, "LDR — Voltage: %.4f V  Intensity: ~%.0f %%",
                     (double)v, (double)pct);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ---- Float Switch (GPIO 35) ---- */

__attribute__((unused))
static void test_float_switch(void)
{
    ESP_LOGI(TAG, "=== TEST: Float Switch (GPIO 35) ===");

    /* actuators_init() configures GPIO 35 as input */
    if (actuators_init() != 0) {
        ESP_LOGE(TAG, "Actuators init FAILED");
        return;
    }

    while (1) {
        bool ok = actuators_water_ok();
        ESP_LOGI(TAG, "Float Switch: %s (GPIO 35 = %s)",
                 ok ? "WATER OK" : "WATER LOW",
                 ok ? "HIGH" : "LOW");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ---- Relay Test Helpers ---- */

/** Configure a single GPIO as push-pull output and set it LOW. */
static void relay_test_init(gpio_num_t gpio)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << gpio),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(gpio, 0);
}

/** Toggle relay: ON 3 s → OFF 3 s, logging state changes. */
__attribute__((unused))
static void relay_test_loop(gpio_num_t gpio, const char *name)
{
    while (1) {
        ESP_LOGI(TAG, "%s — ON", name);
        gpio_set_level(gpio, 1);
        vTaskDelay(pdMS_TO_TICKS(3000));

        ESP_LOGI(TAG, "%s — OFF", name);
        gpio_set_level(gpio, 0);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

/* ---- Individual Relay Tests ---- */

__attribute__((unused))
static void test_relay_pump_ph_up(void)
{
    ESP_LOGI(TAG, "=== TEST: Relay 1 — pH Up Pump (GPIO 19) ===");
    relay_test_init(GPIO_NUM_19);
    relay_test_loop(GPIO_NUM_19, "pH Up");
}

__attribute__((unused))
static void test_relay_pump_ph_down(void)
{
    ESP_LOGI(TAG, "=== TEST: Relay 2 — pH Down Pump (GPIO 18) ===");
    relay_test_init(GPIO_NUM_18);
    relay_test_loop(GPIO_NUM_18, "pH Down");
}

__attribute__((unused))
static void test_relay_pump_nut_a(void)
{
    ESP_LOGI(TAG, "=== TEST: Relay 3 — Nutrient A Pump (GPIO 5) ===");
    relay_test_init(GPIO_NUM_5);
    relay_test_loop(GPIO_NUM_5, "Nutrient A");
}

__attribute__((unused))
static void test_relay_pump_nut_b(void)
{
    ESP_LOGI(TAG, "=== TEST: Relay 4 — Nutrient B Pump (GPIO 17) ===");
    relay_test_init(GPIO_NUM_17);
    relay_test_loop(GPIO_NUM_17, "Nutrient B");
}

__attribute__((unused))
static void test_relay_peltier(void)
{
    ESP_LOGI(TAG, "=== TEST: Relay 5 — Peltier + Fan (GPIO 16) ===");
    ESP_LOGI(TAG, "Relay 6 (Circulation Pump) will also be activated for safety");
    relay_test_init(GPIO_NUM_16);
    relay_test_init(GPIO_NUM_4);

    while (1) {
        ESP_LOGI(TAG, "Peltier + Circ Pump — ON");
        gpio_set_level(GPIO_NUM_16, 1);
        gpio_set_level(GPIO_NUM_4,  1);
        vTaskDelay(pdMS_TO_TICKS(3000));

        ESP_LOGI(TAG, "Peltier + Circ Pump — OFF");
        gpio_set_level(GPIO_NUM_16, 0);
        gpio_set_level(GPIO_NUM_4,  0);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

__attribute__((unused))
static void test_relay_circulation(void)
{
    ESP_LOGI(TAG, "=== TEST: Relay 6 — Circulation Pump (GPIO 4) ===");
    relay_test_init(GPIO_NUM_4);
    relay_test_loop(GPIO_NUM_4, "Circulation Pump");
}

/* ========================================================================= */
/*  SECTION 2 — Full System Diagnostic Test Mode                             */
/* ========================================================================= */

#ifdef RUN_DIAGNOSTIC_TESTS

#define DIAG_READINGS   5
#define DIAG_DELAY_MS   1000
#define RELAY_ON_MS     2000
#define RELAY_GAP_MS    1000

/**
 * @brief Run a sequential diagnostic of every hardware subsystem.
 *
 * Called from app_main() before any FreeRTOS task is created.  Logs all
 * results via ESP_LOGI / ESP_LOGW / ESP_LOGE.  After completion, writes
 * "Diagnostics OK" to the LCD and returns so task creation can proceed.
 */
static void run_full_diagnostics(void)
{
    int pass = 0, warn = 0, fail = 0;
    float temp;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  AQUAHARU — Full System Diagnostics");
    ESP_LOGI(TAG, "========================================");

    /* ---- 1. Temperature Sensor (DS18B20) ---- */
    ESP_LOGI(TAG, "[1/6] DS18B20 Temperature Sensor (5 readings)");
    if (!temp_sensor_init()) {
        ESP_LOGE(TAG, "  DS18B20 init FAILED — check wiring & pull-up");
        fail++;
    } else {
        for (int i = 0; i < DIAG_READINGS; i++) {
            temp = temp_sensor_sample();
            if (temp <= TEMP_SENSOR_NO_DEVICE) {
                ESP_LOGE(TAG, "  [%d/%d] READ ERROR", i + 1, DIAG_READINGS);
                fail++;
            } else if (temp < 10.0f || temp > 45.0f) {
                ESP_LOGW(TAG, "  [%d/%d] %.2f °C — out of expected range [10–45]",
                         i + 1, DIAG_READINGS, (double)temp);
                warn++;
            } else {
                ESP_LOGI(TAG, "  [%d/%d] %.2f °C — OK", i + 1, DIAG_READINGS,
                         (double)temp);
                pass++;
            }
            vTaskDelay(pdMS_TO_TICKS(DIAG_DELAY_MS));
        }
    }

    /* ---- 2. pH Sensor ---- */
    ESP_LOGI(TAG, "[2/6] pH Sensor (5 readings)");
    if (ph_sensor_init() != 0) {
        ESP_LOGE(TAG, "  pH sensor init FAILED");
        fail++;
    } else {
        float temp_ref = (temp > TEMP_SENSOR_NO_DEVICE) ? temp : 25.0f;
        for (int i = 0; i < DIAG_READINGS; i++) {
            float ph = ph_sensor_read_compensated(temp_ref);
            if (ph < 0.0f) {
                ESP_LOGE(TAG, "  [%d/%d] READ ERROR", i + 1, DIAG_READINGS);
                fail++;
            } else if (ph < 4.0f || ph > 10.0f) {
                ESP_LOGW(TAG, "  [%d/%d] pH %.3f — outside expected range [4.0–10.0]",
                         i + 1, DIAG_READINGS, (double)ph);
                warn++;
            } else {
                ESP_LOGI(TAG, "  [%d/%d] pH %.3f — OK", i + 1, DIAG_READINGS,
                         (double)ph);
                pass++;
            }
            vTaskDelay(pdMS_TO_TICKS(DIAG_DELAY_MS));
        }
    }

    /* ---- 3. TDS & EC Sensor ---- */
    ESP_LOGI(TAG, "[3/6] TDS / EC Sensor (5 readings)");
    if (tds_sensor_init(ph_sensor_get_adc_handle()) != 0) {
        ESP_LOGE(TAG, "  TDS sensor init FAILED");
        fail++;
    } else {
        float temp_ref = (temp > TEMP_SENSOR_NO_DEVICE) ? temp : 25.0f;
        for (int i = 0; i < DIAG_READINGS; i++) {
            float tds = tds_sensor_read_tds(temp_ref);
            float ec  = tds_sensor_read_ec(temp_ref);
            if (tds < 0.0f) {
                ESP_LOGE(TAG, "  [%d/%d] READ ERROR", i + 1, DIAG_READINGS);
                fail++;
            } else {
                ESP_LOGI(TAG, "  [%d/%d] TDS: %.1f ppm  EC: %.3f mS/cm — OK",
                         i + 1, DIAG_READINGS, (double)tds, (double)ec);
                pass++;
            }
            vTaskDelay(pdMS_TO_TICKS(DIAG_DELAY_MS));
        }
    }

    /* ---- 4. LCD Display ---- */
    ESP_LOGI(TAG, "[4/6] LCD 20×4 Display");
    if (lcd_init() != 0) {
        ESP_LOGE(TAG, "  LCD init FAILED");
        fail++;
    } else {
        lcd_clear();
        lcd_set_backlight(true);
        lcd_write_at(0, 0, "AQUAHARU DIAGNOSTIC");
        lcd_write_at(1, 0, "LCD 20x4 Test Line 2");
        lcd_write_at(2, 0, "Row 3: abcdefghijklmn");
        lcd_write_at(3, 0, "Row 4: 0123456789ABCDEF");
        ESP_LOGI(TAG, "  LCD test pattern displayed — wait 3 s");
        vTaskDelay(pdMS_TO_TICKS(3000));
        lcd_clear();
        lcd_write_at(0, 2, "LCD OK");
        pass++;
    }

    /* ---- 5. Actuators (Relay Sequential Test) ---- */
    ESP_LOGI(TAG, "[5/6] Relay Sequential Test (2 s ON each)");
    if (actuators_init() != 0) {
        ESP_LOGE(TAG, "  Actuators init FAILED");
        fail++;
    } else if (!actuators_water_ok()) {
        /* Float switch safety check before relay test */
        ESP_LOGW(TAG, "  Float switch indicates LOW WATER — relay test aborted");
        warn++;
    } else {
        /* Run each relay ON 2s → OFF, 1s gap between */
        static const struct {
            gpio_num_t gpio;
            const char *name;
        } relay_seq[] = {
            { GPIO_NUM_19, "Relay 1 — pH Up"         },
            { GPIO_NUM_18, "Relay 2 — pH Down"       },
            { GPIO_NUM_5,  "Relay 3 — Nutrient A"    },
            { GPIO_NUM_17, "Relay 4 — Nutrient B"    },
            { GPIO_NUM_16, "Relay 5 — Peltier+Fan"   },
            { GPIO_NUM_4,  "Relay 6 — Circulation"   },
        };

        for (size_t i = 0; i < sizeof(relay_seq) / sizeof(relay_seq[0]); i++) {
            ESP_LOGI(TAG, "  %s — ON", relay_seq[i].name);
            gpio_set_level(relay_seq[i].gpio, 1);

            /* Peltier test: also turn on circulation pump for safety */
            if (relay_seq[i].gpio == GPIO_NUM_16) {
                gpio_set_level(GPIO_NUM_4, 1);
            }

            vTaskDelay(pdMS_TO_TICKS(RELAY_ON_MS));

            gpio_set_level(relay_seq[i].gpio, 0);
            if (relay_seq[i].gpio == GPIO_NUM_16) {
                gpio_set_level(GPIO_NUM_4, 0);
            }
            ESP_LOGI(TAG, "  %s — OFF", relay_seq[i].name);

            vTaskDelay(pdMS_TO_TICKS(RELAY_GAP_MS));
            pass++;
        }
    }

    /* ---- 6. LDR Light Sensor ---- */
    ESP_LOGI(TAG, "[6/6] LDR Light Sensor (5 readings)");
    {
        for (int i = 0; i < DIAG_READINGS; i++) {
            float v = ldr_read_voltage();
            if (v < 0.0f) {
                ESP_LOGE(TAG, "  [%d/%d] READ ERROR", i + 1, DIAG_READINGS);
                fail++;
            } else {
                float pct = (v / 3.3f) * 100.0f;
                ESP_LOGI(TAG, "  [%d/%d] %.4f V (~%.0f %%) — OK",
                         i + 1, DIAG_READINGS, (double)v, (double)pct);
                pass++;
            }
            vTaskDelay(pdMS_TO_TICKS(DIAG_DELAY_MS));
        }
    }

    /* ---- Summary ---- */
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  Diagnostics Complete");
    ESP_LOGI(TAG, "  PASS: %d  WARN: %d  FAIL: %d", pass, warn, fail);
    ESP_LOGI(TAG, "========================================");

    /* Final LCD message */
    lcd_clear();
    lcd_write_at(0, 3, "Diagnostics OK");
    if (fail > 0) {
        char buf[21];
        snprintf(buf, sizeof(buf), "WARN:%d FAIL:%d", warn, fail);
        lcd_write_at(1, 2, buf);
    }
    lcd_write_at(3, 0, "Booting system...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    lcd_clear();
}

#endif /* RUN_DIAGNOSTIC_TESTS */

/* ========================================================================= */
/*  SECTION 3 — Firebase Config Callback                                     */
/* ========================================================================= */

static void on_firebase_config(const char *json_str)
{
    cJSON *root = cJSON_Parse(json_str);
    if (!root) return;

    if (xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        cJSON *item;

        item = cJSON_GetObjectItem(root, "sp_pH");
        if (item && cJSON_IsNumber(item)) g_config.sp_pH = (float)item->valuedouble;

        item = cJSON_GetObjectItem(root, "sp_ec");
        if (item && cJSON_IsNumber(item)) g_config.sp_ec = (float)item->valuedouble;

        item = cJSON_GetObjectItem(root, "sp_temp");
        if (item && cJSON_IsNumber(item)) g_config.sp_temp = (float)item->valuedouble;

        item = cJSON_GetObjectItem(root, "auto_enabled");
        if (item && cJSON_IsBool(item)) g_config.auto_enabled = cJSON_IsTrue(item);

        item = cJSON_GetObjectItem(root, "auto_off_hour");
        if (item && cJSON_IsNumber(item)) g_config.auto_off_hour = item->valueint;

        item = cJSON_GetObjectItem(root, "auto_on_hour");
        if (item && cJSON_IsNumber(item)) g_config.auto_on_hour = item->valueint;

        xSemaphoreGive(s_config_mutex);
        ESP_LOGI(TAG, "Config updated from Firebase SSE");
    }

    cJSON_Delete(root);
}

/* ========================================================================= */
/*  SECTION 4 — FreeRTOS Task Functions                                      */
/* ========================================================================= */

/* ---- Core 1: taskSensor (Priority 4, 5 s interval) ---- */

static void task_sensor(void *pv)
{
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        float temp = temp_sensor_sample();
        if (temp <= TEMP_SENSOR_NO_DEVICE) {
            ESP_LOGW(TAG, "Temperature sensor error — skipping cycle");
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5000));
            continue;
        }

        float ph    = ph_sensor_read_compensated(temp);
        float tds   = tds_sensor_read_tds(temp);
        float ec    = tds_sensor_read_ec(temp);
        float light = ldr_read_voltage();
        bool  low   = !actuators_water_ok();

        if (xSemaphoreTake(s_sensor_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            g_sensor.pH          = ph;
            g_sensor.tds         = tds;
            g_sensor.ec          = ec;
            g_sensor.temperature = temp;
            g_sensor.light       = (light >= 0.0f) ? light : 0.0f;
            g_sensor.waterLow    = low;
            xSemaphoreGive(s_sensor_mutex);
        }

        /* Peltier cooling — independent hysteresis (not fuzzy) */
        actuators_control_cooling(temp);

        ESP_LOGI(TAG, "Sensors — pH:%.3f TDS:%.1f EC:%.3f T:%.1f°C Light:%.2fV Water:%s",
                 (double)ph, (double)tds, (double)ec, (double)temp,
                 (double)light, low ? "LOW" : "OK");

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5000));
    }
}

/* ---- Core 1: taskControl (Priority 5, 5 s interval) ---- */

static void task_control(void *pv)
{
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        float ph = 0, tds = 0;
        if (xSemaphoreTake(s_sensor_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            ph  = g_sensor.pH;
            tds = g_sensor.tds;
            xSemaphoreGive(s_sensor_mutex);
        }

        float ph_up_ms = 0, ph_dn_ms = 0, nut_ms = 0;
        fuzzy_evaluate(ph, tds, &ph_up_ms, &ph_dn_ms, &nut_ms);

        ESP_LOGI(TAG, "Fuzzy — pH_Up:%.0f ms  pH_Dn:%.0f ms  Nut:%.0f ms",
                 (double)ph_up_ms, (double)ph_dn_ms, (double)nut_ms);

        struct ActCmd cmd = {
            .nut_a_ms = (uint32_t)nut_ms,
            .nut_b_ms = (uint32_t)nut_ms,
            .ph_ms    = (ph_up_ms > ph_dn_ms) ? (uint32_t)ph_up_ms : (uint32_t)ph_dn_ms,
            .ph_is_up = (ph_up_ms > ph_dn_ms),
        };

        if (xQueueSend(s_q_act, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGW(TAG, "Actuator queue full — command dropped");
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5000));
    }
}

/* ---- Core 1: taskActuator (Priority 4, Event-driven) ---- */

static void task_actuator(void *pv)
{
    struct ActCmd cmd;

    while (1) {
        if (xQueueReceive(s_q_act, &cmd, portMAX_DELAY) == pdTRUE) {

            bool auto_en = false;
            if (xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                auto_en = g_config.auto_enabled;
                xSemaphoreGive(s_config_mutex);
            }

            if (!auto_en) {
                ESP_LOGI(TAG, "Auto mode OFF — skipping actuation");
                continue;
            }

            actuators_execute_cycle(cmd.nut_a_ms, cmd.nut_b_ms,
                                    cmd.ph_ms, cmd.ph_is_up);
        }
    }
}

/* ---- Core 0: taskUpload (Priority 1, 60 s interval) ---- */

static void task_upload(void *pv)
{
    ESP_LOGI(TAG, "Waiting for Firebase client…");
    if (!firebase_client_wait_ready(60000)) {
        ESP_LOGW(TAG, "Firebase not ready after 60 s — starting upload attempts");
    }

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        float ph = 0, ec = 0, tds = 0, temp = 0, light = 0;
        bool  low = false;

        if (xSemaphoreTake(s_sensor_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            ph    = g_sensor.pH;
            ec    = g_sensor.ec;
            tds   = g_sensor.tds;
            temp  = g_sensor.temperature;
            light = g_sensor.light;
            low   = g_sensor.waterLow;
            xSemaphoreGive(s_sensor_mutex);
        }

        /* Build live telemetry JSON */
        cJSON *live = cJSON_CreateObject();
        if (live) {
            cJSON_AddNumberToObject(live, "pH",          (double)ph);
            cJSON_AddNumberToObject(live, "ec",          (double)ec);
            cJSON_AddNumberToObject(live, "tds",         (double)tds);
            cJSON_AddNumberToObject(live, "temperature", (double)temp);
            cJSON_AddNumberToObject(live, "light",       (double)light);
            cJSON_AddBoolToObject  (live, "waterLow",    low);

            char *live_str = cJSON_PrintUnformatted(live);
            if (live_str) {
                firebase_put_telemetry(live_str);
                cJSON_free(live_str);
            }
            cJSON_Delete(live);
        }

        /* Build log entry JSON */
        cJSON *log = cJSON_CreateObject();
        if (log) {
            cJSON_AddNumberToObject(log, "pH",          (double)ph);
            cJSON_AddNumberToObject(log, "ec",          (double)ec);
            cJSON_AddNumberToObject(log, "tds",         (double)tds);
            cJSON_AddNumberToObject(log, "temperature", (double)temp);
            cJSON_AddNumberToObject(log, "light",       (double)light);
            cJSON_AddBoolToObject  (log, "waterLow",    low);

            char *log_str = cJSON_PrintUnformatted(log);
            if (log_str) {
                firebase_post_log(log_str);
                cJSON_free(log_str);
            }
            cJSON_Delete(log);
        }

        ESP_LOGI(TAG, "Upload complete");

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(60000));
    }
}

/* ---- Core 0: taskDisplay (Priority 1, 1 s interval) ---- */

static void task_display(void *pv)
{
    vTaskDelay(pdMS_TO_TICKS(200));

    if (lcd_init() != 0) {
        ESP_LOGE(TAG, "LCD init failed — display task exiting");
        vTaskDelete(NULL);
        return;
    }

    lcd_clear();
    lcd_write_at(0, 3, "AQUAHARU v1.0");
    lcd_write_at(1, 5, "Starting…");
    vTaskDelay(pdMS_TO_TICKS(2000));

    TickType_t last_wake = xTaskGetTickCount();
    char line[LCD_COLS + 1];

    while (1) {
        float ph = 0, ec = 0, tds = 0, temp = 0;
        bool  low = false;

        if (xSemaphoreTake(s_sensor_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            ph   = g_sensor.pH;
            ec   = g_sensor.ec;
            tds  = g_sensor.tds;
            temp = g_sensor.temperature;
            low  = g_sensor.waterLow;
            xSemaphoreGive(s_sensor_mutex);
        }

        /* Line 1: "pH: 6.15  TDS: 710" */
        snprintf(line, sizeof(line), "pH: %.2f  TDS: %.0f",
                 (double)ph, (double)tds);
        lcd_write_at(0, 0, line);

        /* Line 2: "EC: 1.42  T: 24.3C" */
        snprintf(line, sizeof(line), "EC: %.2f  T: %.1fC",
                 (double)ec, (double)temp);
        lcd_write_at(1, 0, line);

        /* Line 3: "Peltier: ON " or "Peltier: OFF" */
        snprintf(line, sizeof(line), "Peltier: %s",
                 actuators_is_peltier_on() ? "ON " : "OFF");
        lcd_write_at(2, 0, line);

        /* Line 4: "Pumps: ON   H2O:OK " */
        snprintf(line, sizeof(line), "Pumps: %s  H2O:%s",
                 actuators_is_pumping() ? "ON " : "OFF",
                 low                   ? "LOW" : "OK ");
        lcd_write_at(3, 0, line);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000));
    }
}

/* ========================================================================= */
/*  SECTION 5 — app_main                                                     */
/* ========================================================================= */

void app_main(void)
{
    ESP_LOGI(TAG, "=== AQUAHARU Firmware Starting ===");

    /* ---- Synchronisation primitives ---- */
    s_sensor_mutex = xSemaphoreCreateMutex();
    s_config_mutex = xSemaphoreCreateMutex();
    s_q_act        = xQueueCreate(4, sizeof(struct ActCmd));

    if (!s_sensor_mutex || !s_config_mutex || !s_q_act) {
        ESP_LOGE(TAG, "FATAL: Failed to create synchronisation primitives");
        configASSERT(0);
    }

    /* ---- Hardware initialisation ---- */
    ESP_LOGI(TAG, "Initialising hardware…");

    /* Sensors */
    if (!temp_sensor_init()) {
        ESP_LOGE(TAG, "DS18B20 init failed — continuing without temperature");
    }
    if (ph_sensor_init() != 0) {
        ESP_LOGE(TAG, "pH sensor init failed");
    }
    if (tds_sensor_init(ph_sensor_get_adc_handle()) != 0) {
        ESP_LOGE(TAG, "TDS sensor init failed");
    }

    /* Actuators */
    if (actuators_init() != 0) {
        ESP_LOGE(TAG, "Actuators init failed");
    }

    /* =====================================================================
     * DEVELOPMENT MODE — Isolated Component Testing
     *
     * To test a single component on the bench, comment out the task
     * creation blocks AND the diagnostic block below, then uncomment
     * ONE of these calls:
     *
     *   test_sensor_temp();
     *   test_sensor_ph();
     *   test_sensor_tds();
     *   test_sensor_light();
     *   test_float_switch();
     *   test_relay_pump_ph_up();
     *   test_relay_pump_ph_down();
     *   test_relay_pump_nut_a();
     *   test_relay_pump_nut_b();
     *   test_relay_peltier();
     *   test_relay_circulation();
     *
     * Each function runs an infinite loop with a 1 s internal delay.
     * ===================================================================== */

#ifdef RUN_DIAGNOSTIC_TESTS
    /* ---- Full-system diagnostic (blocking) ---- */
    run_full_diagnostics();
#endif

    /* ---- Core 1 — Control Tasks ---- */
    if (xTaskCreatePinnedToCore(task_sensor,   "sensor",   4096, NULL, 4, NULL, 1) != pdPASS) {
        ESP_LOGE(TAG, "FATAL: Failed to create task_sensor");
        configASSERT(0);
    }
    if (xTaskCreatePinnedToCore(task_control,  "control",  4096, NULL, 5, NULL, 1) != pdPASS) {
        ESP_LOGE(TAG, "FATAL: Failed to create task_control");
        configASSERT(0);
    }
    if (xTaskCreatePinnedToCore(task_actuator, "actuator", 3072, NULL, 4, NULL, 1) != pdPASS) {
        ESP_LOGE(TAG, "FATAL: Failed to create task_actuator");
        configASSERT(0);
    }

    ESP_LOGI(TAG, "Core 1 (control) tasks started — sensor, control, actuator");

    /* ---- Core 0 — Communication Tasks ---- */

    /* LCD display (shows boot screen while Wi-Fi connects) */
    if (xTaskCreatePinnedToCore(task_display, "display", 3072, NULL, 1, NULL, 0) != pdPASS) {
        ESP_LOGE(TAG, "FATAL: Failed to create task_display");
        configASSERT(0);
    }

    /* Firebase client (Wi-Fi + Auth — blocking call, runs on Core 0 via caller) */
    if (firebase_client_init() == 0) {
        /* Start listening for remote config changes via SSE */
        firebase_start_config_stream(on_firebase_config);

        /* Start periodic upload task */
        if (xTaskCreatePinnedToCore(task_upload, "upload", 4096, NULL, 1, NULL, 0) != pdPASS) {
            ESP_LOGE(TAG, "FATAL: Failed to create task_upload");
            configASSERT(0);
        }
        ESP_LOGI(TAG, "Firebase client ready — telemetry & config stream active");
    } else {
        ESP_LOGE(TAG, "Firebase init failed — running offline (no uploads, no config sync)");
    }

    ESP_LOGI(TAG, "Core 0 (comms) tasks started — display, upload");
    ESP_LOGI(TAG, "=== System ready ===");

    /* Prevent app_main() from returning (FreeRTOS scheduler is already running) */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
