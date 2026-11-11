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
 * Bench mode selector: runs at the top of app_main() and does not start the
 * normal control/actuator tasks. Enable at most one mode for bench testing.
 * ========================================================================= */
/* #define RUN_BENCH_LCD_WIFI */
/* #define RUN_BENCH_FIREBASE_SIM */
/* #define RUN_BENCH_TEMP_SENSOR */
/* #define RUN_BENCH_TDS_SENSOR */
/* #define RUN_BENCH_PH_SENSOR */
/* #define RUN_BENCH_ALL_SENSORS */
/* #define RUN_BENCH_RELAY */

#if (defined(RUN_BENCH_LCD_WIFI) + defined(RUN_BENCH_FIREBASE_SIM) + \
     defined(RUN_BENCH_TEMP_SENSOR) + defined(RUN_BENCH_TDS_SENSOR) + \
     defined(RUN_BENCH_PH_SENSOR) + defined(RUN_BENCH_ALL_SENSORS) + \
     defined(RUN_BENCH_RELAY)) > 1
#error "Select only one bench mode"
#endif

/* -------------------------------------------------------------------------- */
/* Includes                                                                   */
/* -------------------------------------------------------------------------- */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "cJSON.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_netif_sntp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "actuators.h"
#include "bench_test.h"
#include "bench_firebase_sim.h"
#include "bench_temp_sensor.h"
#include "bench_tds_sensor.h"
#include "bench_ph_sensor.h"
#include "bench_all_sensors.h"
#include "bench_relay.h"
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
    float ph;
    float ec_ms_cm;
    float tds_ppm;
    float temperature_c;
    bool  light_do_high;
    bool  water_low;
    bool  valid;
};

struct SystemConfig {
    float setpoint_ph;
    float setpoint_ec_ms_cm;
    float setpoint_temperature_c;
    bool  automation_enabled;
    int   schedule_off_hour;
    int   schedule_on_hour;
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
    .setpoint_ph             = 6.5f,
    .setpoint_ec_ms_cm       = 1.5f,
    .setpoint_temperature_c  = 25.0f,
    .automation_enabled      = false, /* OFF until a saved or remote config is loaded */
    .schedule_off_hour       = 21,
    .schedule_on_hour        = 6,
};

/* Synchronisation primitives */
static SemaphoreHandle_t s_sensor_mutex = NULL;
static SemaphoreHandle_t s_config_mutex = NULL;
static QueueHandle_t     s_q_act        = NULL;
static bool              s_nvs_ready   = false;

/* Persist only validated control settings. No Firebase credentials are stored here. */
#define CONTROL_CONFIG_VERSION 1U
typedef struct {
    uint32_t version;
    float setpoint_ph;
    float setpoint_ec_ms_cm;
    float setpoint_temperature_c;
    uint8_t automation_enabled;
    uint8_t schedule_on_hour;
    uint8_t schedule_off_hour;
    uint8_t reserved;
} saved_config_t;

static void load_control_config(void)
{
    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed (%s); automation starts OFF", esp_err_to_name(err));
        return;
    }
    s_nvs_ready = true;

    nvs_handle_t handle;
    err = nvs_open("aquaharu", NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No saved control config; automation starts OFF");
        return;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cannot open saved config: %s", esp_err_to_name(err));
        return;
    }

    saved_config_t saved = {0};
    size_t size = sizeof(saved);
    err = nvs_get_blob(handle, "control", &saved, &size);
    nvs_close(handle);
    if (err != ESP_OK || size != sizeof(saved) ||
        saved.version != CONTROL_CONFIG_VERSION ||
        !isfinite(saved.setpoint_ph) || saved.setpoint_ph < 4 || saved.setpoint_ph > 8 ||
        !isfinite(saved.setpoint_ec_ms_cm) || saved.setpoint_ec_ms_cm < 0.5f || saved.setpoint_ec_ms_cm > 3.5f ||
        !isfinite(saved.setpoint_temperature_c) || saved.setpoint_temperature_c < 18 || saved.setpoint_temperature_c > 30 ||
        saved.automation_enabled > 1 || saved.schedule_on_hour > 23 || saved.schedule_off_hour > 23) {
        ESP_LOGW(TAG, "Saved control config missing or invalid; automation starts OFF");
        return;
    }

    g_config.setpoint_ph = saved.setpoint_ph;
    g_config.setpoint_ec_ms_cm = saved.setpoint_ec_ms_cm;
    g_config.setpoint_temperature_c = saved.setpoint_temperature_c;
    g_config.automation_enabled = saved.automation_enabled != 0;
    g_config.schedule_on_hour = saved.schedule_on_hour;
    g_config.schedule_off_hour = saved.schedule_off_hour;
    ESP_LOGI(TAG, "Restored local config: automation=%s, pH=%.2f EC=%.2f T=%.1f schedule=%02u-%02u WIB",
             g_config.automation_enabled ? "ON" : "OFF", (double)g_config.setpoint_ph,
             (double)g_config.setpoint_ec_ms_cm, (double)g_config.setpoint_temperature_c,
             (unsigned)saved.schedule_on_hour, (unsigned)saved.schedule_off_hour);
}

static void save_control_config(const struct SystemConfig *config)
{
    if (!s_nvs_ready) return;
    const saved_config_t saved = {
        .version = CONTROL_CONFIG_VERSION,
        .setpoint_ph = config->setpoint_ph,
        .setpoint_ec_ms_cm = config->setpoint_ec_ms_cm,
        .setpoint_temperature_c = config->setpoint_temperature_c,
        .automation_enabled = config->automation_enabled ? 1 : 0,
        .schedule_on_hour = (uint8_t)config->schedule_on_hour,
        .schedule_off_hour = (uint8_t)config->schedule_off_hour,
        .reserved = 0,
    };
    nvs_handle_t handle;
    esp_err_t err = nvs_open("aquaharu", NVS_READWRITE, &handle);
    if (err == ESP_OK) {
        err = nvs_set_blob(handle, "control", &saved, sizeof(saved));
        if (err == ESP_OK) err = nvs_commit(handle);
        nvs_close(handle);
    }
    if (err != ESP_OK) ESP_LOGE(TAG, "Failed to save local config: %s", esp_err_to_name(err));
    else ESP_LOGI(TAG, "Control config saved locally for offline operation");
}

/* Control schedule uses WIB (UTC+7). With no valid clock, the last saved
 * automation switch runs locally; time scheduling resumes after NTP sync. */
static bool automation_allowed(struct SystemConfig *snapshot)
{
    struct SystemConfig config;
    if (xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return false;
    config = g_config;
    xSemaphoreGive(s_config_mutex);
    if (snapshot) *snapshot = config;
    if (!config.automation_enabled) return false;

    const time_t now = time(NULL);
    if (now < (time_t)1704067200) {
        static bool warned_no_clock = false;
        if (!warned_no_clock) {
            ESP_LOGW(TAG, "Clock unavailable: using saved automation switch; WIB schedule suspended until time sync");
            warned_no_clock = true;
        }
        return true;
    }
    const time_t wib = now + 7 * 3600;
    struct tm local;
    if (!gmtime_r(&wib, &local)) return false;
    const int hour = local.tm_hour;
    const int start = config.schedule_on_hour;
    const int stop = config.schedule_off_hour;
    if (start == stop) return true; /* equal hours mean 24-hour operation */
    return start < stop ? (hour >= start && hour < stop)
                        : (hour >= start || hour < stop);
}

/* RTDB resolves this REST placeholder to its authoritative Unix timestamp
 * in milliseconds when it stores the payload. */
static void add_server_timestamp(cJSON *object, const char *field_name)
{
    cJSON *server_value = cJSON_CreateObject();
    if (!server_value) return;

    if (!cJSON_AddStringToObject(server_value, ".sv", "timestamp")) {
        cJSON_Delete(server_value);
        return;
    }
    cJSON_AddItemToObject(object, field_name, server_value);
}

/* -------------------------------------------------------------------------- */
/* Forward Declarations (FreeRTOS tasks)                                      */
/* -------------------------------------------------------------------------- */

static void task_sensor(void *pv);
static void task_control(void *pv);
static void task_actuator(void *pv);
static void task_upload(void *pv);
static void task_display(void *pv);
static void task_comms_start(void *pv);

/* -------------------------------------------------------------------------- */
/* Digital LDR (GPIO 34): HIGH = bright on the tested device_01 module.       */
/* -------------------------------------------------------------------------- */

static bool ldr_read_high(void)
{
    return gpio_get_level(GPIO_NUM_34) == 1;
}

/* Two-point mapping used in the successful sensor bench. Reconfirm the
 * pH 4.01 voltage with the final analog wiring before automatic dosing. */
#define PH_V_686 2.736f
#define PH_V_401 3.130f

static bool ph_read_calibrated(float *value)
{
    ph_sensor_sample_t sample;
    if (ph_sensor_read_sample(&sample) != ESP_OK ||
        sample.raw_adc <= 5 || sample.raw_adc >= 4090) return false;
    const float ph = 6.86f + (sample.pin_voltage_v - PH_V_686) *
                     (4.01f - 6.86f) / (PH_V_401 - PH_V_686);
    if (!isfinite(ph) || ph < 0.0f || ph > 14.0f) return false;
    *value = ph;
    return true;
}

/* ========================================================================= */
/*  Firebase Config Callback                                                  */
/* ========================================================================= */

static void on_firebase_config(const char *json_str)
{
    cJSON *root = cJSON_Parse(json_str);
    if (!root) return;

    /* Firebase SSE wraps config in {"path":"/...","data":...}. */
    const cJSON *path = cJSON_GetObjectItem(root, "path");
    const cJSON *data = cJSON_GetObjectItem(root, "data");
    if (!cJSON_IsString(path) || !data || cJSON_IsNull(data)) {
        cJSON_Delete(root);
        return;
    }

    const char *key = path->valuestring;
    if (*key == '/') ++key;
    const bool whole_config = *key == '\0';

    bool switch_off = false;
    bool changed = false;
    struct SystemConfig updated = {0};
    if (xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        const struct SystemConfig previous = g_config;
        const cJSON *item;
#define CONFIG_ITEM(name) (whole_config ? cJSON_GetObjectItem(data, name) : (strcmp(key, name) == 0 ? data : NULL))
        item = CONFIG_ITEM("setpoint_ph");
        if (cJSON_IsNumber(item) && isfinite(item->valuedouble) && item->valuedouble >= 4 && item->valuedouble <= 8) g_config.setpoint_ph = (float)item->valuedouble;
        item = CONFIG_ITEM("setpoint_ec_ms_cm");
        if (cJSON_IsNumber(item) && isfinite(item->valuedouble) && item->valuedouble >= 0.5 && item->valuedouble <= 3.5) g_config.setpoint_ec_ms_cm = (float)item->valuedouble;
        item = CONFIG_ITEM("setpoint_temperature_c");
        if (cJSON_IsNumber(item) && isfinite(item->valuedouble) && item->valuedouble >= 18 && item->valuedouble <= 30) g_config.setpoint_temperature_c = (float)item->valuedouble;
        item = CONFIG_ITEM("schedule_off_hour");
        if (cJSON_IsNumber(item) && item->valuedouble == item->valueint && item->valueint >= 0 && item->valueint <= 23) g_config.schedule_off_hour = item->valueint;
        item = CONFIG_ITEM("schedule_on_hour");
        if (cJSON_IsNumber(item) && item->valuedouble == item->valueint && item->valueint >= 0 && item->valueint <= 23) g_config.schedule_on_hour = item->valueint;
        item = CONFIG_ITEM("automation_enabled");
        if (cJSON_IsBool(item)) g_config.automation_enabled = cJSON_IsTrue(item);
#undef CONFIG_ITEM

        switch_off = previous.automation_enabled && !g_config.automation_enabled;
        changed = previous.setpoint_ph != g_config.setpoint_ph ||
                  previous.setpoint_ec_ms_cm != g_config.setpoint_ec_ms_cm ||
                  previous.setpoint_temperature_c != g_config.setpoint_temperature_c ||
                  previous.automation_enabled != g_config.automation_enabled ||
                  previous.schedule_on_hour != g_config.schedule_on_hour ||
                  previous.schedule_off_hour != g_config.schedule_off_hour;
        updated = g_config;

        xSemaphoreGive(s_config_mutex);
        ESP_LOGI(TAG, "Config received from Firebase SSE%s", changed ? " (updated)" : " (unchanged)");
    }

    if (switch_off) {
        actuators_emergency_stop();
        xQueueReset(s_q_act);
        ESP_LOGW(TAG, "Automation disabled from Web App: all relays OFF");
    }

    if (changed) save_control_config(&updated);

    cJSON_Delete(root);
}

/* ========================================================================= */
/*  FreeRTOS Task Functions                                                   */
/* ========================================================================= */

/* ---- Core 1: taskSensor (Priority 4, 5 s interval) ---- */

static void task_sensor(void *pv)
{
    TickType_t last_wake = xTaskGetTickCount();
    bool outputs_allowed_last_cycle = false;

    while (1) {
        struct SystemConfig config = {0};
        const bool outputs_allowed = automation_allowed(&config);
        if (!outputs_allowed && outputs_allowed_last_cycle) {
            actuators_emergency_stop();
        }
        outputs_allowed_last_cycle = outputs_allowed;
        float temp = temp_sensor_sample();
        if (!isfinite(temp) || temp <= TEMP_SENSOR_NO_DEVICE) {
            ESP_LOGW(TAG, "Temperature sensor error — skipping cycle");
            if (xSemaphoreTake(s_sensor_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                g_sensor.valid = false;
                xSemaphoreGive(s_sensor_mutex);
            }
            actuators_emergency_stop();
            xQueueReset(s_q_act);
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5000));
            continue;
        }

        float ph = 0;
        const bool ph_ok = ph_read_calibrated(&ph);
        float tds   = tds_sensor_read_tds(temp);
        float ec    = tds_sensor_ec_from_tds(tds);
        bool light_high = ldr_read_high();
        bool  low   = !actuators_water_ok();
        const bool valid = ph_ok && isfinite(tds) && tds >= 0.0f &&
                           isfinite(ec) && ec >= 0.0f;

        if (xSemaphoreTake(s_sensor_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            g_sensor.ph             = ph_ok ? ph : 0.0f;
            g_sensor.tds_ppm        = valid ? tds : 0.0f;
            g_sensor.ec_ms_cm       = valid ? ec : 0.0f;
            g_sensor.temperature_c  = temp;
            g_sensor.light_do_high  = light_high;
            g_sensor.water_low      = low;
            g_sensor.valid          = valid;
            xSemaphoreGive(s_sensor_mutex);
        }

        /* Cooling follows the same web automation switch and WIB schedule. */
        if (outputs_allowed) actuators_control_cooling(temp, config.setpoint_temperature_c);

        ESP_LOGI(TAG, "Sensors — pH:%.3f TDS:%.1f EC:%.3f T:%.1f°C LDR:%s Water:%s valid:%d",
                 (double)ph, (double)tds, (double)ec, (double)temp,
                 light_high ? "BRIGHT" : "DARK", low ? "LOW" : "OK", valid);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5000));
    }
}

/* ---- Core 1: taskControl (Priority 5, 5 s interval) ---- */

static void task_control(void *pv)
{
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        float ph = 0, tds = 0, ec = 0;
        bool valid = false;
        if (xSemaphoreTake(s_sensor_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            ph  = g_sensor.ph;
            tds = g_sensor.tds_ppm;
            ec = g_sensor.ec_ms_cm;
            valid = g_sensor.valid;
            xSemaphoreGive(s_sensor_mutex);
        }

        if (!valid) {
            ESP_LOGW(TAG, "Fuzzy skipped: no valid sensor sample");
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5000));
            continue;
        }

        float ph_up_ms = 0, ph_dn_ms = 0, nut_ms = 0;
        fuzzy_evaluate(ph, tds, &ph_up_ms, &ph_dn_ms, &nut_ms);

        ESP_LOGI(TAG, "Fuzzy preview input pH=%.3f TDS=%.1f; output pH_Up=%.0f ms pH_Dn=%.0f ms Nut=%.0f ms",
                 (double)ph, (double)tds,
                 (double)ph_up_ms, (double)ph_dn_ms, (double)nut_ms);

        struct SystemConfig config = {0};
        if (!automation_allowed(&config)) {
            const char *reason = !config.automation_enabled ? "automation switch OFF" :
                                 "outside WIB schedule";
            ESP_LOGI(TAG, "Relay command blocked: %s", reason);
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5000));
            continue;
        }

        /* The MATLAB-comparable fuzzy outputs above remain unchanged.
         * Web setpoints gate relay commands to avoid dosing past the target. */
        if (ph >= config.setpoint_ph - 0.05f) ph_up_ms = 0;
        if (ph <= config.setpoint_ph + 0.05f) ph_dn_ms = 0;
        if (ec >= config.setpoint_ec_ms_cm - 0.05f) nut_ms = 0;
        ESP_LOGI(TAG, "Setpoint gate pH=%.2f EC=%.2f: pH_Up=%.0f pH_Dn=%.0f Nut=%.0f ms",
                 (double)config.setpoint_ph, (double)config.setpoint_ec_ms_cm,
                 (double)ph_up_ms, (double)ph_dn_ms, (double)nut_ms);

        struct ActCmd cmd = {
            .nut_a_ms = (uint32_t)nut_ms,
            .nut_b_ms = (uint32_t)nut_ms,
            .ph_ms    = (ph_up_ms > ph_dn_ms) ? (uint32_t)ph_up_ms : (uint32_t)ph_dn_ms,
            .ph_is_up = (ph_up_ms > ph_dn_ms),
        };

        if (cmd.nut_a_ms == 0 && cmd.nut_b_ms == 0 && cmd.ph_ms == 0) {
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5000));
            continue;
        }

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

            if (!automation_allowed(NULL)) {
                ESP_LOGI(TAG, "Automation OFF or outside schedule — skipping actuation");
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
        float ph = 0, ec = 0, tds = 0, temp = 0;
        bool  low = false, light_high = false, valid = false;

        if (xSemaphoreTake(s_sensor_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            ph    = g_sensor.ph;
            ec    = g_sensor.ec_ms_cm;
            tds   = g_sensor.tds_ppm;
            temp  = g_sensor.temperature_c;
            light_high = g_sensor.light_do_high;
            low   = g_sensor.water_low;
            valid = g_sensor.valid;
            xSemaphoreGive(s_sensor_mutex);
        }

        if (!valid) {
            ESP_LOGW(TAG, "Upload skipped: no valid sensor sample");
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(60000));
            continue;
        }

        actuators_status_t pump_status = {0};
        actuators_get_status(&pump_status);

        /* Build live telemetry JSON */
        cJSON *live = cJSON_CreateObject();
        if (live) {
            cJSON_AddNumberToObject(live, "ph",             (double)ph);
            cJSON_AddNumberToObject(live, "ec_ms_cm",       (double)ec);
            cJSON_AddNumberToObject(live, "tds_ppm",        (double)tds);
            cJSON_AddNumberToObject(live, "temperature_c",  (double)temp);
            cJSON_AddBoolToObject  (live, "light_do_high",  light_high);
            cJSON_AddBoolToObject  (live, "float_switch_high", low);
            cJSON_AddBoolToObject  (live, "water_low",      low);
            cJSON_AddStringToObject(live, "measurement_mode", "normal");
            cJSON_AddBoolToObject(live, "automation_active", automation_allowed(NULL));

            cJSON *pumps = cJSON_CreateObject();
            if (pumps) {
                cJSON_AddBoolToObject(pumps, "ph_up",            pump_status.ph_up);
                cJSON_AddBoolToObject(pumps, "ph_down",          pump_status.ph_down);
                cJSON_AddBoolToObject(pumps, "nutrient_a",       pump_status.nutrient_a);
                cJSON_AddBoolToObject(pumps, "nutrient_b",       pump_status.nutrient_b);
                cJSON_AddBoolToObject(pumps, "peltier",          pump_status.peltier);
                cJSON_AddBoolToObject(pumps, "circulation_pump", pump_status.circulation_pump);
                cJSON_AddItemToObject(live, "pump_status", pumps);
            }
            add_server_timestamp(live, "timestamp_ms");

            char *live_str = cJSON_PrintUnformatted(live);
            if (live_str) {
                firebase_put_telemetry(live_str);
                cJSON_free(live_str);
            }
            cJSON_Delete(live);
        }

        /* Build log entry JSON */
        cJSON *history_record = cJSON_CreateObject();
        if (history_record) {
            cJSON_AddNumberToObject(history_record, "ph",             (double)ph);
            cJSON_AddNumberToObject(history_record, "ec_ms_cm",       (double)ec);
            cJSON_AddNumberToObject(history_record, "tds_ppm",        (double)tds);
            cJSON_AddNumberToObject(history_record, "temperature_c",  (double)temp);
            cJSON_AddBoolToObject  (history_record, "light_do_high",  light_high);
            cJSON_AddBoolToObject  (history_record, "float_switch_high", low);
            cJSON_AddBoolToObject  (history_record, "water_low",      low);
            add_server_timestamp(history_record, "timestamp_ms");

            char *history_str = cJSON_PrintUnformatted(history_record);
            if (history_str) {
                firebase_post_history(history_str);
                cJSON_free(history_str);
            }
            cJSON_Delete(history_record);
        }

        ESP_LOGI(TAG, "Upload complete");

        firebase_put_heartbeat();

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
            ph   = g_sensor.ph;
            ec   = g_sensor.ec_ms_cm;
            tds  = g_sensor.tds_ppm;
            temp = g_sensor.temperature_c;
            low  = g_sensor.water_low;
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
/*  app_main                                                                  */
/* ========================================================================= */

/* Firebase Auth performs TLS handshakes, which exceeded the 3584-byte
 * ESP-IDF main-task stack in the hardware log. Keep network setup on a
 * dedicated task and let app_main return after starting the system. */
static void task_comms_start(void *pv)
{
    if (firebase_client_init() == ESP_OK) {
        const esp_sntp_config_t time_config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
        if (esp_netif_sntp_init(&time_config) == ESP_OK) {
            const esp_err_t sync_result = esp_netif_sntp_sync_wait(pdMS_TO_TICKS(10000));
            ESP_LOGI(TAG, "WIB schedule clock sync: %s", esp_err_to_name(sync_result));
        } else {
            ESP_LOGW(TAG, "NTP init failed; last automation switch remains active, schedule unavailable");
        }
        if (!firebase_start_config_stream(on_firebase_config)) {
            ESP_LOGE(TAG, "Config stream did not start; local saved config remains active");
        }
        if (xTaskCreatePinnedToCore(task_upload, "upload", 12288, NULL, 1, NULL, 0) != pdPASS) {
            ESP_LOGE(TAG, "FATAL: Failed to create task_upload");
            configASSERT(0);
        }
        ESP_LOGI(TAG, "Firebase client ready — telemetry & config stream active");
    } else {
        ESP_LOGE(TAG, "Firebase init failed — running offline (no uploads, no config sync)");
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== AQUAHARU Firmware Starting ===");

#ifdef RUN_BENCH_RELAY
    bench_relay_start();
    return; /* No normal control, sensor, Firebase or actuator tasks. */
#endif
#ifdef RUN_BENCH_ALL_SENSORS
    bench_all_sensors_start();
    return; /* Never start normal control, actuator, SSE or float tasks. */
#endif
#ifdef RUN_BENCH_FIREBASE_SIM
    bench_firebase_sim_start();
    return; /* Never fall through to sensors, actuators, SSE or normal tasks. */
#endif
#ifdef RUN_BENCH_TEMP_SENSOR
    bench_temp_sensor_start();
    return; /* Never fall through to control, actuators, Firebase or other sensors. */
#endif
#ifdef RUN_BENCH_TDS_SENSOR
    bench_tds_sensor_start();
    return; /* Never fall through to control, actuators, Firebase or other sensors. */
#endif
#ifdef RUN_BENCH_PH_SENSOR
    bench_ph_sensor_start();
    return; /* Never fall through to control, actuators, Firebase or other sensors. */
#endif

#ifdef RUN_BENCH_LCD_WIFI
    bench_test_lcd_wifi();   /* never returns */
#endif

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
    const gpio_config_t ldr_input = {
        .pin_bit_mask = 1ULL << GPIO_NUM_34,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&ldr_input));

    if (actuators_init() != ESP_OK) {
        ESP_LOGE(TAG, "FATAL: Actuators init failed");
        configASSERT(0);
    }

    /* Relays are OFF before restoring a saved automation switch. */
    load_control_config();

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

    if (xTaskCreatePinnedToCore(task_comms_start, "comms_start", 16384, NULL, 2, NULL, 0) != pdPASS) {
        ESP_LOGE(TAG, "FATAL: Failed to create task_comms_start");
        configASSERT(0);
    }
    ESP_LOGI(TAG, "=== System tasks started ===");
}
