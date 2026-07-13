/*
 * firebase_client.c
 *
 * WiFi Station + Firebase Realtime Database Client — Implementation
 *
 * Flow:
 *   1. Init NVS, netif, event loop, Wi-Fi STA with auto-reconnect
 *   2. Authenticate via Firebase Auth REST API → obtain idToken
 *   3. Refresh idToken every 50 min via esp_timer
 *   4. PUT telemetry / POST logs on demand
 *   5. SSE config stream in a persistent background task
 *
 * References:
 *   - Firebase REST API: https://firebase.google.com/docs/reference/rest/database
 *   - Firebase Auth REST: https://firebase.google.com/docs/reference/rest/auth
 */

#include "firebase_client.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"

/* -------------------------------------------------------------------------- */
/* Constants                                                                  */
/* -------------------------------------------------------------------------- */

static const char *TAG = "firebase";

/* FreeRTOS event group bits */
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
#define AUTH_READY_BIT      BIT2

#define WIFI_MAX_RETRY      5
#define WIFI_RETRY_DELAY_MS 3000

/* HTTP buffer sizes */
#define HTTP_RX_BUF_SIZE    2048
#define HTTP_TX_BUF_SIZE    1024
#define SSE_RX_BUF_SIZE     4096

/* SSE reconnect delay (ms) */
#define SSE_RECONNECT_MS    5000

/* URL buffer sizes */
#define URL_BUF_SIZE        1536    /* room for token (~1024) + host + path */
#define BODY_BUF_SIZE       512

/* -------------------------------------------------------------------------- */
/* Static State                                                               */
/* -------------------------------------------------------------------------- */

static EventGroupHandle_t s_event_group = NULL;
static char     s_id_token[1024]     = {0};
static char     s_device_id[16]      = {0};
static int      s_wifi_retry         = 0;
static bool     s_authenticated      = false;
static esp_timer_handle_t s_token_timer = NULL;

/* SSE stream */
static TaskHandle_t         s_sse_task      = NULL;
static firebase_config_cb_t s_config_cb     = NULL;
static volatile bool        s_sse_running   = false;

/* -------------------------------------------------------------------------- */
/* Forward Declarations                                                       */
/* -------------------------------------------------------------------------- */

static void token_refresh_cb(void *arg);
static void sse_task_fn(void *arg);
static int  firebase_authenticate(void);

/* -------------------------------------------------------------------------- */
/* MAC Address → Device ID                                                     */
/* -------------------------------------------------------------------------- */

static void fetch_device_id(void)
{
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    snprintf(s_device_id, sizeof(s_device_id),
             "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "Device ID: %s", s_device_id);
}

const char *firebase_get_device_id(void)
{
    return s_device_id;
}

const char *firebase_get_token(void)
{
    return (s_authenticated && s_id_token[0]) ? s_id_token : NULL;
}

bool firebase_is_ready(void)
{
    return s_authenticated && s_id_token[0] &&
           (xEventGroupGetBits(s_event_group) & WIFI_CONNECTED_BIT);
}

/* -------------------------------------------------------------------------- */
/* WiFi Event Handler                                                          */
/* -------------------------------------------------------------------------- */

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {

        if (event_id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();

        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            wifi_event_sta_disconnected_t *disconn = event_data;
            ESP_LOGW(TAG, "Wi-Fi disconnected, reason=%d", (int)disconn->reason);

            xEventGroupClearBits(s_event_group, WIFI_CONNECTED_BIT);

            if (s_wifi_retry < WIFI_MAX_RETRY) {
                s_wifi_retry++;
                ESP_LOGI(TAG, "Reconnecting (attempt %d/%d)…",
                         s_wifi_retry, WIFI_MAX_RETRY);
                vTaskDelay(pdMS_TO_TICKS(WIFI_RETRY_DELAY_MS));
                esp_wifi_connect();
            } else {
                xEventGroupSetBits(s_event_group, WIFI_FAIL_BIT);
                ESP_LOGE(TAG, "Wi-Fi connection failed after %d attempts",
                         WIFI_MAX_RETRY);
            }
        }
    }
}

static void ip_event_handler(void *arg,
                             esp_event_base_t event_base,
                             int32_t event_id, void *event_data)
{
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *evt = event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&evt->ip_info.ip));
        s_wifi_retry = 0;
        xEventGroupSetBits(s_event_group, WIFI_CONNECTED_BIT);
    }
}

/* -------------------------------------------------------------------------- */
/* Wi-Fi Init                                                                  */
/* -------------------------------------------------------------------------- */

static int wifi_init_sta(void)
{
    /* Initialise NVS (required by Wi-Fi stack) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* Create event group before registering handlers */
    s_event_group = xEventGroupCreate();
    if (!s_event_group) {
        ESP_LOGE(TAG, "Failed to create event group");
        return ESP_ERR_NO_MEM;
    }

    /* Initialise TCP/IP stack */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    /* Register event handlers */
    esp_event_handler_instance_t instance_any_id, instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL, &instance_got_ip));

    /* Wi-Fi init + config */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_cfg = {0};
    strncpy((char *)wifi_cfg.sta.ssid,     WIFI_SSID, sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char *)wifi_cfg.sta.password, WIFI_PASS, sizeof(wifi_cfg.sta.password) - 1);
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi STA init complete — connecting to \"%s\"", WIFI_SSID);

    /* Wait for connection or failure */
    EventBits_t bits = xEventGroupWaitBits(
        s_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, pdFALSE,
        pdMS_TO_TICKS(30000));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Wi-Fi connected");
        return 0;
    }
    if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Wi-Fi connection failed");
        return ESP_FAIL;
    }
    ESP_LOGE(TAG, "Wi-Fi connection timed out");
    return ESP_ERR_TIMEOUT;
}

/* -------------------------------------------------------------------------- */
/* HTTP Helper — perform a synchronous request, return body as cJSON          */
/* -------------------------------------------------------------------------- */

/**
 * @brief Execute an HTTP request and return the response body as a cJSON object.
 *
 * The caller must cJSON_Delete() the returned object.
 *
 * @param url     Full URL including ?auth= query.
 * @param method  HTTP_METHOD_GET / PUT / POST.
 * @param body    JSON body string (NULL for GET).
 * @return Parsed cJSON object, or NULL on failure.
 */
static cJSON *http_json_request(const char *url,
                                esp_http_client_method_t method,
                                const char *body)
{
    esp_http_client_config_t http_cfg = {
        .url                  = url,
        .method               = method,
        .timeout_ms           = 10000,
        .buffer_size          = HTTP_RX_BUF_SIZE,
        .buffer_size_tx       = HTTP_TX_BUF_SIZE,
        /* TODO(security): In production, use .cert_pem with a root CA bundle
         * or .crt_bundle_attach = esp_crt_bundle_attach. */
        .skip_cert_common_name_check = true,
    };

    esp_http_client_handle_t client = esp_http_client_init(&http_cfg);
    if (!client) {
        ESP_LOGE(TAG, "Failed to init HTTP client for %s", url);
        return NULL;
    }

    /* Set headers */
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "Accept", "application/json");

    /* Set body for POST/PUT */
    if (body) {
        esp_http_client_set_post_field(client, body, (int)strlen(body));
    }

    /* Perform request */
    esp_err_t err = esp_http_client_perform(client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP request failed: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return NULL;
    }

    int status = esp_http_client_get_status_code(client);
    int content_len = esp_http_client_get_content_length(client);

    ESP_LOGD(TAG, "HTTP %d, Content-Length: %d", status, content_len);

    if (status < 200 || status >= 300) {
        ESP_LOGW(TAG, "HTTP error %d for %s", status, url);
        esp_http_client_cleanup(client);
        return NULL;
    }

    /* Read response body */
    char *rx_buf = malloc(HTTP_RX_BUF_SIZE);
    if (!rx_buf) {
        esp_http_client_cleanup(client);
        return NULL;
    }
    memset(rx_buf, 0, HTTP_RX_BUF_SIZE);

    int total = 0;
    int read_len = 0;
    while ((read_len = esp_http_client_read(client, rx_buf + total,
                                            HTTP_RX_BUF_SIZE - total - 1)) > 0) {
        total += read_len;
        if (total >= HTTP_RX_BUF_SIZE - 1) break;
    }
    rx_buf[total] = '\0';

    esp_http_client_cleanup(client);

    cJSON *json = cJSON_Parse(rx_buf);
    if (!json) {
        ESP_LOGE(TAG, "JSON parse error: %s", cJSON_GetErrorPtr());
    }

    free(rx_buf);
    return json;
}

/* -------------------------------------------------------------------------- */
/* Firebase Authentication                                                     */
/* -------------------------------------------------------------------------- */

static int firebase_authenticate(void)
{
    char auth_url[URL_BUF_SIZE];
    snprintf(auth_url, sizeof(auth_url),
             "https://%s/v1/accounts:signInWithPassword?key=%s",
             FIREBASE_AUTH, FIREBASE_API_KEY);

    char auth_body[BODY_BUF_SIZE];
    snprintf(auth_body, sizeof(auth_body),
             "{\"email\":\"%s\",\"password\":\"%s\",\"returnSecureToken\":true}",
             FIREBASE_EMAIL, FIREBASE_PASS);

    ESP_LOGI(TAG, "Authenticating with Firebase…");

    cJSON *json = http_json_request(auth_url, HTTP_METHOD_POST, auth_body);
    if (!json) {
        ESP_LOGE(TAG, "Firebase authentication failed");
        return ESP_FAIL;
    }

    /* Extract idToken */
    cJSON *token = cJSON_GetObjectItem(json, "idToken");
    if (!token || !cJSON_IsString(token)) {
        ESP_LOGE(TAG, "idToken not found in auth response");
        cJSON_Delete(json);
        return ESP_FAIL;
    }

    strncpy(s_id_token, token->valuestring, sizeof(s_id_token) - 1);
    s_id_token[sizeof(s_id_token) - 1] = '\0';

    /* Extract localId (optional — logged for info) */
    cJSON *uid = cJSON_GetObjectItem(json, "localId");
    if (uid && cJSON_IsString(uid)) {
        ESP_LOGI(TAG, "Authenticated — UID: %s", uid->valuestring);
    }

    cJSON_Delete(json);

    s_authenticated = true;
    xEventGroupSetBits(s_event_group, AUTH_READY_BIT);

    return 0;
}

/* -------------------------------------------------------------------------- */
/* Token Refresh Timer                                                         */
/* -------------------------------------------------------------------------- */

static void token_refresh_cb(void *arg)
{
    ESP_LOGI(TAG, "Refreshing Firebase ID token…");
    if (firebase_authenticate() != 0) {
        ESP_LOGE(TAG, "Token refresh failed — will retry at next interval");
    } else {
        ESP_LOGI(TAG, "Token refreshed successfully");
    }
}

static void start_token_refresh_timer(void)
{
    const esp_timer_create_args_t timer_args = {
        .callback = &token_refresh_cb,
        .arg      = NULL,
        .name     = "token_refresh",
    };
    esp_timer_create(&timer_args, &s_token_timer);

    /* Fire every 50 minutes (in microseconds) */
    uint64_t interval_us = (uint64_t)FIREBASE_TOKEN_REFRESH_MIN * 60ULL * 1000000ULL;
    esp_timer_start_periodic(s_token_timer, interval_us);

    ESP_LOGI(TAG, "Token refresh timer started (interval: %d min)",
             FIREBASE_TOKEN_REFRESH_MIN);
}

/* -------------------------------------------------------------------------- */
/* REST Operations                                                             */
/* -------------------------------------------------------------------------- */

int firebase_put_telemetry(const char *json_payload)
{
    if (!s_authenticated || !s_id_token[0]) {
        ESP_LOGE(TAG, "Cannot PUT — not authenticated");
        return ESP_ERR_INVALID_STATE;
    }

    char url[URL_BUF_SIZE];
    snprintf(url, sizeof(url),
             "https://%s/devices/%s/live.json?auth=%s",
             FIREBASE_HOST, s_device_id, s_id_token);

    cJSON *resp = http_json_request(url, HTTP_METHOD_PUT, json_payload);

    /* PUT to live.json returns the stored data — we can ignore the body
     * as long as the HTTP status was 2xx (already checked in http_json_request) */
    if (resp) {
        cJSON_Delete(resp);
        ESP_LOGI(TAG, "Telemetry PUT successful");
        return 0;
    }

    ESP_LOGE(TAG, "Telemetry PUT failed");
    return ESP_FAIL;
}

/* -------------------------------------------------------------------------- */

int firebase_post_log(const char *json_payload)
{
    if (!s_authenticated || !s_id_token[0]) {
        ESP_LOGE(TAG, "Cannot POST — not authenticated");
        return ESP_ERR_INVALID_STATE;
    }

    char url[URL_BUF_SIZE];
    snprintf(url, sizeof(url),
             "https://%s/devices/%s/logs.json?auth=%s",
             FIREBASE_HOST, s_device_id, s_id_token);

    cJSON *resp = http_json_request(url, HTTP_METHOD_POST, json_payload);

    /* POST returns {"name":"-<unique_key>"} — we can inspect the key if
     * needed, but for now just confirm success. */
    if (resp) {
        cJSON_Delete(resp);
        ESP_LOGI(TAG, "Log POST successful");
        return 0;
    }

    ESP_LOGE(TAG, "Log POST failed");
    return ESP_FAIL;
}

/* -------------------------------------------------------------------------- */
/* SSE Config Stream (persistent — runs in its own task)                       */
/* -------------------------------------------------------------------------- */

static void sse_task_fn(void *arg)
{
    char url[URL_BUF_SIZE];
    snprintf(url, sizeof(url),
             "https://%s/devices/%s/config.json?auth=%s",
             FIREBASE_HOST, s_device_id, s_id_token);

    ESP_LOGI(TAG, "SSE config stream starting — %s", url);

    char *rx_buf = malloc(SSE_RX_BUF_SIZE);
    if (!rx_buf) {
        ESP_LOGE(TAG, "SSE: failed to allocate RX buffer");
        vTaskDelete(NULL);
        return;
    }

    while (s_sse_running) {

        /* If not authenticated, wait and retry */
        if (!s_authenticated || !s_id_token[0]) {
            ESP_LOGW(TAG, "SSE: waiting for authentication…");
            vTaskDelay(pdMS_TO_TICKS(SSE_RECONNECT_MS));
            continue;
        }

        /* Rebuild URL with current token */
        snprintf(url, sizeof(url),
                 "https://%s/devices/%s/config.json?auth=%s",
                 FIREBASE_HOST, s_device_id, s_id_token);

        esp_http_client_config_t http_cfg = {
            .url             = url,
            .method          = HTTP_METHOD_GET,
            .timeout_ms      = 0,             /* no timeout — persistent */
            .keep_alive_enable = true,
            .buffer_size     = HTTP_RX_BUF_SIZE,
            .skip_cert_common_name_check = true,
        };

        esp_http_client_handle_t client = esp_http_client_init(&http_cfg);
        if (!client) {
            ESP_LOGW(TAG, "SSE: client init failed — retrying in %d s",
                     SSE_RECONNECT_MS / 1000);
            vTaskDelay(pdMS_TO_TICKS(SSE_RECONNECT_MS));
            continue;
        }

        esp_http_client_set_header(client, "Accept", "text/event-stream");
        esp_http_client_set_header(client, "Cache-Control", "no-cache");

        esp_err_t err = esp_http_client_open(client, 0);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "SSE: open failed: %s", esp_err_to_name(err));
            esp_http_client_cleanup(client);
            vTaskDelay(pdMS_TO_TICKS(SSE_RECONNECT_MS));
            continue;
        }

        int content_len = esp_http_client_fetch_headers(client);
        ESP_LOGI(TAG, "SSE connected — Content-Length: %d", content_len);

        /* Read SSE data in a loop */
        int buf_pos = 0;
        while (s_sse_running) {
            int rd = esp_http_client_read(client,
                                          rx_buf + buf_pos,
                                          SSE_RX_BUF_SIZE - buf_pos - 1);
            if (rd <= 0) {
                ESP_LOGW(TAG, "SSE: read returned %d — reconnecting…", rd);
                break;   /* connection lost — reconnect */
            }

            buf_pos += rd;
            rx_buf[buf_pos] = '\0';

            /* Parse complete SSE events from the buffer.
             * An SSE event ends with "\n\n".  Look for "data:" lines. */
            char *line_start = rx_buf;
            char *line_end;

            while ((line_end = strstr(line_start, "\n\n")) != NULL) {
                *line_end = '\0';   /* terminate the event block */

                /* Extract the "data:" payload */
                char *data_ptr = strstr(line_start, "data: ");
                if (data_ptr) {
                    data_ptr += 6;   /* skip "data: " */

                    /* Strip trailing newline (if any) */
                    size_t dlen = strlen(data_ptr);
                    while (dlen > 0 &&
                           (data_ptr[dlen - 1] == '\n' || data_ptr[dlen - 1] == '\r')) {
                        data_ptr[--dlen] = '\0';
                    }

                    if (dlen > 0 && s_config_cb) {
                        ESP_LOGI(TAG, "SSE config: %s", data_ptr);
                        s_config_cb(data_ptr);
                    }
                }

                /* Advance past this event */
                line_start = line_end + 2;
            }

            /* Compact: move any partial event to the front of the buffer */
            int remaining = buf_pos - (int)(line_start - rx_buf);
            if (remaining > 0 && line_start != rx_buf) {
                memmove(rx_buf, line_start, remaining);
            }
            buf_pos = remaining;
        }

        esp_http_client_close(client);
        esp_http_client_cleanup(client);

        if (s_sse_running) {
            ESP_LOGI(TAG, "SSE: reconnecting in %d s…", SSE_RECONNECT_MS / 1000);
            vTaskDelay(pdMS_TO_TICKS(SSE_RECONNECT_MS));
        }
    }

    free(rx_buf);
    ESP_LOGI(TAG, "SSE task exiting");
    vTaskDelete(NULL);
}

/* -------------------------------------------------------------------------- */

firebase_sse_handle_t firebase_start_config_stream(firebase_config_cb_t cb)
{
    if (s_sse_running) {
        ESP_LOGW(TAG, "SSE stream already running");
        return s_sse_task;
    }

    s_config_cb   = cb;
    s_sse_running = true;

    BaseType_t ret = xTaskCreate(&sse_task_fn, "sse_stream",
                                 4096, NULL, tskIDLE_PRIORITY + 1,
                                 &s_sse_task);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create SSE task");
        s_sse_running = false;
        return NULL;
    }

    return s_sse_task;
}

void firebase_stop_config_stream(firebase_sse_handle_t handle)
{
    s_sse_running = false;
    /* The SSE task will exit its loop and self-delete on the next
     * failed read or at the top of its reconnect cycle. */
    s_sse_task = NULL;
}

/* -------------------------------------------------------------------------- */
/* Public Init & Status                                                        */
/* -------------------------------------------------------------------------- */

int firebase_client_init(void)
{
    /* Fetch MAC-based device ID */
    fetch_device_id();

    /* Connect Wi-Fi */
    int ret = wifi_init_sta();
    if (ret != 0) {
        ESP_LOGE(TAG, "Wi-Fi init failed");
        return ret;
    }

    /* Authenticate with Firebase */
    ret = firebase_authenticate();
    if (ret != 0) {
        ESP_LOGE(TAG, "Firebase auth failed");
        return ret;
    }

    /* Start periodic token refresh */
    start_token_refresh_timer();

    ESP_LOGI(TAG, "Firebase client ready");
    return 0;
}

/* -------------------------------------------------------------------------- */

bool firebase_client_wait_ready(uint32_t timeout_ms)
{
    EventBits_t bits = xEventGroupWaitBits(
        s_event_group,
        WIFI_CONNECTED_BIT | AUTH_READY_BIT,
        pdFALSE, pdTRUE,
        pdMS_TO_TICKS(timeout_ms));

    return ((bits & WIFI_CONNECTED_BIT) && (bits & AUTH_READY_BIT));
}
