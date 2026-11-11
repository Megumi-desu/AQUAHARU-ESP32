/*
 * firebase_client.c
 *
 * WiFi Station + Firebase Realtime Database Client — Implementation
 *
 * Flow:
 *   1. Init NVS, netif, event loop, Wi-Fi STA with auto-reconnect
 *   2. Authenticate via Firebase Auth REST API → obtain idToken
 *   3. Refresh idToken every 50 min in a dedicated HTTPS task
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
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"

/* -------------------------------------------------------------------------- */
/* Constants                                                                  */
/* -------------------------------------------------------------------------- */

static const char *TAG = "firebase";

/* Public GTS R1-R4 root CAs, embedded as NUL-terminated PEM. Standard mbedTLS
 * chain verification supports cross-signed Google chains without the optional
 * IDF 6.0.x bundle callback. Hostname verification remains enabled.
 * See TLS_CERTIFICATES.md for provenance and maintenance. */
extern const char firebase_google_roots[] asm("_binary_google_roots_pem_start");

/* FreeRTOS event group bits */
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
#define AUTH_READY_BIT      BIT2

#define WIFI_MAX_RETRY      5
#define WIFI_RETRY_DELAY_MS 3000
#define WIFI_CONNECT_TIMEOUT_MS 90000

/* HTTP buffer sizes */
#define HTTP_RX_BUF_SIZE    2048
#define HTTP_RESPONSE_MAX   8192
#define HTTP_TX_BUF_SIZE    2048 /* request line includes an Auth token */
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
static char     s_device_id[32]      = {0};
static int      s_wifi_retry         = 0;
static bool     s_authenticated      = false;
static TaskHandle_t s_token_refresh_task = NULL;

/* SSE stream */
static TaskHandle_t         s_sse_task      = NULL;
static firebase_config_cb_t s_config_cb     = NULL;
static volatile bool        s_sse_running   = false;

/* -------------------------------------------------------------------------- */
/* Forward Declarations                                                       */
/* -------------------------------------------------------------------------- */

static void token_refresh_task_fn(void *arg);
static void sse_task_fn(void *arg);
static int  firebase_authenticate(void);

/* -------------------------------------------------------------------------- */
/* Stable Device ID                                                            */
/* -------------------------------------------------------------------------- */

static void fetch_device_id(void)
{
    snprintf(s_device_id, sizeof(s_device_id), "%s", FIREBASE_DEVICE_ID);
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
           firebase_wifi_is_connected();
}

bool firebase_wifi_is_connected(void)
{
    return s_event_group &&
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
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s (saved control settings preserved)", esp_err_to_name(ret));
        return ret;
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
    /* Several APs share this SSID. Inspect all channels, then try the
     * strongest matching AP first instead of the first one discovered. */
    wifi_cfg.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    wifi_cfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi STA init complete — connecting to \"%s\"", WIFI_SSID);

    /* Wait for connection or failure */
    EventBits_t bits = xEventGroupWaitBits(
        s_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, pdFALSE,
        pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

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
typedef struct {
    char *data;
    size_t length;
    bool overflow;
} http_response_t;

/* perform() consumes the body. Capture chunks while it is running, including
 * chunked transfer responses. Never log response bodies (Auth contains secrets). */
static esp_err_t capture_http_response(esp_http_client_event_t *event)
{
    http_response_t *response = event->user_data;
    if (event->event_id == HTTP_EVENT_ON_DATA && event->data_len > 0) {
        size_t size = (size_t)event->data_len;
        if (size >= HTTP_RESPONSE_MAX - response->length) {
            response->overflow = true;
            return ESP_FAIL;
        }
        memcpy(response->data + response->length, event->data, size);
        response->length += size;
        response->data[response->length] = '\0';
    }
    return ESP_OK;
}

static cJSON *http_json_request(const char *url,
                                esp_http_client_method_t method,
                                const char *body, int *http_status)
{
    if (http_status) *http_status = 0;
    http_response_t response = {.data = calloc(1, HTTP_RESPONSE_MAX)};
    if (!response.data) return NULL;

    esp_http_client_config_t http_cfg = {
        .url                  = url,
        .method               = method,
        .timeout_ms           = 10000,
        .buffer_size          = HTTP_RX_BUF_SIZE,
        .buffer_size_tx       = HTTP_TX_BUF_SIZE,
        .cert_pem             = firebase_google_roots,
        .event_handler        = capture_http_response,
        .user_data            = &response,
        .disable_auto_redirect = true,
    };

    esp_http_client_handle_t client = esp_http_client_init(&http_cfg);
    if (!client) {
        ESP_LOGE(TAG, "Failed to init HTTP client");
        free(response.data);
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
    int status = esp_http_client_get_status_code(client);
    if (http_status) *http_status = status;
    ESP_LOGI(TAG, "REST %s HTTP %d (%s)",
             method == HTTP_METHOD_PUT ? "PUT" :
             method == HTTP_METHOD_POST ? "POST" : "GET",
             status, esp_err_to_name(err));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP request failed: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        free(response.data);
        return NULL;
    }

    if (status < 200 || status >= 300 || response.overflow) {
        ESP_LOGW(TAG, "HTTP response rejected: status=%d overflow=%d",
                 status, response.overflow);
        esp_http_client_cleanup(client);
        free(response.data);
        return NULL;
    }

    esp_http_client_cleanup(client);

    cJSON *json = cJSON_Parse(response.data);
    if (!json) {
        ESP_LOGE(TAG, "Invalid or empty JSON response");
    }

    free(response.data);
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

    cJSON *json = http_json_request(auth_url, HTTP_METHOD_POST, auth_body, NULL);
    if (!json) {
        ESP_LOGE(TAG, "Firebase authentication failed");
        return ESP_FAIL;
    }

    /* Extract idToken */
    cJSON *token = cJSON_GetObjectItem(json, "idToken");
    if (!cJSON_IsString(token) || !token->valuestring[0] ||
        strlen(token->valuestring) >= sizeof(s_id_token)) {
        ESP_LOGE(TAG, "Auth token missing, empty or exceeds client capacity");
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
/* Token Refresh Task                                                          */
/* -------------------------------------------------------------------------- */

static void token_refresh_task_fn(void *arg)
{
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(FIREBASE_TOKEN_REFRESH_MIN * 60 * 1000));
        ESP_LOGI(TAG, "Refreshing Firebase ID token…");
        if (firebase_authenticate() != 0) {
            ESP_LOGE(TAG, "Token refresh failed — will retry at next interval");
        } else {
            ESP_LOGI(TAG, "Token refreshed successfully");
        }
    }
}

static esp_err_t start_token_refresh_task(void)
{
    if (s_token_refresh_task) return ESP_OK;
    if (xTaskCreate(token_refresh_task_fn, "token_refresh", 12288, NULL,
                    tskIDLE_PRIORITY + 1, &s_token_refresh_task) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create token refresh task");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Token refresh task started (interval: %d min)", FIREBASE_TOKEN_REFRESH_MIN);
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* REST Operations                                                             */
/* -------------------------------------------------------------------------- */

int firebase_put_telemetry(const char *json_payload)
{
    return firebase_put_telemetry_with_status(json_payload, NULL);
}

int firebase_put_telemetry_with_status(const char *json_payload, int *http_status)
{
    if (http_status) *http_status = 0;
    if (!s_authenticated || !s_id_token[0]) {
        ESP_LOGE(TAG, "Cannot PUT — not authenticated");
        return ESP_ERR_INVALID_STATE;
    }

    char url[URL_BUF_SIZE];
    snprintf(url, sizeof(url),
             "https://%s/%s/%s/live.json?auth=%s",
             FIREBASE_HOST, FIREBASE_DEVICE_ROOT, s_device_id, s_id_token);

    cJSON *resp = http_json_request(url, HTTP_METHOD_PUT, json_payload, http_status);

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

int firebase_post_history(const char *json_payload)
{
    if (!s_authenticated || !s_id_token[0]) {
        ESP_LOGE(TAG, "Cannot POST — not authenticated");
        return ESP_ERR_INVALID_STATE;
    }

    char url[URL_BUF_SIZE];
    snprintf(url, sizeof(url),
             "https://%s/%s/%s/history.json?auth=%s",
             FIREBASE_HOST, FIREBASE_DEVICE_ROOT, s_device_id, s_id_token);

    cJSON *resp = http_json_request(url, HTTP_METHOD_POST, json_payload, NULL);

    /* POST returns {"name":"-<unique_key>"} — we can inspect the key if
     * needed, but for now just confirm success. */
    if (resp) {
        const cJSON *key = cJSON_GetObjectItemCaseSensitive(resp, "name");
        if (cJSON_IsString(key)) {
            ESP_LOGI(TAG, "History key (for selective cleanup): %s", key->valuestring);
        }
        cJSON_Delete(resp);
        ESP_LOGI(TAG, "History POST successful");
        return 0;
    }

    ESP_LOGE(TAG, "History POST failed");
    return ESP_FAIL;
}

/* -------------------------------------------------------------------------- */

int firebase_put_heartbeat(void)
{
    return firebase_put_heartbeat_with_status(NULL);
}

int firebase_put_heartbeat_with_status(int *http_status)
{
    if (http_status) *http_status = 0;
    if (!s_authenticated || !s_id_token[0]) {
        ESP_LOGE(TAG, "Cannot update heartbeat — not authenticated");
        return ESP_ERR_INVALID_STATE;
    }

    char url[URL_BUF_SIZE];
    snprintf(url, sizeof(url),
             "https://%s/%s/%s/heartbeat.json?auth=%s",
             FIREBASE_HOST, FIREBASE_DEVICE_ROOT, s_device_id, s_id_token);

    const char *payload = "{\"last_seen_ms\":{\".sv\":\"timestamp\"}}";
    cJSON *resp = http_json_request(url, HTTP_METHOD_PUT, payload, http_status);
    if (resp) {
        cJSON_Delete(resp);
        ESP_LOGI(TAG, "Heartbeat PUT successful");
        return 0;
    }

    ESP_LOGE(TAG, "Heartbeat PUT failed");
    return ESP_FAIL;
}

/* -------------------------------------------------------------------------- */
/* SSE Config Stream (persistent — runs in its own task)                       */
/* -------------------------------------------------------------------------- */

static void sse_task_fn(void *arg)
{
    char url[URL_BUF_SIZE];
    snprintf(url, sizeof(url),
             "https://%s/%s/%s/config.json?auth=%s",
             FIREBASE_HOST, FIREBASE_DEVICE_ROOT, s_device_id, s_id_token);

    ESP_LOGI(TAG, "SSE config stream starting for device %s", s_device_id);

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
                 "https://%s/%s/%s/config.json?auth=%s",
                 FIREBASE_HOST, FIREBASE_DEVICE_ROOT, s_device_id, s_id_token);

        esp_http_client_config_t http_cfg = {
            .url             = url,
            .method          = HTTP_METHOD_GET,
            .timeout_ms      = 0,             /* no timeout — persistent */
            .keep_alive_enable = true,
            .buffer_size     = HTTP_RX_BUF_SIZE,
            .buffer_size_tx  = HTTP_TX_BUF_SIZE,
            .cert_pem        = firebase_google_roots,
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
        const int status = esp_http_client_get_status_code(client);
        if (status != 200) {
            ESP_LOGW(TAG, "SSE: HTTP %d (headers=%d) — reconnecting", status, content_len);
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            vTaskDelay(pdMS_TO_TICKS(SSE_RECONNECT_MS));
            continue;
        }
        ESP_LOGI(TAG, "SSE connected — HTTP 200, Content-Length: %d", content_len);

        /* Read SSE data in a loop */
        int buf_pos = 0;
        while (s_sse_running) {
            int rd = esp_http_client_read(client,
                                          rx_buf + buf_pos,
                                          SSE_RX_BUF_SIZE - buf_pos - 1);
            if (rd == -ESP_ERR_HTTP_EAGAIN) {
                /* Idle SSE stream is normal. Reconnect only when Wi-Fi is down. */
                if (!(xEventGroupGetBits(s_event_group) & WIFI_CONNECTED_BIT)) break;
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
            if (rd <= 0) {
                ESP_LOGW(TAG, "SSE: read returned %d — reconnecting…", rd);
                break;   /* connection lost — reconnect */
            }

            buf_pos += rd;
            rx_buf[buf_pos] = '\0';

            /* RTDB SSE may use CRLF; normalize before searching for the
             * blank-line event separator, including split read boundaries. */
            int normalized = 0;
            for (int i = 0; i < buf_pos; ++i) {
                if (rx_buf[i] != '\r') rx_buf[normalized++] = rx_buf[i];
            }
            buf_pos = normalized;
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
                                 12288, NULL, tskIDLE_PRIORITY + 1,
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

    /* HTTPS must not run on the small esp_timer callback-task stack. */
    ret = start_token_refresh_task();
    if (ret != ESP_OK) return ret;

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
