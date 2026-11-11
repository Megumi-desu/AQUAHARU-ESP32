/*
 * firebase_client.h
 *
 * WiFi Station + Firebase Realtime Database Client for AQUAHARU
 *
 * Features:
 *   - Wi-Fi STA mode with auto-reconnect
 *   - Firebase Auth (email/password) with automatic 50-min token refresh
 *   - PUT telemetry to /aquaharu/v1/devices/<device_id>/live.json
 *   - POST history entries to /aquaharu/v1/devices/<device_id>/history.json
 *   - PUT heartbeat to /aquaharu/v1/devices/<device_id>/heartbeat.json
 *   - SSE stream for remote config changes
 *
 * Credentials are configured via sdkconfig (menuconfig) or the macros below.
 */

#ifndef FIREBASE_CLIENT_H
#define FIREBASE_CLIENT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* WiFi Credentials (override via menuconfig or -D flags)                     */
/* -------------------------------------------------------------------------- */

#ifndef WIFI_SSID
#define WIFI_SSID  "rrrah"
#endif

#ifndef WIFI_PASS
#define WIFI_PASS  "bocchi_no1ikz"
#endif

/* -------------------------------------------------------------------------- */
/* Firebase Configuration                                                      */
/* -------------------------------------------------------------------------- */

#define FIREBASE_HOST    "aquaharu-9436f-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_AUTH    "identitytoolkit.googleapis.com"
#define FIREBASE_API_KEY "AIzaSyCXkmK0nqHUMXVbKg-lrCbejstf0LoWI50"

/* Stable non-secret ID for the current prototype board. Set a unique value
 * for each board. Product provisioning should assign and persist this in NVS. */
#define FIREBASE_DEVICE_ID   "device_01"
#define FIREBASE_DEVICE_ROOT "aquaharu/v1/devices"

#define FIREBASE_EMAIL  "rachmatgifari99@gmail.com"
#define FIREBASE_PASS   "bocchi1324"

/** Token refresh interval in minutes. */
#define FIREBASE_TOKEN_REFRESH_MIN  50

/* -------------------------------------------------------------------------- */
/* Types                                                                       */
/* -------------------------------------------------------------------------- */

/** Callback invoked when a new config JSON is received via SSE. */
typedef void (*firebase_config_cb_t)(const char *json_str);

/** Opaque handle to the persistent SSE task (used to stop it). */
typedef void *firebase_sse_handle_t;

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialise Wi-Fi station, connect, and authenticate with Firebase.
 *
 * Blocks until Wi-Fi is connected and the initial idToken is obtained.
 *
 * @return 0 (ESP_OK) on success, or an ESP_ERR_* code on failure.
 */
int firebase_client_init(void);

/**
 * @brief Wait for the client to be fully ready (Wi-Fi connected + authenticated).
 *
 * @param timeout_ms  Maximum time to wait in milliseconds.
 * @return true if ready, false on timeout.
 */
bool firebase_client_wait_ready(uint32_t timeout_ms);

/**
 * @brief PUT (overwrite) a JSON payload to the live telemetry path.
 *
 * URL: /aquaharu/v1/devices/<device_id>/live.json?auth=<token>
 *
 * @param json_payload  Null-terminated JSON string.
 * @return 0 on success (HTTP 2xx), or an ESP_ERR_* code.
 */
int firebase_put_telemetry(const char *json_payload);

/** As above, with per-request HTTP status (0 = no HTTP response).
 * ESP_OK still requires a complete, valid JSON response, not only HTTP 2xx. */
int firebase_put_telemetry_with_status(const char *json_payload, int *http_status);

/**
 * @brief POST (append) a JSON payload to the history path.
 *
 * URL: /aquaharu/v1/devices/<device_id>/history.json?auth=<token>
 *
 * @param json_payload  Null-terminated JSON string.
 * @return 0 on success (HTTP 2xx), or an ESP_ERR_* code.
 */
int firebase_post_history(const char *json_payload);

/** Update the server timestamp used by the Web App's online indicator. */
int firebase_put_heartbeat(void);
int firebase_put_heartbeat_with_status(int *http_status);

/** Wi-Fi has obtained an IP address, independent of Auth status. */
bool firebase_wifi_is_connected(void);

/**
 * @brief Start the Server-Sent Events (SSE) config stream.
 *
 * Opens a persistent HTTPS connection to:
 *   /aquaharu/v1/devices/<device_id>/config.json?auth=<token>
 *
 * When a `data:` event arrives the registered callback is invoked with
 * the raw JSON payload.  The caller is responsible for parsing the JSON
 * and updating sharedConfig under a mutex.
 *
 * @param cb  Callback to receive config JSON strings (or NULL to stop).
 * @return An opaque handle, or NULL on failure.
 */
firebase_sse_handle_t firebase_start_config_stream(firebase_config_cb_t cb);

/**
 * @brief Stop the SSE config stream and free its resources.
 *
 * @param handle  Handle returned by firebase_start_config_stream().
 */
void firebase_stop_config_stream(firebase_sse_handle_t handle);

/**
 * @brief Return the current Firebase ID token.
 *
 * @return Null-terminated token string, or NULL if not yet authenticated.
 */
const char *firebase_get_token(void);

/**
 * @brief Return the configured stable device ID (not an authentication secret).
 */
const char *firebase_get_device_id(void);

/**
 * @brief Check whether Wi-Fi is connected and a valid token is held.
 */
bool firebase_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* FIREBASE_CLIENT_H */
