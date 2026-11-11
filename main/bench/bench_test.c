/*
 * bench_test.c
 *
 * Bench test: I2C/LCD, nearby 2.4 GHz AP scan, standalone Wi-Fi STA test.
 * Enabled from main.c via RUN_BENCH_LCD_WIFI. Holds the six connected relay
 * inputs LOW (OFF), does not start sensors or Firebase, and never returns.
 */

#include "bench_test.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"

#include "firebase_client.h"   /* WIFI_SSID, WIFI_PASS, FIREBASE_HOST */
#include "lcd_display.h"

static const char *TAG = "bench";

#define WIFI_UP_BIT       BIT0
#define WIFI_CONNECTING_BIT BIT1
#define WIFI_RETRY_SEC    5
#define WIFI_LOG_SEC      5
#define WIFI_SCAN_MAX_APS 32

/* -------------------------------------------------------------------------- */
/* LCD helpers                                                                */
/* -------------------------------------------------------------------------- */

/** Write a line padded/truncated to 20 columns so old text is overwritten. */
__attribute__((format(printf, 2, 3)))
static void lcd_line(uint8_t row, const char *fmt, ...)
{
    char tmp[64];
    char buf[LCD_COLS + 1];

    va_list ap;
    va_start(ap, fmt);
    vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);

    snprintf(buf, sizeof(buf), "%-*.*s", LCD_COLS, LCD_COLS, tmp);
    lcd_write_at(row, 0, buf);
}

/* -------------------------------------------------------------------------- */
/* Test 1 — I2C bus scan                                                      */
/* -------------------------------------------------------------------------- */

/** @return true if a device ACKs at LCD_I2C_ADDR. */
static bool i2c_scan(void)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port                     = I2C_NUM_0,
        .sda_io_num                   = GPIO_NUM_21,
        .scl_io_num                   = GPIO_NUM_22,
        .clk_source                   = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus = NULL;

    esp_err_t ret = i2c_new_master_bus(&cfg, &bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C scan: bus init failed: %s", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "I2C scan on SDA=21 SCL=22 (0x08..0x77)…");

    /* Every empty address logs a NACK error from the driver; silence it. */
    esp_log_level_set("i2c.master", ESP_LOG_NONE);

    bool lcd_found = false;
    int  count     = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (i2c_master_probe(bus, addr, 20) == ESP_OK) {
            ESP_LOGI(TAG, "  device found at 0x%02X", addr);
            count++;
            if (addr == LCD_I2C_ADDR) lcd_found = true;
        }
    }

    esp_log_level_set("i2c.master", ESP_LOG_INFO);
    i2c_del_master_bus(bus);

    if (count == 0) {
        ESP_LOGE(TAG, "I2C scan: NO device found — check SDA/SCL wiring, "
                      "VCC (5V) and GND of the LCD backpack");
    } else if (!lcd_found) {
        ESP_LOGE(TAG, "I2C scan: nothing at LCD_I2C_ADDR=0x%02X — set "
                      "LCD_I2C_ADDR to one of the addresses above "
                      "(PCF8574 = 0x20-0x27, PCF8574A = 0x38-0x3F)",
                 LCD_I2C_ADDR);
    } else {
        ESP_LOGI(TAG, "I2C scan: LCD backpack ACKs at 0x%02X", LCD_I2C_ADDR);
    }
    return lcd_found;
}

/* -------------------------------------------------------------------------- */
/* Test 2 — LCD patterns                                                      */
/* -------------------------------------------------------------------------- */

static void lcd_pattern_test(void)
{
    ESP_LOGI(TAG, "LCD: text pattern (4 rows x 20 cols, check nothing is cut/wrapped)");
    lcd_clear();
    lcd_set_backlight(true);
    lcd_write_at(0, 0, "01234567890123456789");
    lcd_write_at(1, 0, "ABCDEFGHIJKLMNOPQRST");
    lcd_write_at(2, 0, "abcdefghijklmnopqrst");
    lcd_write_at(3, 0, "AQUAHARU LCD 20x4 OK");
    vTaskDelay(pdMS_TO_TICKS(4000));

    ESP_LOGI(TAG, "LCD: full fill (every cell should show '#')");
    for (uint8_t r = 0; r < LCD_ROWS; r++) {
        lcd_write_at(r, 0, "####################");
    }
    vTaskDelay(pdMS_TO_TICKS(2500));

    ESP_LOGI(TAG, "LCD: corner markers");
    lcd_clear();
    lcd_write_at(0, 0,  "<");
    lcd_write_at(0, 19, ">");
    lcd_write_at(3, 0,  "[");
    lcd_write_at(3, 19, "]");
    lcd_write_at(1, 4,  "Corners: < > [ ]");
    vTaskDelay(pdMS_TO_TICKS(3000));

    ESP_LOGI(TAG, "LCD: backlight blink x3");
    for (int i = 0; i < 3; i++) {
        lcd_set_backlight(false);
        vTaskDelay(pdMS_TO_TICKS(500));
        lcd_set_backlight(true);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* -------------------------------------------------------------------------- */
/* Test 3 — Wi-Fi station                                                     */
/* -------------------------------------------------------------------------- */

static EventGroupHandle_t s_ev;
static volatile int       s_last_reason = 0;
static volatile unsigned  s_disconnects = 0;
static int64_t            s_attempt_started_us = 0;
static TickType_t         s_last_disconnect_tick = 0;
static char               s_ip[16]      = "";

static const char *reason_text(int reason)
{
    switch (reason) {
    case 2:   return "auth expired";
    case 8:   return "left by AP";
    case 15:  return "handshake timeout (password?)";
    case 200: return "beacon timeout";
    case 201: return "SSID not found (2.4 GHz only!)";
    case 202: return "auth failed (password?)";
    case 204: return "handshake timeout";
    case 205: return "connection failed";
    default:  return "see wifi_err_reason_t";
    }
}

static void wifi_evt(void *arg, esp_event_base_t base,
                     int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        /* Scan nearby APs first; wifi_start() connects after the scan. */

    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *d = data;
        s_last_reason = d->reason;
        s_disconnects++;
        s_last_disconnect_tick = xTaskGetTickCount();
        xEventGroupClearBits(s_ev, WIFI_UP_BIT | WIFI_CONNECTING_BIT);
        ESP_LOGW(TAG, "Wi-Fi disconnected, reason=%d (%s)",
                 (int)d->reason, reason_text(d->reason));

    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = data;
        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&e->ip_info.ip));
        s_last_reason = 0;
        xEventGroupClearBits(s_ev, WIFI_CONNECTING_BIT);
        xEventGroupSetBits(s_ev, WIFI_UP_BIT);
        ESP_LOGI(TAG, "Got IP: %s  gw: " IPSTR "  connect_ms=%lld",
                 s_ip, IP2STR(&e->ip_info.gw),
                 (long long)((esp_timer_get_time() - s_attempt_started_us) / 1000));
    }
}

static const char *auth_name(wifi_auth_mode_t mode)
{
    switch (mode) {
    case WIFI_AUTH_OPEN:          return "OPEN";
    case WIFI_AUTH_WEP:           return "WEP";
    case WIFI_AUTH_WPA_PSK:       return "WPA";
    case WIFI_AUTH_WPA2_PSK:      return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK:  return "WPA/WPA2";
    case WIFI_AUTH_WPA3_PSK:      return "WPA3";
    case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2/WPA3";
    default:                      return "OTHER";
    }
}

/* Bench-only scan before connecting. ESP-IDF returns APs strongest first and
 * releases its internal scan list when get_ap_records() is called. */
static void scan_nearby_aps(void)
{
    ESP_LOGI(TAG, "Scanning nearby 2.4 GHz Wi-Fi networks...");
    esp_err_t err = esp_wifi_scan_start(NULL, true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Wi-Fi scan failed: %s", esp_err_to_name(err));
        return;
    }

    uint16_t found = 0;
    err = esp_wifi_scan_get_ap_num(&found);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Cannot read scan count: %s", esp_err_to_name(err));
        esp_wifi_clear_ap_list();
        return;
    }
    if (found == 0) {
        ESP_LOGW(TAG, "No 2.4 GHz AP found in this scan");
        esp_wifi_clear_ap_list();
        return;
    }

    uint16_t count = found < WIFI_SCAN_MAX_APS ? found : WIFI_SCAN_MAX_APS;
    wifi_ap_record_t *aps = calloc(count, sizeof(*aps));
    if (!aps) {
        ESP_LOGE(TAG, "No memory for Wi-Fi scan records");
        esp_wifi_clear_ap_list();
        return;
    }
    err = esp_wifi_scan_get_ap_records(&count, aps);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Wi-Fi scan: %u found, showing %u strongest; password never shown",
                 (unsigned)found, (unsigned)count);
        ESP_LOGI(TAG, " #  RSSI  CH  SECURITY    BSSID              SSID");
        for (uint16_t i = 0; i < count; ++i) {
            char ssid[33];
            size_t length = 0;
            while (length < 32 && aps[i].ssid[length]) {
                uint8_t ch = aps[i].ssid[length];
                ssid[length++] = (ch >= 32 && ch <= 126) ? (char)ch : '?';
            }
            ssid[length] = '\0';
            ESP_LOGI(TAG, "%2u  %4d  %2u  %-11s %02x:%02x:%02x:%02x:%02x:%02x  %s",
                     (unsigned)(i + 1), aps[i].rssi, (unsigned)aps[i].primary,
                     auth_name(aps[i].authmode),
                     (unsigned)aps[i].bssid[0], (unsigned)aps[i].bssid[1],
                     (unsigned)aps[i].bssid[2], (unsigned)aps[i].bssid[3],
                     (unsigned)aps[i].bssid[4], (unsigned)aps[i].bssid[5],
                     length ? ssid : "<hidden>");
        }
    } else {
        ESP_LOGW(TAG, "Cannot read scan records: %s", esp_err_to_name(err));
        esp_wifi_clear_ap_list();
    }
    free(aps);
}

static bool start_connection(void)
{
    s_attempt_started_us = esp_timer_get_time();
    xEventGroupSetBits(s_ev, WIFI_CONNECTING_BIT);
    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK) {
        xEventGroupClearBits(s_ev, WIFI_CONNECTING_BIT);
        s_last_disconnect_tick = xTaskGetTickCount();
        ESP_LOGW(TAG, "Wi-Fi connect request failed: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

static bool wifi_start(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(ret));
        return false;
    }

    s_ev = xEventGroupCreate();
    if (!s_ev) return false;

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &wifi_evt, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               &wifi_evt, NULL));

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    wifi_config_t wifi_cfg = {0};
    strncpy((char *)wifi_cfg.sta.ssid,     WIFI_SSID, sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char *)wifi_cfg.sta.password, WIFI_PASS, sizeof(wifi_cfg.sta.password) - 1);
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_cfg.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    wifi_cfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    scan_nearby_aps();
    ESP_LOGI(TAG, "Wi-Fi scan finished; connecting to configured AP");
    start_connection();
    return true;
}

/** Resolve the Firebase host: proves DNS + internet, not just the AP link. */
static bool dns_check(void)
{
    struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_STREAM };
    struct addrinfo *res  = NULL;

    int rc = getaddrinfo(FIREBASE_HOST, "443", &hints, &res);
    if (rc != 0 || !res) {
        ESP_LOGE(TAG, "DNS: cannot resolve %s (rc=%d) — AP has no internet/DNS?",
                 FIREBASE_HOST, rc);
        return false;
    }

    char ip[16];
    inet_ntop(AF_INET, &((struct sockaddr_in *)res->ai_addr)->sin_addr,
              ip, sizeof(ip));
    ESP_LOGI(TAG, "DNS: %s -> %s", FIREBASE_HOST, ip);
    freeaddrinfo(res);
    return true;
}

/* -------------------------------------------------------------------------- */
/* Entry point                                                                */
/* -------------------------------------------------------------------------- */

void bench_test_lcd_wifi(void)
{
    ESP_LOGW(TAG, "=== BENCH TEST: I2C LCD 20x4 + Wi-Fi only ===");

    /* BENCH ONLY: the six wired relay inputs are active HIGH. Hold their
     * output latches LOW before enabling output mode; never start actuators. */
    const gpio_num_t relay_pins[] = {GPIO_NUM_19, GPIO_NUM_18, GPIO_NUM_5,
                                     GPIO_NUM_17, GPIO_NUM_16, GPIO_NUM_4};
    uint64_t relay_mask = 0;
    for (size_t i = 0; i < sizeof(relay_pins) / sizeof(relay_pins[0]); ++i) {
        ESP_ERROR_CHECK(gpio_set_level(relay_pins[i], 0));
        relay_mask |= 1ULL << relay_pins[i];
    }
    const gpio_config_t relay_config = {
        .pin_bit_mask = relay_mask, .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE, .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&relay_config));
    ESP_LOGI(TAG, "Six connected relay inputs held LOW (OFF)");

    /* ---- LCD ---- */
    i2c_scan();

    bool lcd_ok = (lcd_init() == 0);
    if (lcd_ok) {
        lcd_pattern_test();
    } else {
        ESP_LOGE(TAG, "lcd_init() failed — continuing with Wi-Fi test only");
    }

    /* ---- Wi-Fi ---- */
    if (strcmp(WIFI_SSID, "your-ssid") == 0) {
        ESP_LOGE(TAG, "WIFI_SSID/WIFI_PASS are still placeholders — set them "
                      "in firebase_client.h (do not commit them)");
        if (lcd_ok) {
            lcd_clear();
            lcd_line(0, "WiFi test SKIPPED");
            lcd_line(1, "Set WIFI_SSID/PASS");
            lcd_line(2, "in firebase_client.h");
        }
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (lcd_ok) {
        lcd_clear();
        lcd_line(0, "AQUAHARU WiFi Test");
        lcd_line(1, "Configured AP");
        lcd_line(2, "Scanning WiFi...");
    }

    if (!wifi_start()) {
        ESP_LOGE(TAG, "Wi-Fi start failed");
        if (lcd_ok) lcd_line(2, "WiFi START FAILED");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    int  sec        = 0;
    int  attempts   = 1;
    bool dns_done   = false;
    bool dns_ok     = false;

    while (1) {
        bool up = (xEventGroupGetBits(s_ev) & WIFI_UP_BIT) != 0;

        if (up) {
            wifi_ap_record_t ap;
            esp_err_t ap_err = esp_wifi_sta_get_ap_info(&ap);
            bool ap_ok = ap_err == ESP_OK;

            if (!dns_done || (!dns_ok && sec % WIFI_LOG_SEC == 0)) {
                dns_ok   = dns_check();
                dns_done = true;
            }

            if (lcd_ok) {
                if (ap_ok) {
                    lcd_line(2, "%ddBm CH%d DNS:%s", ap.rssi, ap.primary,
                             dns_ok ? "OK" : "FAIL");
                } else {
                    lcd_line(2, "WiFi UP DNS:%s", dns_ok ? "OK" : "FAIL");
                }
                lcd_line(3, "IP:%s", s_ip);
            }
            if (sec % WIFI_LOG_SEC == 0) {
                if (ap_ok) {
                    ESP_LOGI(TAG, "Wi-Fi UP ip=%s rssi=%d dBm ch=%u "
                             "bssid=%02x:%02x:%02x:%02x:%02x:%02x "
                             "dns=%s disconnects=%u",
                             s_ip, ap.rssi, (unsigned)ap.primary,
                             (unsigned)ap.bssid[0], (unsigned)ap.bssid[1],
                             (unsigned)ap.bssid[2], (unsigned)ap.bssid[3],
                             (unsigned)ap.bssid[4], (unsigned)ap.bssid[5],
                             dns_ok ? "ok" : "FAIL", s_disconnects);
                } else {
                    ESP_LOGW(TAG, "Wi-Fi has IP but AP info unavailable: %s",
                             esp_err_to_name(ap_err));
                }
            }
        } else {
            dns_done = false;

            if (lcd_ok) {
                lcd_line(2, "Connecting #%d", attempts);
                if (s_last_reason) lcd_line(3, "reason=%d", s_last_reason);
            }
            TickType_t now = xTaskGetTickCount();
            bool connecting = (xEventGroupGetBits(s_ev) & WIFI_CONNECTING_BIT) != 0;
            if (!connecting &&
                (TickType_t)(now - s_last_disconnect_tick) >= pdMS_TO_TICKS(WIFI_RETRY_SEC * 1000)) {
                attempts++;
                ESP_LOGI(TAG, "Wi-Fi retry #%d (last reason=%d)",
                         attempts, s_last_reason);
                start_connection();
            }
        }

        sec++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
