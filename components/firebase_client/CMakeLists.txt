idf_component_register(SRCS "firebase_client.c"
                       INCLUDE_DIRS "include"
                       EMBED_TXTFILES "google_roots.pem"
                       PRIV_REQUIRES driver esp_wifi esp_event esp_netif esp_http_client esp_timer nvs_flash espressif__cjson mbedtls)
