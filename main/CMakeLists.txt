idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES temp_sensor ph_sensor tds_sensor actuators fuzzy_control firebase_client lcd_display esp_driver_gpio esp_adc espressif__cjson)
