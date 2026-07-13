idf_component_register(SRCS "actuators.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES driver esp_timer esp_driver_gpio)
