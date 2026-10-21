idf_component_register(SRCS "toph_belt_main.c"
                            "vl53l5cx/vl53l5cx_api.c"
                            "vl53l5cx/platform.c"
                       PRIV_REQUIRES esp_driver_gpio esp_driver_i2c esp_driver_ledc esp_timer
                       INCLUDE_DIRS "vl53l5cx")
