idf_component_register(SRCS "shutter.c" "inputs.c" "sense.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES nvs_flash esp_driver_gpio esp_timer esp_pm esp_adc)
