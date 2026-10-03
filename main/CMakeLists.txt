idf_component_register(SRCS "main.c" "ota.c" "ota_file_parser.c"
                       PRIV_INCLUDE_DIRS "."
                       PRIV_REQUIRES shutter nvs_flash esp_driver_gpio esp_timer esp_pm app_update esp_app_format)
