idf_component_register(SRCS "submersion_module.c"
                    INCLUDE_DIRS "."
                    REQUIRES nmea_parser espnow-lib gps_control touch_sensor gps_parser soc nvs_flash ulp driver esp_adc esp_event esp_netif esp_wifi)
