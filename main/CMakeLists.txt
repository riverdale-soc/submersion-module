idf_component_register(SRCS "submersion_module.c"
                    INCLUDE_DIRS "."
                    REQUIRES nmea_parser gps_control soc esp_timer nvs_flash ulp driver esp_adc esp_event esp_netif esp_wifi)
