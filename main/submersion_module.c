/**
 * @file submersion_module.c
 * Author: Dmitri Lyalikov (Dlyalikov01@Manhattan.edu)
 * Date: 10/6/2023
 * Revision: 0.1
 * @brief This is the entry point and main application for the submersion module.
 * It will read from a water sensor and send the data to a receiver over ESP-NOW protocol.
 * 
 * 
 * 
 *   In order to get the MAC address of the other device, Device1 firstly send broadcast ESPNOW data with 'state' set as 0.
 *   When Device2 receiving broadcast ESPNOW data from Device1 with 'state' as 0, adds Device1 into the peer list. 
 *   Then start sending broadcast ESPNOW data with 'state' set as 1.
 *   When Device1 receiving broadcast ESPNOW data with 'state' as 1, compares the local magic number with that in the data. 
 *   If the local one is bigger than that one, stop sending broadcast ESPNOW data and starts sending unicast ESPNOW data to Device2.
 *   If Device2 receives unicast ESPNOW data, also stop sending broadcast ESPNOW data.
*
*/

#include <stdlib.h>
#include <time.h>
#include <inttypes.h>
#include "esp_sleep.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/dac.h"
#include "esp_adc/adc_oneshot.h"
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "submersion_module.h"
#include "gps_parser.h"
#include "nmea_parser.h"
#include "touch_sensor.h"
#include "gps_control.h"
#include "espnow-lib.h"

// Submersion Module GPIO Pin on ESP32-PICO-D4 (GPIO25)
static const int ext_wakeup_pin_0 = 25;

static const char *TAG = "Submersion Module";

// define gps-payload struct that holds longitude and latitude
typedef struct {
    float longitude;
    float latitude;
} gps_payload_t;

// Queue functions that hold GPS longitude and latitude data
QueueHandle_t gps_queue;

// Queue functions that hold GPS longitude and latitude data
static void gps_queue_init(void)
{
    gps_queue = xQueueCreate(1, sizeof(gps_payload_t));
    assert(gps_queue != NULL);
}

// Queue functions that hold GPS longitude and latitude data
static void gps_queue_deinit(void)
{
    vQueueDelete(gps_queue);
}

// Queue functions that hold GPS longitude and latitude data
static void gps_queue_send(gps_payload_t *payload)
{
    xQueueSend(gps_queue, payload, portMAX_DELAY);
}

// Queue functions that hold GPS longitude and latitude data
static void gps_queue_receive(gps_payload_t *payload)
{
    xQueueReceive(gps_queue, payload, portMAX_DELAY);
}

// Queue functions that hold GPS longitude and latitude data
static void gps_queue_reset(void)
{
    xQueueReset(gps_queue);
}

// Queue functions that hold GPS longitude and latitude data
static void gps_queue_print(void)
{
    gps_payload_t payload;
    gps_queue_receive(&payload);
    ESP_LOGI(TAG, "GPS Longitude: %f", payload.longitude);
    ESP_LOGI(TAG, "GPS Latitude: %f", payload.latitude);
}

/**
 * @brief GPS Event Handler
 *
 * @param event_handler_arg handler specific arguments
 * @param event_base event base, here is fixed to ESP_NMEA_EVENT
 * @param event_id event id
 * @param event_data event specific arguments
 */
static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    gps_t *gps = NULL;
    switch (event_id) {
    case GPS_UPDATE:
        gps = (gps_t *)event_data;
        // construct payload struct
        gps_payload_t payload;
        payload.longitude = gps->longitude;
        payload.latitude = gps->latitude;
        // send payload struct to queue
        gps_queue_send(&payload);
        break;
    case GPS_UNKNOWN:
        /* print unknown statements */
        ESP_LOGW(TAG, "Unknown statement:%s", (char *)event_data);
        break;
    default:
        break;
    }
}


static void config_ext0_wakeup(void)
{
    ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(ext_wakeup_pin_0, 1));
    // Configure pullup/downs via RTCIO to tie wakeup pins to inactive level during deepsleep.
    // EXT0 resides in the same power domain (RTC_PERIPH) as the RTC IO pullup/downs.
    // No need to keep that power domain explicitly, unlike EXT1.
    ESP_ERROR_CHECK(rtc_gpio_pullup_dis(ext_wakeup_pin_0));
    ESP_ERROR_CHECK(rtc_gpio_pulldown_en(ext_wakeup_pin_0));

    // Isolate GPIO12 pin from external circuits. This is needed for modules
    // which have an external pull-up resistor on GPIO12 (such as ESP32-WROVER)
    // to minimize current consumption.
    rtc_gpio_isolate(GPIO_NUM_12);
}
/**
 * @brief Main application entry point. Starts WiFi and ESP-NOW.
 * 
 * Submersion Module State Machine
 *  1: Check wake up cause
 *      1a: If wake up cause is not ULP, initialize ULP program and load it into RTC memory, enter deep sleep
 *      1b: If wake up cause is ULP, initialize WiFi and ESP-NOW, construct and send MOB payload, start GPS event handler 
 */
void app_main(void)
{
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause != ESP_SLEEP_WAKEUP_EXT0) {
        printf("Regular Reset Wake\n");
        // Initialize NVS
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK( nvs_flash_erase() );
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK( ret );

    } else {
        // Turn on NEO-7M GPS module gated power supply
        gps_enable_init();
        gps_power_on();
        // Initialize GPS parser library
        /* NMEA parser configuration */
        gps_queue_init();
        nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();
        /* init NMEA parser library */
        nmea_parser_handle_t nmea_hdl = nmea_parser_init(&config);
        nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL);
        // Start WiFi before using ESP-NOW
        wifi_init();
        // Initialize ESP-NOW and begin task
        espnow_init();
    }
    // Enter Deep Sleep
    config_ext0_wakeup();
    esp_deep_sleep_start();
}
