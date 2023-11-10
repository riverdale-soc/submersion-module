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
#include "esp32/ulp.h"
#include "ulp_main.h"
#include "esp_adc/adc_oneshot.h"
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
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


// Base and end addresses of ULP coprocessor program binary blob
extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

/* This function is called once after power-on reset, to load ULP program into
 * RTC memory and configure the ADC.
 */
static void init_ulp_program(void);

/* This function is called every time before going into deep sleep.
 * It starts the ULP program and resets measurement counter.
 */
static void start_ulp_program(void);


static void init_ulp_program(void)
{
    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
            (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

    //-------------ADC1 Init---------------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_FSM,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Channel Config---------------//
    // Note: when changing channel here, also change 'adc_channel' constant in adc.S
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &config));

    /* Set low and high thresholds: approx 1.35V - 1.75V */
    ulp_low_thr = 0;
    ulp_high_thr = 300;

    rtc_gpio_init(25);
    rtc_gpio_set_direction(25, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_set_level(25,0);

    /* set ULP wake up period to 20 ms */
    ulp_set_wakeup_period(0, 20000);

    /* Disconnect GPIO12 and GPIO15 to remove current drain through 
       pullup/pulldown resistors 
       GPIO12 may be puled high to select flash voltage 
    */
    rtc_gpio_isolate(GPIO_NUM_12);
    rtc_gpio_isolate(GPIO_NUM_15);
    esp_deep_sleep_disable_rom_logging(); // suppress boot messages
}

static void start_ulp_program(void) 
{
    /* Reset sample counter */
    ulp_sample_counter = 0;

    /* Start the program */
    esp_err_t err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}



/**
    // Set latitude and longitude fields in the payload array
    payload->payload[0] = state;
    memcpy(&payload->payload[1], &latitude, sizeof(float));
    memcpy(&payload->payload[1 + sizeof(float)], &longitude, sizeof(float));

    // Calculate and set the CRC16 value
    payload->crc = calculateCRC(payload, sizeof(espnow_data_t) - sizeof(uint16_t));
*/ 
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
    if (cause != ESP_SLEEP_WAKEUP_ULP) {
        printf("Not ULP wakeup\n");
        init_ulp_program();
            // Initialize NVS
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK( nvs_flash_erase() );
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK( ret );
        // Start WiFi before using ESP-NOW
        wifi_init();
        // Initialize ESP-NOW and begin task
        espnow_init();
    } else {
        printf("Deep sleep wakeup\n");
        printf("ULP did %"PRIu32" measurements since last reset\n", ulp_sample_counter & UINT16_MAX);
        printf("Thresholds:  low=%"PRIu32"  high=%"PRIu32"\n", ulp_low_thr, ulp_high_thr);
        ulp_last_result &= UINT16_MAX;
        printf("Value=%"PRIu32" was %s threshold\n", ulp_last_result,
                ulp_last_result < ulp_low_thr ? "below" : "above");
    }
    printf("Entering deep sleep\n\n");
    start_ulp_program();
    ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );
    esp_deep_sleep_start();
}
