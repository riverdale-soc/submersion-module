/**
 * @file water-sensor.c
 * @author Dmitri Lyalikov (Dlyalikov01@Manhattan.edu)
 * @brief ADC Driver and Calibration for Water Sensor
 * @version 0.1
 * @date 2023-10-06
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "water-sensor.h"

const static char *TAG = "ADC_CALI";

/*---------------------------------------------------------------
        ADC General Macros
---------------------------------------------------------------*/
//ADC1 Channels
#if CONFIG_IDF_TARGET_ESP32
#define ADC1_CHAN0          ADC_CHANNEL_4
#define ADC1_CHAN1          ADC_CHANNEL_5
#else
#define ADC1_CHAN0          ADC_CHANNEL_2
#define ADC1_CHAN1          ADC_CHANNEL_3
#endif

#if (SOC_ADC_PERIPH_NUM >= 2)
//ADC2 Channels
#if CONFIG_IDF_TARGET_ESP32
#define ADC2_CHAN0          ADC_CHANNEL_0
#else
#define ADC2_CHAN0          ADC_CHANNEL_0
#endif
#endif

static int adc_raw[2][10];
static int voltage[2][10];

/**
 * @brief ADC Read
 */
void adc_read_oneshot(void)
{
    //-------------ADC1 Init---------------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_CHAN0, &config));

    
    adc_cali_handle_t adc1_cali_handle = NULL;
    bool do_calibration = adc_calibration_init(ADC_UNIT_1, ADC_ATTEN_DB_11, &adc1_cali_handle);

    while(1) {
        ESP_LOGI(TAG, "ADC1 CHAN0 Raw: %d", adc_oneshot_read(adc1_handle, ADC1_CHAN0, &adc_raw[0][0]));
        if (do_calibration) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw[0][0], &voltage[0][0]));
            ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, ADC1_CHAN0, voltage[0][0]);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Tear Down
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    if (do_calibration) {
        adc_calibration_deinit(adc1_cali_handle);
    }
}

/**
 * @brief Calibration Init
 * 
 * @param unit unit of ADC (ADC_UNIT_1 or ADC_UNIT_2)
 * @param atten set attenuation of ADC
 * @param out_handle output calibration handle
 * @return true 
 * @return false 
 */
static bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if(!calibrated) {
        ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Scheme: Curve Fitting", unit + 1, ADC1_CHAN0);
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif
    *out_handle = handle;   
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "CALIBRATION SUCCESSFUL");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}


    static void adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}