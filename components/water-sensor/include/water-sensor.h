#ifndef WATER_SENSOR_H_
#define WATER_SENSOR_H_

static bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void adc_calibration_deinit(adc_cali_handle_t handle);
void adc_read_oneshot(void);

#endif // WATER_SENSOR_H_