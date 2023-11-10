#ifndef TOUCH_SENSOR_H
#define TOUCH_SENSOR_H

#include <stdio.h>
#include "touch_sensor.h"
#include "driver/touch_pad.h"
#include "esp_sleep.h"

#define TOUCH_THRESH_NO_USE 0

/**
 * @brief  Register touch pad interrupt function, which will print a message on console when touch pad is pressed.
 * 
 * @param pad 
 */
static void calibrate_touch_pad(touch_pad_t pad);

/**
 * @brief  Register touch pad interrupt function, which will print a message on console when touch pad is pressed.
 * 
 */
void deep_sleep_register_touch_wakeup(void);

#endif /* TOUCH_SENSOR_H */
