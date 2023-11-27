#include <stdio.h>
#include "driver/gpio.h"
#include "gps_control.h"

void gps_enable_init(void) 
{
    // Enable GPS
    gpio_reset_pin(GPS_ENABLE);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPS_ENABLE, GPIO_MODE_OUTPUT);
    gpio_set_level(GPS_ENABLE, 0);
}

void gps_power_on(void)
{
    gpio_set_level(GPS_ENABLE, 1);
}

void gps_power_off(void)
{
    gpio_set_level(GPS_ENABLE, 0);
}
