#ifndef GPS_CONTROL_H
#define GPS_CONTROL_H

#include <stdio.h>
#include "driver/gpio.h"    
#include "gps_control.h"

// Allocate GPIO32 - GPS Enable as an GPIO Output
#define GPS_ENABLE 32

/**
 * @brief  Initialize GPS Enable Pin
 * 
 */
static void gps_enable_init(void);

/**
 * @brief  Power on GPS
 * 
 */
static void gps_power_on(void);

/**
 * @brief  Power off GPS
 * 
 */
static void gps_power_off(void);

#endif /* GPS_CONTROL_H */
