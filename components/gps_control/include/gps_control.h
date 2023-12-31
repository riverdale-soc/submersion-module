#ifndef GPS_CONTROL_H
#define GPS_CONTROL_H

#include <stdio.h>
#include "driver/gpio.h"    
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "gps_control.h"

// define gps-payload struct that holds longitude and latitude
typedef struct {
    float longitude;
    float latitude;
} gps_payload_t;

// Allocate GPIO32 - GPS Enable as an GPIO Output
#define GPS_ENABLE 32

/**
 * @brief  Initialize GPS Enable Pin
 * 
 */
void gps_enable_init(void);

/**
 * @brief  Power on GPS
 * 
 */
void gps_power_on(void);

/**
 * @brief  Power off GPS
 * 
 */
void gps_power_off(void);


#endif /* GPS_CONTROL_H */
