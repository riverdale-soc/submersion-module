#include <stdio.h>
#include <string.h>
#include "gps_parser.h"
#include "lwgps.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
/**
 * This example uses direct processing function
 * to process dummy NMEA data from GPS receiver
 */

/* GPS handle */
lwgps_t hgps;

/**
 * \brief           Dummy data from GPS receiver
 */
const char gps_rx_data[] = ""
                           "$GPRMC,183729,A,3907.356,N,12102.482,W,000.0,360.0,080301,015.5,E*6F\r\n"
                           "$GPRMB,A,,,,,,,,,,,,V*71\r\n"
                           "$GPGGA,183730,3907.356,N,12102.482,W,1,05,1.6,646.4,M,-24.1,M,,*75\r\n"
                           "$GPGSA,A,3,02,,,07,,09,24,26,,,,,1.6,1.6,1.0*3D\r\n"
                           "$GPGSV,2,1,08,02,43,088,38,04,42,145,00,05,11,291,00,07,60,043,35*71\r\n"
                           "$GPGSV,2,2,08,08,02,145,00,09,46,303,47,24,16,178,32,26,18,231,43*77\r\n"
                           "$PGRME,22.0,M,52.9,M,51.0,M*14\r\n"
                           "$GPGLL,3907.360,N,12102.481,W,183730,A*33\r\n"
                           "$PGRMZ,2062,f,3*2D\r\n"
                           "$PGRMM,WGS84*06\r\n"
                           "$GPBOD,,T,,M,,*47\r\n"
                           "$GPRTE,1,1,c,0*07\r\n"
                           "$GPRMC,183731,A,3907.482,N,12102.436,W,000.0,360.0,080301,015.5,E*67\r\n"
                           "$GPRMB,A,,,,,,,,,,,,V*71\r\n";

int parse() {
    /* Init GPS */
    lwgps_init(&hgps);

    /* Process all input data */
    lwgps_process(&hgps, gps_rx_data, strlen(gps_rx_data));

    /* Print messages */
    printf("Valid status: %d\r\n", hgps.is_valid);
    printf("Latitude: %f degrees\r\n", hgps.latitude);
    printf("Longitude: %f degrees\r\n", hgps.longitude);
    printf("Altitude: %f meters\r\n", hgps.altitude);

    return 0;
}

/**
 * @brief Initialize Mailbox Queue
 * @returns QueueHandle_T: Handle to initialize queue  
 */
QueueHandle_t vQueueInit(void)
{   
    return xQueueCreate(1, sizeof(hgps));
}

/**
 * @brief Write a value to queue
 * 
 * @param Queue : queue handle of type QueueHandle_t
 * @param ulNewValue : uin16_t value to write
 */
void vUpdateQueue(QueueHandle_t Queue, lwgps_t pxData)
{
    xQueueOverwrite(Queue, &pxData);
}

/**
 * @brief Read queue and pop value read, 
 * 
 * @param pxData : Pointer to struct of type weather_data to read into
 * @param Queue  : Queue handle of type QueueHandle_t to read from
 */
void vReadQueue(lwgps_t *pxData, QueueHandle_t Queue)
{
    xQueuePeek(Queue, pxData,  pdMS_TO_TICKS(100));
}