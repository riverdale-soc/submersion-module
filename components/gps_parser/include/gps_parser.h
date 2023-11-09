#ifndef GPS_PARSER_H
#define GPS_PARSER_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "lwgps.h"

int parse();

void vReadQueue(lwgps_t *pxData, QueueHandle_t Queue);
void vUpdateQueue(QueueHandle_t Queue, lwgps_t pxData);
QueueHandle_t vQueueInit(void);


#endif // GPS_PARSER_H