#ifndef APP_WIFI_H_
#define APP_WIFI_H_
#include "freertos/queue.h"

void esp_sta_init(QueueHandle_t);

extern QueueHandle_t telemetry_queue;

#endif