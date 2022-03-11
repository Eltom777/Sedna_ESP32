#ifndef APP_WIFI_H_
#define APP_WIFI_H_
#include "freertos/queue.h"

typedef float (*fetch_telemetry_event_f)(void);
typedef void (*update_config_event_f)(float);
typedef void (*execute_feed_command_event_f)(void);

typedef struct {
    fetch_telemetry_event_f fetch_telemetry_event;
    update_config_event_f update_config_event;
    execute_feed_command_event_f feed_command_event; 
} mqtt_callback_t;

void esp_sta_init(mqtt_callback_t);

#endif