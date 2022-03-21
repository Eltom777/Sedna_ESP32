#ifndef APP_WIFI_H_
#define APP_WIFI_H_
#include "freertos/queue.h"

typedef struct device_telemetry_t {
    float current_temp;
    int food_count;
} device_telemetry_t;

typedef void (*fetch_telemetry_event_f)(struct device_telemetry_t*);
typedef bool (*fetch_switchSW_event_f)(void);
typedef bool(*fetch_waterlvl_event_f)(void);
typedef void (*update_config_event_f)(char*, size_t);
typedef void (*execute_feed_command_event_f)(void);
typedef void (*execute_refill_command_event_f)(void);

typedef struct {
    fetch_telemetry_event_f fetch_telemetry_event;
    fetch_switchSW_event_f fetch_floatSW_event;
    fetch_waterlvl_event_f fetch_waterlvl_event;
    update_config_event_f update_config_event;
    execute_feed_command_event_f feed_command_event;
    execute_refill_command_event_f refill_command_event; 
} mqtt_callback_t;

void esp_sta_init(mqtt_callback_t);

#endif