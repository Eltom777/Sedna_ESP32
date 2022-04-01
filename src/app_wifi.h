#ifndef APP_WIFI_H_
#define APP_WIFI_H_
#include "freertos/queue.h"

typedef struct telemtry_message_t {
    float current_temp;
    int food_count;
    bool low_level_switch;
    bool water_leak;
} telemtry_message_t;

typedef bool (*fetch_telemetry_event_f)(struct telemtry_message_t*);
typedef void (*update_config_event_f)(char*, size_t);
typedef void (*execute_feed_command_event_f)(void);
typedef void (*execute_refill_command_event_f)(void);

typedef struct {
    fetch_telemetry_event_f fetch_telemetry_event;
    update_config_event_f update_config_event;
    execute_feed_command_event_f feed_command_event;
    execute_refill_command_event_f refill_command_event; 
} mqtt_callback_t;

void esp_sta_init(mqtt_callback_t);

#endif