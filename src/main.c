#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"

#include "app_wifi.h"

#define app_core 0
#define DUMMY_TEMP 25.0

static QueueHandle_t data_queue;
static int queue_size = 10;


typedef struct {
    float min_temp;
    float max_temp;
} device_config_t;

static void send_dummy_values(void *pvParameters)
{
    float data;
    for(;;)
    {
        data = DUMMY_TEMP + rand() % 10;
        xQueueSendToBack(telemetry_queue, &data, (TickType_t)0);
        
        vTaskDelay(500 / portTICK_RATE_MS);
    }
}

void app_main()
{
    /*TODO: figure out a way to share device config with 
    iotc_mqttlogic_subscribe_callback inside app_wifi.*/
    device_config_t device_config = 
    {
        .min_temp = 25.0,
        .max_temp = 26.0
    };

    data_queue = xQueueCreate(queue_size, sizeof(float));  

    /*Start STA Wifi connection*/
    esp_sta_init(data_queue);

    /* Create a dummy task to publish data every 10 seconds. */
    xTaskCreatePinnedToCore(
                    &send_dummy_values,         
                    "Generate Dummy Data",        
                    configMINIMAL_STACK_SIZE,    
                    NULL,               
                    5,                  
                    NULL,               
                    app_core);
} 