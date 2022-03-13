#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"

#include "app_wifi.h"

#include "driver/gpio.h"

#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"

#include "cJSON.h"

#define app_core 0

static QueueHandle_t data_queue;
static int queue_size = 10;

#define GPIO_DS18B20_0 16
#define DS18B20_RESOLUTION 12
#define SAMPLE_PERIOD 1000

#define ctrl_core 1

#define RELAY_PIN 4
#define GPIO_RELAY_PIN_SEL (1ULL << RELAY_PIN)

#define temp_threshold -5.0f

static DS18B20_Info * ds18b20_info;
static TickType_t last_wake_time;
static OneWireBus * owb;
static owb_rmt_driver_info rmt_driver_info;
static OneWireBus_ROMCode rom_code;
static owb_status status;

static float currentTemp;

static TaskHandle_t feeding_task_handler;

static const char *TAG = "CTRL_APP";
static int relayPower;

typedef struct device_config_t{
    //temperature system
    float desiredTemp;
    float currTemp;
    //feeder system
    int feedTime;
    bool feedNow; //force feed
    int foodLeft; //feedings remaining
    //wavermaker system
    int waveOnTime;
    int waveOffTime;
    bool waveForce; //forcing device to stay on/off
    //lighting system
    int lightOnTime;
    int lightOffTime;
    bool lightForce; //forcing device to stay on/off
    //water detectors
    bool waterLevel; //true if water level is low
    bool waterLeak; //true if water is detected
} device_config_t;
static device_config_t device_config;
static SemaphoreHandle_t device_config_mutex;

typedef enum FSMstates{
        Idle_State,
        Measure_State,
        Temp_Low_State,
        Temp_High_State,
        Turn_On_Relay_State,
        Turn_Off_Relay_State
} FSMstates;

void feed_command_event() {
    ESP_LOGI(TAG, "Resuming feeding task...");
    vTaskResume(feeding_task_handler);
}

float dequeue_telemetry()
{
    float data = temp_threshold;
    if(data_queue != NULL)
    {
        if((int) uxQueueMessagesWaiting(data_queue) > 0)
        {
            xQueueReceive(data_queue, &data, (TickType_t)0);
        }
        else
        {
            ESP_LOGI(TAG, "Telemetry Queue is empty...");
        }
    }
    else
    {
        ESP_LOGE(TAG, "Telemetry Queue is not set...");
    }
    return data;
}

static void enqueue_telemetry(void* pvParameters)
{

    if(data_queue == NULL)
    {
        ESP_LOGE(TAG, "Telemetry Queue is not set...");
        return;
    }

    for(;;)
    {
        ESP_LOGI(TAG, "Queueing the value : %f", currentTemp);
        xQueueSendToBack(data_queue, &currentTemp, (TickType_t)0);

        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }    
}

static void update_device_config_callback(char* new_device_config, size_t buffer_size) {
    
    //omit string termination char
    size_t buffer_size_parsing = buffer_size - 1;

    cJSON* root = cJSON_ParseWithLength(new_device_config, buffer_size_parsing);
    if(root != NULL) {
        char * rendered = cJSON_Print(root);
        ESP_LOGI(TAG, "%s\n", rendered);

        free(rendered);
        cJSON_Delete(root);
    }   
    else {
        ESP_LOGE(TAG, "JSON PARSER FAILED");
    }
}

static void init_hw(void){
    gpio_config_t io_conf;
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    io_conf.pin_bit_mask = GPIO_RELAY_PIN_SEL;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_set_level(RELAY_PIN, 0);

    // Create a 1-Wire bus, using the RMT timeslot driver
    owb = owb_rmt_initialize(&rmt_driver_info, GPIO_DS18B20_0, RMT_CHANNEL_1, RMT_CHANNEL_0);
    owb_use_crc(owb, true); // enable CRC check for ROM code

    // For a single device only:
    status = owb_read_rom(owb, &rom_code);
    if (status == OWB_STATUS_OK){
        char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];
        owb_string_from_rom_code(rom_code, rom_code_s, sizeof(rom_code_s));
        printf("Single device %s present\n", rom_code_s);
    }
    else{
        printf("An error occurred reading ROM code: %d", status);
    }

    ds18b20_info = ds18b20_malloc();
    printf("Single device optimisations enabled\n");
    ds18b20_init_solo(ds18b20_info, owb);
    ds18b20_use_crc(ds18b20_info, true); // enable CRC check on all reads
    ds18b20_set_resolution(ds18b20_info, DS18B20_RESOLUTION);

    last_wake_time = xTaskGetTickCount();
}

static float TempRead() {

        static float readTemp;
        ds18b20_convert_all(owb);
        ds18b20_wait_for_conversion(ds18b20_info);

        ds18b20_read_temp(ds18b20_info, &readTemp);
        printf("Temperature readings (degrees C): ");

        printf("%.1f\n", readTemp);

        vTaskDelayUntil(&last_wake_time, SAMPLE_PERIOD / portTICK_PERIOD_MS);
        ESP_LOGI(TAG,"Ram Left %d" ,xPortGetFreeHeapSize());
        return readTemp;
}

static void FSMTempCtrl(void* pvParameters)
{
    device_config_t* device_config =  (device_config_t*) pvParameters;
    enum FSMstates state = Idle_State;
    for(;;)
    {
        ESP_LOGI(TAG,"Device Desired Temp: %f" ,device_config->desiredTemp);
        switch(state){
            case Idle_State:
                state = Measure_State;
            break;
            case Measure_State:
                ESP_LOGI(TAG,"At Measure State");
                currentTemp = TempRead();
                if(currentTemp < device_config->desiredTemp){
                    ESP_LOGI(TAG,"Going to Low Temp State");
                    state = Temp_Low_State;
                }
                if(currentTemp >= device_config->desiredTemp){
                    ESP_LOGI(TAG,"Going to High Temp State");
                    state = Temp_High_State;
                }
            break;
            case Temp_Low_State:
                ESP_LOGI(TAG,"At Low Temp State");
                relayPower = gpio_get_level(RELAY_PIN);
                if(relayPower == 1)
                {
                    ESP_LOGI(TAG,"Going to Turn Relay On State");
                    state = Turn_On_Relay_State;
                }
                else 
                {
                    ESP_LOGI(TAG,"Going to Measure State.");
                    state = Measure_State;
                }
            break;
            case Temp_High_State:
                ESP_LOGI(TAG,"At High Temp State");
                relayPower = gpio_get_level(RELAY_PIN);
                if(relayPower ==  0)
                {
                    ESP_LOGI(TAG,"Going to Turn Relay Off State");
                    state = Turn_Off_Relay_State;
                }
                else 
                {
                    ESP_LOGI(TAG,"Going to Measure State.");
                    state = Measure_State;
                }
            break;
            case Turn_On_Relay_State:
                ESP_LOGI(TAG,"At Turn Relay On State");
                gpio_set_level(RELAY_PIN, 0);
                ESP_LOGI(TAG,"Relay is On!");
                ESP_LOGI(TAG,"Relay State %i" ,gpio_get_level(RELAY_PIN));
                ESP_LOGI(TAG,"Going to Measure State.");
                state = Measure_State;
            break;
            case Turn_Off_Relay_State:
                ESP_LOGI(TAG,"At Turn Relay Off State");
                gpio_set_level(RELAY_PIN, 1);
                ESP_LOGI(TAG,"Relay is Off!");
                ESP_LOGI(TAG,"Relay State %i" ,gpio_get_level(RELAY_PIN));
                ESP_LOGI(TAG,"Going to Measure State.");
                state = Measure_State;
            break;
        }
        vTaskDelayUntil(&last_wake_time, SAMPLE_PERIOD / portTICK_PERIOD_MS);
    }
}

static void feed_fish(void* pvParameters) {
    for(;;) {
        vTaskSuspend(NULL);
        ESP_LOGI(TAG, "Feeding fish...");
        //TODO: create and call servo control function
    }
}

void app_main()
{
    device_config.desiredTemp = 17.0;

    data_queue = xQueueCreate(queue_size, sizeof(float));  

    /*Start STA Wifi connection*/
    mqtt_callback_t mqtt_callback = 
    {
        .fetch_telemetry_event = dequeue_telemetry,
        .update_config_event = update_device_config_callback,
        .feed_command_event = feed_command_event
    };
    esp_sta_init(mqtt_callback);
    
    init_hw();
    xTaskCreatePinnedToCore(&FSMTempCtrl, 
                            "FSM run.", 
                            3*configMINIMAL_STACK_SIZE,
                            &device_config, 
                            5, 
                            NULL, 
                            ctrl_core);

    /* Create a task to queue data every 10 seconds. */ 
    xTaskCreatePinnedToCore(&enqueue_telemetry, 
                            "Enqueue Telemetry.", 
                            3*configMINIMAL_STACK_SIZE,
                            NULL, 
                            5, 
                            NULL, 
                            ctrl_core);

    /* Create a task to control feeding servo. */
    xTaskCreatePinnedToCore(&feed_fish, 
                            "Feed Fish.", 
                            configMINIMAL_STACK_SIZE,
                            NULL, 
                            6, 
                            &feeding_task_handler, 
                            ctrl_core);
    

    ESP_LOGI(TAG, "Program quits");
} 