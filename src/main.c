#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"

#include "app_wifi.h"

#include "driver/gpio.h"

#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"

#define app_core 0

static QueueHandle_t data_queue;
static int queue_size = 10;

#define GPIO_DS18B20_0 16
#define DS18B20_RESOLUTION 12
#define SAMPLE_PERIOD 1000

#define ctrl_core 1

#define RELAY_PIN 4
#define GPIO_RELAY_PIN_SEL (1ULL << RELAY_PIN)

static DS18B20_Info * ds18b20_info;
static TickType_t last_wake_time;
static OneWireBus * owb;
static owb_rmt_driver_info rmt_driver_info;
static OneWireBus_ROMCode rom_code;
static owb_status status;

static float currentTemp;
static float desiredTemp = 26;
static const char *TAG = "CTRL_APP";
static int relayPower;

typedef struct device_config_t{
    float desiredTemp;
} device_config_t;

static void queue_telemetry_data(void *pvParameters)
{
    for(;;)
    {   
        xQueueSendToBack(telemetry_queue, &currentTemp, (TickType_t)0);
        
        vTaskDelay(30000 / portTICK_RATE_MS);
    }
}

typedef enum FSMstates{
        Idle_State,
        Measure_State,
        Temp_Low_State,
        Temp_High_State,
        Turn_On_Relay_State,
        Turn_Off_Relay_State
}FSMstates;

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

static float TempRead(){

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
        ESP_LOGI(TAG, "Desired Temp: %p", device_config);

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
                if(relayPower == 1){
                    ESP_LOGI(TAG,"Going to Turn Relay On State");
                    state = Turn_On_Relay_State;
                }else {
                    ESP_LOGI(TAG,"Going to Measure State.");
                    state = Measure_State;
                }
            break;
            case Temp_High_State:
                ESP_LOGI(TAG,"At High Temp State");
                relayPower = gpio_get_level(RELAY_PIN);
                if(relayPower ==  0){
                    ESP_LOGI(TAG,"Going to Turn Relay Off State");
                    state = Turn_Off_Relay_State;
                }else {
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

void app_main()
{
    /*TODO: figure out a way to share device config with 
    iotc_mqttlogic_subscribe_callback inside app_wifi.*/
    device_config_t* device_config = malloc(sizeof(device_config_t));
    device_config->desiredTemp = 26.0;
    
    ESP_LOGI(TAG, "Init Desired Temp: %p", device_config);

    data_queue = xQueueCreate(queue_size, sizeof(float));  

    /*Start STA Wifi connection*/
    esp_sta_init(data_queue);
    
    init_hw();
    xTaskCreatePinnedToCore(&FSMTempCtrl, 
                            "FSM run.", 
                            3*configMINIMAL_STACK_SIZE,
                            &device_config, 
                            5, 
                            NULL, 
                            ctrl_core);
    
    /* Create a task to queue data every 30 seconds. */
    xTaskCreatePinnedToCore(
                    &queue_telemetry_data,         
                    "Data read",        
                    configMINIMAL_STACK_SIZE,    
                    NULL,               
                    5,                  
                    NULL,               
                    app_core);

    free(device_config);
    ESP_LOGI(TAG, "Program quits");
} 