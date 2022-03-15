#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"

#include "hw_definition.h"
#include "gpio_definition.h"

#include "app_wifi.h"
#include "driver/gpio.h"

#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"

#include "cJSON.h"
#include <sys/time.h>
// #include "led_strip.h"

/* Global Variables----------------------------------------------------------------------------------------------------------*/
//ESPI_LOG Tag
static const char *TAG = "CTRL_APP";

//Com btwn app-wifi cores
static QueueHandle_t data_queue;
static int queue_size = 10;

//Temp sub-system
static DS18B20_Info * ds18b20_info;
static TickType_t last_wake_time;
static OneWireBus * owb;
static owb_rmt_driver_info rmt_driver_info;
static OneWireBus_ROMCode rom_code;
static owb_status status;
static float currentTemp;
static int relayPower;

typedef enum FSMstates{
        Idle_State,
        Measure_State,
        Temp_Low_State,
        Temp_High_State,
        Turn_On_Relay_State,
        Turn_Off_Relay_State
} FSMstates;

//Planner sub-system
#define LOCAL_TIME_ZONE "EST5EDT,M3.2.0/2,M11.1.0"
#define ONE_SEC_SPEED_UP 700 // Speed up a sec by 300ms so deadline is not missed
#define ONE_MINUTE 60000L

enum planner_states_t
{
    WAIT_FOR_SET_TIME,
    CHECK_TIME,
    SLEEP
};
static TaskHandle_t planner_task_handler;
static int ONE_MINUTE_SPEED_UP = 60 * ONE_SEC_SPEED_UP;

static struct timeval time_now = {0};
static struct tm timeinfo_now = {0};

static struct tm timeinfo_planner_task = {0};

// //LED brightness settings
// static const rgb_t led_brightness[] = {
//     { .r = 0x00, .g = 0x00, .b = 0x00 },
//     { .r = 0x01, .g = 0x01, .b = 0x01 },
//     { .r = 0x03, .g = 0x03, .b = 0x03 },
//     { .r = 0x05, .g = 0x05, .b = 0x05 },
//     { .r = 0x06, .g = 0x06, .b = 0x06 },
//     { .r = 0x08, .g = 0x08, .b = 0x08 },
//     { .r = 0x0a, .g = 0x0a, .b = 0x0a },
//     { .r = 0x0c, .g = 0x0c, .b = 0x0c },
//     { .r = 0x0e, .g = 0x0e, .b = 0x0e },
//     { .r = 0x0f, .g = 0x0f, .b = 0x0f }
// };
// #define COLORS_TOTAL (sizeof(led_brightness) / sizeof(rgb_t))

// led_strip_t strip = {
//         .type = LED_TYPE,
//         .length = CONFIG_LED_STRIP_LEN,
//         .gpio = GPIO_LED_LIGHT,
//         .buf = NULL,
//         #ifdef LED_STRIP_BRIGHTNESS
//         .brightness = 255,
//         #endif
// };

/*Interupt handlers----------------------------------------------------------------------------------------------------------*/
static TaskHandle_t feeding_task_handler;
TaskHandle_t float_switch_handler = NULL;
void switch_handler(void*);

/*Controller Config----------------------------------------------------------------------------------------------------------*/
//Variables
typedef struct device_config_t {
    //temperature system
    float desired_temp;
    
    //light control
    bool light_force;
    bool light_auto;
    int light_intensity;
    time_t light_on_time;
    time_t light_off_time;
    
    //wave control
    bool wave_force;
    bool wave_auto;
    time_t wave_on_time;
    time_t wave_off_time;

    //feed control
    bool feed_auto;
    time_t feed_time;

} device_config_t;
static device_config_t device_config;
static SemaphoreHandle_t device_config_mutex;

//Print Variables
/* device_config_mutex must be captured before calling*/
static void print_device_config() {
    ESP_LOGI(TAG, "==========device_config==========");
    ESP_LOGI(TAG, "desired_temp: %f", device_config.desired_temp);
    ESP_LOGI(TAG, "light_force: %d", device_config.light_force);
    ESP_LOGI(TAG, "light_auto: %d", device_config.light_auto);
    ESP_LOGI(TAG, "light_on_time: %ld", device_config.light_on_time);
    ESP_LOGI(TAG, "light_off_time: %ld", device_config.light_off_time);
    ESP_LOGI(TAG, "wave_force: %d", device_config.wave_force);
    ESP_LOGI(TAG, "wave_auto: %d", device_config.wave_auto);
    ESP_LOGI(TAG, "wave_on_time: %ld", device_config.wave_on_time);
    ESP_LOGI(TAG, "wave_off_time: %ld", device_config.wave_off_time);
    ESP_LOGI(TAG, "feed_auto: %d", device_config.feed_auto);
    ESP_LOGI(TAG, "feed_time: %ld", device_config.feed_time);
}

/*Controller Config Updater----------------------------------------------------------------------------------------------------------*/
static void update_device_config_callback(char* new_device_config, size_t buffer_size) {
    
    //omit string termination char
    size_t buffer_size_parsing = buffer_size - 1;

    cJSON* root = cJSON_ParseWithLength(new_device_config, buffer_size_parsing);
    if(root == NULL) {
        ESP_LOGE(TAG, "Json parser failed");
        return;
    }

    // Updated sharde device_config
    xSemaphoreTake(device_config_mutex, portMAX_DELAY);
    
    cJSON* attribute;
    attribute = cJSON_GetObjectItem(root, "desiredTemperature");
    if(attribute != NULL)
        device_config.desired_temp = (float) attribute->valuedouble;
    else
        ESP_LOGI(TAG, "desired_temperature not set in new config");
    
    attribute = cJSON_GetObjectItem(root, "lightForce");
    if(attribute != NULL)
        device_config.light_force = cJSON_IsTrue(attribute);
    else
        ESP_LOGI(TAG, "lightForce not set in new config");

    attribute = cJSON_GetObjectItem(root, "lightAuto");
    if(attribute != NULL)
        device_config.light_auto = cJSON_IsTrue(attribute);
    else
        ESP_LOGI(TAG, "lightAuto not set in new config");
    
    attribute = cJSON_GetObjectItem(root, "lightIntensity");
    if(attribute != NULL)
        device_config.light_intensity = attribute->valueint;
    else
        ESP_LOGI(TAG, "lightIntensity not set in new config");
    
    attribute = cJSON_GetObjectItem(root, "lightOnTime");
    if(attribute != NULL)
        device_config.light_on_time = (time_t) attribute->valueint;
    else
        ESP_LOGI(TAG, "lightOnTime not set in new config");
    
    attribute = cJSON_GetObjectItem(root, "lightOffTime");
    if(attribute != NULL)
        device_config.light_off_time = (time_t) attribute->valueint;
    else
        ESP_LOGI(TAG, "lightOffTime not set in new config");
    
    attribute = cJSON_GetObjectItem(root, "waveForce");
    if(attribute != NULL)
        device_config.wave_force = cJSON_IsTrue(attribute);
    else
        ESP_LOGI(TAG, "waveForce not set in new config");
    
    attribute = cJSON_GetObjectItem(root, "waveAuto");
    if(attribute != NULL)
        device_config.wave_auto = cJSON_IsTrue(attribute);
    else
        ESP_LOGI(TAG, "waveAuto not set in new config");
    
    attribute = cJSON_GetObjectItem(root, "waveOnTime");
    if(attribute != NULL)
        device_config.wave_on_time = (time_t) attribute->valueint;
    else
        ESP_LOGI(TAG, "waveOnTime not set in new config");
    
    attribute = cJSON_GetObjectItem(root, "waveOffTime");
    if(attribute != NULL)
        device_config.wave_off_time = (time_t) attribute->valueint;
    else
        ESP_LOGI(TAG, "waveOffTime not set in new config");

    attribute = cJSON_GetObjectItem(root, "feedAuto");
    if(attribute != NULL)
        device_config.feed_auto = cJSON_IsTrue(attribute);
    else
        ESP_LOGI(TAG, "feedTime not set in new config");
    
    attribute = cJSON_GetObjectItem(root, "feedTime");
    if(attribute != NULL)
        device_config.feed_time = (time_t) attribute->valueint;
    else
        ESP_LOGI(TAG, "feedTime not set in new config");

    print_device_config();

    vTaskResume(planner_task_handler);

    xSemaphoreGive(device_config_mutex);
    
    cJSON_Delete(root);
}

/*Ctrler-Wifi core communication----------------------------------------------------------------------------------------------------------*/
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

/*Hardware initialization----------------------------------------------------------------------------------------------------------*/
static void init_hw(void){

    //Temp probe
    gpio_config_t io_conf;
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    io_conf.pin_bit_mask = GPIO_HEATER_RELAY_PIN_SEL;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(GPIO_HEATER_RELAY, 0);

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
    ds18b20_set_resolution(ds18b20_info, DS18B20_RES);

    last_wake_time = xTaskGetTickCount();

    //Float switch
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_FLOAT_SWITCH;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_FLOAT_SWITCH, switch_handler, NULL);

    //LED strip
    //ESP_ERROR_CHECK(led_strip_init(&strip));
}

/*Feeder sub-system----------------------------------------------------------------------------------------------------------*/
static void feed_fish(void* pvParameters) {
    for(;;) {
        vTaskSuspend(NULL);
        ESP_LOGI(TAG, "Feeding fish...");
        //TODO: create and call servo control function
    }
}

void feed_command_event() {
    ESP_LOGI(TAG, "Resuming feeding task...");
    vTaskResume(feeding_task_handler);
}


/*Planner sub-system----------------------------------------------------------------------------------------------------------*/
static long adjust_time(time_t* time_of_task)
{
    long milis_of_exec_time, milis_of_current_time;
    long milis_to_wait;

    gettimeofday(&time_now, LOCAL_TIME_ZONE);
    localtime_r(&time_now.tv_sec, &timeinfo_now);
    localtime_r(time_of_task, &timeinfo_planner_task);

    milis_of_current_time = (((timeinfo_now.tm_hour * 60) + timeinfo_now.tm_min) * 60 + timeinfo_now.tm_sec) * 1000;
    milis_of_exec_time = (((timeinfo_planner_task.tm_hour * 60) + timeinfo_planner_task.tm_min) * 60 + timeinfo_planner_task.tm_sec) * 1000;
    milis_to_wait=(milis_of_exec_time-milis_of_current_time>0) ? (milis_of_exec_time-milis_of_current_time) : (milis_of_exec_time-milis_of_current_time+(24*3600*1000));

    return milis_to_wait;
}

static void planner_task(void *pvParameters)
{
    long milis_to_wait;
    enum planner_states_t planner_state = WAIT_FOR_SET_TIME;
    ESP_LOGI(TAG,"Start of Planner State Machine");
    for(;;)
    {
        switch (planner_state) 
        {
            case WAIT_FOR_SET_TIME:
                ESP_LOGI(TAG, "Waiting for config to be set.");
                vTaskSuspend(NULL);
                planner_state = CHECK_TIME;
            break;
            case CHECK_TIME:
                xSemaphoreTake(device_config_mutex, portMAX_DELAY);
                if(!device_config.light_force && device_config.light_auto) {
                    milis_to_wait = adjust_time(&device_config.light_on_time);
                    if(milis_to_wait < ONE_MINUTE) {
                        //TODO: Turn lights on
                        ESP_LOGI(TAG, "Turning light on");
                    }
                    else {
                        ESP_LOGI(TAG, "Turning light on in %ld milisec", milis_to_wait);
                    }

                    milis_to_wait = adjust_time(&device_config.light_off_time);
                    if(milis_to_wait < ONE_MINUTE) {
                        //TODO: Turn lights off
                        ESP_LOGI(TAG, "Turning light off");
                    }
                    else {
                        ESP_LOGI(TAG, "Turning light off in %ld milisec", milis_to_wait);
                    }
                }
                
                if(!device_config.wave_force && device_config.wave_auto) {
                    milis_to_wait = adjust_time(&device_config.wave_on_time);
                    if(milis_to_wait < ONE_MINUTE) {
                        //TODO: Turn wave on
                        ESP_LOGI(TAG, "Turning wave maker on");

                    }
                    else {
                        ESP_LOGI(TAG, "Turning wave maker on in %ld milisec", milis_to_wait);
                    }

                    milis_to_wait = adjust_time(&device_config.wave_off_time);
                    if(milis_to_wait < ONE_MINUTE) {
                        //TODO: Turn wave off
                        ESP_LOGI(TAG, "Turning wave maker off");
                    }
                    else {
                        ESP_LOGI(TAG, "Turning wave maker off in %ld milisec", milis_to_wait);
                    }
                }

                if(device_config.feed_auto) {
                    milis_to_wait = adjust_time(&device_config.feed_time);
                    if(milis_to_wait < ONE_MINUTE) {
                        //TODO: feed fish
                        ESP_LOGI(TAG, "Feeding fish");
                    }
                    else {
                        ESP_LOGI(TAG, "feeding fish in %ld milisec", milis_to_wait);
                    }
                }

                planner_state = SLEEP;
                xSemaphoreGive(device_config_mutex);
            break;
            case SLEEP:
                planner_state = CHECK_TIME;
                ESP_LOGI(TAG, "sleeping");
                vTaskDelay(ONE_MINUTE_SPEED_UP / portTICK_RATE_MS);
            break;
        }  
    } 
}

/*Temperature sub-system----------------------------------------------------------------------------------------------------------*/
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
        ESP_LOGI(TAG,"Device Desired Temp: %f" ,device_config->desired_temp);
        switch(state){
            case Idle_State:
                state = Measure_State;
            break;
            case Measure_State:
                ESP_LOGI(TAG,"At Measure State");
                currentTemp = TempRead();
                if(currentTemp < device_config->desired_temp){
                    ESP_LOGI(TAG,"Going to Low Temp State");
                    state = Temp_Low_State;
                }
                if(currentTemp >= device_config->desired_temp){
                    ESP_LOGI(TAG,"Going to High Temp State");
                    state = Temp_High_State;
                }
            break;
            case Temp_Low_State:
                ESP_LOGI(TAG,"At Low Temp State");
                relayPower = gpio_get_level(GPIO_HEATER_RELAY);
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
                relayPower = gpio_get_level(GPIO_HEATER_RELAY);
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
                gpio_set_level(GPIO_HEATER_RELAY, 0);
                ESP_LOGI(TAG,"Relay is On!");
                ESP_LOGI(TAG,"Relay State %i" ,gpio_get_level(GPIO_HEATER_RELAY));
                ESP_LOGI(TAG,"Going to Measure State.");
                state = Measure_State;
            break;
            case Turn_Off_Relay_State:
                ESP_LOGI(TAG,"At Turn Relay Off State");
                gpio_set_level(GPIO_HEATER_RELAY, 1);
                ESP_LOGI(TAG,"Relay is Off!");
                ESP_LOGI(TAG,"Relay State %i" ,gpio_get_level(GPIO_HEATER_RELAY));
                ESP_LOGI(TAG,"Going to Measure State.");
                state = Measure_State;
            break;
        }
        vTaskDelayUntil(&last_wake_time, SAMPLE_PERIOD / portTICK_PERIOD_MS);
    }
}

/*Lighting sub-system----------------------------------------------------------------------------------------------------------*/
static void set_light(int brightness) {
    //birghtness is from 0-10 increments of 0.5
    //ESP_ERROR_CHECK(led_strip_fill(&strip, 0, strip.length, led_brightness[brightness]));
    //ESP_ERROR_CHECK(led_strip_flush(&strip));
}

/*Float switch sub-system----------------------------------------------------------------------------------------------------------*/
void floatSwitchTask (void *p){
    for(;;){
        vTaskSuspend(NULL);
        ESP_LOGI(TAG, "Float Switch triggered!");
        //TODO: create and call send data function to cloud more like setting variable
    }
}

void switch_handler(void *arg){
    BaseType_t checkIfYieldRequired;
    checkIfYieldRequired = xTaskResumeFromISR(float_switch_handler);
    vPortEvaluateYieldFromISR(checkIfYieldRequired);
    //Send data to cloud
}

/*main----------------------------------------------------------------------------------------------------------*/
void app_main()
{
    device_config.desired_temp = 17.0;

    data_queue = xQueueCreate(queue_size, sizeof(float));  

    /*Start STA Wifi connection*/
    mqtt_callback_t mqtt_callback = 
    {
        .fetch_telemetry_event = dequeue_telemetry,
        .update_config_event = update_device_config_callback,
        .feed_command_event = feed_command_event
    };
    esp_sta_init(mqtt_callback);

    device_config_mutex = xSemaphoreCreateMutex();
    
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

    /* Create a task to check float switch trigger.*/ 
    xTaskCreatePinnedToCore(&floatSwitchTask, 
                            "Float Switch Triggered.", 
                            configMINIMAL_STACK_SIZE,
                            NULL, 
                            6, 
                            &float_switch_handler, 
                            ctrl_core);
    

    ESP_LOGI(TAG,"Starting Planner task");
    /* Create a task to check time and trigger control */ 
    xTaskCreatePinnedToCore(&planner_task,         
                            "Planner Task",        
                            3*configMINIMAL_STACK_SIZE,    
                            NULL,               
                            5,                  
                            &planner_task_handler,               
                            ctrl_core);
            
    ESP_LOGI(TAG, "Program quits");
} 