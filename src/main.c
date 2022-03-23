#include <stdlib.h>
#include <string.h>
#include <math.h>
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
#include "led_strip.h"

#include "driver/mcpwm.h"

/* Global Variables----------------------------------------------------------------------------------------------------------*/
//ESPI_LOG Tag
static const char *TAG = "CTRL_APP";
static const char *TAGTEMP = "CTRL_TEMP";
static const char *TAGFEED = "CTRL_FEED";
static const char *TAGLED = "CTRL_LED";
static const char *TAGWAVE = "CTRL_WAVE";

//ISR Queues
static xQueueHandle float_switch_ISR_queue;
static xQueueHandle liquid_sensor_ISR_queue;

//Temp sub-system variables
static DS18B20_Info * ds18b20_info;
static TickType_t last_wake_time;
static OneWireBus * owb;
static owb_rmt_driver_info rmt_driver_info;
static OneWireBus_ROMCode rom_code;
static owb_status status;
static int relayPower;

typedef enum FSMstates{
        Idle_State,
        Measure_State,
        Temp_Low_State,
        Temp_High_State,
        Turn_On_Relay_State,
        Turn_Off_Relay_State
} FSMstates;

//Planner sub-system variables
#define LOCAL_TIME_ZONE "EST5EDT,M3.2.0/2,M11.1.0"
#define ONE_SEC_SPEED_UP 700 // Speed up a sec by 300ms so deadline is not missed
#define ONE_MINUTE 60000L

enum planner_states_t
{
    WAIT_FOR_SET_TIME,
    CHECK_TIME,
    SLEEP
};
static int ONE_MINUTE_SPEED_UP = 60 * ONE_SEC_SPEED_UP;

static struct timeval time_now = {0};
static struct tm timeinfo_now = {0};

static struct tm timeinfo_planner_task = {0};

//LED brightness varibles
void set_light(int);
static const rgb_t led_brightness[] = { //0 - 10 values
    { .r = 0x00, .g = 0x00, .b = 0x00 },//0
    { .r = 0x01, .g = 0x01, .b = 0x01 },//1
    { .r = 0x03, .g = 0x03, .b = 0x03 },//2
    { .r = 0x05, .g = 0x05, .b = 0x05 },//3
    { .r = 0x06, .g = 0x06, .b = 0x06 },//4
    { .r = 0x07, .g = 0x07, .b = 0x07 },//5
    { .r = 0x08, .g = 0x08, .b = 0x08 },//6
    { .r = 0x0a, .g = 0x0a, .b = 0x0a },//7
    { .r = 0x0c, .g = 0x0c, .b = 0x0c },//8
    { .r = 0x0e, .g = 0x0e, .b = 0x0e },//9
    { .r = 0x0f, .g = 0x0f, .b = 0x0f }//10
};
#define COLORS_TOTAL (sizeof(led_brightness) / sizeof(rgb_t))

int set_brightness = 0;

led_strip_t strip = {
        .type = LED_TYPE,
        .length = CONFIG_LED_STRIP_LEN,
        .gpio = GPIO_LED_LIGHT,
        .buf = NULL,
        #ifdef LED_STRIP_BRIGHTNESS
        .brightness = 255,
        #endif
};

/*Interupt handlers----------------------------------------------------------------------------------------------------------*/
static TaskHandle_t feeding_task_handler;
TaskHandle_t float_switch_handle = NULL;
void fswitch_handler(void*);
TaskHandle_t liquid_sensor_handle = NULL;
void lsensor_handler(void*);

static bool notification_flag = true;

/*Controller Config----------------------------------------------------------------------------------------------------------*/
//config & telemetry
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

//Device status for temperature and food count of system
static device_telemetry_t device_status ={0};
static SemaphoreHandle_t device_status_mutex;

//Planner task synchronization
static SemaphoreHandle_t planner_task_mutex;

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

    xSemaphoreGive(planner_task_mutex);

    xSemaphoreGive(device_config_mutex);
    
    cJSON_Delete(root);
}

bool dequeue_telemetry(struct device_telemetry_t* device_telemetry_wifi)
{
    bool telemetry_changed = false;
    
    xSemaphoreTake(device_status_mutex, portMAX_DELAY);

    ESP_LOGI(TAG, "Grabbing Current Temp for Telemetry: %f", device_status.current_temp);
    ESP_LOGI(TAG, "Grabbing food count for Telemetry : %d", device_status.food_count);

    float abs_difference = fabs(device_telemetry_wifi->food_count - device_status.food_count);
    if(abs_difference >= temperature_telemetry_thresold_change) {
        device_telemetry_wifi->food_count = device_status.food_count;
        telemetry_changed = true;
    }

    if(device_telemetry_wifi->food_count != device_status.food_count) {
        device_telemetry_wifi->food_count = device_status.food_count;
        telemetry_changed = true;
    }
    
    xSemaphoreGive(device_status_mutex);
    
    return telemetry_changed;
}

static bool dequeue_floatSW_notification_queue()
{

    bool data = false;
    if(float_switch_ISR_queue != NULL)
    {
        if((int) uxQueueMessagesWaiting(float_switch_ISR_queue) > 0)
        {
            xQueueReceiveFromISR(float_switch_ISR_queue, &data, (TickType_t)0);
        }
        else
        {
            ESP_LOGI(TAG, "Float switch ISR queue is empty...");
        }
    }
    else
    {
        ESP_LOGE(TAG, "Float switch ISR queue is not set...");
    }
    return data;   
}

static bool dequeue_lsensor_notification_queue()
{

    bool data = false;
    if(liquid_sensor_ISR_queue != NULL)
    {
        if((int) uxQueueMessagesWaiting(liquid_sensor_ISR_queue) > 0)
        {
            xQueueReceiveFromISR(liquid_sensor_ISR_queue, &data, (TickType_t)0);
        }
        else
        {
            ESP_LOGI(TAG, "Liquid sensor ISR queue is empty...");
        }
    }
    else
    {
        ESP_LOGE(TAG, "Liquid sensor ISR queue is not set...");
    }
    return data;   
}

/*ISR Handler----------------------------------------------------------------------------------------------------------------------*/

// Liquid sensor ISR
void lsensor_handler(void *arg){
    xQueueSendToBackFromISR(liquid_sensor_ISR_queue, &notification_flag, NULL);
}

// Float Switch ISR
void fswitch_handler(void *arg){
    xQueueSendToBackFromISR(float_switch_ISR_queue, &notification_flag, NULL);
}

/*Hardware initialization----------------------------------------------------------------------------------------------------------*/
static void init_hw(void){

    //Temp relay
    gpio_config_t io_conf;
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    io_conf.pin_bit_mask = GPIO_HEATER_RELAY_PIN_SEL;
    gpio_config(&io_conf);
    gpio_set_level(GPIO_HEATER_RELAY, 1);

    //Wave relay
    io_conf.pin_bit_mask = GPIO_WAVE_RELAY_PIN_SEL;
    gpio_config(&io_conf);
    gpio_set_level(GPIO_WAVE_RELAY, 1);

    //Temp probe
    // Create a 1-Wire bus, using the RMT timeslot driver
    owb = owb_rmt_initialize(&rmt_driver_info, GPIO_DS18B20_0, RMT_CHANNEL_1, RMT_CHANNEL_2);
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
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pull_up_en = 1;
    io_conf.pin_bit_mask = GPIO_FLOAT_SWITCH_SEL;
    gpio_config(&io_conf);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_FLOAT_SWITCH, fswitch_handler, NULL);

    //Liquid sensor
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = GPIO_LIQUID_SENSOR_SEL;
    gpio_config(&io_conf);
    gpio_isr_handler_add(GPIO_LIQUID_SENSOR, lsensor_handler, NULL);

    //LED strip
    led_strip_install();
    ESP_ERROR_CHECK(led_strip_init(&strip));
    ESP_ERROR_CHECK(led_strip_fill(&strip, 0, strip.length, led_brightness[1]));
    ESP_ERROR_CHECK(led_strip_flush(&strip));
    ESP_LOGI(TAG, "Exit HW INIT");

    //Fish Feeder Servo
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_FEEDER_SERVO);

    mcpwm_config_t pwm_config = {
        .frequency = 50, // frequency = 50Hz, i.e. for every servo motor time period should be 20ms
        .cmpr_a = 0,     // duty cycle of PWMxA = 0
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
}

/*Feeder sub-system----------------------------------------------------------------------------------------------------------*/
static inline uint32_t convert_servo_angle_to_duty_us(int angle)
{
     return (angle + SERVO_MAX_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (2 * SERVO_MAX_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

static void feed_fish(void* pvParameters) {
    for(;;) {
        vTaskSuspend(NULL);
        ESP_LOGI(TAGFEED, "Feeding fish...");
        xSemaphoreTake(device_status_mutex, portMAX_DELAY);
        device_status.food_count--;
        xSemaphoreGive(device_status_mutex);

        ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, convert_servo_angle_to_duty_us(100)));
        vTaskDelay(63 / portTICK_RATE_MS);
        ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0));
        
    }
}

void feed_command_event() {
    ESP_LOGI(TAGFEED, "Resuming feeding task...");
    vTaskResume(feeding_task_handler);
}

void refill_command_event() {
    ESP_LOGI(TAGFEED, "refilling");
    xSemaphoreTake(device_status_mutex, portMAX_DELAY);
    device_status.food_count = MAX_FEED_COUNT;
    xSemaphoreGive(device_status_mutex);
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
    for(;;)
    {
        switch (planner_state) 
        {
            case WAIT_FOR_SET_TIME:
                ESP_LOGI(TAG, "Waiting for config to be set.");
                xSemaphoreTake(planner_task_mutex, portMAX_DELAY);
                planner_state = CHECK_TIME;
            break;
            case CHECK_TIME:
                xSemaphoreTake(device_config_mutex, portMAX_DELAY);

                //set light state
                if(device_config.light_auto){
                    milis_to_wait = adjust_time(&device_config.light_on_time);
                    if(milis_to_wait < ONE_MINUTE) {
                        ESP_LOGI(TAGLED, "Turning light on");
                        set_brightness = device_config.light_intensity;
                        set_light(set_brightness);
                    }
                    else {
                        ESP_LOGI(TAGLED, "Turning light on in %ld milisec", milis_to_wait);
                    }

                    milis_to_wait = adjust_time(&device_config.light_off_time);
                    if(milis_to_wait < ONE_MINUTE) {
                        ESP_LOGI(TAGLED, "Turning light off");
                        set_light(0); //0 intensity = off
                    }
                    else {
                        ESP_LOGI(TAGLED, "Turning light off in %ld milisec", milis_to_wait);
                    }
                }else if(device_config.light_force){
                    ESP_LOGI(TAGLED, "Forcing light on");
                    set_brightness = device_config.light_intensity;
                    set_light(set_brightness);
                }else{
                    ESP_LOGI(TAGLED, "Forcing light off");
                    set_light(0);
                }
                
                //set wavemaker state
                if(device_config.wave_auto) {
                    milis_to_wait = adjust_time(&device_config.wave_on_time);
                    if(milis_to_wait < ONE_MINUTE) {
                        ESP_LOGI(TAGWAVE, "Turning wave maker on");
                        gpio_set_level(GPIO_WAVE_RELAY, 1);
                        ESP_LOGI(TAGWAVE,"Wave Relay is On!");
                        ESP_LOGI(TAGWAVE,"Wave Relay State %i" ,gpio_get_level(GPIO_WAVE_RELAY));

                    }
                    else {
                        ESP_LOGI(TAGWAVE, "Turning wave maker on in %ld milisec", milis_to_wait);
                    }

                    milis_to_wait = adjust_time(&device_config.wave_off_time);
                    if(milis_to_wait < ONE_MINUTE) {
                        ESP_LOGI(TAGWAVE, "Turning wave maker off");
                        gpio_set_level(GPIO_WAVE_RELAY, 0);
                        ESP_LOGI(TAGWAVE,"Wave Relay is Off!");
                        ESP_LOGI(TAGWAVE,"Wave Relay State %i" ,gpio_get_level(GPIO_WAVE_RELAY));
                    }
                    else {
                        ESP_LOGI(TAG, "Turning wave maker off in %ld milisec", milis_to_wait);
                    }
                } else if(device_config.wave_force){
                    ESP_LOGI(TAGWAVE, "Forcing wave maker on");
                    if(gpio_get_level(GPIO_WAVE_RELAY) == 0){
                        gpio_set_level(GPIO_WAVE_RELAY, 1);
                        ESP_LOGI(TAGWAVE,"Wave Relay is On!");
                        ESP_LOGI(TAGWAVE,"Wave Relay State %i" ,gpio_get_level(GPIO_WAVE_RELAY));
                    }else{
                        ESP_LOGI(TAGWAVE,"Wave Relay already On!");
                        ESP_LOGI(TAGWAVE,"Wave Relay State %i" ,gpio_get_level(GPIO_WAVE_RELAY));
                    }
                
                } else{
                    ESP_LOGI(TAG, "Forcing wave maker off");
                    if(gpio_get_level(GPIO_WAVE_RELAY) == 1){
                        gpio_set_level(GPIO_WAVE_RELAY, 0);
                        ESP_LOGI(TAGWAVE,"Wave Relay is Off!");
                        ESP_LOGI(TAGWAVE,"Wave Relay State %i" ,gpio_get_level(GPIO_WAVE_RELAY));
                    }else{
                        ESP_LOGI(TAGWAVE,"Wave Relay already Off!");
                        ESP_LOGI(TAGWAVE,"Wave Relay State %i" ,gpio_get_level(GPIO_WAVE_RELAY));
                    }
                }

                //set feeder state
                if(device_config.feed_auto) {
                    milis_to_wait = adjust_time(&device_config.feed_time);
                    if(milis_to_wait < ONE_MINUTE) {
                        ESP_LOGI(TAGFEED, "Scheduled Feeding fish");
                        feed_command_event();
                    }
                    else {
                        ESP_LOGI(TAGFEED, "feeding fish in %ld milisec", milis_to_wait);
                    }
                }

                planner_state = SLEEP;
                xSemaphoreGive(device_config_mutex);
            break;
            case SLEEP:
                planner_state = CHECK_TIME;
                ESP_LOGI(TAG, "sleeping");
                xSemaphoreTake(planner_task_mutex, ONE_MINUTE_SPEED_UP / portTICK_RATE_MS);
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
        return readTemp;
}

static void FSMTempCtrl(void* pvParameters)
{
    device_config_t* device_config =  (device_config_t*) pvParameters;
    enum FSMstates state = Idle_State;
    for(;;)
    {
        ESP_LOGI(TAGTEMP,"Device Desired Temp: %f" ,device_config->desired_temp);
        switch(state){
            case Idle_State:
                state = Measure_State;
            break;
            case Measure_State:
                ESP_LOGI(TAGTEMP,"At Measure State");
                device_status.current_temp = TempRead();
                if(device_status.current_temp < device_config->desired_temp){
                    ESP_LOGI(TAGTEMP,"Going to Low Temp State");
                    state = Temp_Low_State;
                }
                if(device_status.current_temp >= device_config->desired_temp){
                    ESP_LOGI(TAGTEMP,"Going to High Temp State");
                    state = Temp_High_State;
                }
            break;
            case Temp_Low_State:
                ESP_LOGI(TAGTEMP,"At Low Temp State");
                relayPower = gpio_get_level(GPIO_HEATER_RELAY);
                if(relayPower == 0)
                {
                    ESP_LOGI(TAGTEMP,"Going to Turn Relay On State");
                    state = Turn_On_Relay_State;
                }
                else 
                {
                    ESP_LOGI(TAGTEMP,"Going to Measure State.");
                    state = Measure_State;
                }
            break;
            case Temp_High_State:
                ESP_LOGI(TAGTEMP,"At High Temp State");
                relayPower = gpio_get_level(GPIO_HEATER_RELAY);
                if(relayPower ==  1)
                {
                    ESP_LOGI(TAGTEMP,"Going to Turn Relay Off State");
                    state = Turn_Off_Relay_State;
                }
                else 
                {
                    ESP_LOGI(TAGTEMP,"Going to Measure State.");
                    state = Measure_State;
                }
            break;
            case Turn_On_Relay_State:
                ESP_LOGI(TAGTEMP,"At Turn Relay On State");
                gpio_set_level(GPIO_HEATER_RELAY, 1);
                ESP_LOGI(TAGTEMP,"Relay is On!");
                ESP_LOGI(TAGTEMP,"Relay State %i" ,gpio_get_level(GPIO_HEATER_RELAY));
                ESP_LOGI(TAGTEMP,"Going to Measure State.");
                state = Measure_State;
            break;
            case Turn_Off_Relay_State:
                ESP_LOGI(TAGTEMP,"At Turn Relay Off State");
                gpio_set_level(GPIO_HEATER_RELAY, 0);
                ESP_LOGI(TAGTEMP,"Relay is Off!");
                ESP_LOGI(TAGTEMP,"Relay State %i" ,gpio_get_level(GPIO_HEATER_RELAY));
                ESP_LOGI(TAGTEMP,"Going to Measure State.");
                state = Measure_State;
            break;
        }
        vTaskDelayUntil(&last_wake_time, SAMPLE_PERIOD / portTICK_PERIOD_MS);
    }
}

/*Lighting sub-system----------------------------------------------------------------------------------------------------------*/
void set_light(int brightness) {
    //brightness is from 0-10 increments of 1
    ESP_ERROR_CHECK(led_strip_fill(&strip, 0, strip.length, led_brightness[brightness]));
    ESP_ERROR_CHECK(led_strip_flush(&strip));
}

void init_mutex() {
    device_config_mutex = xSemaphoreCreateMutex();
    device_status_mutex = xSemaphoreCreateMutex();
    planner_task_mutex = xSemaphoreCreateBinary();
}

/*main----------------------------------------------------------------------------------------------------------*/
void app_main()
{
    device_config.desired_temp = default_desired_temp;
    device_status.food_count = MAX_FEED_COUNT;

    float_switch_ISR_queue = xQueueCreate(1, sizeof(bool));
    liquid_sensor_ISR_queue = xQueueCreate(1, sizeof(bool));

    /*Start STA Wifi connection*/
    mqtt_callback_t mqtt_callback = 
    {
        .fetch_telemetry_event = dequeue_telemetry,
        .fetch_floatSW_event = dequeue_floatSW_notification_queue,
        .fetch_waterlvl_event = dequeue_lsensor_notification_queue,
        .update_config_event = update_device_config_callback,
        .feed_command_event = feed_command_event,
        .refill_command_event = refill_command_event
    };
    esp_sta_init(mqtt_callback);

    init_mutex();
    
    init_hw();

    /* Create a task to control temperature feedback control*/
    xTaskCreatePinnedToCore(&FSMTempCtrl, 
                            "FSM run.", 
                            3*configMINIMAL_STACK_SIZE,
                            &device_config, 
                            5, 
                            NULL, 
                            ctrl_core);

    /* Create a task to control feeding servo. */
    xTaskCreatePinnedToCore(&feed_fish, 
                            "Feed Fish.", 
                            3*configMINIMAL_STACK_SIZE,
                            NULL, 
                            6, 
                            &feeding_task_handler, 
                            ctrl_core);
    
    /* Create a task to check time and trigger control */ 
    xTaskCreatePinnedToCore(&planner_task,         
                            "Planner Task",        
                            3*configMINIMAL_STACK_SIZE,    
                            NULL,               
                            5,                  
                            NULL,               
                            ctrl_core);
            
    ESP_LOGI(TAG, "Program quits");
} 