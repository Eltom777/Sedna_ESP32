#ifndef hw_definition
#define hw_definition

//Hardware config
#define app_core 0
#define ctrl_core 1
#define ESP_INTR_FLAG_DEFAULT 0

//Temperature prob config
#define DS18B20_RES 12
#define SAMPLE_PERIOD 1000
#define temperature_telemetry_thresold_change 0.5f
#define default_desired_temp 20.0f

#define temp_threshold -5.0f

//LED config
#define LED_TYPE LED_STRIP_WS2812
#define CONFIG_LED_STRIP_LEN 51

//Servo config
#define SERVO_MAX_PULSEWIDTH_US 2300
#define SERVO_MIN_PULSEWIDTH_US 700
#define SERVO_MAX_DEGREE 180
#define MAX_FEED_COUNT 14

#endif //hw_definition