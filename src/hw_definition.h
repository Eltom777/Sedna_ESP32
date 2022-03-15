#ifndef hw_definition
#define hw_definition

//Hardware config
#define app_core 0
#define ctrl_core 1
#define ESP_INTR_FLAG_DEFAULT 0

//Temperature prob config
#define DS18B20_RES 12
#define SAMPLE_PERIOD 1000

#define temp_threshold -5.0f

//LED config
#define LED_TYPE LED_STRIP_WS2812
#define CONFIG_LED_STRIP_LEN 60

#endif //hw_definition