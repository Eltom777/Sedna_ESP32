#ifndef gpio_definition
#define gpio_definition

// Defines all the GPIO pins
//Temp probe
#define GPIO_DS18B20_0 32
//Heater
#define GPIO_HEATER_RELAY 27
#define GPIO_HEATER_RELAY_PIN_SEL (1ULL << GPIO_HEATER_RELAY)
//Lights
#define GPIO_LED_LIGHT 25
//Feeder
#define GPIO_FEEDER_SERVO 26
//Wavemaker
#define GPIO_WAVE_RELAY 2
#define GPIO_WAVE_RELAY_PIN_SEL (1ULL << GPIO_WAVE_RELAY)
//Float switch
#define GPIO_FLOAT_SWITCH 33
#define GPIO_FLOAT_SWITCH_SEL (1ULL << GPIO_FLOAT_SWITCH)
//Liquid sensor
#define GPIO_LIQUID_SENSOR 34
#define GPIO_LIQUID_SENSOR_SEL (1ULL << GPIO_LIQUID_SENSOR)
#endif //gpio_definition