/*!
    @file:	led_strip_handler.h
    @brief:	Manage the LED strip initialization and animation
*/
#ifndef LED_STRIP_HANDLER_H
#define LED_STRIP_HANDLER_H

#include <stdint.h>
#include "esp_err.h"
#ifdef __cplusplus
extern "C" {
#endif

/*
    LED state request struct
*/
struct led_state_req_s {
    uint8_t power;
    uint8_t brightness;
    uint8_t brightness_valid;
    uint8_t color_temp;
    uint8_t color_temp_valid;
};

/*
    Functions
*/
esp_err_t led_strip_init(
    uint32_t gpio_pin,
    uint32_t num_leds
);

void led_strip_task(
    void *args
);

esp_err_t update_led_state(
    struct led_state_req_s *led_state_req
);

#ifdef __cplusplus
}
#endif

#endif