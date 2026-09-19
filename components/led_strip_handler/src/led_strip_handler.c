#include "led_strip_handler.h"

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "led_strip.h"
#include "math.h"

/*
    Message tag used for ESP_LOGI messages
*/
#define DEBUG_TAG "LED_STRIP"

/*
    LED strip initial conditions
*/
#define INITIAL_BRIGHTNESS  32
#define INITIAL_COLOR_TEMP  0

struct led_state_s {
    uint8_t brightness;
    uint8_t color_temp;
};

static led_strip_handle_t led_strip;
static QueueHandle_t state_req_queue;

/*******************************************************************************
-- PRIVATE FUNCTIONS --
*******************************************************************************/

/*!
    @brief Initialize the LED strip
*/
static esp_err_t led_strip_configure(uint32_t gpio_pin, uint32_t num_leds) {
    /* Create LED strip configuration */
    led_strip_config_t strip_config = {
        .strip_gpio_num = gpio_pin,
        .max_leds = num_leds,
        .led_model = LED_MODEL_WS2812,
        /* The color component format doesn't really matter much since we only
            have 2 "colors" (cool white and warm white) */
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = {
            .invert_out = 0,
        }
    };

    /* Create LED strip driver configuration */
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10e6,
        .mem_block_symbols = 0,
        .flags = {
            .with_dma = 0
        }
    };

    /* Create the LED strip object */
    esp_err_t err = led_strip_new_rmt_device(
        &strip_config,
        &rmt_config,
        &led_strip
    );

    led_strip_clear(led_strip);
    return err;
}

/*!
    @brief Set the LED strip warm and cool pixel values for the full strip

    @param led_strip Handle for the LED strip that is to be updated
    @param warm_white Value for the warm white color (0 - 255)
    @param cool_white Value for the cool white color (0 - 255)
*/
static void set_led_strip(
    led_strip_handle_t led_strip,
    uint8_t warm_white,
    uint8_t cool_white
) {
    for (int i = 0; i < CONFIG_LED_STRIP_LED_COUNT; i++) {
        led_strip_set_pixel(
            led_strip,
            i,
            warm_white,
            cool_white,
            0
        );
    }
    led_strip_refresh(led_strip);
}

/*!
    @brief Set the LED strip pixels based on a normalized brightness and color
    temperature and apply gamma correction

    @param brightness Normalized brightness (0.0 - 1.0)
    @param color_temp Normalized color temperature (0.0 - 1.0)

    @details The function assumes a gamma correction factor of 2. With that, the
    basic white value calculations are as follows:
        cool_white = 255 * brightness ^ 2 * color_temp
        warm_white = 255 * brightness ^ 2 * (1 - color_temp)
*/
static void set_led_strip_from_norm_led_state(
    led_strip_handle_t led_strip,
    float brightness,
    float color_temp
) {
    float b_2 = brightness * brightness;
    float ww = b_2 * (1 - color_temp);
    float cw = b_2 * color_temp;

    uint8_t warm_white = roundf(255 * ww);
    uint8_t cool_white = roundf(255 * cw);
    set_led_strip(led_strip, warm_white, cool_white);
}

static void interpolate_led_strip(
    led_strip_handle_t led_strip,
    struct led_state_req_s *led_state_req,
    struct led_state_s *current_led_state
) {
    /* Override the brightness if the LED is actually supposed to be off since
        the brightness value doesn't change for that */
    uint8_t actual_brightness = led_state_req->brightness;
    if (led_state_req->power != 1) {
        actual_brightness = 0;
    }

    /* Interpolating brightness and color temperature are 2 independent
        operations, so the largest delta is used to determine how fast the
        interpolation should be */
    uint8_t delta_brightness = abs(actual_brightness - current_led_state->brightness);
    uint8_t delta_color_temp = abs(led_state_req->color_temp - current_led_state->color_temp);
    uint8_t max_delta = MAX(delta_brightness, delta_color_temp);

    /* Pre-compute the normalized brightness and color temperature values used
        for the linear interpolation because these values don't change during
        the interpolation steps */
    float b0_norm = current_led_state->brightness / 255.0;
    float ct0_norm = current_led_state->color_temp / 255.0;
    float b1_norm = actual_brightness / 255.0;
    float ct1_norm = led_state_req->color_temp / 255.0;
    float b1_0_norm = b1_norm - b0_norm;
    float ct1_0_norm = ct1_norm - ct0_norm;
    float b_i = b0_norm;
    float ct_i = ct0_norm;
    uint8_t interpolation_interrupted = 0;
    for (int i = 1; i <= max_delta; i += 8) {
        vTaskDelay(pdMS_TO_TICKS(20));

        /* Check if there's any new LED state requests in the queue. If there
            are, then stop this interpolation so the next one can start
            immediately. */
        if (uxQueueMessagesWaiting(state_req_queue)) {
            interpolation_interrupted = 1;
            break;
        }
        
        /* Calculate intermediate normalized brightness and color temperature */
        float a = i / (float) max_delta;
        a = sinf(M_PI_2 * a);
        b_i = b0_norm + a * b1_0_norm;
        ct_i = ct0_norm + a * ct1_0_norm;

        set_led_strip_from_norm_led_state(
            led_strip,
            b_i,
            ct_i
        );
    }

    if (interpolation_interrupted) {
        /* In the event that the interpolation is interrupted because of a new
            LED state request, the LED state is updated with the last
            interpolated value so that the next interpolation starts from where
            this one left off */
        current_led_state->brightness = 255 * b_i;
        current_led_state->color_temp = 255 * ct_i;
    } else {
        /* If the interpolation completed, then set the LED strip and LED state
            to the final brightness and color temperature values. This is done
            because the interpolation loop end might not be a multiple of the
            interpolation step. */
        set_led_strip_from_norm_led_state(
            led_strip,
            b1_norm,
            ct1_norm
        );

        current_led_state->brightness = actual_brightness;
        current_led_state->color_temp = led_state_req->color_temp;
    }
}

/*******************************************************************************
-- PUBLIC FUNCTIONS --
*******************************************************************************/

/*!
    @brief Initialize the LED strip and request queue
*/
esp_err_t led_strip_init(uint32_t gpio_pin, uint32_t num_leds) {
    state_req_queue = xQueueCreate(4, sizeof(struct led_state_req_s));
    esp_err_t err = led_strip_configure(gpio_pin, num_leds);
    if (err != ESP_OK) {
        ESP_LOGE(DEBUG_TAG, "Error initializing the LED strip");
    }
    return err;
}

/*!
    @brief Task that updates the LED strip when there's a new desired state
*/
void led_strip_task(void *args) {
    /* Provide initial values for the brightness and color temperature since
        Home Assistant won't send these when the strip is simply turned on */
    struct led_state_req_s led_state_req = {
        .power = 0,
        .brightness = INITIAL_BRIGHTNESS,
        .brightness_valid = 1,
        .color_temp = INITIAL_COLOR_TEMP,
        .color_temp_valid = 1
    };
    struct led_state_s current_led_state = {
        .brightness = INITIAL_BRIGHTNESS,
        .color_temp = INITIAL_COLOR_TEMP
    };
    while (1) {
        /* Block until there's a new requested LED state */
        struct led_state_req_s new_led_state_req;
        BaseType_t stat = xQueueReceive(
            state_req_queue,
            (void*) &new_led_state_req,
            portMAX_DELAY
        );
        if (stat == pdTRUE) {
            /* Update the LED state request with only the UPDATED state members.
                This ensures that when the LED strip is turned off and turned
                back on, it goes back to the brightness and color temperature it
                was at prior to being turned off. */
            if (new_led_state_req.power) {
                led_state_req.power = 1;
                if (new_led_state_req.brightness_valid) {
                    led_state_req.brightness = new_led_state_req.brightness;
                }
                if (new_led_state_req.color_temp_valid) {
                    led_state_req.color_temp = new_led_state_req.color_temp;
                }
            } else {
                led_state_req.power = 0;
            }

            /* Interpolate the LED strip to the desired state */
            interpolate_led_strip(
                led_strip,
                &led_state_req,
                &current_led_state
            );
        }
    }
}

/*!
    @brief Push a new LED state request into the LED state request queue

    @param led_state_req LED state request struct that contains the new
    requested state

    @return
    - ESP_OK if the update was added to the queue
    - ESP_FAIL if the update wasn't able to be added to the queue
*/
esp_err_t update_led_state(struct led_state_req_s *led_state_req) {
    BaseType_t stat = xQueueSend(
        state_req_queue,
        (void*) led_state_req,
        pdFALSE
    );
    if (stat != pdTRUE) {
        ESP_LOGE(DEBUG_TAG, "Failed to add new led state to queue");
        return ESP_FAIL;
    }
    return ESP_OK;
}