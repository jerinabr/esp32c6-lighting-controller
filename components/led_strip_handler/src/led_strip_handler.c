#include "led_strip_handler.h"

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "led_strip.h"

/*
    Message tag used for ESP_LOGI messages
*/
#define DEBUG_TAG "LED_STRIP"

/*
    LED strip initial conditions
*/
#define INITIAL_BRIGHTNESS  32
#define INITIAL_COLOR_TEMP  0

/*
    Gamma LUT
*/
static const uint8_t GAMMA_LUT[256] = {
	0, 0, 0, 0,
	0, 0, 0, 0,
	0, 0, 0, 0,
	1, 1, 1, 1,
	1, 1, 1, 1,
	2, 2, 2, 2,
	2, 2, 3, 3,
	3, 3, 4, 4,
	4, 4, 5, 5,
	5, 5, 6, 6,
	6, 7, 7, 7,
	8, 8, 8, 9,
	9, 9, 10, 10,
	11, 11, 11, 12,
	12, 13, 13, 14,
	14, 15, 15, 16,
	16, 17, 17, 18,
	18, 19, 19, 20,
	20, 21, 21, 22,
	23, 23, 24, 24,
	25, 26, 26, 27,
	28, 28, 29, 30,
	30, 31, 32, 32,
	33, 34, 35, 35,
	36, 37, 38, 38,
	39, 40, 41, 42,
	42, 43, 44, 45,
	46, 47, 47, 48,
	49, 50, 51, 52,
	53, 54, 55, 56,
	56, 57, 58, 59,
	60, 61, 62, 63,
	64, 65, 66, 67,
	68, 69, 70, 71,
	73, 74, 75, 76,
	77, 78, 79, 80,
	81, 82, 84, 85,
	86, 87, 88, 89,
	91, 92, 93, 94,
	95, 97, 98, 99,
	100, 102, 103, 104,
	105, 107, 108, 109,
	111, 112, 113, 115,
	116, 117, 119, 120,
	121, 123, 124, 126,
	127, 128, 130, 131,
	133, 134, 136, 137,
	139, 140, 142, 143,
	145, 146, 148, 149,
	151, 152, 154, 155,
	157, 158, 160, 162,
	163, 165, 166, 168,
	170, 171, 173, 175,
	176, 178, 180, 181,
	183, 185, 186, 188,
	190, 192, 193, 195,
	197, 199, 200, 202,
	204, 206, 207, 209,
	211, 213, 215, 217,
	218, 220, 222, 224,
	226, 228, 230, 232,
	233, 235, 237, 239,
	241, 243, 245, 247,
	249, 251, 253, 255
};

/*
    Sin LUT from 0 degrees to 90 degrees
*/
static const uint8_t SIN_LUT_0_90[256] = {
	0, 2, 3, 5,
	6, 8, 9, 11,
	13, 14, 16, 17,
	19, 20, 22, 24,
	25, 27, 28, 30,
	31, 33, 34, 36,
	38, 39, 41, 42,
	44, 45, 47, 48,
	50, 51, 53, 55,
	56, 58, 59, 61,
	62, 64, 65, 67,
	68, 70, 71, 73,
	74, 76, 77, 79,
	80, 82, 83, 85,
	86, 88, 89, 91,
	92, 94, 95, 96,
	98, 99, 101, 102,
	104, 105, 107, 108,
	109, 111, 112, 114,
	115, 116, 118, 119,
	121, 122, 123, 125,
	126, 127, 129, 130,
	132, 133, 134, 136,
	137, 138, 140, 141,
	142, 143, 145, 146,
	147, 149, 150, 151,
	152, 154, 155, 156,
	157, 159, 160, 161,
	162, 164, 165, 166,
	167, 168, 169, 171,
	172, 173, 174, 175,
	176, 178, 179, 180,
	181, 182, 183, 184,
	185, 186, 187, 188,
	190, 191, 192, 193,
	194, 195, 196, 197,
	198, 199, 200, 201,
	202, 203, 203, 204,
	205, 206, 207, 208,
	209, 210, 211, 212,
	213, 213, 214, 215,
	216, 217, 218, 218,
	219, 220, 221, 222,
	222, 223, 224, 225,
	225, 226, 227, 228,
	228, 229, 230, 230,
	231, 232, 232, 233,
	234, 234, 235, 235,
	236, 237, 237, 238,
	238, 239, 239, 240,
	241, 241, 242, 242,
	243, 243, 243, 244,
	244, 245, 245, 246,
	246, 247, 247, 247,
	248, 248, 248, 249,
	249, 249, 250, 250,
	250, 251, 251, 251,
	251, 252, 252, 252,
	252, 253, 253, 253,
	253, 253, 254, 254,
	254, 254, 254, 254,
	254, 255, 255, 255,
	255, 255, 255, 255,
	255, 255, 255, 255,
};

struct led_state_s {
    uint8_t brightness;
    uint8_t color_temp;
};

static led_strip_handle_t led_strip;
static QueueHandle_t state_req_queue;
static uint32_t config_num_leds = 0;

/*******************************************************************************
-- PRIVATE FUNCTIONS --
*******************************************************************************/

/*!
    @brief Scale a number by an alpha value

    @param alpha Number that is used for scaling (0 - 255)
    @param n Number that will be scaled (0 - 255)

    @return Results of alpha scaling

    @details The calculation here is equivalent to
        y = (alpha / 255) * n

    The calculations were taken from this arXiv paper:
    https://arxiv.org/html/2202.02864v2#S2
*/
static uint8_t alpha_blend(uint8_t alpha, uint8_t n) {
    uint32_t a = alpha * n;
    a += 0x80U;
    a += (a >> 8);
    return (a >> 8);
}

/*!
    @brief Linear interpolate between 2 numbers given an alpha

    @param start Starting value (0 - 255)
    @param end Ending value (0 - 255)
    @param alpha Interpolation alpha (0 - 255)

    @returns Interpolated result

    @details The calculation is equivalent to
        y = a + (alpha / 255) * (b - a)
*/
static uint8_t interpolate(uint8_t start, uint8_t end, uint8_t alpha) {
    uint8_t a_i = alpha_blend(255 - alpha, start);
    uint8_t b_i = alpha_blend(alpha, end);
    uint16_t result = (uint16_t) a_i + (uint16_t) b_i;
    return (result > 255 ? 255 : result);
}

/*!
    @brief Initialize the LED strip

    @param gpio_pin GPIO pin the LED strip is connected to
    @param num_leds Number of LEDs in the strip

    @return
    - ESP_OK if the LED strip successfully initialized
    - error codes if the LED strip failed to initialize
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
    for (int i = 0; i < config_num_leds; i++) {
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

    @param brightness Normalized brightness (0 - 255)
    @param color_temp Normalized color temperature (0 - 255)

    @details The function assumes a gamma correction factor of 2. With that, the
    basic white value calculations are as follows:
        warm_white = (brightness / 255) ^ gamma * (255 - color_temp)
        cool_white = (brightness / 255) ^ gamma * color_temp
*/
static void set_led_strip_from_norm_led_state(
    led_strip_handle_t led_strip,
    uint8_t brightness,
    uint8_t color_temp
) {
    uint8_t g_b = GAMMA_LUT[brightness];
    uint8_t warm_white = alpha_blend(g_b, 255 - color_temp);
    uint8_t cool_white = alpha_blend(g_b, color_temp);
    set_led_strip(led_strip, warm_white, cool_white);
}

static void interpolate_led_strip(
    led_strip_handle_t led_strip,
    struct led_state_req_s *led_state_req,
    struct led_state_s *current_led_state
) {
    /* Override the brightness if the LED is actually supposed to be off since
        the brightness value doesn't change for that */
    uint8_t actual_req_brightness = led_state_req->brightness;
    if (led_state_req->power != 1) {
        actual_req_brightness = 0;
    }

    /* Interpolating brightness and color temperature are 2 independent
        operations, so the largest delta is used to determine how fast the
        interpolation should be */
    uint8_t delta_brightness = abs(actual_req_brightness - current_led_state->brightness);
    uint8_t delta_color_temp = abs(led_state_req->color_temp - current_led_state->color_temp);
    uint8_t max_delta = MAX(delta_brightness, delta_color_temp);

    /*
        Calculate the interpolation step by remapping the max delta (0-255) to a
        range of 4 to 7
        
        The linear equation for this is y = (1 / 64) * x + 4
        Rewriting this for fixed point math, we get y = (x + 256) / 64

        Since a division by 64 is just a bit-shift, this can be simplified even
        further which gives us the resulting equation!
    */
    uint32_t interpolation_step = (max_delta + 256) >> 6;

    uint8_t b_i = current_led_state->brightness;
    uint8_t ct_i = current_led_state->color_temp;
    uint8_t interpolation_interrupted = 0;
    for (int i = interpolation_step; i < 256; i += interpolation_step) {
        /* Check if there's any new LED state requests in the queue. If there
            are, then stop this interpolation so the next one can start
            immediately. */
        if (uxQueueMessagesWaiting(state_req_queue)) {
            interpolation_interrupted = 1;
            break;
        }

        /* Use sine interpolation for a smoother fade */
        uint8_t alpha = SIN_LUT_0_90[i];

        /* Interpolate brightness and color temperature */
        b_i = interpolate(
            current_led_state->brightness,
            actual_req_brightness,
            alpha
        );
        ct_i = interpolate(
            current_led_state->color_temp,
            led_state_req->color_temp,
            alpha
        );

        set_led_strip_from_norm_led_state(
            led_strip,
            b_i,
            ct_i
        );

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (interpolation_interrupted) {
        /* In the event that the interpolation is interrupted because of a new
            LED state request, the LED state is updated with the last
            interpolated value so that the next interpolation starts from where
            this one left off */
        current_led_state->brightness = b_i;
        current_led_state->color_temp = ct_i;
    } else {
        /* If the interpolation completed, then set the LED strip and LED state
            to the final brightness and color temperature values. This is done
            because the interpolation step might not be a factor of 256. */
        set_led_strip_from_norm_led_state(
            led_strip,
            actual_req_brightness,
            led_state_req->color_temp
        );

        current_led_state->brightness = actual_req_brightness;
        current_led_state->color_temp = led_state_req->color_temp;
    }
}

/*******************************************************************************
-- PUBLIC FUNCTIONS --
*******************************************************************************/

/*!
    @brief Initialize the LED strip and request queue

    @param gpio_pin GPIO pin the LED strip is connected to
    @param num_leds Number of LEDs in the strip

    @return
    - ESP_OK if the LED strip initialized successfully
    - Error codes if the LED strip failed to initialize
*/
esp_err_t led_strip_init(uint32_t gpio_pin, uint32_t num_leds) {
    state_req_queue = xQueueCreate(4, sizeof(struct led_state_req_s));
    esp_err_t err = led_strip_configure(gpio_pin, num_leds);
    if (err != ESP_OK) {
        ESP_LOGE(DEBUG_TAG, "Error initializing the LED strip");
        return err;
    }
    config_num_leds = num_leds;
    return ESP_OK;
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