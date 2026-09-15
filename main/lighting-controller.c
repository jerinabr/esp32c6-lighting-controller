#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "cJSON.h"
#include "led_strip.h"
#include "driver/gpio.h"
#include "math.h"

/*
    Message tag used for ESP_LOGI messages
*/
#define DEBUG_TAG   "app"

/*
    LED strip initial conditions
*/
#define INITIAL_BRIGHTNESS  32
#define INITIAL_COLOR_TEMP  0

/*
    GPIO pins
*/
#define STATUS_LED_N        15
#define RF_SWITCH_EN_N      3
#define RF_SWITCH_PORT_SEL  14

/*
    GPIO states
*/
#define EXT_ANTENNA_SEL     1
#define CHIP_ANTENNA_SEL    0

/*
    Event group status bits
*/
#define WIFI_CONNECT_SUCCESS_BIT    BIT0
#define WIFI_CONNECT_FAILED_BIT     BIT1

#define MQTT_CLIENT_CONNECTED       BIT0

/*
    MQTT message queue properties
*/
#define MQTT_MSG_MAX_TOPIC_LEN  64
#define MQTT_MSG_MAX_DATA_LEN   256

/*
    MQTT message struct
*/
struct mqtt_msg_s {
    char topic[MQTT_MSG_MAX_TOPIC_LEN];
    int topic_len;
    char data[MQTT_MSG_MAX_DATA_LEN];
    int data_len;
};

/*
    LED state and state request structs
*/
struct led_state_s {
    uint8_t brightness;
    uint8_t color_temp;
};

struct led_state_req_s {
    uint8_t power;
    uint8_t brightness;
    uint8_t brightness_valid;
    uint8_t color_temp;
    uint8_t color_temp_valid;
};

/*
    Constants
*/
const float COLOR_TEMP_MAP_SCALE = 255.0 / (CONFIG_LED_STRIP_MAX_KELVIN - CONFIG_LED_STRIP_MIN_KELVIN);

/*
    Variables
*/
static EventGroupHandle_t wifi_event_group;
static EventGroupHandle_t mqtt_event_group;
static QueueHandle_t mqtt_msg_queue;
static QueueHandle_t led_state_req_queue;

static uint8_t wifi_reconnect_attempt_count = 0;

static uint8_t sys_ready = 0;

/*!
    @brief Callback for default events

    @param handler_args Arguments that are passed from the event handler
    registration
    @param event_base Event base of the event that happened
    @param event_id ID of the event that happened
    @param event_data Additional data about the event
*/
static void default_event_handler(
    void *handler_args,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data
) {
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            /* When the station starts, try to connect to the WIFI with the
                provided credentials. */
            case WIFI_EVENT_STA_START: {
                esp_wifi_connect();
                break;
            }
            /* When the station disconnects from the network, try to reconnect
                until the maximum number of retries has been reached. Once that
                happens, indicate in the event group that the connection
                failed. */
            case WIFI_EVENT_STA_DISCONNECTED: {
                if (wifi_reconnect_attempt_count < CONFIG_WIFI_MAX_RECONNECT_ATTEMPTS) {
                    esp_wifi_connect();
                    wifi_reconnect_attempt_count++;
                    ESP_LOGI(
                        DEBUG_TAG,
                        "Lost Wi-Fi connection. Reconnecting..."
                    );
                } else {
                    xEventGroupSetBits(
                        wifi_event_group,
                        WIFI_CONNECT_FAILED_BIT
                    );
                }
                break;
            }
            /* Log any other event. I'll need to compare the event_id printed to
                the enum list so that's a drag. */
            default: {
                ESP_LOGI(DEBUG_TAG, "Wi-Fi event: %d", event_id);
                break;
            }
        }
    } else if (event_base == IP_EVENT) {
        switch (event_id) {
            /* When an IP address has been acquired, indicate in the event group
                that the connection was a success */
            case IP_EVENT_STA_GOT_IP: {
                ip_event_got_ip_t *event = (ip_event_got_ip_t*) event_data;
                ESP_LOGI(
                    DEBUG_TAG,
                    "Connected to Wi-Fi with IP address: " IPSTR,
                    IP2STR(&event->ip_info.ip)
                );
                xEventGroupSetBits(
                    wifi_event_group,
                    WIFI_CONNECT_SUCCESS_BIT
                );
                /* Reset the retry count once connected to WIFI */
                wifi_reconnect_attempt_count = 0;
                break;
            }
            /* Log any other event. I'll need to compare the event_id printed to
                the enum list so that's a drag. */
            default: {
                ESP_LOGI(DEBUG_TAG, "IP event: %d", event_id);
                break;
            }
        }
    }
}

/*!
    @brief Initialize the WIFI station and connect to the network
*/
void wifi_sta_init() {
    wifi_event_group = xEventGroupCreate();

    /* Initialize the TCP/IP stack */
    esp_netif_init();

    /* Create the default event loop. This event loop is used to monitor the
        WIFI and IP events. */
    esp_event_loop_create_default();

    /* Create a ESP-NETIF object with default configuration for WIFI station.
        This has to be called AFTER the default event loop is created. */
    esp_netif_create_default_wifi_sta();
    
    /* Initialize the WIFI driver with default configuration */
    wifi_init_config_t wifi_driver_config = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_driver_config);

    /* Register the event handler as a callback to the specified WIFI and IP
        events */
    esp_event_handler_instance_t wifi_event_inst;
    esp_event_handler_instance_t ip_event_inst;
    esp_event_handler_instance_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &default_event_handler,
        NULL,
        &wifi_event_inst
    );
    esp_event_handler_instance_register(
        IP_EVENT, 
        ESP_EVENT_ANY_ID, 
        &default_event_handler,
        NULL,
        &ip_event_inst
    );

    /* Create the WIFI station configuration */
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .disable_wpa3_compatible_mode = 0,
        },
    };

    /* Start WIFI in station mode with the above configuration */
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();

    /* Block until either the WIFI has successfully connected or the maximum
        number of connection retries has been reached */
    EventBits_t wifi_event_group_bits = xEventGroupWaitBits(
        wifi_event_group,
        WIFI_CONNECT_SUCCESS_BIT | WIFI_CONNECT_FAILED_BIT,
        pdFALSE,
        pdFALSE,
        portMAX_DELAY
    );

    /* Verify which event actually happened */
    if (wifi_event_group_bits & WIFI_CONNECT_SUCCESS_BIT) {
        ESP_LOGI(DEBUG_TAG, "Connected to Wi-Fi!");
        
        wifi_ap_record_t ap_info;
        esp_wifi_sta_get_ap_info(&ap_info);
        ESP_LOGI(DEBUG_TAG, "RSSI: %d dBm", ap_info.rssi);
    } else if (wifi_event_group_bits & WIFI_CONNECT_FAILED_BIT) {
        ESP_LOGE(DEBUG_TAG, "Couldn't connect to Wi-Fi!!!!!");
    } else {
        ESP_LOGE(DEBUG_TAG, "Bruh what happened");
    }
}

/*!
    @brief Handle different MQTT events
    
    @param handler_args Arguments that are passed from the event handler
    registration
    @param event_base Event base of the event that happened
    @param event_id ID of the event that happened
    @param event_data Additional data about the event
*/
static void mqtt_event_handler(
    void *handler_args,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data
) {
    esp_mqtt_event_handle_t event = event_data;
    /* The event base is always an MQTT event since this was registered with the
        MQTT client so we don't bother checking that */
    switch (event_id) {
        /* Log when the MQTT client connects */
        case MQTT_EVENT_CONNECTED: {
            xEventGroupSetBits(
                mqtt_event_group,
                MQTT_CLIENT_CONNECTED
            );
            break;
        }

        case MQTT_EVENT_DISCONNECTED: {
            ESP_LOGI(DEBUG_TAG, "MQTT client disconnected");
            break;
        }

        case MQTT_EVENT_SUBSCRIBED: {
            ESP_LOGI(DEBUG_TAG, "MQTT client subscribed to topic");
            break;
        }

        case MQTT_EVENT_UNSUBSCRIBED: {
            ESP_LOGI(DEBUG_TAG, "MQTT client unsubscribed from topic");
            break;
        }

        case MQTT_EVENT_PUBLISHED: {
            ESP_LOGI(DEBUG_TAG, "MQTT client published message");
            break;
        }

        case MQTT_EVENT_DATA: {
            ESP_LOGI(DEBUG_TAG, "MQTT client received data");
            struct mqtt_msg_s mqtt_msg;

            /* Don't put the message in the queue if it's too long */
            if (
                event->topic_len > MQTT_MSG_MAX_TOPIC_LEN ||
                event->data_len > MQTT_MSG_MAX_DATA_LEN
            ) {
                break;
            } else {
                mqtt_msg.topic_len = event->topic_len;
                mqtt_msg.data_len = event->data_len;
            }
            
            /* Copy the contents of the message topic and payload. The message
                doesn't need to be null-terminated because the length of the
                message is used instead. */
            memcpy(mqtt_msg.topic, event->topic, event->topic_len);
            memcpy(mqtt_msg.data, event->data, event->data_len);
            
            /* Put the MQTT message struct into the queue */
            BaseType_t stat = xQueueSend(mqtt_msg_queue, (void*) &mqtt_msg, 0);
            if (stat != pdTRUE) {
                ESP_LOGE(DEBUG_TAG, "Failed to add MQTT message to queue");
            }
            break;
        }

        case MQTT_EVENT_ERROR: {
            ESP_LOGI(
                DEBUG_TAG,
                "MQTT client error. Return code: %d",
                event->error_handle->connect_return_code
            );
            break;
        }

        default: {
            ESP_LOGI(DEBUG_TAG, "MQTT event: %d", event->event_id);
            break;
        }
    }
}

/*!
    @brief Start the MQTT5 client

    @param client Pointer to the MQTT client
*/
static void mqtt5_client_init(esp_mqtt_client_handle_t *client) {
    mqtt_event_group = xEventGroupCreate();

    /* Initialize MQTT client */
    esp_mqtt_client_config_t mqtt5_cfg = {
        .broker.address.uri = CONFIG_MQTT_BROKER_URL,
        .credentials.username = CONFIG_MQTT_CLIENT_USER,
        .credentials.authentication.password = CONFIG_MQTT_CLIENT_PASS,
        .session.last_will.topic = "topic/device-last-will",
        .session.last_will.msg = "ESP32C6 disconnected from MQTT",
        .session.last_will.msg_len = 30,
        .session.last_will.qos = 1,
        .session.last_will.retain = true,
        .session.protocol_ver = MQTT_PROTOCOL_V_5,
        .network.disable_auto_reconnect = true,
    };
    *client = esp_mqtt_client_init(&mqtt5_cfg);

    /* Set connection properties */
    esp_mqtt5_connection_property_config_t connect_property = {
        /* Session expires 2 minutes after disconnection */
        .session_expiry_interval = 120,
        .maximum_packet_size = 1024,
        .receive_maximum = 1024,
        .topic_alias_maximum = 2,
        .request_resp_info = true,
        .request_problem_info = true,
        /* If the device doesn't reconnect to the broker within 10 seconds, the
            broker will send the last will message out. */
        .will_delay_interval = 10,
        .message_expiry_interval = 120,
        .response_topic = "test/response",
        .correlation_data = "124356",
        .correlation_data_len = 6,
    };
    esp_mqtt5_client_set_connect_property(
        *client,
        &connect_property
    );

    /* Register the MQTT event handler and start the MQTT client */
    esp_mqtt_client_register_event(
        *client,
        ESP_EVENT_ANY_ID,
        mqtt_event_handler,
        NULL
    );
    esp_mqtt_client_start(*client);

    /* Block until the MQTT client has connected */
    xEventGroupWaitBits(
        mqtt_event_group,
        MQTT_CLIENT_CONNECTED,
        pdFALSE,
        pdTRUE,
        portMAX_DELAY
    );
    ESP_LOGI(DEBUG_TAG, "MQTT client connected!");
}

/*!
    @brief Parse MQTT message and add the requested LED state to the LED state
    request queue
    @param msg Pointer to the MQTT message struct that contains the MQTT message
    to be processed
*/
static void process_mqtt_msg(const struct mqtt_msg_s *msg) {
    const char *err;
    cJSON *msg_json = cJSON_ParseWithLengthOpts(
        msg->data,
        msg->data_len,
        &err,
        0
    );

    /* If the JSON message couldn't be parsed, delete the JSON object and print
        the error */
    if (msg_json == NULL) {
        if (err != NULL) {
            ESP_LOGI(DEBUG_TAG, "JSON parsing error at: %s", err);
        }
        cJSON_Delete(msg_json);
        return;
    }

    /* Initialize the LED state struct */
    struct led_state_req_s led_state_req = {
        .power = 0,
        .brightness = 0,
        .brightness_valid = 0,
        .color_temp = 0,
        .color_temp_valid = 0
    };

    /* Extract JSON values and populate the led state struct with them. Because
        Home Assistant only sends the UPDATED values when the LED state is
        changed, the "_valid" members are set to indicate the associated member
        was updated. */
    const cJSON *power_json = cJSON_GetObjectItemCaseSensitive(
        msg_json,
        "power"
    );
    if (cJSON_IsNumber(power_json)) {
        ESP_LOGI(DEBUG_TAG, "Power: %d", power_json->valueint);

        /* The power value in the message should only ever be 0 or 1, but if for
            some reason the format changes this will keep it from breaking. */
        if (power_json->valueint > 0) {
            led_state_req.power = 1;
        }
    } else {
        ESP_LOGW(DEBUG_TAG, "LED state not indicated in message");
    }

    const cJSON *brightness_json = cJSON_GetObjectItemCaseSensitive(
        msg_json,
        "brightness"
    );
    if (cJSON_IsNumber(brightness_json)) {
        ESP_LOGI(DEBUG_TAG, "Brightness: %d", brightness_json->valueint);

        if (brightness_json->valueint > CONFIG_LED_STRIP_MAX_BRIGHTNESS) {
            led_state_req.brightness = CONFIG_LED_STRIP_MAX_BRIGHTNESS;
        } else if (brightness_json->valueint < 0) {
            led_state_req.brightness = 0;
        } else {
            led_state_req.brightness = brightness_json->valueint;
        }
        led_state_req.brightness_valid = 1;
    }

    const cJSON *color_temp_json = cJSON_GetObjectItemCaseSensitive(
        msg_json,
        "temp"
    );
    if (cJSON_IsNumber(color_temp_json)) {
        ESP_LOGI(DEBUG_TAG, "Color Temperature: %dK", color_temp_json->valueint);

        uint32_t color_temp = color_temp_json->valueint;
        if (color_temp_json->valueint > CONFIG_LED_STRIP_MAX_KELVIN) {
            color_temp = CONFIG_LED_STRIP_MAX_KELVIN;
        } else if (color_temp_json->valueint < CONFIG_LED_STRIP_MIN_KELVIN) {
            color_temp = CONFIG_LED_STRIP_MIN_KELVIN;
        }

        /* Map the color temperature to a value between 0 and 255 */
        color_temp -= CONFIG_LED_STRIP_MIN_KELVIN;
        float color_temp_map = roundf(color_temp * COLOR_TEMP_MAP_SCALE);

        led_state_req.color_temp = (uint8_t) color_temp_map;
        led_state_req.color_temp_valid = 1;

        ESP_LOGI(DEBUG_TAG, "Mapped color temp: %d", led_state_req.color_temp);
    }

    /* Add the new LED state to the queue for processing */
    BaseType_t stat = xQueueSend(
        led_state_req_queue,
        (void*) &led_state_req,
        0
    );
    if (stat != pdTRUE) {
        ESP_LOGE(DEBUG_TAG, "Failed to add new led state to queue");
    }
}

/*!
    @brief Task that blocks until an MQTT message is available
*/
static void mqtt_msg_handler_task(void *args) {
    while (1) {
        struct mqtt_msg_s msg;
        BaseType_t stat = xQueueReceive(
            mqtt_msg_queue,
            (void*) &msg,
            portMAX_DELAY
        );
        if (stat == pdTRUE) {
            process_mqtt_msg(&msg);
        }
    }
}

/*!
    @brief Initialize the LED strip

    @param led_strip Pointer to the LED strip handle that will be initialized
*/
static esp_err_t led_strip_init(led_strip_handle_t *led_strip) {
    /* Create LED strip configuration */
    led_strip_config_t strip_config = {
        .strip_gpio_num = CONFIG_LED_STRIP_GPIO_PIN,
        .max_leds = CONFIG_LED_STRIP_LED_COUNT,
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
        led_strip
    );
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
        if (uxQueueMessagesWaiting(led_state_req_queue)) {
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

/*!
    @brief Task that initializes and updates the LED strip when there's a new
    desired state
*/
static void led_strip_task(void *args) {
    /* Initialize the LED strip */
    led_strip_handle_t led_strip;
    esp_err_t err = led_strip_init(&led_strip);
    if (err != ESP_OK) {
        ESP_LOGE(DEBUG_TAG, "Error initializing the LED strip");
        vTaskDelete(NULL);
    }
    led_strip_clear(led_strip);
    
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
            led_state_req_queue,
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
    @brief Task that blinks the status LED during system initialization and
    holds it steady once initialized
*/
static void status_led_task(void *args) {
    /* Configure status LED GPIO */
    gpio_reset_pin(STATUS_LED_N);
    gpio_set_direction(STATUS_LED_N, GPIO_MODE_OUTPUT);

    /* Blink the LED while the system isnt ready */
    int led_state = 1;
    while (!sys_ready) {
        gpio_set_level(STATUS_LED_N, led_state);
        led_state = !led_state;
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    /* Keep the status LED on once initialized and delete the task since this
        task is complete */
    gpio_set_level(STATUS_LED_N, 0);
    vTaskDelete(NULL);
}

void app_main(void) {
    /* Create status LED task to blink during setup */
    xTaskCreate(
        status_led_task,
        "status_led_task",
        1024,
        NULL,
        0,
        NULL
    );

    /* WIFI driver configuration is stored in NVS so it needs to be initialized
        before WIFI is initialized. If the NVS partition has no empty pages or
        the data is in a bad format, the flash needs to be erased before being
        initialized again. */
    esp_err_t err = nvs_flash_init();
    if (
        err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND
    ) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* Enable the RF switch and select the chip antenna as the RF path. If this
        isn't done, the WIFI will still work because of RF leakage, but it will
        work very poorly. */
    gpio_reset_pin(RF_SWITCH_EN_N);
    gpio_set_direction(RF_SWITCH_EN_N, GPIO_MODE_OUTPUT);
    gpio_set_level(RF_SWITCH_EN_N, 0);

    gpio_reset_pin(RF_SWITCH_PORT_SEL);
    gpio_set_direction(RF_SWITCH_PORT_SEL, GPIO_MODE_OUTPUT);
    gpio_set_level(RF_SWITCH_PORT_SEL, CHIP_ANTENNA_SEL);

    /* Connect to the WIFI */
    wifi_sta_init();

    /* Create queues */
    mqtt_msg_queue = xQueueCreate(8, sizeof(struct mqtt_msg_s));
    led_state_req_queue = xQueueCreate(4, sizeof(struct led_state_req_s));

    /* Initialize the MQTT5 client */
    esp_mqtt_client_handle_t client;
    mqtt5_client_init(&client);

    /* Create publish configuration */
    esp_mqtt5_publish_property_config_t pub_property = {
        .payload_format_indicator = 1,
        .message_expiry_interval = 1000,
        .topic_alias = 0,
        .response_topic = "topic/test/response",
        .correlation_data = "123456",
        .correlation_data_len = 6,
    };
    esp_mqtt5_client_set_publish_property(client, &pub_property);

    /* Subscribe to the control topic */
    esp_mqtt5_subscribe_property_config_t sub_property = {
        .subscribe_id = 25555,
        .no_local_flag = false,
        .retain_as_published_flag = false,
        .retain_handle = 0,
    };
    esp_mqtt5_client_set_subscribe_property(client, &sub_property);
    esp_mqtt_client_subscribe(client, "kitchen/under-cabinet-light/cmd", 0);

    /* Indicate system ready to make the status LED stop blinking */
    sys_ready = 1;

    /* Create tasks */
    xTaskCreate(
        mqtt_msg_handler_task,
        "mqtt_message_handler_task",
        4096,
        NULL,
        1,
        NULL
    );
    xTaskCreate(
        led_strip_task,
        "led_strip_task",
        4096,
        NULL,
        0,
        NULL
    );
}