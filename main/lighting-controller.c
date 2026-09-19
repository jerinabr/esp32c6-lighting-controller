#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "cJSON.h"
#include "driver/gpio.h"
#include "math.h"
#include "wifi_handler.h"
#include "led_strip_handler.h"

/*
    Message tag used for ESP_LOGI messages
*/
#define DEBUG_TAG   "app"

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
#define MQTT_CLIENT_CONNECTED   BIT0

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
    Constants
*/
const float COLOR_TEMP_MAP_SCALE = 255.0 / (CONFIG_LED_STRIP_MAX_KELVIN - CONFIG_LED_STRIP_MIN_KELVIN);

/*
    Variables
*/
static EventGroupHandle_t mqtt_event_group;
static QueueHandle_t mqtt_msg_queue;

static uint8_t sys_ready = 0;

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
    update_led_state(&led_state_req);
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
    esp_err_t err;

    /* Create status LED task to blink during setup */
    xTaskCreate(
        status_led_task,
        "status_led_task",
        1024,
        NULL,
        0,
        NULL
    );

    /* Initialize the LED strip */
    err = led_strip_init(
        CONFIG_LED_STRIP_GPIO_PIN,
        CONFIG_LED_STRIP_LED_COUNT
    );
    if (err != ESP_OK) {
        return;
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

    /* Create the WIFI station configuration */
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .disable_wpa3_compatible_mode = 0,
        },
    };
    
    /* Initialize the WIFI system */
    err = wifi_init(&wifi_config);
    if (err != ESP_OK) {

        return;
    }

    /* Connect to the WIFI network */
    err = wifi_connect(
        CONFIG_WIFI_MAX_CONNECTION_ATTEMPTS,
        CONFIG_WIFI_CONNECT_RETRY_DELAY_MS
    );
    if (err != ESP_OK) {
        return;
    }

    /* Create queues */
    mqtt_msg_queue = xQueueCreate(8, sizeof(struct mqtt_msg_s));

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