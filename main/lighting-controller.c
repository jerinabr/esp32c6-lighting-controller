#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "cJSON.h"

/*
    Message tag used for ESP_LOGI messages
*/
#define DEBUG_TAG   "app"

/*
    Configuration Settings
*/
#define WIFI_SSID                   CONFIG_WIFI_SSID
#define WIFI_PASS                   CONFIG_WIFI_PASS
#define WIFI_MAX_RECONNECT_ATTEMPTS CONFIG_WIFI_MAX_RECONNECT_ATTEMPTS

#define MQTT_BROKER_URL             CONFIG_MQTT_BROKER_URL
#define MQTT_CLIENT_USER            CONFIG_MQTT_CLIENT_USER
#define MQTT_CLIENT_PASS            CONFIG_MQTT_CLIENT_PASS

/*
    Event Group Status Bits
*/
#define WIFI_CONNECT_SUCCESS_BIT    BIT0
#define WIFI_CONNECT_FAILED_BIT     BIT1

#define MQTT_CLIENT_CONNECTED       BIT0

/*
    MQTT Message Queue Properties
*/
#define MQTT_MSG_QUEUE_DEPTH    8
#define MQTT_MSG_MAX_TOPIC_LEN  64
#define MQTT_MSG_MAX_DATA_LEN   256

/*
    MQTT Message Struct
*/
struct mqtt_msg_t {
    char topic[MQTT_MSG_MAX_TOPIC_LEN];
    int topic_len;
    char data[MQTT_MSG_MAX_DATA_LEN];
    int data_len;
};

/*
    Variables
*/
static EventGroupHandle_t wifi_event_group;
static EventGroupHandle_t mqtt_event_group;
static QueueHandle_t mqtt_msg_queue;

static uint8_t wifi_reconnect_attempt_count = 0;

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
                if (wifi_reconnect_attempt_count < WIFI_MAX_RECONNECT_ATTEMPTS) {
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
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
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
    } else if (wifi_event_group_bits & WIFI_CONNECT_FAILED_BIT) {
        ESP_LOGI(DEBUG_TAG, "Couldn't connect to Wi-Fi!!!!!");
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
            struct mqtt_msg_t mqtt_msg;

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
            if (stat == errQUEUE_FULL) {
                ESP_LOGI(DEBUG_TAG, "MQTT message queue full!");
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
    mqtt_msg_queue = xQueueCreate(
        MQTT_MSG_QUEUE_DEPTH,
        sizeof(struct mqtt_msg_t)
    );

    /* Initialize MQTT client */
    esp_mqtt_client_config_t mqtt5_cfg = {
        .broker.address.uri = MQTT_BROKER_URL,
        .credentials.username = MQTT_CLIENT_USER,
        .credentials.authentication.password = MQTT_CLIENT_PASS,
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
    @brief Parse MQTT message and 
*/
static void process_mqtt_msg(const struct mqtt_msg_t *msg) {
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

    /* Extract JSON values */
    const cJSON *pwr = cJSON_GetObjectItemCaseSensitive(
        msg_json,
        "power"
    );
    if (cJSON_IsString(pwr) && (pwr->valuestring != NULL)) {
        ESP_LOGI(DEBUG_TAG, "Power %s", pwr->valuestring);
    }

    const cJSON *brightness = cJSON_GetObjectItemCaseSensitive(
        msg_json,
        "brightness"
    );
    if (cJSON_IsNumber(brightness)) {
        ESP_LOGI(DEBUG_TAG, "Brightness: %d", brightness->valueint);
    }

    const cJSON *color_temp = cJSON_GetObjectItemCaseSensitive(
        msg_json,
        "temp"
    );
    if (cJSON_IsNumber(color_temp)) {
        ESP_LOGI(DEBUG_TAG, "Color Temperature: %dK", color_temp->valueint);
    }
}

/*!
    @brief Task that blocks until an MQTT message is available
*/
static void mqtt_msg_handler_task(void *arg) {
    while (1) {
        struct mqtt_msg_t msg;
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

void app_main(void) {
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

    /* Connect to the WIFI */
    wifi_sta_init();

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

    /* Subscribe to a test topic */
    esp_mqtt5_subscribe_property_config_t sub_property = {
        .subscribe_id = 25555,
        .no_local_flag = false,
        .retain_as_published_flag = false,
        .retain_handle = 0,
    };
    esp_mqtt5_client_set_subscribe_property(client, &sub_property);
    esp_mqtt_client_subscribe(client, "kitchen/under-cabinet-light/cmd", 0);

    /* Create tasks */
    xTaskCreate(
        mqtt_msg_handler_task,
        "mqtt_message_handler_task",
        4096,
        NULL,
        0,
        NULL
    );
}