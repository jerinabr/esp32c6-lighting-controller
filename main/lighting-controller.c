#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "mqtt_client.h"

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
    Event group status bits
*/
#define WIFI_CONNECT_FAILED_BIT     BIT1
#define WIFI_CONNECT_SUCCESS_BIT    BIT0

#define MQTT_CONNECTED  BIT0

/*
    Variables
*/
static EventGroupHandle_t wifi_event_group;
static EventGroupHandle_t mqtt_event_group;

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
                MQTT_CONNECTED
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
            ESP_LOGI(
                DEBUG_TAG,
                "TOPIC: %.*s",
                event->topic_len,
                event->topic
            );
            ESP_LOGI(
                DEBUG_TAG,
                "DATA: %.*s",
                event->data_len,
                event->data
            );
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
            ESP_LOGI(
                DEBUG_TAG,
                "MQTT event: %d",
                event->event_id
            );
            break;
        }
    }
}

/*!
    @brief Start the MQTT5 client

    @param client Pointer to the MQTT client
*/
static void mqtt5_client_init(esp_mqtt_client_handle_t *client) {
    /* Create the MQTT event group */
    mqtt_event_group = xEventGroupCreate();

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
        MQTT_CONNECTED,
        pdFALSE,
        pdTRUE,
        portMAX_DELAY
    );
    ESP_LOGI(DEBUG_TAG, "MQTT client connected!");
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
    esp_mqtt_client_subscribe(client, "topic/qos0", 0);

    /* Publish a message every 10 seconds */
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        esp_mqtt_client_publish(
            client,
            "topic/qos1",
            "Hello from ESP32C6",
            0,
            0,
            0
        );
        ESP_LOGI(DEBUG_TAG, "MQTT client published a message");
    }
}