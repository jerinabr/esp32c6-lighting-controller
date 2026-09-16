#include "wifi.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

/*
    Message tag used for ESP_LOGI messages
*/
#define DEBUG_TAG "WIFI"

/*
    Event group status bits
*/
#define WIFI_CONNECT_SUCCESS_BIT    BIT0
#define WIFI_CONNECT_FAILED_BIT     BIT1

static EventGroupHandle_t wifi_event_group;
static uint8_t reconnect_attempt_count = 0;

/*******************************************************************************
-- PRIVATE FUNCTIONS --
*******************************************************************************/

/*!
    @brief Callback for WIFI events

    @param handler_args Arguments that are passed from the event handler
    registration
    @param event_base Event base of the event that happened
    @param event_id ID of the event that happened
    @param event_data Additional data about the event
*/
static void wifi_event_handler(
    void *handler_args,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data
) {
    switch (event_id) {
        /* When the station starts, try to connect to the WIFI with the provided
            credentials. */
        case WIFI_EVENT_STA_START: {
            esp_wifi_connect();
            break;
        }
        /* When the station disconnects from the network, try to reconnect until
            the maximum number of retries has been reached. Once that happens,
            indicate in the event group that the connection failed. */
        case WIFI_EVENT_STA_DISCONNECTED: {
            if (reconnect_attempt_count < CONFIG_WIFI_MAX_RECONNECT_ATTEMPTS) {
                esp_wifi_connect();
                reconnect_attempt_count++;
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
        /* Log any other event. I'll need to compare the event_id printed to the
            enum list so that's a drag. */
        default: {
            ESP_LOGI(DEBUG_TAG, "Wi-Fi event: %d", event_id);
            break;
        }
    }
}

/*!
    @brief Callback for IP events

    @param handler_args Arguments that are passed from the event handler
    registration
    @param event_base Event base of the event that happened
    @param event_id ID of the event that happened
    @param event_data Additional data about the event
*/
static void ip_event_handler(
    void *handler_args,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data
) {
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
            reconnect_attempt_count = 0;
            break;
        }
        /* Log any other event. I'll need to compare the event_id printed to the
            enum list so that's a drag. */
        default: {
            ESP_LOGI(DEBUG_TAG, "IP event: %d", event_id);
            break;
        }
    }
}

/*!
    @brief Initialize the WIFI station and connect to the network
*/
static void wifi_sta_init() {
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
        &wifi_event_handler,
        NULL,
        &wifi_event_inst
    );
    esp_event_handler_instance_register(
        IP_EVENT, 
        ESP_EVENT_ANY_ID, 
        &ip_event_handler,
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

/*******************************************************************************
-- PUBLIC FUNCTIONS --
*******************************************************************************/

void wifi_init() {
    wifi_event_group = xEventGroupCreate();

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
}