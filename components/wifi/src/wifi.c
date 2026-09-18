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
#define WIFI_CONNECTED_BIT      BIT0
#define WIFI_DISCONNECTED_BIT   BIT1
#define IP_ACQUIRED_BIT         BIT2

static EventGroupHandle_t network_event_group;
static uint32_t connect_attempt_count = 0;

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
        case WIFI_EVENT_STA_CONNECTED: {
            xEventGroupSetBits(
                network_event_group,
                WIFI_CONNECTED_BIT
            );
            break;
        }
        
        case WIFI_EVENT_STA_DISCONNECTED: {
            xEventGroupSetBits(
                network_event_group,
                WIFI_DISCONNECTED_BIT
            );
            break;
        }
        
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
        case IP_EVENT_STA_GOT_IP: {
            xEventGroupSetBits(
                network_event_group,
                IP_ACQUIRED_BIT
            );
            break;
        }
        
        default: {
            ESP_LOGI(DEBUG_TAG, "IP event: %d", event_id);
            break;
        }
    }
}

/*!
    @brief Initialize the WIFI station
*/
static esp_err_t wifi_sta_init(
    wifi_config_t *wifi_config
) {
    esp_err_t err;

    /* Initialize the TCP/IP stack */
    err = esp_netif_init();
    if (err != ESP_OK) {
        return err;
    }

    /* Create the default event loop. This event loop is used to monitor the
        WIFI and IP events. */
    err = esp_event_loop_create_default();
    if (err != ESP_OK) {
        return err;
    }

    /* Create a ESP-NETIF object with default configuration for WIFI station.
        This has to be called AFTER the default event loop is created. */
    esp_netif_create_default_wifi_sta();
    
    /* Initialize the WIFI driver with default configuration */
    wifi_init_config_t wifi_driver_config = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&wifi_driver_config);
    if (err != ESP_OK) {
        return err;
    }

    /* Register the event handler as a callback to the specified WIFI and IP
        events */
    esp_event_handler_instance_t wifi_event_inst;
    err = esp_event_handler_instance_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &wifi_event_handler,
        NULL,
        &wifi_event_inst
    );
    if (err != ESP_OK) {
        return err;
    }

    esp_event_handler_instance_t ip_event_inst;
    err = esp_event_handler_instance_register(
        IP_EVENT, 
        ESP_EVENT_ANY_ID, 
        &ip_event_handler,
        NULL,
        &ip_event_inst
    );
    if (err != ESP_OK) {
        return err;
    }

    /* Start WIFI in station mode with the above configuration */
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_wifi_set_config(WIFI_IF_STA, wifi_config);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_wifi_start();
    if (err != ESP_OK) {
        return err;
    }
    
    return ESP_OK;
}

/*******************************************************************************
-- PUBLIC FUNCTIONS --
*******************************************************************************/

/*!
    @brief Initialize the WIFI system as a station
    @param wifi_config Pointer to the wifi configuration struct
    @return
    - ESP_OK if the WIFI station was initialized
    - error code if the WIFI station failed to initialize
*/
esp_err_t wifi_init(wifi_config_t *wifi_config) {
    network_event_group = xEventGroupCreate();
    esp_err_t err;

    /* WIFI driver configuration is stored in NVS so it needs to be initialized
        before WIFI is initialized. If the NVS partition has no empty pages or
        the data is in a bad format, the flash needs to be erased before being
        initialized again. */
    err = nvs_flash_init();
    if (
        err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND
    ) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    err = nvs_flash_init();
    if (err != ESP_OK) {
        return err;
    }

    /* Connect to the WIFI */
    err = wifi_sta_init(wifi_config);
    if (err != ESP_OK) {
        return err;
    }

    return ESP_OK;
}

/*!
    @brief Connect to the WIFI network provided by the WIFI configuration
    @param max_connect_attempts Maximum number of times the device will try to
    connect to the network. Set this to 0 if the device should try to connect
    indefinitely.
    @param connect_retry_delay_ms Milliseconds to wait before trying to
    reconnect to the network
    @return
    - ESP_OK if the device connected to the WIFI network
    - error code if the device failed to connect to the WIFI network
*/
esp_err_t wifi_connect(
    uint32_t max_connect_attempts,
    uint32_t connect_retry_delay_ms
) {
    /* Try to connect to the provided WIFI network */
    const uint8_t connect_forever = (max_connect_attempts == 0);
    uint8_t wifi_connect_success = 0;
    while (connect_forever || (connect_attempt_count < max_connect_attempts)) {
        connect_attempt_count++;
        ESP_LOGI(DEBUG_TAG, "Connecting to Wi-Fi...");
        if (esp_wifi_connect() == ESP_OK) {
            wifi_connect_success = 1;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(connect_retry_delay_ms));
    }

    /* This will never be reached if the device is configured to try to connect
        to the WIFI forever */
    if (!wifi_connect_success) {
        ESP_LOGE(DEBUG_TAG, "Failed to connect to Wi-Fi");
        return ESP_FAIL;
    }

    /* Once the WIFI is connected, wait till an IP address has been acquired */
    xEventGroupWaitBits(
        network_event_group,
        IP_ACQUIRED_BIT,
        pdFALSE,
        pdFALSE,
        portMAX_DELAY
    );

    ESP_LOGI(DEBUG_TAG, "Wi-Fi connected!");
    return ESP_OK;
}