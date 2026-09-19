/*!
    @file:	wifi_handler.h
    @brief:	Manage the WIFI station initialization and connection
*/
#ifndef WIFI_HANDLER_H
#define WIFI_HANDLER_H

#include "esp_err.h"
#include "esp_wifi_types_generic.h"
#ifdef __cplusplus
extern "C" {
#endif

/*
    Functions
*/
esp_err_t wifi_init(
    wifi_config_t *wifi_config
);

esp_err_t wifi_connect(
    uint32_t max_connect_attempts,
    uint32_t connect_retry_delay_ms
);

#ifdef __cplusplus
}
#endif

#endif