/*!
    @file:	wifi.h
    @brief:	Device WIFI management
*/
#ifndef WIFI_H
#define WIFI_H

#ifdef __cplusplus
extern "C" {
#endif

void wifi_init();
void wifi_task(void *args);

#ifdef __cplusplus
}
#endif

#endif