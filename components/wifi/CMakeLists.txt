set(component_srcs "src/wifi.c")

idf_component_register(
    SRCS "${component_srcs}"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS ""
    REQUIRES esp_common esp_wifi
    PRIV_REQUIRES nvs_flash log esp_event 
)