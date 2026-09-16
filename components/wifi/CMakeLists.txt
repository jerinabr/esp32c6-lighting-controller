set(component_srcs "src/wifi.c")

idf_component_register(
    SRCS "${component_srcs}"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS ""
    REQUIRES ""
    PRIV_REQUIRES nvs_flash esp_common log esp_event esp_wifi
)