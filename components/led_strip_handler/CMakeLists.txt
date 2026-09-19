set(component_srcs "src/led_strip_handler.c")

idf_component_register(
    SRCS "${component_srcs}"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS ""
    REQUIRES ""
    PRIV_REQUIRES espressif__led_strip
)