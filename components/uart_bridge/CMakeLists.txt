idf_component_register(
    SRCS "uart_bridge.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_uart data_bridge
)
