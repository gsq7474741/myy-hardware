set(ESP_WIFI_SSID pendulum)
set(ESP_WIFI_PASSWORD 03010925)

idf_component_register(
        SRCS "main.c"
        INCLUDE_DIRS "E:/VSCODE/esp-project/myy-hardware-1.0.1/components/C-SDK"
        INCLUDE_DIRS "E:/VSCODE/ESP-IDF/esp-idf/v5.3.1/esp-idf/components"
)
