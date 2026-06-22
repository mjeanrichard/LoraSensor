FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/main/*.*)

idf_component_register(
    SRCS ${app_sources}
    REQUIRES
        esp_https_ota 
        app_update
        esp_adc 
        esp_driver_ledc 
        esp_driver_gpio 
        esp_driver_i2c 
        esp_driver_uart
        esp_driver_spi
        esp_timer
        esp_wifi 
        esp_netif 
        esp_event
        nvs_flash 
        mbedtls
        efuse 
        esp_driver_usb_serial_jtag
    )

# idf_component_register(SRCS "wifiClient.cpp" "settings.cpp" "main.cpp" "SHTC3/SHTC3.cpp" "LoraClient.cpp" "adc.cpp" "settings.cpp" "EspHal.cpp"
#                     INCLUDE_DIRS ".")
