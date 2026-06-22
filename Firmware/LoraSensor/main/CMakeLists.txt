FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/main/*.*)

idf_component_register(SRCS ${app_sources}
                        REQUIRES esp_https_ota app_update)

# idf_component_register(SRCS "wifiClient.cpp" "settings.cpp" "main.cpp" "SHTC3/SHTC3.cpp" "LoraClient.cpp" "adc.cpp" "settings.cpp" "EspHal.cpp"
#                     INCLUDE_DIRS ".")
