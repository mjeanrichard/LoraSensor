#pragma once

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_check.h"
#include "driver/ledc.h"
#include "config.h"
#include "freertos/FreeRTOS.h"

class Adc
{
private:
    adc_oneshot_unit_handle_t _adcHandle = nullptr;
    adc_cali_handle_t _calibrationHandle = nullptr;


    esp_err_t readRaw(uint16_t &rawValue, adc_channel_t channel);

public:
    esp_err_t initialize();
    esp_err_t readValues(float &batteryVolts, uint8_t &moisturePercent);
    esp_err_t stop();

private:
    esp_err_t readBatteryRaw(uint16_t &rawValue);
    esp_err_t readMoistureRaw(uint16_t &rawValue);
};