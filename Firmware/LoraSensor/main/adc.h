#pragma once

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_check.h"
#include "driver/ledc.h"
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "settings.h"

struct AdcMeasurements
{
    float BatteryVolts = 0;
    uint8_t BatteryPercent = 0;
    uint16_t MoistureRaw = 0;
    uint8_t MoisturePercent = 0;
};


class Adc
{
private:
    adc_oneshot_unit_handle_t _adcHandle = nullptr;
    adc_cali_handle_t _calibrationHandle = nullptr;
    Settings *_settings = nullptr;

    esp_err_t readRaw(uint16_t &rawValue, adc_channel_t channel);

public:
    esp_err_t initialize(Settings &settings);
    esp_err_t readValues(AdcMeasurements &measurements);
    esp_err_t stop();

private:
    esp_err_t readBatteryRaw(uint16_t &rawValue);
    esp_err_t readMoistureRaw(uint16_t &rawValue);
};