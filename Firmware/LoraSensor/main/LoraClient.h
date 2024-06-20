#pragma once

#include "EspHal.h"
#include "RadioLib.h"

class LoraClient
{
private:
    EspHal *_hal;
    SX1262 _radio;

public:
    esp_err_t initialize();
    esp_err_t sendMeasurements(float temp, float humidity, float battery, float moisture);
};