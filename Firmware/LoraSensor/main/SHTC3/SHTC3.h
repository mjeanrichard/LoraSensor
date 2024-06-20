#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "elapsedMillis.h"
#include "esp_check.h"
#include "rom/ets_sys.h"

class SHTC3
{
private:
    i2c_master_dev_handle_t _devHandle;

    elapsedMicros _elapsedMicrosStartMeasurement = 0;

    esp_err_t writeCommand(uint16_t cmd);
    esp_err_t readPacket(uint16_t &value);

    esp_err_t checkId();
    esp_err_t wakeUp();
    esp_err_t sleep();
    esp_err_t readData(float &temperature, float &humidity);
    esp_err_t reset();

    static esp_err_t checkCrc(uint8_t data[], uint8_t len);
    static uint8_t calculateCrc(uint8_t data[], uint8_t len);

public:
    esp_err_t measure(float &temperature, float &humidity);
    esp_err_t begin(i2c_master_bus_handle_t _busHandle);
};