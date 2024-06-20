#include "LoraClient.h"

esp_err_t LoraClient::initialize()
{
    _hal = new EspHal(10, 6, 7);
    _radio = new Module(hal, 8, 3, 5, 4);

    int state = radio.begin(868, 125, 7, 5, 0x12, 10, 8);
    if (state != RADIOLIB_ERR_NONE)
    {
        ESP_LOGE(TAG, "Raido initialization failed (code %d)", state);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t LoraClient::sendMeasurements(float temp, float humidity, float battery, float moisture)
{
    return ESP_OK;
}
