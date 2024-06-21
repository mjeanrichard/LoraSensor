#include "LoraClient.h"

static const char *TAG = "LoRa";

esp_err_t LoraClient::initialize()
{
    _hal = new EspHal(PIN_RADIO_SCK, PIN_RADIO_MISO, PIN_RADIO_MOSI);
    _radio = new SX1262(new Module(_hal, PIN_RADIO_NSS, PIN_RADIO_DIO1, PIN_RADIO_RST, PIN_RADIO_BUSY));

    int state = _radio->begin(868, 125, 7, 5, 0x12, 10, 8);
    if (state != RADIOLIB_ERR_NONE)
    {
        ESP_LOGE(TAG, "Raido initialization failed (code %d)", state);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t LoraClient::sendMeasurements(float temp, float humidity, float battery, uint8_t moisture)
{
    char buffer[150];
    sprintf(buffer, "{\"model\":\"ESP32TEMP\",\"id\":\"myLoraNode\",\"tempc\":%.1f,\"hum\":%.0f,\"bat\":%.1f,\"moi\":%i}", temp, humidity*100, battery, moisture);
    ESP_LOGI(TAG, "Sending: %s", buffer);
    int16_t state = _radio->transmit(buffer);
    if (state != RADIOLIB_ERR_NONE)
    {
        ESP_LOGE(TAG, "Raido transmit failed (code %d)", state);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t LoraClient::sleep()
{
    int16_t state = _radio->sleep();
    if (state != RADIOLIB_ERR_NONE)
    {
        ESP_LOGE(TAG, "Radio could not enter sleep state (%d)", state);
        return ESP_FAIL;
    }
    return ESP_OK;
}