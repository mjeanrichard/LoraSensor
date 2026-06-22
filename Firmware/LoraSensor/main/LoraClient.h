#pragma once

#include "EspHal.h"
#include "RadioLib.h"
#include "adc.h"
#include "cJSON.h"
#include "config.h"
#include "esp_mac.h"
#include "settings.h"
#include "wifiClient.h"

class LoraClient
{
  private:
    EspHal *_hal;
    SX1262 *_radio;
    Settings *_settings;

    bool _forceTest = false;
    std::string _pendingOtaUrl;

    static void dataReady(void);

    esp_err_t transmitData(const char *buffer);

    esp_err_t sendConfig();

    esp_err_t processData(char *buffer);
    esp_err_t updateConfig(const cJSON *json);

    esp_err_t getJsonString(const char *name, const cJSON *json, std::string &value);
    esp_err_t getJsonInt(const char *name, const cJSON *json, int32_t &value);
    esp_err_t getJsonBool(const char *name, const cJSON *json, bool &value);

  public:
    LoraClient(Settings *settings) : _settings(settings) {};
    esp_err_t initialize(bool forceTest);
    esp_err_t sendMeasurements(float temp, float humidity, AdcMeasurements &adcMeasurements, uint8_t wakeupCount);
    esp_err_t sendWifiInfo(WifiClient *wifiClient);
    esp_err_t sleep();

    esp_err_t getData();
    esp_err_t startListening();

    std::string _chipId = "";

    bool hasPendingOta() const { return !_pendingOtaUrl.empty(); }
    const std::string &pendingOtaUrl() const { return _pendingOtaUrl; }
};