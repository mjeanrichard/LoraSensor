#pragma once

#include "nvs_flash.h"
#include "esp_check.h"
#include "helpers.h"
#include <string>

class Settings
{
private:
    std::string _name = "Test";
    bool _isTest = false;
    uint16_t _sleepSeconds = 300;
    uint8_t _waitSeconds = 10;
    uint8_t _transmitPower = 15;
    uint8_t _retransmits = 2;
    uint16_t _configVersion = 0;
    uint16_t _moiDry = 2700;
    uint16_t _moiWet = 700;
    std::string _ssid;
    std::string _wifiPwd;

    esp_err_t readString(nvs_handle_t nvs, const char *key, std::string &field);
    esp_err_t readBool(nvs_handle_t nvs, const char *key, bool *value);

public:
    esp_err_t load();
    esp_err_t save();

    const std::string &getName() const { return _name; };
    void setName(std::string name) { _name = name; }

    bool isTest() const { return _isTest; };
    void setIsTest(bool isTest) { _isTest = isTest; };

    uint16_t sleepSeconds() const { return _sleepSeconds; }
    void setSleepSeconds(uint16_t sleepSeconds) { _sleepSeconds = sleepSeconds; }

    uint8_t waitSeconds() const { return _waitSeconds; }
    void setWaitSeconds(uint8_t waitSeconds) { _waitSeconds = waitSeconds; }

    uint8_t transmitPower() const { return _transmitPower; }
    void setTransmitPower(uint8_t transmitPower) { _transmitPower = transmitPower; }

    uint8_t retransmits() const { return _retransmits; }
    void setRetransmits(uint8_t retransmits) { _retransmits = retransmits; }

    uint16_t version() const { return _configVersion; }

    uint16_t moiDry() const { return _moiDry; }
    void setMoiDry(uint16_t moiDry) { _moiDry = moiDry; }

    uint16_t moiWet() const { return _moiWet; }
    void setMoiWet(uint16_t moiWet) { _moiWet = moiWet; }

    const std::string &getSsid() const { return _ssid; }
    void setSsid(const std::string &ssid) { _ssid = ssid; }

    const std::string &getWifiPwd() const { return _wifiPwd; }
    void setWifiPwd(const std::string &pwd) { _wifiPwd = pwd; }
};