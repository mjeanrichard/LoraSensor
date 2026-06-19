#pragma once

#include "esp_check.h"
#include "esp_timer.h"
#include "helpers.h"
#include "settings.h"
#include <cstring>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

class WifiClient
{
  private:
    esp_netif_t *_wifiNetIf = NULL;

    Settings *_settings = nullptr;
    uint16_t _connectTime = 0;

    static void wifiEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

    bool _isConnected = false;

    void sendData();

  public:
    WifiClient(Settings *settings) : _settings(settings) {};

    esp_err_t start();

    bool isConnected() { return _isConnected; }

    esp_ip4_addr_t getIp();
    int8_t getRssi();

    uint16_t getConnectTime() { return _connectTime; }
};
