#pragma once

#include "LoraClient.h"
#include "wifiClient.h"

class OtaClient
{
  public:
    // Connects to WiFi, downloads and flashes the firmware at url.
    // Never returns: calls esp_restart() on both success and failure.
    void update(const char *url, WifiClient *wifi, LoraClient *lora);
};
