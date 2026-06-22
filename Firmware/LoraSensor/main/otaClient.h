#pragma once

#include "wifiClient.h"

class OtaClient
{
    WifiClient *_wifi;

  public:
    OtaClient(WifiClient *wifi) : _wifi(wifi) {}

    // Connects to WiFi, downloads and flashes the firmware at url.
    // Never returns: calls esp_restart() on both success and failure.
    void update(const char *url);
};
