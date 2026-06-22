#include "wifiClient.h"

static const char *TAG = "WIFI_CLIENT";

void WifiClient::wifiEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    WifiClient *wifi = (WifiClient *)arg;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        wifi->_isConnected = false;
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        wifi->_isConnected = false;
        esp_wifi_connect();
        ESP_LOGI(TAG, "retry to connect to the AP");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        wifi->_isConnected = true;
        wifi->_connectTime = esp_timer_get_time() / 1000;
    }
}

esp_err_t WifiClient::start()
{
    RETURN_ON_ERROR(esp_netif_init());

    RETURN_ON_ERROR(esp_event_loop_create_default());
    _wifiNetIf = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    RETURN_ON_ERROR(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    RETURN_ON_ERROR(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifiEventHandler, this, &instance_any_id));
    RETURN_ON_ERROR(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifiEventHandler, this, &instance_got_ip));

    wifi_config_t wifi_config = {.sta = {}};
    strncpy((char *)wifi_config.sta.ssid, _settings->getSsid().c_str(), sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, _settings->getWifiPwd().c_str(), sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.scan_method = WIFI_FAST_SCAN;
    wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SECURITY;
    wifi_config.sta.threshold = {
        .rssi = -127, 
        .authmode = WIFI_AUTH_WPA2_PSK,
        .rssi_5g_adjustment = 0,
    };

    RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA));
    RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    RETURN_ON_ERROR(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    return ESP_OK;
}

esp_err_t WifiClient::stop()
{
    _isConnected = false;
    RETURN_ON_ERROR(esp_wifi_stop());
    RETURN_ON_ERROR(esp_wifi_deinit());
    if (_wifiNetIf)
    {
        esp_netif_destroy(_wifiNetIf);
        _wifiNetIf = nullptr;
    }
    esp_event_loop_delete_default();
    return ESP_OK;
}

esp_ip4_addr_t WifiClient::getIp()
{
    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(_wifiNetIf, &ip_info);

    return ip_info.ip;
}

int8_t WifiClient::getRssi()
{
    wifi_ap_record_t wifidata;
    esp_wifi_sta_get_ap_info(&wifidata);
    ESP_LOGD(TAG, "RSSI: %i", wifidata.rssi);
    return wifidata.rssi;
}