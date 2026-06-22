#include "otaClient.h"
#include "esp_crt_bundle.h"
#include "esp_https_ota.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "OTA";

#define WIFI_TIMEOUT_US (30LL * 1000 * 1000)

void OtaClient::update(const char *url, WifiClient *wifi, LoraClient *lora)
{
    ESP_LOGI(TAG, "Starting OTA from %s", url);
    lora->sendOtaStart();

    wifi->start();

    int64_t start = esp_timer_get_time();
    while (!wifi->isConnected())
    {
        if (esp_timer_get_time() - start > WIFI_TIMEOUT_US)
        {
            ESP_LOGE(TAG, "WiFi connect timeout");
            lora->sendOtaFail("wifi_timeout");
            esp_restart();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGI(TAG, "WiFi connected, running HTTPS OTA");

    esp_http_client_config_t http_cfg = {
        .url = url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .keep_alive_enable = true,
    };
    esp_https_ota_config_t ota_cfg = {
        .http_config = &http_cfg,
    };

    esp_err_t ret = esp_https_ota(&ota_cfg);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "OTA succeeded, restarting");
        esp_restart();
    }

    ESP_LOGE(TAG, "OTA failed (%d)", ret);
    lora->sendOtaFail("download_failed");
    esp_restart();
}
