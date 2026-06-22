
// #include "esp_log.h"
// #define RADIOLIB_DEBUG_PRINT(...) char buf[200]; sprintf(buf, __VA_ARGS__); ESP_LOGW("RAD", "%s", buf)
// #define RADIOLIB_DEBUG_PRINT_LVL(LEVEL, M, ...) ESP_LOGW("RAD", M, ##__VA_ARGS__)
// #define RADIOLIB_DEBUG_PRINTLN(M, ...) ESP_LOGW("RAD", M, ##__VA_ARGS__)
// #define RADIOLIB_DEBUG_PRINTLN_LVL(LEVEL, M, ...)  ESP_LOGW("RAD", M, ##__VA_ARGS__)

#include "LoraClient.h"
#include "SHTC3/SHTC3.h"
#include "adc.h"
#include "otaClient.h"
#include "esp_ota_ops.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "settings.h"
#include "wifiClient.h"
#include <driver/rtc_io.h>

static const char *TAG = "main";

Settings settings;
LoraClient *_loraClient = new LoraClient(&settings);
Adc *_adc = new Adc();
i2c_master_bus_handle_t _i2cHandle;
SHTC3 *_sht;
RTC_DATA_ATTR uint8_t _wakeupCount = 0;

void sleep()
{
    esp_err_t ret;
    ret = _sht->sleep();
    if (ret != ESP_OK) ESP_LOGE(TAG, "SHTC3 sleep failed (%d)", ret);
    ret = _sht->deinit();
    if (ret != ESP_OK) ESP_LOGE(TAG, "SHTC3 deinit failed (%d)", ret);
    ret = _loraClient->sleep();
    if (ret != ESP_OK) ESP_LOGE(TAG, "LoRa sleep failed (%d)", ret);
    vTaskDelay(1);
    ret = i2c_del_master_bus(_i2cHandle);
    if (ret != ESP_OK) ESP_LOGE(TAG, "I2C bus delete failed (%d)", ret);
    esp_sleep_enable_gpio_wakeup_on_hp_periph_powerdown(BIT(PIN_BTN), ESP_GPIO_WAKEUP_GPIO_LOW);
    esp_sleep_enable_timer_wakeup(settings.sleepSeconds() * 1000 * 1000);
    esp_deep_sleep_start();
}

void setupI2c()
{
    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = 0,
            .allow_pd = 0,
        }
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &_i2cHandle));

    _sht = new SHTC3();
    _sht->begin(_i2cHandle);
}

extern "C" void app_main(void)
{
    esp_ota_mark_app_valid_cancel_rollback();

    bool hasUsb = usb_serial_jtag_is_connected();

    esp_log_level_set("SHTC3", ESP_LOG_DEBUG);
    esp_log_level_set("ADC", ESP_LOG_DEBUG);
    esp_log_level_set("LoRa", ESP_LOG_DEBUG);

    uint32_t wakeupSource = esp_sleep_get_wakeup_causes();

    uart_driver_delete(UART_NUM_0);
    uart_driver_delete(UART_NUM_1);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    settings.load();

    _adc->initialize(settings);
    setupI2c();
    _loraClient->initialize(hasUsb);

    float humidity = 0.0;
    float temperature = 0.0;
    ret = _sht->measure(temperature, humidity);
    if (ret != ESP_OK)
    {
        ret = _sht->measure(temperature, humidity);
    }

    if (ret == ESP_OK)
    {
        AdcMeasurements adcMeasurements;
        _adc->readValues(adcMeasurements);
        _adc->stop();

        _loraClient->sendMeasurements(temperature, humidity, adcMeasurements, _wakeupCount++);
    }
    else
    {
        ESP_LOGE(TAG, "Skipping measurement, because of sensor failure.");
    }

    if (wakeupSource & BIT(ESP_SLEEP_WAKEUP_GPIO) || hasUsb || _wakeupCount % 5 == 0)
    {
        int64_t time = esp_timer_get_time();
        while (esp_timer_get_time() - time < 10 * 1000 * 1000)
        {
            _loraClient->getData();
            vTaskDelay(1);
        }
    }

    if (_loraClient->hasPendingOta())
    {
        WifiClient wifiClient(&settings);
        OtaClient otaClient;
        otaClient.update(_loraClient->pendingOtaUrl().c_str(), &wifiClient, _loraClient);
        // never returns
    }

    sleep();
}
