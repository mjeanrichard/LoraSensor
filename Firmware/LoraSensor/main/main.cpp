
// #include "esp_log.h"
// #define RADIOLIB_DEBUG_PRINT(...) char buf[200]; sprintf(buf, __VA_ARGS__); ESP_LOGW("RAD", "%s", buf)
// #define RADIOLIB_DEBUG_PRINT_LVL(LEVEL, M, ...) ESP_LOGW("RAD", M, ##__VA_ARGS__)
// #define RADIOLIB_DEBUG_PRINTLN(M, ...) ESP_LOGW("RAD", M, ##__VA_ARGS__)
// #define RADIOLIB_DEBUG_PRINTLN_LVL(LEVEL, M, ...)  ESP_LOGW("RAD", M, ##__VA_ARGS__)

#include "LoraClient.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "driver/i2c_master.h"
#include "SHTC3/SHTC3.h"
#include <driver/rtc_io.h>
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "driver/ledc.h"
#include "adc.h"
#include "driver/uart.h"

static const char *TAG = "main";

LoraClient *_loraClient = new LoraClient();
Adc *_adc = new Adc();
i2c_master_bus_handle_t _i2cHandle;
SHTC3 *_sht;

// static void event_handler(void *arg, esp_event_base_t event_base,
//                           int32_t event_id, void *event_data)
// {
//   if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
//   {
//     esp_wifi_connect();
//   }
//   else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
//   {
//     esp_wifi_connect();
//     ESP_LOGI(TAG, "retry to connect to the AP");
//   }
//   else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
//   {
//     ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
//     ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
//   }
// }

// void startWifi()
// {
//   ESP_ERROR_CHECK(esp_netif_init());

//   ESP_ERROR_CHECK(esp_event_loop_create_default());
//   esp_netif_create_default_wifi_sta();

//   wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//   ESP_ERROR_CHECK(esp_wifi_init(&cfg));

//   esp_event_handler_instance_t instance_any_id;
//   esp_event_handler_instance_t instance_got_ip;
//   ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
//                                                       ESP_EVENT_ANY_ID,
//                                                       &event_handler,
//                                                       NULL,
//                                                       &instance_any_id));
//   ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
//                                                       IP_EVENT_STA_GOT_IP,
//                                                       &event_handler,
//                                                       NULL,
//                                                       &instance_got_ip));

//   wifi_config_t wifi_config = {
//       .sta = {
//           .ssid = "zuhause-iot",
//           .password = "BEKrsN2eYTEzGeL",
//           /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (pasword len => 8).
//            * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
//            * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
//            * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
//            */
//           // .threshold.authmode = WIFI_AUTH_WPA2_PSK,
//           // .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
//           // .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
//       },
//   };
//   ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
//   ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
//   ESP_ERROR_CHECK(esp_wifi_start());

//   ESP_LOGI(TAG, "wifi_init_sta finished.");
// }

void sleep()
{
  _sht->sleep();
  _loraClient->sleep();
  vTaskDelay(1);
  i2c_del_master_bus(_i2cHandle);
  //esp_sleep_config_gpio_isolate();
  gpio_pulldown_dis(GPIO_NUM_20);
  gpio_pulldown_dis(GPIO_NUM_19);
  gpio_pullup_en(GPIO_NUM_20);
  gpio_pullup_en(GPIO_NUM_19);
  gpio_deep_sleep_hold_en();
  esp_deep_sleep_enable_gpio_wakeup(BIT(PIN_BTN), ESP_GPIO_WAKEUP_GPIO_LOW);
  esp_sleep_enable_timer_wakeup(60 * 1000 * 1000);
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
      .flags = {
          .enable_internal_pullup = true}};

  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &_i2cHandle));

  _sht = new SHTC3();
  _sht->begin(_i2cHandle);
}

extern "C" void app_main(void)
{
  esp_log_level_set("SHTC3", ESP_LOG_DEBUG);
  esp_log_level_set("ADC", ESP_LOG_DEBUG);

  esp_sleep_wakeup_cause_t wakeupSource = esp_sleep_get_wakeup_cause();

  uart_driver_delete(UART_NUM_0);
  uart_driver_delete(UART_NUM_1);


  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  _adc->initialize();
  setupI2c();
  _loraClient->initialize();

  float humidity = 0.0;
  float temperature = 0.0;
  float battery = 0.0;
  uint8_t moisture = 0.0;
  _sht->measure(temperature, humidity);
  _adc->readValues(battery, moisture);
  _adc->stop();

  _loraClient->sendMeasurements(temperature, humidity, battery, moisture);

  if (wakeupSource == ESP_SLEEP_WAKEUP_GPIO)
  {
    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }

  sleep();
}
