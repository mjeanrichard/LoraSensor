/*
  RadioLib SX126x Transmit with Interrupts Example

  This example transmits LoRa packets with one second delays
  between them. Each packet contains up to 256 bytes
  of data, in the form of:
  - Arduino String
  - null-terminated char array (C-string)
  - arbitrary binary data (byte array)

  Other modules from SX126x family can also be used.

  For default module settings, see the wiki page
  https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx126x---lora-modem

  For full API reference, see the GitHub Pages
  https://jgromes.github.io/RadioLib/
*/

// #include "esp_log.h"
// #define RADIOLIB_DEBUG_PRINT(...) char buf[200]; sprintf(buf, __VA_ARGS__); ESP_LOGW("RAD", "%s", buf)
// #define RADIOLIB_DEBUG_PRINT_LVL(LEVEL, M, ...) ESP_LOGW("RAD", M, ##__VA_ARGS__)
// #define RADIOLIB_DEBUG_PRINTLN(M, ...) ESP_LOGW("RAD", M, ##__VA_ARGS__)
// #define RADIOLIB_DEBUG_PRINTLN_LVL(LEVEL, M, ...)  ESP_LOGW("RAD", M, ##__VA_ARGS__)

// include the library
#include "LoraClient.h"
#include "EspHal.h"
#include <RadioLib.h>
#include "esp_timer.h"
#include "esp_sleep.h"
#include "driver/i2c_master.h"
#include "SHTC3/SHTC3.h"
#include <driver/rtc_io.h>

LoraClient * _loraClient;


// create a new instance of the HAL class
EspHal *hal = new EspHal(10, 6, 7);

// SX1262 has the following connections:
// NSS pin:   10
// DIO1 pin:  2
// NRST pin:  3
// BUSY pin:  9
SX1262 radio = new Module(hal, 8, 3, 5, 4);

// or using RadioShield
// https://github.com/jgromes/RadioShield
// SX1262 radio = RadioShield.ModuleA;

// or using CubeCell
// SX1262 radio = new Module(RADIOLIB_BUILTIN_MODULE);

// save transmission state between loops
int transmissionState = RADIOLIB_ERR_NONE;

static const char *TAG = "main";

void sleep()
{
  radio.sleep();
  esp_sleep_config_gpio_isolate();
  esp_sleep_enable_timer_wakeup(30 * 1000 * 1000);
  esp_deep_sleep_start();
}

i2c_master_bus_handle_t bus_handle;
SHTC3 *sht;

void readSensor()
{
  float humidity = 0.0;
  float temperature = 0.0;

  sht->measure(temperature, humidity);

  ESP_LOGI(TAG, "humidity:%.0f", humidity*100);
  ESP_LOGI(TAG, "temperature:%.1f", temperature);
}

void setupI2c()
{
  i2c_master_bus_config_t i2c_mst_config = {
      .i2c_port = I2C_NUM_0,
      .sda_io_num = GPIO_NUM_21,
      .scl_io_num = GPIO_NUM_2,
      .clk_source = I2C_CLK_SRC_DEFAULT,
  };

  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

  sht = new SHTC3();
  sht->begin(bus_handle);
}

extern "C" void app_main(void)
{
  esp_log_level_set("SHTC3", ESP_LOG_DEBUG);

  // initialize just like with Arduino
  ESP_LOGI(TAG, "[SX1276] Initializing ... ");
  int state = radio.begin(868, 125, 7, 5, 0x12, 10, 8);
  if (state != RADIOLIB_ERR_NONE)
  {
    ESP_LOGI(TAG, "failed, code %d\n", state);
    while (true)
    {
      hal->delay(1000);
    }
  }
  ESP_LOGI(TAG, "success!\n");

  // loop forever
  // send a packet
  ESP_LOGI(TAG, "[SX1276] Transmitting packet ... ");
  state = radio.transmit("{\"model\":\"ESP32TEMP\",\"id\":\"myLoraNode\",\"tempc\":13.25}");
  if (state == RADIOLIB_ERR_NONE)
  {
    // the packet was successfully transmitted
    ESP_LOGI(TAG, "success!");
  }
  else
  {
    ESP_LOGI(TAG, "failed, code %d\n", state);
  }

  setupI2c();
  readSensor();

  // while (true)
  // {
  //   ESP_LOGI(TAG, "Reading sensor...");
  //   readSensor();
  //   vTaskDelay(1000 / portTICK_PERIOD_MS);
  // }

  // wait for a second before transmitting again
  vTaskDelay(10000 / portTICK_PERIOD_MS);

  sleep();
}
