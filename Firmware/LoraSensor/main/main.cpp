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
#include "EspHal.h"
#include <RadioLib.h>


// create a new instance of the HAL class
EspHal* hal = new EspHal(10, 6, 7);

// SX1262 has the following connections:
// NSS pin:   10
// DIO1 pin:  2
// NRST pin:  3
// BUSY pin:  9
SX1262 radio = new Module(hal, 8, 3, 5, 4);

// or using RadioShield
// https://github.com/jgromes/RadioShield
//SX1262 radio = RadioShield.ModuleA;

// or using CubeCell
//SX1262 radio = new Module(RADIOLIB_BUILTIN_MODULE);

// save transmission state between loops
int transmissionState = RADIOLIB_ERR_NONE;

static const char *TAG = "main";

extern "C" void app_main(void) {
  // initialize just like with Arduino
  ESP_LOGI(TAG, "[SX1276] Initializing ... ");
  int state = radio.begin(868, 125, 7, 5, 0x12, 10, 8);
  if (state != RADIOLIB_ERR_NONE) {
    ESP_LOGI(TAG, "failed, code %d\n", state);
    while(true) {
      hal->delay(1000);
    }
  }
  ESP_LOGI(TAG, "success!\n");

  // loop forever
  for(;;) {
    // send a packet
    ESP_LOGI(TAG, "[SX1276] Transmitting packet ... ");
    state = radio.transmit("{\"model\":\"ESP32TEMP\",\"id\":\"myLoraNode\",\"tempc\":13.25}");
    if(state == RADIOLIB_ERR_NONE) {
      // the packet was successfully transmitted
      ESP_LOGI(TAG, "success!");

    } else {
      ESP_LOGI(TAG, "failed, code %d\n", state);

    }

    // wait for a second before transmitting again
    hal->delay(1000);

  }

}

