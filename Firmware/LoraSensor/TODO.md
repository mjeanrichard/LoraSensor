# LoraSensor — TODO

## Minor

- [*] **Extract LoRa radio parameters into `config.h` constants**
  `LoraClient.cpp:28` — Frequency (868), bandwidth (125), SF (7), CR (5), sync word (0x12) are magic numbers. Move to named constants so they are easy to change for a different region or gateway.

## Bugs

- [*] **`gpio_install_isr_service` called on every `attachInterrupt`** (`EspHal.cpp:63`)
  Must be called exactly once. A second call returns `ESP_ERR_INVALID_STATE`, which
  `ESP_ERROR_CHECK` turns into `abort()`. Guard with:
  ```cpp
  esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(err);
  ```

## Reliability

- [ ] **`uint16_t` ADC accumulators overflow if `NUM_ADC_READS` > 19** (`adc.cpp:63`)
  Change `batterySum` and `moistureSum` to `uint32_t`.

- [*] **`new` called at global scope before FreeRTOS is ready** (`main.cpp:26–27`)
  `_loraClient` and `_adc` are heap-allocated via `new` before `app_main`, before the
  IDF heap is initialised. Declare as raw pointers and construct inside `app_main`.

- [ ] **WiFi retries forever on wrong credentials** (`wifiClient.cpp:13`)
  `WIFI_EVENT_STA_DISCONNECTED` handler calls `esp_wifi_connect()` unconditionally.
  Add a retry counter; stop and let the OTA timeout fire after N failures.

- [ ] **OTA proceeds with empty SSID** (`LoraClient.cpp:processData`)
  Receiving an `ota` command before WiFi credentials are set causes a 30-second
  timeout then restart. Reject the command in `processData` if `_settings->getSsid()` is empty.

- [*] **No length validation on `name`, `ssid`, `wifiPwd` in `set_config`** (`LoraClient.cpp:updateConfig`)
  Oversized values are stored without bounds checking. A long `name` can push JSON
  payloads past the buffer limit (now caught by `CHECK_SNPRINTF` as abort). `ssid` > 32
  and `wifiPwd` > 64 are silently truncated by `strncpy`, so stored value won't match
  what's used. Clamp: name ≤ 24, ssid ≤ 32, wifiPwd ≤ 64.

## Minor

- [*] **`_wakeupCount` overflows silently at 255** (`main.cpp:30`)
  Wraps to 0 after ~21 hours at 5-minute intervals, causing `idx` to jump backward.
  Change `RTC_DATA_ATTR uint8_t _wakeupCount` to `uint32_t`.

- [*] **`_isReceiving` is dead code** (`LoraClient.h:20`)
  Never read or written; state is tracked entirely by `_state`. Remove the field.

- [*] **Redundant `memset` after value-initialisation** (`EspHal.cpp:137`)
  `spi_device_interface_config_t cfg = {}` already zero-initialises all fields.
  Remove the subsequent `memset(&cfg, 0, sizeof(cfg))`.
