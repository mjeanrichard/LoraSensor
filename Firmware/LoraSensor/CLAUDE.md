# LoraSensor — Project Guide

## What this is

ESP32-C3 firmware (ESP-IDF 6.0.1, C++) for a plant sensor called **PlantSense**. Measures temperature/humidity (SHTC3 over I2C), soil moisture and battery voltage (ADC + LEDC PWM excitation), transmits JSON over **raw LoRa** (SX1262 via RadioLib). Settings stored in NVS. Supports OTA config changes via LoRa commands (`set_config` / `get_config`). Uses deep sleep between measurements.

## Repository layout

```
LoraSensor/
├── Board/LoraSensor/       KiCad schematic + PCB (ESP32-C3, SX1262, SHTC3)
└── Firmware/LoraSensor/
    ├── main/               Application source (all files to edit)
    │   ├── main.cpp        Entry point: init, measure, transmit, sleep
    │   ├── config.h        GPIO pin assignments, PWM freq, ADC constants, FIRMWARE_VERSION
    │   ├── LoraClient.{h,cpp}  SX1262 transmit/receive, JSON build/parse
    │   ├── adc.{h,cpp}     Battery + moisture ADC (with LEDC PWM drive)
    │   ├── settings.{h,cpp}    NVS-backed config (name, sleep, power, …)
    │   ├── wifiClient.{h,cpp}  WiFi STA (currently provisioned at compile time)
    │   ├── EspHal.{h,cpp}  RadioLib HAL implementation for ESP-IDF
    │   ├── SHTC3/          I2C driver for Sensirion SHTC3
    │   ├── elapsedMillis.h Arduino-style elapsed time helpers (adapted for ESP-IDF)
    │   └── helpers.h       RETURN_ON_ERROR macro
    ├── CMakeLists.txt      Top-level CMake
    ├── sdkconfig.defaults  Minimal: targets esp32c3
    └── managed_components/ ESP-IDF component manager output (RadioLib, cJSON)
```

## Hardware (ESP32-C3)

| Signal | GPIO |
|---|---|
| I2C SDA | 21 |
| I2C SCL | 20 |
| Soil sense (ADC) | 0 / ADC_CH0 |
| Soil PWM drive | 9 |
| Battery (ADC) | 1 / ADC_CH1 |
| Button (wake) | 2 |
| SX1262 DIO1 | 3 |
| SX1262 BUSY | 4 |
| SX1262 RST | 5 |
| SX1262 MISO | 6 |
| SX1262 MOSI | 7 |
| SX1262 NSS | 8 |
| SX1262 SCK | 10 |

LoRa parameters (must match gateway): 868 MHz, BW 125 kHz, SF7, CR 4/5, sync word 0x12.

## Build & flash

**Toolchain:** ESP-IDF 6.0.1 at `C:\esp\v6.0.1\esp-idf`, RISC-V GCC 15.2.0, target `esp32c3`.

**Environment setup (once per terminal session):**
```powershell
C:\esp\v6.0.1\esp-idf\export.ps1
```

**Common commands** (run from `Firmware/LoraSensor/`):
```powershell
idf.py build                        # compile
idf.py flash                        # flash over USB (auto-detects port)
idf.py flash monitor                # flash then open serial monitor
idf.py monitor                      # open serial monitor only
idf.py menuconfig                   # interactive sdkconfig editor
idf.py update-dependencies          # pull latest managed_components versions
```

**Build output:** `build/LoraSensor.bin` (merged flashable image), `build/LoraSensor.elf`.

**Managed components** (resolved by ESP-IDF Component Manager into `managed_components/`):
- `jgromes/radiolib` ≥ 7.7.0 (currently 7.7.0; 7.7.1 available)
- `espressif/cjson` ≥ 1.7 (currently 1.7.19~2)

**Debug builds:** Pass `-DRADIOLIB_DEBUG=ON` to cmake (or `idf.py -DRADIOLIB_DEBUG=ON build`) to enable RadioLib serial debug output. Off by default.

## NVS settings

| Key | Type | Default | Description |
|---|---|---|---|
| `name` | str | "Test" | Friendly name in JSON |
| `isTest` | i8 | false | Marks packets as test |
| `sleep` | u16 | 300 | Deep sleep seconds |
| `wait` | u8 | 10 | Receive window seconds |
| `pwr` | u8 | 15 | LoRa TX power (dBm) |
| `retx` | u8 | 2 | Number of retransmits |
| `moiDry` | u16 | 2700 | Moisture dry-point calibration (mV, open air) |
| `moiWet` | u16 | 700 | Moisture wet-point calibration (mV, saturated soil) |
| `v` | u16 | auto | Config version (auto-incremented on save) |

## JSON protocol (raw LoRa, no LoRaWAN)

**Uplink — data:**
```json
{"model":"PlantSense","msg":"data","id":"<mac>","name":"<name>",
 "tempc":<float>,"hum":<float>,"bat":<float>,"batPct":<int>,
 "moi":<int>,"moiRaw":<int>,"test":<bool>,"idx":<uint>,"v":<uint>,"fw":"<version>"}
```

**Uplink — wifi:**
```json
{"model":"PlantSense","msg":"wifi","id":"<mac>","name":"<name>",
 "test":<bool>,"wifiRssi":<int>,"uptime":<uint>,"fw":"<version>"}
```

**Downlink — set config:**
```json
{"id":"<mac>","cmd":"set_config","name":"...","sleep":300,"wait":10,
 "txPower":15,"retransmits":2,"test":false,"moiDry":2700,"moiWet":700}
```
All fields optional. `moiDry`/`moiWet` are raw ADC millivolt readings; observe `moiRaw` in data packets under dry and wet conditions to determine values.

**Downlink — get config:**
```json
{"id":"<mac>","cmd":"get_config"}
```

**Uplink — config response:**
```json
{"model":"PlantSense","msg":"config","id":"<mac>","name":"<name>",
 "test":<bool>,"sleep":<int>,"retx":<uint>,"wait":<uint>,"pwr":<uint>,
 "moiDry":<uint>,"moiWet":<uint>,"v":<uint>,"fw":"<version>"}
```

Gateway used: OpenMQTTGateway on a LilyGO board.

## Known issues & pending work

See task list (created in Claude Code session). Key points:
- WiFi credentials are currently hardcoded in `wifiClient.cpp` — must be moved to NVS
- Received config values are not range-validated before being saved to NVS

## Coding conventions

- ESP-IDF style: `ESP_RETURN_ON_ERROR` / `RETURN_ON_ERROR` for error propagation
- All functions return `esp_err_t`; callers check with the macros
- Log tags are `static const char *TAG` at file scope
- No dynamic allocation after init; stack VLAs are used but should be bounded
- `#pragma once` for headers (except EspHal.h which uses include guards)
