#if !defined(_RADIOLIB_USER_BUILD_OPTIONS_H)
#define _RADIOLIB_USER_BUILD_OPTIONS_H

// this file can be used to define any user build options
// most commonly, RADIOLIB_EXCLUDE_* macros
// or enabling debug output

//#define RADIOLIB_DEBUG_BASIC        (1)   // basic debugging (e.g. reporting GPIO timeouts or module not being found)
//#define RADIOLIB_DEBUG_PROTOCOL     (1)   // protocol information (e.g. LoRaWAN internal information)
//#define RADIOLIB_DEBUG_SPI          (1)   // verbose transcription of all SPI communication - produces large debug logs!

#include "esp_log.h"
#define RADIOLIB_DEBUG_PRINT(...) char buf[200]; sprintf(buf, __VA_ARGS__); ESP_LOGW("RAD", "%s", buf)
#define RADIOLIB_DEBUG_PRINT_LVL(LEVEL, M, ...) ESP_LOGW("RAD", M, ##__VA_ARGS__)
#define RADIOLIB_DEBUG_PRINTLN(M, ...) ESP_LOGW("RAD", M, ##__VA_ARGS__)
#define RADIOLIB_DEBUG_PRINTLN_LVL(LEVEL, M, ...)  ESP_LOGW("RAD", M, ##__VA_ARGS__)


#endif
