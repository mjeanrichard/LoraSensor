#ifndef ESP_HAL_H
#define ESP_HAL_H

// include RadioLib
#include <RadioLib.h>

#if CONFIG_IDF_TARGET_ESP32C3 == 0
#error Target is not ESP32C3!
#endif

// include all the dependencies
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp32/rom/gpio.h"
#include "soc/rtc.h"
#include "soc/spi_reg.h"
#include "soc/spi_struct.h"
#include "driver/gpio.h"
#include "hal/gpio_hal.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "rom/ets_sys.h"

// define Arduino-style macros
#define LOW (0x0)
#define HIGH (0x1)
#define INPUT (0x01)
#define OUTPUT (0x03)
#define RISING (0x01)
#define FALLING (0x02)
#define NOP() asm volatile("nop")

#define MATRIX_DETACH_OUT_SIG (0x100)
#define MATRIX_DETACH_IN_LOW_PIN (0x30)

// all of the following is needed to calculate SPI clock divider
#define ClkRegToFreq(reg) (apb_freq / (((reg)->clkdiv_pre + 1) * ((reg)->clkcnt_n + 1)))

typedef union
{
    uint32_t value;
    struct
    {
        uint32_t clkcnt_l : 6;
        uint32_t clkcnt_h : 6;
        uint32_t clkcnt_n : 6;
        uint32_t clkdiv_pre : 13;
        uint32_t clk_equ_sysclk : 1;
    };
} spiClk_t;

// create a new ESP-IDF hardware abstraction layer
// the HAL must inherit from the base RadioLibHal class
// and implement all of its virtual methods
// this is pretty much just copied from Arduino ESP32 core
class EspHal : public RadioLibHal
{
public:
    // default constructor - initializes the base HAL and any needed private members
    EspHal(int8_t sck, int8_t miso, int8_t mosi)
        : RadioLibHal(GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, LOW, HIGH, GPIO_INTR_POSEDGE, GPIO_INTR_NEGEDGE),
          _spiSCK(sck), _spiMISO(miso), _spiMOSI(mosi)
    {
    }

    void init() override;
    void term() override;
    void pinMode(uint32_t pin, uint32_t mode) override;
    void digitalWrite(uint32_t pin, uint32_t value) override; 
    uint32_t digitalRead(uint32_t pin) override;
    void attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) override;
    void detachInterrupt(uint32_t interruptNum) override;
    void delay(unsigned long ms) override;
    void delayMicroseconds(unsigned long us) override;
    unsigned long millis() override;
    unsigned long micros() override;
    long pulseIn(uint32_t pin, uint32_t state, unsigned long timeout) override;

    void spiBegin() override;
    void spiBeginTransaction() override;
    void spiTransfer(uint8_t *out, size_t len, uint8_t *in) override;
    void spiEndTransaction() override;
    void spiEnd() override;

private:
    int8_t _spiSCK;
    int8_t _spiMISO;
    int8_t _spiMOSI;
    spi_host_device_t _spi = SPI2_HOST;
    spi_device_handle_t _spiDeviceHandle;

    esp_err_t waitForPinState(gpio_num_t pin, int state, uint32_t timeoutMilliseconds);
};

#endif