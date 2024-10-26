#include "EspHal.h"

static const char *TAG = "EspHal";

void EspHal::init()
{
    // we only need to init the SPI here
    spiBegin();
}

void EspHal::term()
{
    // we only need to stop the SPI here
    spiEnd();
}

// GPIO-related methods (pinMode, digitalWrite etc.) should check
// RADIOLIB_NC as an alias for non-connected pins
void EspHal::pinMode(uint32_t pin, uint32_t mode)
{
    if (pin == RADIOLIB_NC)
    {
        return;
    }

    gpio_config_t conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = (gpio_mode_t)mode,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&conf);
}

void EspHal::digitalWrite(uint32_t pin, uint32_t value)
{
    if (pin == RADIOLIB_NC)
    {
        return;
    }

    gpio_set_level((gpio_num_t)pin, value);
}

uint32_t EspHal::digitalRead(uint32_t pin)
{
    if (pin == RADIOLIB_NC)
    {
        return (0);
    }

    return (gpio_get_level((gpio_num_t)pin));
}

void EspHal::attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode)
{
    if (interruptNum == RADIOLIB_NC)
    {
        return;
    }

    ESP_ERROR_CHECK(gpio_install_isr_service((int)ESP_INTR_FLAG_IRAM));

    ESP_ERROR_CHECK(gpio_set_intr_type((gpio_num_t)interruptNum, (gpio_int_type_t)mode));
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)interruptNum, (gpio_isr_t)interruptCb, nullptr));
}

void EspHal::detachInterrupt(uint32_t interruptNum)
{
    if (interruptNum == RADIOLIB_NC)
    {
        return;
    }

    gpio_isr_handler_remove((gpio_num_t)interruptNum);
    gpio_wakeup_disable((gpio_num_t)interruptNum);
    gpio_set_intr_type((gpio_num_t)interruptNum, GPIO_INTR_DISABLE);
}

void EspHal::delay(unsigned long ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

void EspHal::delayMicroseconds(unsigned long us)
{
    ets_delay_us(us);
}

unsigned long EspHal::millis()
{
    return ((unsigned long)(esp_timer_get_time() / 1000ULL));
}

unsigned long EspHal::micros()
{
    return ((unsigned long)(esp_timer_get_time()));
}

long EspHal::pulseIn(uint32_t pin, uint32_t state, unsigned long timeout)
{
    if (pin == RADIOLIB_NC)
    {
        return (0);
    }

    if (waitForPinState((gpio_num_t)pin, state, timeout) == ESP_ERR_TIMEOUT)
    {
        return 0;
    }

    int64_t start = esp_timer_get_time();

    if (waitForPinState((gpio_num_t)pin, !state, timeout) == ESP_ERR_TIMEOUT)
    {
        return 0;
    }

    return (esp_timer_get_time() - start) / 1000;
}

void EspHal::spiBegin()
{
    spi_bus_config_t bus_config = {};
    bus_config.sclk_io_num = _spiSCK;
    bus_config.mosi_io_num = _spiMOSI;
    bus_config.miso_io_num = _spiMISO;
    bus_config.quadwp_io_num = -1;
    bus_config.quadhd_io_num = -1;
    bus_config.max_transfer_sz = 0;
    bus_config.flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS;

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t cfg = {};
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode = 0;
    cfg.clock_speed_hz = 3E6;
    cfg.spics_io_num = -1;
    cfg.queue_size = 16;
    cfg.command_bits = 0;
    cfg.dummy_bits = 0;

    ESP_ERROR_CHECK(spi_bus_add_device(_spi, &cfg, &_spiDeviceHandle));
}

void EspHal::spiBeginTransaction()
{
    // not needed - in ESP32 Arduino core, this function
    // repeats clock div, mode and bit order configuration
}

void EspHal::spiTransfer(uint8_t *out, size_t len, uint8_t *in)
{
    spi_transaction_t trx = {};
    trx.cmd = 0;
    trx.tx_buffer = out;
    trx.length = len * 8;
    trx.rxlength = len * 8;
    trx.rx_buffer = in;

    ESP_ERROR_CHECK(spi_device_transmit(_spiDeviceHandle, (spi_transaction_t *)&trx));
}

void EspHal::spiEndTransaction()
{
    // nothing needs to be done here
}

void EspHal::spiEnd()
{
    ESP_ERROR_CHECK(spi_bus_remove_device(_spiDeviceHandle));
    ESP_ERROR_CHECK(spi_bus_free(SPI2_HOST));
}

esp_err_t EspHal::waitForPinState(gpio_num_t pin, int state, uint32_t timeoutMilliseconds)
{
    int64_t startTime = esp_timer_get_time();
    while (gpio_get_level(pin) != state)
    {
        if (esp_timer_get_time() - startTime > timeoutMilliseconds * 1000)
        {
            return ESP_ERR_TIMEOUT;
        }
    }
    return ESP_OK;
}
