#include "SHTC3.h"

static const char *TAG = "SHTC3";

#define CMD_NORMAL_STRECH_TEMP_FIRST 0x7CA2
#define CMD_NORMAL_TEMP_FIRST 0x7866
#define CMD_READ_ID 0xEFC8
#define CMD_WAKEUP 0x3517
#define CMD_SLEEP 0xB098
#define CMD_SOFT_RESET 0xEFC8
#define crcPolynom 0x31

#define SHTC3_RETURN_ON_ERROR(x)         \
    do                                   \
    {                                    \
        esp_err_t err_rc_ = (x);         \
        if (unlikely(err_rc_ != ESP_OK)) \
        {                                \
            return err_rc_;              \
        }                                \
    } while (0)

esp_err_t SHTC3::measure(float &temperature, float &humidity)
{
    SHTC3_RETURN_ON_ERROR(wakeUp());
    SHTC3_RETURN_ON_ERROR(readData(temperature, humidity));
    SHTC3_RETURN_ON_ERROR(sleep());
    return ESP_OK;
}

esp_err_t SHTC3::begin(i2c_master_bus_handle_t busHandle)
{

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x70,
        .scl_speed_hz = 100000,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(busHandle, &dev_cfg, &_devHandle));

    SHTC3_RETURN_ON_ERROR(wakeUp());
    esp_err_t err = checkId();
    if (err != ESP_OK)
    {
        reset();
        SHTC3_RETURN_ON_ERROR(checkId());
    }

    return sleep();
}

esp_err_t SHTC3::wakeUp()
{
    ESP_RETURN_ON_ERROR(writeCommand(CMD_WAKEUP), TAG, "Failed to send wake up command");
    ets_delay_us(300);
    return ESP_OK;
}

esp_err_t SHTC3::sleep()
{
    ESP_RETURN_ON_ERROR(writeCommand(CMD_SLEEP), TAG, "Failed to send sleep command");
    ets_delay_us(500);
    return ESP_OK;
}

esp_err_t SHTC3::reset()
{
    ESP_RETURN_ON_ERROR(writeCommand(CMD_SOFT_RESET), TAG, "Failed to send reset command");
    return ESP_OK;
}

esp_err_t SHTC3::readData(float &temperature, float &humidity)
{
    ESP_RETURN_ON_ERROR(writeCommand(CMD_NORMAL_TEMP_FIRST), TAG, "Failed to write read command");
    vTaskDelay(2);

    uint8_t data[6];
    ESP_RETURN_ON_ERROR(i2c_master_receive(_devHandle, data, 6, -1), TAG, "Failed to read data");
    ESP_LOGD(TAG, "Data read:");
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, data, 6, ESP_LOG_DEBUG);

    ESP_RETURN_ON_ERROR(checkCrc(data, 3), TAG, "CRC Check failed");
    ESP_RETURN_ON_ERROR(checkCrc(data + 3, 3), TAG, "CRC Check failed");

    temperature = -45 + 175 * ((float)((data[0] << 8) | data[1])) / (1 << 16);
    humidity = ((float)(((data[3] << 8) | data[4])) / UINT16_MAX);

    return ESP_OK;
}

esp_err_t SHTC3::checkId()
{
    uint16_t chipId = 0;
    ESP_RETURN_ON_ERROR(writeCommand(CMD_READ_ID), TAG, "Failed to write read chip id.");
    ESP_RETURN_ON_ERROR(readPacket(chipId), TAG, "Failed to read chipId");

    if ((chipId & 0x0807) == 0x0807)
    {
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Failed to read chip id (%04X)", chipId);
    return ESP_FAIL;
}

esp_err_t SHTC3::writeCommand(uint16_t cmd)
{
    uint8_t data[2];
    data[0] = cmd >> 8;
    data[1] = cmd & 0xff;
    ESP_RETURN_ON_ERROR(i2c_master_transmit(_devHandle, data, 2, -1), TAG, "Failed to transmit command.");

    return ESP_OK;
}

esp_err_t SHTC3::readPacket(uint16_t &value)
{
    uint8_t data[3];
    ESP_RETURN_ON_ERROR(i2c_master_receive(_devHandle, data, 3, -1), TAG, "Failed to read data");
    ESP_LOGD(TAG, "Data read:");
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, data, 3, ESP_LOG_DEBUG);

    ESP_RETURN_ON_ERROR(checkCrc(data, 3), TAG, "CRC Check failed");

    value = (data[0] << 8) | data[1];
    return ESP_OK;
}

esp_err_t SHTC3::checkCrc(uint8_t data[], uint8_t len)
{
    uint8_t crc = calculateCrc(data, len - 1);
    if (crc == data[len - 1])
    {
        return ESP_OK;
    }
    return ESP_FAIL;
}

uint8_t SHTC3::calculateCrc(uint8_t data[], uint8_t len)
{
    // initialization value
    uint8_t crc = 0xff;

    // iterate over all bytes
    for (int i = 0; i < len; i++)
    {
        crc ^= data[i];

        for (int i = 0; i < 8; i++)
        {
            bool isXor = crc & 0x80;
            crc = crc << 1;
            crc = isXor ? crc ^ crcPolynom : crc;
        }
    }

    return crc;
}