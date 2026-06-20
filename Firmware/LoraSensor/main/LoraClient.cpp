#include "LoraClient.h"
#include <algorithm>

static const char *TAG = "LoRa";

#define LORA_STATE_IDLE 0
#define LORA_STATE_RECEIVING 1
#define LORA_STATE_DATA_READY 2
#define LORA_STATE_TRANSMIT 3

static volatile int _state = LORA_STATE_IDLE;

#define CHECK_SNPRINTF(x)                                                                                                                                                                              \
    do                                                                                                                                                                                                 \
    {                                                                                                                                                                                                  \
        if (unlikely(x == 0))                                                                                                                                                                          \
        {                                                                                                                                                                                              \
            ESP_LOGE(TAG, "%s(%d): Failed to do SPRINTF.", __FUNCTION__, __LINE__);                                                                                                                    \
            abort();                                                                                                                                                                                   \
        }                                                                                                                                                                                              \
    } while (0)

esp_err_t LoraClient::initialize(bool forceTest)
{
    _forceTest = forceTest;
    _hal = new EspHal(PIN_RADIO_SCK, PIN_RADIO_MISO, PIN_RADIO_MOSI);
    _radio = new SX1262(new Module(_hal, PIN_RADIO_NSS, PIN_RADIO_DIO1, PIN_RADIO_RST, PIN_RADIO_BUSY));

    int state = _radio->begin(868, 125, 7, 5, 0x12, _settings->transmitPower(), 8);
    if (state != RADIOLIB_ERR_NONE)
    {
        ESP_LOGE(TAG, "Radio initialization failed (code %d)", state);
        return ESP_FAIL;
    }

    _radio->setPacketReceivedAction(dataReady);

    uint8_t chipid[6];
    esp_efuse_mac_get_default(chipid);
    char buffer[13];
    sprintf(buffer, "%02x%02x%02x%02x%02x%02x", chipid[0], chipid[1], chipid[2], chipid[3], chipid[4], chipid[5]);
    _chipId.assign(buffer);

    return ESP_OK;
}

esp_err_t LoraClient::sendMeasurements(float temp, float humidity, AdcMeasurements &adcMeasurements, uint8_t wakeupCount)
{
    char buffer[250];
    CHECK_SNPRINTF(snprintf(
        buffer,
        250,
        R"({"model":"PlantSense","msg":"data","id":"%s","name":"%s","tempc":%.1f,"hum":%.0f,"bat":%.2f,"batPct":%i,"moi":%i,"moiRaw":%i,"test":%s,"idx":%u,"v":%u,"fw":"%s"})",
        _chipId.c_str(),
        _settings->getName().c_str(),
        temp,
        humidity * 100,
        adcMeasurements.BatteryVolts,
        adcMeasurements.BatteryPercent,
        adcMeasurements.MoisturePercent,
        adcMeasurements.MoistureRaw,
        _forceTest || _settings->isTest() ? "true" : "false",
        wakeupCount,
        _settings->version(),
        FIRMWARE_VERSION
    ));

    return transmitData(buffer);
}

esp_err_t LoraClient::sendConfig()
{
    char buffer[250];
    CHECK_SNPRINTF(snprintf(
        buffer,
        250,
        R"({"model":"PlantSense","msg":"config","id":"%s","name":"%s","test":%s,"sleep":%i,"retx":%u,"wait":%u,"pwr":%u,"moiDry":%u,"moiWet":%u,"v":%u,"fw":"%s"})",
        _chipId.c_str(),
        _settings->getName().c_str(),
        _settings->isTest() ? "true" : "false",
        _settings->sleepSeconds(),
        _settings->retransmits(),
        _settings->waitSeconds(),
        _settings->transmitPower(),
        _settings->moiDry(),
        _settings->moiWet(),
        _settings->version(),
        FIRMWARE_VERSION
    ));

    return transmitData(buffer);
}

esp_err_t LoraClient::sendWifiInfo(WifiClient *wifiClient)
{
    char buffer[250];
    CHECK_SNPRINTF(snprintf(
        buffer,
        250,
        R"({"model":"PlantSense","msg":"wifi","id":"%s","name":"%s","test":%s,"wifiRssi":%i,"uptime":%u,"fw":"%s"})",
        _chipId.c_str(),
        _settings->getName().c_str(),
        _forceTest || _settings->isTest() ? "true" : "false",
        wifiClient->getRssi(),
        wifiClient->getConnectTime(),
        FIRMWARE_VERSION
    ));

    return transmitData(buffer);
}

esp_err_t LoraClient::transmitData(const char *buffer)
{
    _state = LORA_STATE_TRANSMIT;

    ESP_LOGD(TAG, "Sending: %s", buffer);
    for (size_t i = 0; i < _settings->retransmits(); i++)
    {
        ESP_LOGI(TAG, "Retransmit %i", i);
        int16_t state = _radio->transmit(buffer);
        if (state != RADIOLIB_ERR_NONE)
        {
            ESP_LOGE(TAG, "Radio transmit failed (code %d)", state);
            return ESP_FAIL;
        }
        vTaskDelay(1);
    }

    return startListening();
}

esp_err_t LoraClient::getData()
{
    if (_state == LORA_STATE_DATA_READY)
    {
        _state = LORA_STATE_RECEIVING;

        size_t len = _radio->getPacketLength();
        if (len > 255)
        {
            ESP_LOGE(TAG, "Packet too long (%u bytes), discarding.", len);
            return ESP_OK;
        }
        uint8_t buffer[len + 1];
        buffer[len] = 0;
        int state = _radio->readData(buffer, len);
        ESP_LOGI(TAG, "Lora Command received (len: %u).", len);

        if (state == RADIOLIB_ERR_NONE)
        {
            processData((char *)buffer);
        }
        else if (state == RADIOLIB_ERR_RX_TIMEOUT)
        {
            ESP_LOGE(TAG, "Radio readData timed out!");
        }
        else if (state == RADIOLIB_ERR_CRC_MISMATCH)
        {
            ESP_LOGE(TAG, "Lora Command received with CRC error.");
        }
        else
        {
            ESP_LOGE(TAG, "Radio get Data failed with code %i", state);
        }
    }
    return ESP_OK;
}

esp_err_t LoraClient::processData(char *buffer)
{
    cJSON *json = cJSON_Parse(buffer);
    if (json == NULL)
    {
        ESP_LOGE(TAG, "Received non JSON command '%s'.", buffer);
        return ESP_FAIL;
    }

    std::string id;
    if (getJsonString("id", json, id) != ESP_OK)
    {
        ESP_LOGW(TAG, "Command does not contain a device id.");
        cJSON_free(json);
        return ESP_FAIL;
    }

    if (strcasecmp(_chipId.c_str(), id.c_str()) != 0)
    {
        ESP_LOGI(TAG, "Device id of the command (%s) does not match ours (%s).", id.c_str(), _chipId.c_str());
        cJSON_free(json);
        return ESP_FAIL;
    }

    std::string cmd;
    if (getJsonString("cmd", json, cmd) != ESP_OK)
    {
        ESP_LOGW(TAG, "Command does not contain a command.");
        cJSON_free(json);
        return ESP_FAIL;
    }

    esp_err_t result = ESP_FAIL;
    if (cmd == "set_config")
    {
        ESP_LOGI(TAG, "Got set_config command.");
        result = updateConfig(json);
        if (result == ESP_OK)
        {
            result = sendConfig();
        }
    }
    else if (cmd == "get_config")
    {
        ESP_LOGI(TAG, "Got get_config command.");
        result = sendConfig();
    }
    else
    {
        ESP_LOGW(TAG, "Unknown command '%s'.", cmd.c_str());
    }

    cJSON_free(json);
    return result;
}

esp_err_t LoraClient::updateConfig(const cJSON *json)
{
    std::string strValue;
    bool boolValue;
    int32_t intValue = 0;

    if (getJsonString("name", json, strValue) == ESP_OK)
    {
        ESP_LOGI(TAG, "Setting new name '%s'.", strValue.c_str());
        _settings->setName(strValue);
    }

    if (getJsonInt("sleep", json, intValue) == ESP_OK)
    {
        intValue = std::max<int32_t>(10, std::min<int32_t>(3600, intValue));
        ESP_LOGI(TAG, "Setting sleep time to %li.", intValue);
        _settings->setSleepSeconds((uint16_t)intValue);
    }

    if (getJsonInt("wait", json, intValue) == ESP_OK)
    {
        intValue = std::max<int32_t>(1, std::min<int32_t>(60, intValue));
        ESP_LOGI(TAG, "Setting wait time to %li.", intValue);
        _settings->setWaitSeconds(intValue);
    }

    if (getJsonInt("txPower", json, intValue) == ESP_OK)
    {
        intValue = std::max<int32_t>(0, std::min<int32_t>(22, intValue));
        ESP_LOGI(TAG, "Setting transmit power to %li.", intValue);
        _settings->setTransmitPower(intValue);
    }

    if (getJsonInt("retransmits", json, intValue) == ESP_OK)
    {
        intValue = std::max<int32_t>(1, std::min<int32_t>(10, intValue));
        ESP_LOGI(TAG, "Setting retransmits to %li.", intValue);
        _settings->setRetransmits(intValue);
    }

    if (getJsonBool("test", json, boolValue) == ESP_OK)
    {
        ESP_LOGI(TAG, "Setting is test to %i.", boolValue);
        _settings->setIsTest(boolValue);
    }

    if (getJsonInt("moiDry", json, intValue) == ESP_OK)
    {
        intValue = std::max<int32_t>(0, std::min<int32_t>(3300, intValue));
        ESP_LOGI(TAG, "Setting moisture dry point to %li.", intValue);
        _settings->setMoiDry((uint16_t)intValue);
    }

    if (getJsonInt("moiWet", json, intValue) == ESP_OK)
    {
        intValue = std::max<int32_t>(0, std::min<int32_t>(3300, intValue));
        ESP_LOGI(TAG, "Setting moisture wet point to %li.", intValue);
        _settings->setMoiWet((uint16_t)intValue);
    }

    _settings->save();

    return ESP_OK;
}

esp_err_t LoraClient::startListening()
{
    _state = LORA_STATE_RECEIVING;
    int ret = _radio->startReceive();
    if (ret != RADIOLIB_ERR_NONE)
    {
        ESP_LOGE(TAG, "Failed to start receive (%i)", ret);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t LoraClient::sleep()
{
    int16_t state = _radio->sleep();
    if (state != RADIOLIB_ERR_NONE)
    {
        ESP_LOGE(TAG, "Radio could not enter sleep state (%d)", state);
        return ESP_FAIL;
    }
    return ESP_OK;
}

void IRAM_ATTR LoraClient::dataReady(void)
{
    if (_state == LORA_STATE_RECEIVING)
    {
        _state = LORA_STATE_DATA_READY;
    }
}

esp_err_t LoraClient::getJsonString(const char *name, const cJSON *json, std::string &value)
{
    cJSON *property = cJSON_GetObjectItem(json, name);

    if (property == NULL)
    {
        ESP_LOGI(TAG, "Json %s is not present.", name);
        return ESP_FAIL;
    }

    if (!cJSON_IsString(property) || property->valuestring == NULL)
    {
        ESP_LOGE(TAG, "Json %s is not a string.", name);
        return ESP_FAIL;
    }

    value.assign(property->valuestring);

    return ESP_OK;
}

esp_err_t LoraClient::getJsonInt(const char *name, const cJSON *json, int32_t &value)
{
    cJSON *property = cJSON_GetObjectItem(json, name);

    if (property == NULL)
    {
        ESP_LOGI(TAG, "Json %s is not present.", name);
        return ESP_FAIL;
    }

    if (!cJSON_IsNumber(property))
    {
        ESP_LOGE(TAG, "Json %s is not an int.", name);
        return ESP_FAIL;
    }

    value = property->valueint;
    return ESP_OK;
}

esp_err_t LoraClient::getJsonBool(const char *name, const cJSON *json, bool &value)
{
    cJSON *property = cJSON_GetObjectItem(json, name);

    if (property == NULL)
    {
        ESP_LOGI(TAG, "Json %s is not present.", name);
        return ESP_FAIL;
    }

    if (!cJSON_IsBool(property))
    {
        ESP_LOGE(TAG, "Json %s is not a bool.", name);
        return ESP_FAIL;
    }

    value = cJSON_IsTrue(property);
    return ESP_OK;
}
