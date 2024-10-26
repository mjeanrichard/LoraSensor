#include "settings.h"

static const char *TAG = "SETTINGS";
static const char *NVS_NAMESPACE = "settings";

esp_err_t Settings::save()
{
    nvs_handle_t nvs;

    ESP_RETURN_ON_ERROR(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs), TAG, "Could not open nvs in write mode.");
    ESP_RETURN_ON_ERROR(nvs_set_str(nvs, "name", _name.c_str()), TAG, "Could not write name to nvs.");
    ESP_RETURN_ON_ERROR(nvs_set_i8(nvs, "isTest", _isTest), TAG, "Could not write isTest to nvs.");
    ESP_RETURN_ON_ERROR(nvs_set_u16(nvs, "sleep", _sleepSeconds), TAG, "Could not write sleep to nvs.");
    ESP_RETURN_ON_ERROR(nvs_set_u8(nvs, "wait", _waitSeconds), TAG, "Could not write wait to nvs.");
    ESP_RETURN_ON_ERROR(nvs_set_u8(nvs, "pwr", _transmitPower), TAG, "Could not write pwr to nvs.");
    ESP_RETURN_ON_ERROR(nvs_set_u8(nvs, "retx", _retransmits), TAG, "Could not write retx to nvs.");
    ESP_RETURN_ON_ERROR(nvs_commit(nvs), TAG, "Could not commit nvs.");
    nvs_close(nvs);

    return ESP_OK;
}

esp_err_t Settings::load()
{
    nvs_handle_t nvs;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (ret == ESP_ERR_NVS_NOT_FOUND)
    {
        _name = "Test";
        save();
        ESP_RETURN_ON_ERROR(nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs), TAG, "Could not open NVS in qrite mode.");
    }
    else if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open NVS settings (%i).", ret);
        return ret;
    }

    RETURN_ON_ERROR(readString(nvs, "name", _name));
    RETURN_ON_ERROR(readBool(nvs, "isTest", &_isTest));
    RETURN_ON_ERROR(nvs_get_u16(nvs, "sleep", &_sleepSeconds));
    RETURN_ON_ERROR(nvs_get_u8(nvs, "wait", &_waitSeconds));
    RETURN_ON_ERROR(nvs_get_u8(nvs, "pwr", &_transmitPower));
    RETURN_ON_ERROR(nvs_get_u8(nvs, "retx", &_retransmits));

    nvs_close(nvs);
    return ESP_OK;
}

esp_err_t Settings::readString(nvs_handle_t nvs, const char *key, std::string &field)
{
    size_t len = 0;
    ESP_RETURN_ON_ERROR(nvs_get_str(nvs, key, nullptr, &len), TAG, "Could not get length of '%s' from NVS.", key);
    char buffer[len];
    ESP_RETURN_ON_ERROR(nvs_get_str(nvs, key, buffer, &len), TAG, "Could not get length of '%s' from NVS.", key);
    field.assign(buffer);
    return ESP_OK;
}

esp_err_t Settings::readBool(nvs_handle_t nvs, const char *key, bool *value)
{
    int8_t isTest = false;
    RETURN_ON_ERROR(nvs_get_i8(nvs, key, &isTest));
    *value = isTest;
    return ESP_OK;
}