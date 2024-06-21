#include "adc.h"

static const char *TAG = "ADC";

esp_err_t Adc::initialize()
{
    adc_cali_curve_fitting_config_t calibrationConfig = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(adc_cali_create_scheme_curve_fitting(&calibrationConfig, &_calibrationHandle), TAG, "Failed to create claibration data.");

    adc_oneshot_unit_init_cfg_t adcConfig = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&adcConfig, &_adcHandle), TAG, "Failed to create adc unit.");

    adc_oneshot_chan_cfg_t oneShotConfig = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(_adcHandle, BATTERY_ADC_CHANNEL, &oneShotConfig), TAG, "Failed to create channel for battery measurement.");
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(_adcHandle, MOISTURE_ADC_CHANNEL, &oneShotConfig), TAG, "Failed to create channel for moisture measurement.");

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_2_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_USE_RC_FAST_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .gpio_num = PIN_SOIL_PWM,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 2,
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    return ESP_OK;
}

esp_err_t Adc::stop()
{
    ESP_RETURN_ON_ERROR(ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0), TAG, "Could not stop PWM.");
    return adc_oneshot_del_unit(_adcHandle);
}


esp_err_t Adc::readValues(float &batteryVolts, uint8_t &moisturePercent)
{
    uint16_t batterySum = 0;
    uint16_t moistureSum = 0;

    for (size_t i = 0; i < NUM_ADC_READS; i++)
    {
        uint16_t rawBattery = 0;
        uint16_t rawMoisture = 0;
        ESP_RETURN_ON_ERROR(readRaw(rawBattery, BATTERY_ADC_CHANNEL), TAG, "Failed to read raw voltage.");
        ESP_RETURN_ON_ERROR(readRaw(rawMoisture, MOISTURE_ADC_CHANNEL), TAG, "Failed to read raw voltage.");
        batterySum += rawBattery;
        moistureSum += rawMoisture;
        vTaskDelay(1);
    }

    batteryVolts = (batterySum * 2) / (float)(1000 * NUM_ADC_READS);
    moisturePercent = 135 - moistureSum / (20 * NUM_ADC_READS);
    return ESP_OK;
}

esp_err_t Adc::readRaw(uint16_t &rawValue, adc_channel_t channel)
{
    int raw = 0;
    ESP_RETURN_ON_ERROR(adc_oneshot_read(_adcHandle, channel, &raw), TAG, "Failed to read adc.");
    int volts = 0;
    ESP_RETURN_ON_ERROR(adc_cali_raw_to_voltage(_calibrationHandle, raw, &volts), TAG, "Failed to convert to calibrated voltage.");
    rawValue = volts;
    return ESP_OK;
}