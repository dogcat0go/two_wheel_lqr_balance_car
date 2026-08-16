#include "CurrentSensor.h"

void CurrentSensor::init(const Params& params)
{
    params_ = params;
    if (params_.sensitivity_v_per_a < 1e-6f) {
        params_.sensitivity_v_per_a = 0.185f;
    }
    if (params_.lpf_alpha <= 0.0f) {
        params_.lpf_alpha = 1.0f;
    }
    if (params_.lpf_alpha > 1.0f) {
        params_.lpf_alpha = 1.0f;
    }
    if (params_.zero_samples < 1) {
        params_.zero_samples = 64;
    }

    switch (params_.pin) {
    case 36: channel_ = ADC1_CHANNEL_0; break;
    case 37: channel_ = ADC1_CHANNEL_1; break;
    case 38: channel_ = ADC1_CHANNEL_2; break;
    case 39: channel_ = ADC1_CHANNEL_3; break;
    case 32: channel_ = ADC1_CHANNEL_4; break;
    case 33: channel_ = ADC1_CHANNEL_5; break;
    case 34: channel_ = ADC1_CHANNEL_6; break;
    case 35: channel_ = ADC1_CHANNEL_7; break;
    default:
        ok_ = false;
        return;
    }

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(channel_, ADC_ATTEN_DB_11);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &chars_);
    ok_ = true;
}

float CurrentSensor::readVoltage()
{
    const int raw = adc1_get_raw(channel_);
    return esp_adc_cal_raw_to_voltage(raw < 0 ? 0 : (uint32_t)raw, &chars_) * 1e-3f;
}

void CurrentSensor::calibrateZero()
{
    if (!ok_) {
        return;
    }
    float sum = 0.0f;
    for (int i = 0; i < params_.zero_samples; i++) {
        sum += readVoltage();
        delay(2);
    }
    v_zero_ = sum / static_cast<float>(params_.zero_samples);
    current_raw_a_ = 0.0f;
    current_lpf_a_ = 0.0f;
}

void CurrentSensor::calibrateZeroFast()
{
    if (!ok_) {
        return;
    }
    float sum = 0.0f;
    const int n = params_.zero_samples < 32 ? params_.zero_samples : 32;
    for (int i = 0; i < n; i++) {
        sum += readVoltage();
    }
    v_zero_ = sum / static_cast<float>(n);
    current_raw_a_ = 0.0f;
    current_lpf_a_ = 0.0f;
}

void CurrentSensor::update()
{
    if (!ok_) {
        return;
    }
    voltage_v_ = readVoltage();
    current_raw_a_ =
        params_.sign * (voltage_v_ - v_zero_) / params_.sensitivity_v_per_a;
    current_lpf_a_ +=
        params_.lpf_alpha * (current_raw_a_ - current_lpf_a_);
}
