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

    pinMode(params_.pin, INPUT);
    // 11dB：满量程约 0~3.3V；ACS712@5V 零点≈2.5V，±5A 约 1.58~3.43V（正满幅略贴顶）
    analogSetPinAttenuation(params_.pin, ADC_11db);
    ok_ = true;
}

void CurrentSensor::calibrateZero()
{
    if (!ok_) {
        return;
    }
    uint32_t sum_mv = 0;
    for (int i = 0; i < params_.zero_samples; i++) {
        sum_mv += analogReadMilliVolts(params_.pin);
        delay(2);
    }
    v_zero_ = (sum_mv / static_cast<float>(params_.zero_samples)) * 1e-3f;
    current_raw_a_ = 0.0f;
    current_lpf_a_ = 0.0f;
}

void CurrentSensor::update()
{
    if (!ok_) {
        return;
    }
    voltage_v_ = analogReadMilliVolts(params_.pin) * 1e-3f;
    current_raw_a_ =
        params_.sign * (voltage_v_ - v_zero_) / params_.sensitivity_v_per_a;
    current_lpf_a_ +=
        params_.lpf_alpha * (current_raw_a_ - current_lpf_a_);
}
