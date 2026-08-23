/*
 * @Author: LCOIT dogcat.let@gmail.com
 * @Date: 2026-08-10 10:36:09
 * @LastEditors: LCOIT dogcat.let@gmail.com
 * @LastEditTime: 2026-08-22 04:02:40
 * @FilePath: /fishbot_esp32_mt_example/lib/Drive/CurrentSensor.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
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
    constexpr int kOversample = 8;              // 8 次 ≈ 240µs,200Hz 下绰绰有余
    uint32_t acc = 0;
    for (int i = 0; i < kOversample; ++i) {
        int raw = adc1_get_raw(channel_);
        acc += (raw < 0) ? 0 : (uint32_t)raw;
    }
    return esp_adc_cal_raw_to_voltage(acc / kOversample, &chars_) * 1e-3f;
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
    v_zero_ref_ = v_zero_;
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
    v_zero_ref_ = v_zero_;
    current_raw_a_ = 0.0f;
    current_lpf_a_ = 0.0f;
}

void CurrentSensor::update(bool track_zero)
{
    if (!ok_) {
        return;
    }
    voltage_v_ = readVoltage();

    // 量程合理性门：断线/输出拉轨时读数钉在物理不可能的值（E2_v1 实测 -11.7A）。
    // 超限样本不进低通、不进零点跟踪，只更新 raw 供遥测抓现场；
    // 恢复确认期内闭环侧仍视为故障，确认满拍后用当前 raw 重播种低通。
    if (params_.fault_abs_a > 0.0f) {
        const float probe_a =
            params_.sign * (voltage_v_ - v_zero_) / params_.sensitivity_v_per_a;
        if (fabsf(probe_a) > params_.fault_abs_a) {
            current_raw_a_ = probe_a;
            good_n_ = 0;
            if (bad_n_ < params_.fault_ticks) {
                bad_n_++;
            }
            if (bad_n_ >= params_.fault_ticks) {
                fault_ = true;
            }
            return;
        }
        bad_n_ = 0;
        if (fault_) {
            current_raw_a_ = probe_a;
            if (++good_n_ >= params_.recover_ticks) {
                fault_ = false;
                current_lpf_a_ = probe_a;
            }
            return;
        }
    }

    if (track_zero && params_.zero_track_alpha > 0.0f) {
        constexpr float kZeroTrackClampV = 0.05f; // ±0.05V ≈ ±270mA @185mV/A
        v_zero_ += params_.zero_track_alpha * (voltage_v_ - v_zero_);
        if (v_zero_ > v_zero_ref_ + kZeroTrackClampV) {
            v_zero_ = v_zero_ref_ + kZeroTrackClampV;
        } else if (v_zero_ < v_zero_ref_ - kZeroTrackClampV) {
            v_zero_ = v_zero_ref_ - kZeroTrackClampV;
        }
    }
    current_raw_a_ =
        params_.sign * (voltage_v_ - v_zero_) / params_.sensitivity_v_per_a;
    current_lpf_a_ +=
        params_.lpf_alpha * (current_raw_a_ - current_lpf_a_);
}
