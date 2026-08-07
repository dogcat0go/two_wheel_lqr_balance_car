#include "WheelSensor.h"

void WheelSensor::init(const Params& params)
{
    params_ = params;
    if (params_.diff_window < 1) params_.diff_window = 1;
    if (params_.diff_window > kCapacity - 1) params_.diff_window = kCapacity - 1;
    encoder_.init(params_.pcnt_unit, params_.pin_a, params_.pin_b);
}

void WheelSensor::update(uint32_t now_us)
{
    last_ticks_ = encoder_.getTicks();
    position_m_ = last_ticks_ * params_.m_per_tick * params_.dir;

    buffer_[head_] = {last_ticks_, now_us};
    head_ = (head_ + 1) % kCapacity;
    if (count_ < kCapacity) count_++;

    // 与 diff_window 个采样之前的样本做差分，窗口越长量化噪声越小、滞后越大
    int window = params_.diff_window;
    if (count_ - 1 < window) window = count_ - 1;
    if (window <= 0) return;

    int newest = (head_ + kCapacity - 1) % kCapacity;
    int oldest = (newest + kCapacity - window) % kCapacity;
    uint32_t dt_us = buffer_[newest].t_us - buffer_[oldest].t_us;
    if (dt_us == 0) return;

    int32_t delta = buffer_[newest].ticks - buffer_[oldest].ticks;
    speed_raw_ = delta * params_.m_per_tick * params_.dir / (dt_us * 1e-6f);
    speed_lpf_ += params_.lpf_alpha * (speed_raw_ - speed_lpf_);
}
