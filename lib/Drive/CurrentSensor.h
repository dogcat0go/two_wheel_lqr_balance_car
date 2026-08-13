#pragma once

#include <Arduino.h>

// 霍尔电流传感（硬件边界层）：ADC → 安培
// 默认面向 ACS712-05B（±5A，灵敏度 185 mV/A）；零点上电校零，方向由 sign 收口。
// currentRaw() = 本拍采样；current() = 一阶低通（压 PWM 纹波/噪声，闭环主用）。
class CurrentSensor {
public:
    struct Params {
        int   pin;                 // ADC 引脚（须为 ADC1，WiFi 共存）
        float sensitivity_v_per_a; // V/A，ACS712-05B = 0.185
        float sign;                // +1/-1，前进为正电流
        float lpf_alpha;           // 一阶低通 (0~1]，1 = 不滤波
        int   zero_samples;        // calibrateZero() 平均采样数
    };

    void init(const Params& params);
    // 电机停转时调用：采平均零点电压；上电 setup 里调一次
    void calibrateZero();
    // 每个控制周期调用一次
    void update();

    float current() const { return current_lpf_a_; }     // A，低通后
    float currentRaw() const { return current_raw_a_; }  // A，本拍原始
    float voltage() const { return voltage_v_; }         // V，本拍 ADC 电压
    float zeroVoltage() const { return v_zero_; }        // V，校零结果
    bool  ok() const { return ok_; }

private:
    Params params_{};
    bool   ok_ = false;
    float  v_zero_ = 0.0f;
    float  voltage_v_ = 0.0f;
    float  current_raw_a_ = 0.0f;
    float  current_lpf_a_ = 0.0f;
};
