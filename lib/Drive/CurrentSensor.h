#pragma once

#include <Arduino.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"

// 霍尔电流传感（硬件边界层）：ADC → 安培
// 默认面向 ACS712-05B（±5A，灵敏度 185 mV/A）；零点上电校零，方向由 sign 收口。
// currentRaw() = 本拍采样；current() = 一阶低通（压 PWM 纹波/噪声，闭环主用）。
// 采样只用 adc1_get_raw，禁止控制环里 analogReadMilliVolts（会反复 pinMode(ANALOG)
// 并改 RTC/ADC1，与 GPIO32/33 编码器和 WiFi 抢同一套外设）。
class CurrentSensor {
public:
    struct Params {
        int   pin;                 // ADC 引脚（须为 ADC1，WiFi 共存）
        float sensitivity_v_per_a; // V/A，ACS712-05B = 0.185
        float sign;                // +1/-1，前进为正电流
        float lpf_alpha;           // 一阶低通 (0~1]，1 = 不滤波
        int   zero_samples;        // calibrateZero() 平均采样数
        float zero_track_alpha; // 在线零点跟踪系数；0=关（不填=0）
        // 量程合理性门：|I_raw| > fault_abs_a 连续 fault_ticks 拍挂故障，
        // 连续 recover_ticks 拍回到量程内才解除；0=关（不填=0）
        float fault_abs_a;
        int   fault_ticks;
        int   recover_ticks;
    };

    void init(const Params& params);
    // 电机停转时调用：采平均零点电压；上电 setup 里调一次
    void calibrateZero();
    // 控制环内重校：无 delay，停转后调用
    void calibrateZeroFast();
    // 每个控制周期调用一次。track_zero=true（须保证 PWM=0 且轮停稳）时
    // 零点慢速跟踪本拍电压，压上电偏移与运行温漂；钳位在校零结果 ±0.05V 内。
    void update(bool track_zero = false);

    float current() const { return current_lpf_a_; }     // A，低通后
    float currentRaw() const { return current_raw_a_; }  // A，本拍原始
    float voltage() const { return voltage_v_; }         // V，本拍 ADC 电压
    float zeroVoltage() const { return v_zero_; }        // V，校零结果
    bool  ok() const { return ok_; }
    // false = 读数钉在物理不可能的值（断线/输出拉轨），闭环侧应弃用 current()
    bool  healthy() const { return ok_ && !fault_; }

private:
    float readVoltage();

    Params params_{};
    bool   ok_ = false;
    adc1_channel_t channel_{};
    esp_adc_cal_characteristics_t chars_{};
    float  v_zero_ = 0.0f;
    float  v_zero_ref_ = 0.0f; // calibrateZero* 的结果，在线跟踪的钳位中心
    float  voltage_v_ = 0.0f;
    float  current_raw_a_ = 0.0f;
    float  current_lpf_a_ = 0.0f;
    bool   fault_ = false;
    int    bad_n_ = 0;
    int    good_n_ = 0;
};
