#pragma once

#include <Arduino.h>
#include <Esp32PcntEncoder.h>

// 单轮测量（硬件边界层）：ticks → 位移 x(m)、速度 v(m/s)、轮角速度 ω(rad/s)
// 方向 dir 在此收口，上层永远认为 正 = 车体前进。
// 位移由累计 ticks 直接换算（无积分漂移）；速度用滑动窗口差分 + 一阶低通。
class WheelSensor {
public:
    struct Params {
        int   pcnt_unit;      // PCNT 单元号
        int   pin_a;
        int   pin_b;
        float dir;            // +1/-1，正 = 前进
        float m_per_tick;     // m/tick，正值
        float wheel_radius_m; // 轮半径，omega() 换算用
        int   diff_window;    // 差分窗口（采样数，1 ~ kCapacity-1）
        float lpf_alpha;      // 速度低通系数 (0~1]，1 = 不滤波
    };

    void init(const Params& params);
    // 每个控制周期调用一次，now_us 用 micros()
    void update(uint32_t now_us);

    float   position() const { return position_m_; }   // m，参考锁存/里程计用
    float   speed() const { return speed_lpf_; }        // m/s，低通后，闭环反馈用
    float   speedRaw() const { return speed_raw_; }      // m/s，窗口差分原始值
    float   omega() const { return speed_lpf_ / params_.wheel_radius_m; } // rad/s
    int32_t ticks() const { return last_ticks_; }        // 原始计数（调试用）

private:
    static constexpr int kCapacity = 8; // 环形缓冲容量（样本数）

    struct Sample {
        int32_t  ticks;
        uint32_t t_us;
    };

    Esp32PcntEncoder encoder_;
    Params params_{};
    Sample buffer_[kCapacity]{};
    int    head_ = 0;   // 下一个写入位置
    int    count_ = 0;  // 已有样本数
    int32_t last_ticks_ = 0;
    float  position_m_ = 0.0f;
    float  speed_raw_ = 0.0f;
    float  speed_lpf_ = 0.0f;
};
