#pragma once

#include <MPU6050_light.h>

// 姿态估计（硬件边界层）：I2C → pitch / pitch_rate / yaw_rate
// 航向角 ψ 用轮式里程计（Ahrs 不积分 yaw，避免陀螺漂移当 heading hold 参考）。
//
// 为什么不直接用 MPU6050_light::update()：
// 该函数内部用 millis() 求 dt，200Hz 下 dt=5ms 会被量化成 4/5/6ms（±20% 误差），
// 陀螺积分项直接跟着抖。这里只借它做 I2C 收发与量纲换算（fetchData），
// 互补滤波自己算，dt 由控制环按固定周期传入。
class Ahrs {
public:
    struct Params {
        int      sda;
        int      scl;
        uint32_t i2c_hz;
        float    pitch_sign; // +1/-1，IMU 装反时翻这里
        float    yaw_sign;   // +1/-1，陀螺 Z 与车体逆时针不一致时翻这里
        float    gyro_coef;  // 互补滤波系数，越接近 1 越信陀螺
    };

    // 含静止陀螺零偏标定（阻塞约 1s），只在 setup() 里调；加速度不参与上电标定
    bool init(const Params& params);

    // 每个控制周期调一次，dt_s 用控制环的名义周期
    void update(float dt_s);

    float    pitch() const { return pitch_rad_; }          // rad，正 = 前倾
    float    pitchRate() const { return pitch_rate_rps_; } // rad/s
    float    pitchAcc() const { return pitch_acc_rad_; }   // rad，仅加速度计
    float    accX() const { return acc_g_[0]; }            // g，减 offset 后
    float    accY() const { return acc_g_[1]; }
    float    accZ() const { return acc_g_[2]; }
    float    yawRate() const { return yaw_rate_rps_; }     // rad/s，正 = 逆时针
    bool     ok() const { return ok_; }                    // 初始化成功且总线有数据
    uint32_t stampMs() const { return stamp_ms_; }         // 最后一次有效更新时刻

private:
    MPU6050  imu_{Wire};
    Params   params_{};
    bool     ok_ = false;
    float    pitch_rad_ = 0.0f;
    float    pitch_acc_rad_ = 0.0f;
    float    pitch_rate_rps_ = 0.0f;
    float    yaw_rate_rps_ = 0.0f;
    float    acc_g_[3] = {};
    uint32_t stamp_ms_ = 0;
};
