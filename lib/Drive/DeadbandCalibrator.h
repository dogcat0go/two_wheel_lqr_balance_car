#pragma once

#include <stdint.h>

#include "CurrentSensor.h"
#include "WheelActuator.h"
#include "WheelSensor.h"

// 死区自标定（架空）+ r 前电流探轮自检。共用 ticks 门槛 / 停等 / 独占电机状态机。
// 每拍 update() 推进一步；不依赖上层 config，参数经 Params 传入。
class DeadbandCalibrator {
public:
    struct Params {
        float   ramp_pct_per_tick; // 标定：每拍 duty 增量 (%)
        float   duty_max_pct;      // 标定：爬升上限 (%)
        int32_t ticks_thresh;      // 判起转的编码器增量（标定与自检共用）
        int     settle_ticks;      // 标定：阶段间停等拍数
        float   scale;             // 标定：存库前欠补系数
        float   probe_i_a;              // 自检：探轮电流 (A)，两轮同向
        int     probe_hold_ticks;       // 自检：最长给电流拍数
        int     probe_settle_ticks;     // 自检：起步停等拍数
        bool    probe_enable;           // false：requestArm 直接通过，不推轮
        float   probe_abort_pitch_rad;  // 离 trim 超过则不探 / 探中停
        float   probe_abort_omega;      // rad/s，探中角速度过大则停
        float   dt_s;                   // 控制周期，电流环用
    };

    void init(WheelActuator* actuators, WheelSensor* sensors,
              CurrentSensor* currents, const Params& params);
    void loadStored(float default_l, float default_r);
    void start();       // 架空死区斜坡
    void requestArm(float e_pitch, float omega); // r：关探轮则立刻通过，否则共模探轮
    void abortPrearm(); // s / 切开环 / 倾角跑飞：立刻停探轮，不武装
    bool active() const { return kind_ != kIdle; }
    bool prearming() const { return kind_ == kPrearm; }
    uint8_t snapMode() const { return (kind_ == kPrearm) ? 8 : 9; }
    void update(float e_pitch = 0.0f, float omega = 0.0f);
    // 自检刚通过时为 true，读走后清掉。失败不置位。
    bool takePrearmPass();

private:
    enum Kind { kIdle = 0, kCalib, kPrearm };

    bool ticksMoved(int wheel, float dir) const;
    void stopBoth();
    void startPrearm();
    void finishPrearm(bool ok);

    WheelActuator* act_ = nullptr;
    WheelSensor*   sen_ = nullptr;
    CurrentSensor* cur_ = nullptr;
    Params  p_{};
    Kind    kind_ = kIdle;
    int     phase_ = 0;
    float   duty_ = 0.0f;
    int32_t start_ticks_[2] = {0, 0};
    int     settle_ = 0;
    int     hold_ = 0;
    bool    capped_any_ = false;
    bool    prearm_pass_ = false;
    float   meas_[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
};
