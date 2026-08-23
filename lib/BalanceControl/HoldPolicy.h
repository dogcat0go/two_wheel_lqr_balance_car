#pragma once

#include <stdint.h>

// 车辆级 in-position（TRACK / CONFIRM / HOLD）。
// 不进 LQR 公式、不碰电流环：只根据状态盒子决定「两轮是否同时切到重力补偿」。
// 进入 = 与门 + 饱和计数（满足 +1 / 不满足或退出 -1，下限 0）；退出 = 或门、当拍。左右必须同一状态。
// 与 BalanceController 同层，stage2 / stage5 共用；阈值由调用方注入。
class HoldPolicy {
public:
    enum State : uint8_t {
        kTrack   = 0,
        kConfirm = 1,
        kHold    = 2,
    };

    struct Limits {
        float theta_in_rad;  // 进入 |e_θ|
        float theta_out_rad; // 退出 |e_θ|
        float omega_in;      // rad/s
        float omega_out;
        float vel_in;        // m/s
        float vel_out;
        float pos_in;        // m；<=0 关闭位置门
        float pos_out;
        float tau_in;        // |u_sum|/2；<=0 关闭力矩门
        float tau_out;
        float v_cmd_eps;     // |v_cmd| 小于此才允许进
        float w_cmd_eps;
        int   enter_ticks;   // CONFIRM 连续拍数；<=0 整策略关闭
        int   settle_ticks;  // HOLD 进入后再观望拍数；0=不用
    };

    struct Sample {
        float e_theta;  // pitch - pitch_ref (rad)
        float omega;    // pitch_rate (rad/s)
        float vel;      // 两轮均值 (m/s)
        float e_pos;    // pos - pos_ref (m)
        float tau_half; // |u_sum|/2，与 tau_in 同单位
        float v_cmd;
        float w_cmd;
        bool  allow;    // armed && !hard_fault
    };

    void setLimits(const Limits& lim) { lim_ = lim; }
    // 本车 config.h 默认门槛；torque_gate=false 时关掉力矩门（PWM 环单位不是 N·m）
    static Limits robotLimits(bool torque_gate);
    void reset();

    State update(const Sample& s);

    State   state() const { return state_; }
    bool    holding() const { return state_ == kHold; }
    uint8_t confirmCount() const { return confirm_; }
    bool    justEntered() const { return just_entered_; }

private:
    bool canEnter(const Sample& s) const;
    bool shouldExit(const Sample& s) const;

    Limits  lim_{};
    State   state_ = kTrack;
    uint8_t confirm_ = 0;
    bool    just_entered_ = false;
};
