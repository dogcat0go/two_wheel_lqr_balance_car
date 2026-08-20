#pragma once

#include <Esp32McpwmMotor.h>

// 单轮执行器（硬件边界层）：车体坐标系指令 → 电机 PWM
// 方向 dir 在此收口；Esp32McpwmMotor 是多路共享驱动，由外部持有。
// applyRawPwm：平衡 / 开环占空比；applyCurrent：I_ref 电流 PI，内部仍落到 PWM。
class WheelActuator {
public:
    struct Params {
        int   motor_id;
        int   pin_a;
        int   pin_b;
        float dir;
        float max_duty;
        float deadband;
        float cmd_eps;
        float tau_eps; // applyTorque 休息区 (N·m)，0=关
        float i_max;
        float kt; // N·m/A，applyTorque 用
        float kp;
        float ki;
    };

    void init(Esp32McpwmMotor* driver, const Params& params);

    void applyRawPwm(float duty);
    // I_ref / I_meas：车体前进为正 (A)。dt_s = kCtrlDt。
    // compensate_db=false 时不补死区；ff_dir≠0 时前馈方向跟 ff_dir，否则跟 duty。
    void applyCurrent(float i_ref, float i_meas, float dt_s,
                      bool compensate_db = true, float ff_dir = 0.0f);
    void applyTorque(float tau_nm, float i_meas, float dt_s,
                     bool compensate_db = true, float ff_dir = 0.0f);
    void stop() { applyRawPwm(0.0f); }
    void resetCurrentLoop();
    float lastDuty() const { return last_duty_; }

    // 死区标定用：绕过死区前馈直接下发原始 duty；标定完把测得门槛写回。
    void writePwmRaw(float duty) { writePwm(duty); }
    void setDeadband(float d) { params_.deadband = d; }
    float deadband() const { return params_.deadband; }

private:
    float compensateDeadband(float duty, float dir) const; // 唯一死区前馈；dir=0 跟 duty
    void writePwm(float duty);

private:
    Esp32McpwmMotor* driver_ = nullptr;
    Params params_{};
    float integ_ = 0.0f;
    float last_duty_ = 0.0f;
    bool  torque_rest_ = true; // applyTorque 休息区状态（回差）
};
