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
        float i_max;
        float kt; // N·m/A，applyTorque 用
        float kp;
        float ki;
    };

    void init(Esp32McpwmMotor* driver, const Params& params);

    void applyRawPwm(float duty);
    // I_ref / I_meas：车体前进为正 (A)。dt_s = kCtrlDt。
    void applyCurrent(float i_ref, float i_meas, float dt_s);
    void applyTorque(float tau_nm, float i_meas, float dt_s);
    void stop() { applyRawPwm(0.0f); }
    void resetCurrentLoop();
    float lastDuty() const { return last_duty_; }

private:
    float compensateDeadband(float duty) const; // 唯一死区前馈
    void writePwm(float duty);

private:
    Esp32McpwmMotor* driver_ = nullptr;
    Params params_{};
    float integ_ = 0.0f;
    float last_duty_ = 0.0f;
};
