#pragma once

#include <Esp32McpwmMotor.h>

// 单轮执行器（硬件边界层）：车体坐标系指令 → 电机 PWM
// 方向 dir 在此收口；Esp32McpwmMotor 是多路共享驱动，由外部持有。
// 阶段5标定出 R/Kt/Kb 后，在这里补 applyTorque(tau, omega)，上层出口不变。
class WheelActuator {
public:
    struct Params {
        int   motor_id;  // Esp32McpwmMotor 通道号
        int   pin_a;
        int   pin_b;
        float dir;       // +1/-1，正 = 前进
        float max_duty;  // 占空比限幅 (0~100)
        float deadband;  // 死区前馈 (%)，0 = 关；|cmd|>eps 时把输出抬到 [deadband,max]
        float cmd_eps;   // 指令死区 (%)，|cmd| 低于此不出力，防平衡点零点抖
    };

    void init(Esp32McpwmMotor* driver, const Params& params);

    // 开环占空比，车体坐标系（+ = 前进），范围 ±max_duty。阶段0验证与台架标定用。
    void applyRawPwm(float duty);
    void stop() { applyRawPwm(0.0f); }

private:
    Esp32McpwmMotor* driver_ = nullptr;
    Params params_{};
};
