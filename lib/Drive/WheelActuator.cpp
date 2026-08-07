#include "WheelActuator.h"

#include <math.h>

void WheelActuator::init(Esp32McpwmMotor* driver, const Params& params)
{
    driver_ = driver;
    params_ = params;
    driver_->attachMotor(params_.motor_id, params_.pin_a, params_.pin_b);
}

void WheelActuator::applyRawPwm(float duty)
{
    if (duty > params_.max_duty) duty = params_.max_duty;
    if (duty < -params_.max_duty) duty = -params_.max_duty;

    // 死区前馈：把小指令抬过电机静摩擦门槛，使 [eps,max] 线性映射到 [deadband,max]。
    // eps 以下保持 0，避免平衡点附近微小指令被抬成 ±deadband 造成嗡鸣。
    float out = duty;
    if (params_.deadband > 0.0f && fabsf(duty) >= params_.cmd_eps) {
        const float span = params_.max_duty - params_.deadband;
        out = (duty > 0.0f ? params_.deadband : -params_.deadband) +
              duty * span / params_.max_duty;
    } else if (params_.deadband > 0.0f) {
        out = 0.0f;
    }
    driver_->updateMotorSpeed(params_.motor_id, (int16_t)(out * params_.dir));
}
