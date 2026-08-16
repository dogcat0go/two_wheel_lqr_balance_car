#include "WheelActuator.h"

#include <math.h>

void WheelActuator::init(Esp32McpwmMotor* driver, const Params& params)
{
    driver_ = driver;
    params_ = params;
    driver_->attachMotor(params_.motor_id, params_.pin_a, params_.pin_b);
    integ_ = 0.0f;
    last_duty_ = 0.0f;
}

void WheelActuator::resetCurrentLoop()
{
    integ_ = 0.0f;
}

float WheelActuator::compensateDeadband(float duty) const
{
    if (duty > params_.max_duty) duty = params_.max_duty;
    if (duty < -params_.max_duty) duty = -params_.max_duty;
    if (params_.deadband <= 0.0f || duty == 0.0f) {
        return duty;
    }
    const float span = params_.max_duty - params_.deadband;
    return (duty > 0.0f ? params_.deadband : -params_.deadband) +
           duty * span / params_.max_duty;
}

void WheelActuator::writePwm(float duty)
{
    if (duty > params_.max_duty) duty = params_.max_duty;
    if (duty < -params_.max_duty) duty = -params_.max_duty;
    last_duty_ = duty;
    driver_->updateMotorSpeed(params_.motor_id, (int16_t)(duty * params_.dir));
}

void WheelActuator::applyRawPwm(float duty)
{
    if (fabsf(duty) < params_.cmd_eps) {
        writePwm(0.0f);
        return;
    }
    writePwm(compensateDeadband(duty));
}

void WheelActuator::applyCurrent(float i_ref, float i_meas, float dt_s)
{
    if (dt_s < 1e-6f) {
        dt_s = 0.005f;
    }
    if (i_ref > params_.i_max) {
        i_ref = params_.i_max;
    } else if (i_ref < -params_.i_max) {
        i_ref = -params_.i_max;
    }

    if (fabsf(i_ref) < 1e-3f) {
        integ_ = 0.0f;
        writePwm(0.0f);
        return;
    }

    const float err = i_ref - i_meas;
    float duty = params_.kp * err + params_.ki * integ_;
    const bool sat_hi = duty > params_.max_duty;
    const bool sat_lo = duty < -params_.max_duty;
    if (sat_hi) {
        duty = params_.max_duty;
    } else if (sat_lo) {
        duty = -params_.max_duty;
    }
    if ((!sat_hi && !sat_lo) || (sat_hi && err < 0.0f) || (sat_lo && err > 0.0f)) {
        integ_ += err * dt_s;
    }
    writePwm(compensateDeadband(duty));
}

void WheelActuator::applyTorque(float tau_nm, float i_meas, float dt_s)
{
    const float kt = (params_.kt > 1e-6f) ? params_.kt : 1.0f;
    applyCurrent(tau_nm / kt, i_meas, dt_s);
}
