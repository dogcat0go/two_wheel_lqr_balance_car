#include "WheelActuator.h"

#include <math.h>

void WheelActuator::init(Esp32McpwmMotor* driver, const Params& params)
{
    driver_ = driver;
    params_ = params;
    driver_->attachMotor(params_.motor_id, params_.pin_a, params_.pin_b);
    integ_ = 0.0f;
    last_duty_ = 0.0f;
    torque_rest_ = true;
}

void WheelActuator::resetCurrentLoop()
{
    integ_ = 0.0f;
    torque_rest_ = true;
}

float WheelActuator::compensateDeadband(float duty, float dir) const
{
    if (duty > params_.max_duty) duty = params_.max_duty;
    if (duty < -params_.max_duty) duty = -params_.max_duty;
    if (params_.deadband <= 0.0f || duty == 0.0f) {
        return duty;
    }
    const float s = (dir > 1e-9f) ? 1.0f
                  : (dir < -1e-9f) ? -1.0f
                  : (duty > 0.0f ? 1.0f : -1.0f);
    const float span = params_.max_duty - params_.deadband;
    return s * params_.deadband + duty * span / params_.max_duty;
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
    writePwm(compensateDeadband(duty, 0.0f));
}

void WheelActuator::applyCurrent(float i_ref, float i_meas, float dt_s,
                                 bool compensate_db, float ff_dir)
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
    writePwm(compensate_db ? compensateDeadband(duty, ff_dir) : duty);
}

void WheelActuator::applyTorque(float tau_nm, float i_meas, float dt_s,
                                bool compensate_db, float ff_dir)
{
    const float kt = (params_.kt > 1e-6f) ? params_.kt : 1.0f;
    const float eps = params_.tau_eps;
    if (eps > 0.0f) {
        const float mag = fabsf(tau_nm);
        if (torque_rest_) {
            if (mag <= eps * 1.5f) {
                integ_ = 0.0f;
                writePwm(0.0f);
                return;
            }
            torque_rest_ = false;
        } else if (mag < eps) {
            torque_rest_ = true;
            integ_ = 0.0f;
            writePwm(0.0f);
            return;
        }
    }
    applyCurrent(tau_nm / kt, i_meas, dt_s, compensate_db, ff_dir);
}
