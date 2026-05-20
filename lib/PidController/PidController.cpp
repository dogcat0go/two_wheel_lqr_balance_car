/*
 * @Author: LCOIT dogcat.let@gmail.com
 * @Date: 2026-05-18 04:04:35
 * @LastEditors: LCOIT dogcat.let@gmail.com
 * @LastEditTime: 2026-05-18 05:01:13
 * @FilePath: /fishbot_esp32_mt_example/lib/PidController/PidController.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "PidController.h"
#include "Arduino.h"


PidController::PidController(float kp, float ki, float kd)
    : kp_(kp), ki_(ki), kd_(kd) {}

float PidController::update(float current, float dt_s)
{
    if (dt_s <= 0.0f) return 0.0f;

    float error = target_ - current;

    float p_term = kp_ * error;
    float d_term = kd_ * (error - prev_error_) / dt_s;
    prev_error_  = error;

    // 先预算未饱和的输出，判断是否需要冻结积分（条件积分抗饱和）
    float unsat = p_term + ki_ * sum_error_ + d_term;
    bool saturating_high = (unsat >= out_max_) && (error > 0.0f);
    bool saturating_low  = (unsat <= out_min_) && (error < 0.0f);
    if (!saturating_high && !saturating_low) {
        sum_error_ += error * dt_s;
        if (sum_error_ > intergral_max_) sum_error_ = intergral_max_;
        if (sum_error_ < intergral_min_) sum_error_ = intergral_min_;
    }

    float output = p_term + ki_ * sum_error_ + d_term;
    if (output > out_max_) output = out_max_;
    if (output < out_min_) output = out_min_;
    return output;
}

void PidController::update_target(float target) { target_ = target; }

void PidController::update_pid(float kp, float ki, float kd)
{
    kp_ = kp; ki_ = ki; kd_ = kd;
}

void PidController::out_limit(float out_min, float out_max)
{
    out_min_ = out_min; out_max_ = out_max;
}

void PidController::integral_limit(float i_min, float i_max)
{
    intergral_min_ = i_min; intergral_max_ = i_max;
}

void PidController::reset()
{
    prev_error_ = 0.0f;
    sum_error_  = 0.0f;
}
