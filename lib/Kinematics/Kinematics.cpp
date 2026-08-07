/*
 * @Author: LCOIT dogcat.let@gmail.com
 * @Date: 2026-05-18 18:40:30
 * @LastEditors: LCOIT dogcat.let@gmail.com
 * @LastEditTime: 2026-05-20 01:36:21
 * @FilePath: /fishbot_esp32_mt_example/lib/Kinematics/Kinematics.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "Kinematics.h"
#include <cmath>

void Kinematics::set_motor_param(int motor_id, float per_pulse_mm, float dir, int64_t last_encoder_ticks)
{
    motor_param_[motor_id].per_pulse_mm = per_pulse_mm;
    motor_param_[motor_id].dir = dir;
    motor_param_[motor_id].motor_speed = 0.0f;
    motor_param_[motor_id].filtered_speed = 0.0f;
    motor_param_[motor_id].last_encoder_ticks = last_encoder_ticks;
}

void Kinematics::get_motor_param(int motor_id, float *per_pulse_mm, float *motor_speed, int64_t *last_encoder_ticks)
{
    *per_pulse_mm = motor_param_[motor_id].per_pulse_mm;
    *motor_speed = motor_param_[motor_id].motor_speed;
    *last_encoder_ticks = motor_param_[motor_id].last_encoder_ticks;
}

void Kinematics::set_motor_distance(float wheel_distance)
{
    wheel_distance_ = wheel_distance;
}

/**
    * @brief 更新电机速度
    * @param now 当前时间
    * @param left_ticks 左轮编码器计数值
    * @param right_ticks 右轮编码器计数值
    * @return void
**/
void Kinematics::update_motor_speed(uint64_t now, int64_t left_ticks, int64_t right_ticks)
{
    uint32_t dt_ms = now - last_update_time_;
    if (dt_ms == 0) return;
    float dt_s = dt_ms / 1000.0f;
    int64_t dl = left_ticks - motor_param_[0].last_encoder_ticks;
    int64_t dr = right_ticks - motor_param_[1].last_encoder_ticks;
    motor_param_[0].last_encoder_ticks = left_ticks;
    motor_param_[1].last_encoder_ticks = right_ticks;
    last_update_time_ = now;
    motor_param_[0].motor_speed = (dl * motor_param_[0].per_pulse_mm * motor_param_[0].dir) / dt_s;
    motor_param_[1].motor_speed = (dr * motor_param_[1].per_pulse_mm * motor_param_[1].dir) / dt_s;
    for (int i = 0; i < 2; i++) {
        motor_param_[i].filtered_speed = kSpeedFilterAlpha * motor_param_[i].motor_speed +
                                         (1 - kSpeedFilterAlpha) * motor_param_[i].filtered_speed;
    }

    update_odometry(dt_ms);
}

float Kinematics::get_motor_speed(int motor_id)
{
    return motor_param_[motor_id].filtered_speed;
}

/**
    * @brief 逆运动学
    * @param linear_speed 线速度(mm/s)
    * @param angular_speed 角速度
    * @param left_speed 左轮速度(mm/s)
    * @param right_speed 右轮速度(mm/s)
    * @return void
**/
void Kinematics::kinematic_inverse(float linear_speed, float angular_speed, float *left_speed, float *right_speed)
{
    *left_speed = linear_speed - angular_speed * wheel_distance_ / 2.0f;
    *right_speed = linear_speed + angular_speed * wheel_distance_ / 2.0f;
}

void Kinematics::kinematic_forward(float left_speed, float right_speed, float *linear_speed, float *angular_speed)
{
    *linear_speed = (left_speed + right_speed) / 2;
    *angular_speed = (right_speed - left_speed) / wheel_distance_;
}


void Kinematics::update_odometry(uint32_t dt_ms)
{
    if (dt_ms == 0) return;
    float dt_s = dt_ms / 1000.0f;
    this->kinematic_forward(motor_param_[0].motor_speed, motor_param_[1].motor_speed, &odometry_.linear_speed, &odometry_.angular_speed);

    odometry_.theta += odometry_.angular_speed * dt_s;
    TransAngleInPi(odometry_.theta, &odometry_.theta);

    // 转换位置（距离 = 线速度 * 时间，必须用秒）
    float delta_distance = odometry_.linear_speed * dt_s;
    odometry_.x += delta_distance * std::cos(odometry_.theta);
    odometry_.y += delta_distance * std::sin(odometry_.theta);
}

odometry_t* Kinematics::get_odometry()
{
    return &odometry_;
}

void Kinematics::TransAngleInPi(float angle, float *angle_in_pi)
{
    if(angle_in_pi == nullptr) return;
    if (angle > kPi) *angle_in_pi -= 2 * kPi;
    if (angle < -kPi) *angle_in_pi += 2 * kPi;
}

