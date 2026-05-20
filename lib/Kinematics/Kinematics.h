/*
 * @Author: LCOIT dogcat.let@gmail.com
 * @Date: 2026-05-18 18:40:21
 * @LastEditors: LCOIT dogcat.let@gmail.com
 * @LastEditTime: 2026-05-20 00:42:15
 * @FilePath: /fishbot_esp32_mt_example/lib/Kinematics/Kinematics.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#include <Arduino.h>

typedef struct {
    float per_pulse_mm; // mm/pulse
    float motor_speed; // mm/s
    int64_t last_encoder_ticks; // ticks
} motor_param_t;

typedef struct {
    float x; // mm
    float y; // mm
    float theta; // -PI ~ PI rad
    float linear_speed; // mm/s
    float angular_speed; // rad/s
} odometry_t;

static constexpr float kPi = 3.1415926f;

class Kinematics {
public:
    Kinematics() = default;
    ~Kinematics() = default;

    // 设置电机参数
    void set_motor_param(int motor_id, float per_pulse_mm, float motor_speed, int64_t last_encoder_ticks);
    void get_motor_param(int motor_id, float *per_pulse_mm, float *motor_speed, int64_t *last_encoder_ticks);
    void set_motor_distance(float wheel_distance);
    void update_motor_speed(uint64_t now, int64_t left_ticks, int64_t right_ticks);
    float get_motor_speed(int motor_id);

    // 正逆运动学
    void kinematic_inverse(float linear_speed, float angular_speed, float *left_speed, float *right_speed);
    void kinematic_forward(float left_speed, float right_speed, float *linear_speed, float *angular_speed);

    // 里程计
    void update_odometry(uint32_t dt_ms);
    odometry_t* get_odometry();
    static void TransAngleInPi(float angle, float *angle_in_pi);

private:
    odometry_t odometry_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    motor_param_t motor_param_[2];
    uint64_t last_update_time_  = 0;
    float wheel_distance_ = 0;
};

#endif