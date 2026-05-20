/*
 * @Author: LCOIT dogcat.let@gmail.com
 * @Date: 2026-05-11 16:38:47
 * @LastEditors: LCOIT dogcat.let@gmail.com
 * @LastEditTime: 2026-05-20 00:47:27
 * @FilePath: /fishbot_esp32_example/src/main.cpp
 * @Description: 这是默认设置,请设置`customMade`
 */
#include <Arduino.h>
#include <Esp32McpwmMotor.h>
#include <Esp32PcntEncoder.h>
#include "PidController.h"
#include "Kinematics.h"

Esp32McpwmMotor motor;        // 创建一个名为motor的对象，用于控制电机
Esp32PcntEncoder encoders[2]; // 创建一个数组用于存储两个编码器
PidController pid_controller[2];
Kinematics kinematics;

static const uint32_t kCtrlPeriodMs = 20;       // 控制周期 20ms
static const float    kAlpha        = 0.3f;     // 速度低通滤波系数(0~1, 越小越平滑)
static const float    kMmPerTick    = 0.105805f;
static const float    kWheelDistance = 175.0f;

// test motor speed
static float target_speed = 20.0f; // mm/s
static float target_angular_speed = 0.1f; // rad/s
float output_left_motor_speed = 0.0f;
float output_right_motor_speed = 0.0f;

int64_t last_ticks[2] = {0, 0};
int16_t delta_ticks[2] = {0, 0};
int64_t last_update_time = 0;
float current_speed[2] = {0, 0};
float   filtered_speed[2]  = {0.0f, 0.0f};

void motorSpeedControl()
{
    uint32_t now = millis();
    uint32_t dt_ms = now - last_update_time;
    if (dt_ms == 0) return;
    float dt_s = dt_ms / 1000.0f;
    int64_t t0 = encoders[0].getTicks();
    int64_t t1 = encoders[1].getTicks();
    int32_t d0 = (int32_t)(t0 - last_ticks[0]);
    int32_t d1 = (int32_t)(t1 - last_ticks[1]);
    last_ticks[0] = t0;
    last_ticks[1] = t1;
    last_update_time = now;
    current_speed[0] = (d0 * kMmPerTick) / dt_s;
    current_speed[1] = (d1 * kMmPerTick) / dt_s;
    filtered_speed[0] = kAlpha * current_speed[0] + (1 - kAlpha) * filtered_speed[0];
    filtered_speed[1] = kAlpha * current_speed[1] + (1 - kAlpha) * filtered_speed[1];
    motor.updateMotorSpeed(0, pid_controller[0].update(filtered_speed[0], dt_s));
    motor.updateMotorSpeed(1, pid_controller[1].update(filtered_speed[1], dt_s));
    kinematics.update_motor_speed(now,encoders[0].getTicks(), encoders[1].getTicks());
}

void setup()
{
    Serial.begin(115200); // 初始化串口通信，波特率为115200
    // 2.设置编码器
    encoders[0].init(0, 32, 33);  // 初始化第一个编码器，使用GPIO 32和33连接
    encoders[1].init(1, 26, 25);  // 初始化第二个编码器，使用GPIO 26和25连接
    motor.attachMotor(0, 22, 23); // 将电机0连接到引脚33和引脚25
    motor.attachMotor(1, 12, 13); // 将电机1连接到引脚26和引脚27
    Serial.printf("motor and encoder init success!\n");

    // 初始化PID控制器参数
    pid_controller[0].update_pid(0.6, 0.5, 0.0);
    pid_controller[1].update_pid(0.6, 0.5, 0.0);
    pid_controller[0].out_limit(-100, 100);
    pid_controller[1].out_limit(-100, 100);
    // pid_controller[0].update_target(100);
    // pid_controller[1].update_target(100);

    // 初始化运动学参数
    kinematics.set_motor_param(0, kMmPerTick, 0, encoders[0].getTicks());
    kinematics.set_motor_param(1, kMmPerTick, 0, encoders[1].getTicks());
    kinematics.set_motor_distance(kWheelDistance);

    // 测试运动学逆运算
    kinematics.kinematic_inverse(target_speed, target_angular_speed, &output_left_motor_speed, &output_right_motor_speed);
    Serial.printf("output_left_motor_speed=%.2f output_right_motor_speed=%.2f\n", output_left_motor_speed, output_right_motor_speed);
    pid_controller[0].update_target(output_left_motor_speed);
    pid_controller[1].update_target(output_right_motor_speed);

    // Serial.printf("set motor speed success!\n");
    
}

void loop()
{
    static uint32_t next_tick = 0;
    uint32_t now = millis();
    if ((int32_t)(now - next_tick) >= 0) {
        next_tick = now + kCtrlPeriodMs;
        motorSpeedControl();
        // Serial.printf("L=%.2f R=%.2f\n", filtered_speed[0], filtered_speed[1]);
        Serial.printf("x=%.2f y=%.2f theta=%.2f linear_speed=%.2f angular_speed=%.2f\n", kinematics.get_odometry()->x, kinematics.get_odometry()->y, kinematics.get_odometry()->theta, kinematics.get_odometry()->linear_speed, kinematics.get_odometry()->angular_speed);
    }
}
