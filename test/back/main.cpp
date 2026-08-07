/*
 * @Author: LCOIT dogcat.let@gmail.com
 * @Date: 2026-05-11 16:38:47
 * @LastEditors: LCOIT dogcat.let@gmail.com
 * @LastEditTime: 2026-07-29 19:13:33
 * @FilePath: /fishbot_esp32_example/src/main.cpp
 * @Description: 这是默认设置,请设置`customMade`
 */
#include <Arduino.h>
#include <Esp32McpwmMotor.h>
#include <Esp32PcntEncoder.h>
#include "PidController.h"
#include "Kinematics.h"
#include "rmw_microros/time_sync.h"

// 引入Microros和wifi相关的库
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <rosidl_runtime_c/string_functions.h>

void microros_error_loop();

#define RCCHECK(fn)                          \
    {                                        \
        rcl_ret_t rc = fn;                   \
        if (rc != RCL_RET_OK) {              \
            Serial.printf("RCL error %d at line %d: %s\n", (int)rc, __LINE__, #fn); \
            microros_error_loop();           \
        }                                    \
    }

#define RCSOFTCHECK(fn)                      \
    {                                        \
        rcl_ret_t rc = fn;                   \
        if (rc != RCL_RET_OK) {              \
            Serial.printf("RCL soft error %d at line %d: %s\n", (int)rc, __LINE__, #fn); \
        }                                    \
    }

Esp32McpwmMotor motor;        // 创建一个名为motor的对象，用于控制电机
Esp32PcntEncoder encoders[2]; // 创建一个数组用于存储两个编码器
PidController pid_controller[2];
Kinematics kinematics;

// test motor speed
static float target_speed = 20.0f; // mm/s
static float target_angular_speed = 0.1f; // rad/s
float output_left_motor_speed = 0.0f;
float output_right_motor_speed = 0.0f;

int64_t last_update_time = 0;

// micro-ros
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;
rcl_node_t node;

geometry_msgs__msg__Twist cmd_vel_sub_msg;
rcl_subscription_t cmd_vel_sub;
rcl_publisher_t odom_pub;
nav_msgs__msg__Odometry odom_msg;
rcl_timer_t odom_timer;
static uint32_t odom_publish_count = 0;

static void fill_odom_stamp() {
    int64_t stamp_ms = rmw_uros_epoch_millis();
    if (stamp_ms <= 0) {
        // 对时未完成时用本地单调时钟，保证仍能发布（下游 slam 建议完成对时）
        stamp_ms = static_cast<int64_t>(millis());
    }
    odom_msg.header.stamp.sec = static_cast<int32_t>(stamp_ms / 1000);
    odom_msg.header.stamp.nanosec = static_cast<uint32_t>((stamp_ms % 1000) * 1000000);
}

void odom_timer_callback(rcl_timer_t * timer, int64_t last_call) {
    
    (void)timer;
    (void)last_call;
    odometry_t* odom_data = kinematics.get_odometry();
    fill_odom_stamp();

    // nav_msgs__msg__Odometry 中postion的位置是m, twis'zxa's'z'w'wst的线速度是m/s, 角速度是rad/s
    odom_msg.pose.pose.position.x = odom_data->x / 1000.0f;
    odom_msg.pose.pose.position.y = odom_data->y / 1000.0f;

    odom_msg.pose.pose.orientation.x = 0;
    odom_msg.pose.pose.orientation.y = 0;
    odom_msg.pose.pose.orientation.z = std::sin(odom_data->theta / 2.0f);
    odom_msg.pose.pose.orientation.w = std::cos(odom_data->theta / 2.0f);

    odom_msg.twist.twist.linear.x = odom_data->linear_speed / 1000.0f;
    odom_msg.twist.twist.angular.z = odom_data->angular_speed;

    rcl_ret_t pub_rc = rcl_publish(&odom_pub, &odom_msg, NULL);
    if (pub_rc != RCL_RET_OK) {
        Serial.printf("odom publish failed rc=%d\n", (int)pub_rc);
    } else {
        odom_publish_count++;
        if ((odom_publish_count % 40) == 1) {
            Serial.printf("odom pub ok #%lu stamp=%d.%09u\n",
                static_cast<unsigned long>(odom_publish_count),
                odom_msg.header.stamp.sec,
                odom_msg.header.stamp.nanosec);
        }
    }
}

void twist_callback(const void* msg) {
    const geometry_msgs__msg__Twist* twist = (const geometry_msgs__msg__Twist*)msg;
    Serial.printf("twist_callback: linear.x=%.2f, angular.z=%.2f\n", twist->linear.x, twist->angular.z);
    cmd_vel_sub_msg.linear.x = twist->linear.x;
    cmd_vel_sub_msg.angular.z = twist->angular.z;
    kinematics.kinematic_inverse(cmd_vel_sub_msg.linear.x * 1000.0f, 
        cmd_vel_sub_msg.angular.z, 
        &output_left_motor_speed, 
        &output_right_motor_speed);
    pid_controller[0].update_target(output_left_motor_speed);
    pid_controller[1].update_target(output_right_motor_speed);
}

// FreeRTOS 任务函数不能 return，否则 PC 跳到 0 触发 IllegalInstruction (A0=0)
void microros_error_loop() {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// 单独创建一个任务运行 micro-ROS 相当于一个线程
void microros_task(void* args) {
    (void)args;

    // 1. 设置传输协议并等待 WiFi 就绪
    IPAddress agent_ip;
    char wifi_ssid[] = "CHY";
    char wifi_password[] = "13705558902";
    agent_ip.fromString("192.168.5.62");
    set_microros_wifi_transports(wifi_ssid, wifi_password, agent_ip, 8888);
    vTaskDelay(pdMS_TO_TICKS(3000));
    while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
        vTaskDelay(pdMS_TO_TICKS(500));
        Serial.printf("ping agent failed\n");
    }
    Serial.printf("ping agent success\n");
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "fishbot_motion_control", "", &support));

    // 尚无 publisher/timer 时不要对 0 个 handle 调用 rclc_executor_spin：
    // 会立刻返回，任务函数结束 → Guru Meditation (IllegalInstruction)
    // 添加订阅/定时器后：num_handles>=1，rclc_executor_init + add_*，再改用 spin_some 循环

    // 初始化执行器
    unsigned int num_handles = 2;
    RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
    
    // 添加订阅
    RCCHECK(rclc_subscription_init_best_effort(&cmd_vel_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_sub_msg, &twist_callback, ON_NEW_DATA));
    
    // 初始化发布者
    RCCHECK(rclc_publisher_init_best_effort(&odom_pub, 
        &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), 
        "/odom"));
    nav_msgs__msg__Odometry__init(&odom_msg);
    rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom");
    rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_footprint");

    // 先注册定时器再 spin；切勿在 while(epoch==0) 里阻塞，否则永远走不到这里
    RCCHECK(rclc_timer_init_default(&odom_timer,
        &support,
        RCL_MS_TO_NS(50),
        odom_timer_callback));
    RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));
    Serial.printf("odom timer ready, publishing /odom at 20Hz\n");

    for (;;) {
        if (rmw_uros_epoch_millis() == 0) {
            RCSOFTCHECK(rmw_uros_sync_session(100));
        }
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        vTaskDelay(pdMS_TO_TICKS(10));
    }

}

static const uint32_t kCtrlPeriodMs = 20;       // 控制周期 20ms
static const float    kMmPerTick    = 0.105805f; // 正值：每脉冲行程(mm)
static const float    kWheelDistance = 175.0f;
// 车体坐标系：+ = 前进。方向不对只改这里（左、右可不同）
static const float    kWheelDir[2]  = {-1.f, -1.f};

void motorSpeedControl()
{
    uint32_t now = millis();
    uint32_t dt_ms = now - last_update_time;
    if (dt_ms == 0) return;
    float dt_s = dt_ms / 1000.0f;
    last_update_time = now;
    // 速度唯一出口：Kinematics 用已存的电机参数(含方向)把 ticks 换成车体速度
    kinematics.update_motor_speed(now, encoders[0].getTicks(), encoders[1].getTicks());
    for (int i = 0; i < 2; i++) {
        float output = pid_controller[i].update(kinematics.get_motor_speed(i), dt_s);
        motor.updateMotorSpeed(i, output * kinematics.get_motor_dir(i));
    }
}

void setup()
{
    Serial.begin(115200); // 初始化串口通信，波特率为115200
    // 2.设置编码器
    encoders[0].init(0, 32, 33);  // 初始化第一个编码器，使用GPIO 32和33连接
    encoders[1].init(1, 26, 25);  // 初始化第二个编码器，使用GPIO 26和25连接
    motor.attachMotor(0, 23, 22); // 将电机0连接到引脚33和引脚25
    motor.attachMotor(1, 13, 12); // 将电机1连接到引脚26和引脚27
    Serial.printf("motor and encoder init success!\n");

    // 初始化PID控制器参数
    pid_controller[0].update_pid(0.6, 0.5, 0.0);
    pid_controller[1].update_pid(0.6, 0.5, 0.0);
    pid_controller[0].out_limit(-100, 100);
    pid_controller[1].out_limit(-100, 100);
    // pid_controller[0].update_target(100);
    // pid_controller[1].update_target(100);

    // 初始化运动学参数：方向在此配置一次，之后测速/里程计/PWM 都从这里取
    kinematics.set_motor_param(0, kMmPerTick, kWheelDir[0], encoders[0].getTicks());
    kinematics.set_motor_param(1, kMmPerTick, kWheelDir[1], encoders[1].getTicks());
    kinematics.set_motor_distance(kWheelDistance);

    // 测试运动学逆运算
    // kinematics.kinematic_inverse(target_speed, target_angular_speed, &output_left_motor_speed, &output_right_motor_speed);
    // Serial.printf("output_left_motor_speed=%.2f output_right_motor_speed=%.2f\n", output_left_motor_speed, output_right_motor_speed);
    // pid_controller[0].update_target(output_left_motor_speed);
    // pid_controller[1].update_target(output_right_motor_speed);

    // Serial.printf("set motor speed success!\n");
    // 创建一个任务运行 micro-ROS
    // WiFi + micro-ROS 栈需求较大，建议 >= 16KB
      /**
   * @brief 创建一个任务在Core 0 上
   * microros_task    任务函数
   * "microros_task"  任务名称
   * 10240      任务占用内存大小
   * NULL         任务参数，为空
   * 1               任务优先级
   * NULL     任务Handle可以为空
   * 0                 内核编号
   */
    xTaskCreatePinnedToCore(microros_task, "microros_task", 16384, NULL, 1, NULL, 0);   
}

void loop()
{
    static uint32_t last_print = 0;
    static uint32_t next_tick = 0;
    uint32_t now = millis();
    // 每2分钟做一次时间的同步
    // MCU 晶振典型漂移 ±50ppm
    // 30秒累积：120 × 50e-6 = 6.0ms → 在 10ms 预算内
    static uint32_t last_sync = 0;
    if (millis() - last_sync > 120000) {
        rmw_uros_sync_session(100);
        last_sync = millis();
    }
    if ((int32_t)(now - next_tick) >= 0) {
        next_tick = now + kCtrlPeriodMs;
        motorSpeedControl();

        // 调试打印可降到 200~500ms 一次
        if (now - last_print >= 500) {
            last_print = now;
            Serial.printf("x=%.2f y=%.2f theta=%.2f linear_speed=%.2f angular_speed=%.2f\n", 
                kinematics.get_odometry()->x, 
                kinematics.get_odometry()->y, 
                kinematics.get_odometry()->theta, 
                kinematics.get_odometry()->linear_speed, 
                kinematics.get_odometry()->angular_speed);
        }
    }
}
