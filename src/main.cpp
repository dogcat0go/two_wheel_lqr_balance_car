/*
 * @Author: LCOIT dogcat.let@gmail.com
 * @Date: 2026-07-29 10:53:59
 * @LastEditors: LCOIT dogcat.let@gmail.com
 * @LastEditTime: 2026-08-03 06:40:50
 * @FilePath: /fishbot_esp32_mt_example/test/main.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <WiFi.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <Esp32McpwmMotor.h>
#include <Esp32PcntEncoder.h>

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

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
/**
 * @brief MicroROSTASK,打印ID
 *
 * @param param
 */
void microros_task(void *param)
{
  // 设置通过WIFI进行MicroROS通信
  IPAddress agent_ip;
  agent_ip.fromString("192.168.5.62");
  // 设置wifi名称，密码，电脑IP,端口号
  set_microros_wifi_transports("CHY", "13705558902", agent_ip, 8888);
  // 延时时一段时间，等待设置完成
  delay(2000);
  // 初始化内存分配器
  allocator = rcl_get_default_allocator();
  // 创建初始化选项
  rclc_support_init(&support, 0, NULL, &allocator);
  // 创建节点 microros_wifi
  rclc_node_init_default(&node, "microros_wifi", "", &support);
  // 创建执行器
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  while (true)
  {
    delay(100);
    // Serial.printf("microros_task on core:%d, set motor speed to 50\n", xPortGetCoreID());
    motor.updateMotorSpeed(0, 20);
    motor.updateMotorSpeed(1, 20);
    // 循环处理数据
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
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
  /**
   * @brief 创建一个人物在Core 0 上
   * microros_task    任务函数
   * "microros_task"  任务名称
   * 10240      任务占用内存大小
   * NULL         任务参数，为空
   * 1               任务优先级
   * NULL     任务Handle可以为空
   * 0                 内核编号
   */
  xTaskCreatePinnedToCore(microros_task, "microros_task", 10240, NULL, 1, NULL, 0);

}

// 阶段0空载测速：1s 长窗口平均，不用短滑动窗口
static const float kMmPerTick = 0.105805f; // TODO: 换电机后按实测改

void loop()
{
  static int32_t last_ticks[2] = {0, 0};
  static uint32_t last_ms = 0;
  static bool inited = false;

  uint32_t now_ms = millis();
  int32_t t0 = encoders[0].getTicks();
  int32_t t1 = encoders[1].getTicks();

  if (!inited) {
    last_ticks[0] = t0;
    last_ticks[1] = t1;
    last_ms = now_ms;
    inited = true;
    delay(1000);
    return;
  }

  uint32_t dt_ms = now_ms - last_ms;
  if (dt_ms < 1000) {
    delay(10);
    return;
  }

  float dt_s = dt_ms / 1000.0f;
  float dps0 = (t0 - last_ticks[0]) / dt_s; // ticks/s
  float dps1 = (t1 - last_ticks[1]) / dt_s;
  float v0 = dps0 * kMmPerTick; // mm/s
  float v1 = dps1 * kMmPerTick;

  Serial.printf("dt=%.2fs | L: ticks=%ld  %.1f tick/s  %.1f mm/s | R: ticks=%ld  %.1f tick/s  %.1f mm/s\n",
      dt_s,
      (long)t0, dps0, v0,
      (long)t1, dps1, v1);

  last_ticks[0] = t0;
  last_ticks[1] = t1;
  last_ms = now_ms;
}
