/*
 * 阶段1：MPU6050 姿态观察（不接电机控制）
 * 注意：ESP32 默认 I2C 为 SDA=21 / SCL=22，而本车电机 AIN2=GPIO22，
 *       若 Wire.begin() 不指定引脚，SCL 会打在电机脚上导致轮子狂转。
 *
 * 双核：imu_task 钉在 Core0 全速 mpu.update()；loop 在 Core1 每秒统计更新频率。
 */
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>

static const int kImuSda = 18;
static const int kImuScl = 19;
static const int kMotorPins[] = {22, 23, 12, 13};

MPU6050 mpu(Wire);
bool imu_ok = false;

// imu_task(Core0) 写，loop(Core1) 读
volatile uint32_t g_imu_update_count = 0;
volatile float g_angle_x = 0.0f;
volatile float g_angle_y = 0.0f;
volatile float g_angle_z = 0.0f;

static void holdMotorsSafe()
{
  for (int pin : kMotorPins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
}

void imu_task(void *param)
{
  (void)param;
  for (;;) {
    if (!imu_ok) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    mpu.update();
    g_angle_x = mpu.getAngleX();
    g_angle_y = mpu.getAngleY();
    g_angle_z = mpu.getAngleZ();
    g_imu_update_count++;
  }
}

void setup()
{
  Serial.begin(115200);
  delay(200);
  holdMotorsSafe();

  Wire.begin(kImuSda, kImuScl);
  Wire.setClock(400000);

  byte status = mpu.begin();
  Serial.printf("MPU6050 begin status=%u (0=ok), I2C SDA=%d SCL=%d\n",
                status, kImuSda, kImuScl);
  if (status != 0) {
    Serial.println("MPU6050 not found. Check wiring / kImuSda,kImuScl. Motors held LOW.");
    imu_ok = false;
  } else {
    Serial.println("Calculating offsets, keep IMU still...");
    delay(1000);
    mpu.calcOffsets();
    Serial.println("Done. imu_task on core0, loop reports Hz on core1.");
    imu_ok = true;
  }

  /**
   * imu_task    任务函数
   * "imu_task"  任务名称
   * 4096        栈大小
   * NULL        参数
   * 2           优先级（高于 loop 默认优先级）
   * NULL        handle
   * 0           钉在 Core0（与 main.cpp 里 microros_task 同核策略：专用任务占一核）
   */
  xTaskCreatePinnedToCore(imu_task, "imu_task", 4096, NULL, 2, NULL, 0);
}

void loop()
{
  static uint32_t last_ms = 0;
  static uint32_t last_count = 0;
  static uint32_t max_hz = 0;

  holdMotorsSafe();

  uint32_t now = millis();
  if (now - last_ms < 1000) {
    delay(10);
    return;
  }

  uint32_t count = g_imu_update_count;
  uint32_t hz = count - last_count;
  last_count = count;
  last_ms = now;
  if (hz > max_hz) {
    max_hz = hz;
  }

  // 每秒：本秒更新频率、历史最大频率、当前角度、loop 所在核
  Serial.printf("imu_hz=%u  imu_hz_max=%u  X=%.2f Y=%.2f Z=%.2f  loop_core=%u\n",
                hz, max_hz,
                g_angle_x, g_angle_y, g_angle_z,
                (unsigned)xPortGetCoreID());
}
