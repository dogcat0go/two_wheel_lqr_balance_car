// ============================================================
// 阶段0：裸机验证（Sim2Real_list.md 阶段0）
// 目的：确认「正指令 = 车体前进 = 编码器速度为正」三者一致。
// 不跑任何控制算法，不含 micro-ROS，串口交互：
//   l <duty>   左轮开环占空比 (-100~100，车体坐标系)
//   r <duty>   右轮
//   b <duty>   双轮
//   s          停止
// 每 200ms 打印 ticks / x / v。
// 判据：给 +duty，轮子朝前进方向转且 v > 0；不满足改 config.h 的 kWheelDir。
// ============================================================

#include <Arduino.h>
#include "WheelActuator.h"
#include "WheelSensor.h"
#include "config.h"

static Esp32McpwmMotor motor_driver;
static WheelActuator actuators[2];
static WheelSensor sensors[2];

void setup()
{
    Serial.begin(115200);

    for (int i = 0; i < 2; i++) {
        sensors[i].init({
            .pcnt_unit = i,
            .pin_a = cfg::kEncoderPinA[i],
            .pin_b = cfg::kEncoderPinB[i],
            .dir = cfg::kWheelDir[i],
            .m_per_tick = cfg::kMPerTick,
            .wheel_radius_m = cfg::kWheelRadiusM,
            .diff_window = cfg::kSpeedDiffWindow,
            .lpf_alpha = cfg::kSpeedLpfAlpha,
        });
        actuators[i].init(&motor_driver, {
            .motor_id = i,
            .pin_a = cfg::kMotorPinA[i],
            .pin_b = cfg::kMotorPinB[i],
            .dir = cfg::kWheelDir[i],
            .max_duty = cfg::kMaxDuty,
        });
    }

    Serial.println("stage0: l/r/b <duty(-100~100)>, s=stop");
    Serial.println("check: +duty -> wheel forward & v>0, else flip cfg::kWheelDir");
}

static void handleCommand()
{
    if (!Serial.available()) return;
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    char op = line[0];
    float duty = line.substring(1).toFloat();
    switch (op) {
    case 'l': actuators[cfg::kLeft].applyRawPwm(duty); break;
    case 'r': actuators[cfg::kRight].applyRawPwm(duty); break;
    case 'b':
        actuators[cfg::kLeft].applyRawPwm(duty);
        actuators[cfg::kRight].applyRawPwm(duty);
        break;
    case 's':
        actuators[cfg::kLeft].stop();
        actuators[cfg::kRight].stop();
        break;
    default:
        Serial.printf("unknown cmd: %s\n", line.c_str());
        return;
    }
    Serial.printf("cmd ok: %c %.1f\n", op, duty);
}

void loop()
{
    static uint32_t next_sample_ms = 0;
    static uint32_t next_print_ms = 0;

    handleCommand();

    uint32_t now_ms = millis();
    if ((int32_t)(now_ms - next_sample_ms) >= 0) {
        next_sample_ms = now_ms + 20; // 50Hz 采样
        sensors[cfg::kLeft].update(micros());
        sensors[cfg::kRight].update(micros());
    }
    if ((int32_t)(now_ms - next_print_ms) >= 0) {
        next_print_ms = now_ms + 200;
        Serial.printf("L: ticks=%ld x=%.3fm v=%.3fm/s | R: ticks=%ld x=%.3fm v=%.3fm/s\n",
            (long)sensors[cfg::kLeft].ticks(), sensors[cfg::kLeft].position(), sensors[cfg::kLeft].speed(),
            (long)sensors[cfg::kRight].ticks(), sensors[cfg::kRight].position(), sensors[cfg::kRight].speed());
    }
}
