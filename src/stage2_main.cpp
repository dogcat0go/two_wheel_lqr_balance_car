// ============================================================
// 阶段2~4：频带分离 + 安全层 + 姿态/速度反馈 + 偏航差速 + WiFi cmd_vel
//
// 频带分离：
//   Core1  control_task  200Hz：采样 → 姿态 → 控制器 → 安全层 → 执行
//   Core0  comm_task     串口调参 + 遥测
//   Core0  microros_task WiFi micro-ROS，订阅 /cmd_vel → linear_x / angular_z
//   控制与通信只经 shared_state.h 快照交换。
//
// 速度/角速度：串口 v/a，或 ROS /cmd_vel（有新帧时覆盖串口；超时清零）
// WiFi 参数：include/config.h 的 kWifi* / kAgent*
//
// 串口指令：
//   m 0|1     切模式          p/d/i/w/y 增益     t <deg> trim    r 复位
//   v <mps>   线速度目标      a <rad/s> 角速度    z/n 航向增益
//   e/el/er   开环            s 停机
//   x / o     模拟断链 / 恢复（仅影响串口侧 stamp 保活）
//   f <ms>    遥测周期
// ============================================================

#include <Arduino.h>
#include <string.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <rmw_microros/rmw_microros.h>

#include "Ahrs.h"
#include "BalanceController.h"
#include "Safety.h"
#include "WheelActuator.h"
#include "WheelSensor.h"
#include "config.h"
#include "shared_state.h"

#define RCSOFTCHECK(fn)                                                      \
    {                                                                        \
        rcl_ret_t rc = (fn);                                                 \
        if (rc != RCL_RET_OK) {                                              \
            Serial.printf("RCL soft error %d at %d: %s\n", (int)rc, __LINE__, #fn); \
        }                                                                    \
    }

static Esp32McpwmMotor motor_driver;
static WheelActuator actuators[2];
static WheelSensor sensors[2];
static Ahrs ahrs;
static Safety safety;
static BalanceController balance;

namespace {
inline float wrapPi(float a)
{
    return atan2f(sinf(a), cosf(a));
}

inline float clampAbs(float v, float lim)
{
    if (v > lim) return lim;
    if (v < -lim) return -lim;
    return v;
}
} // namespace

// ---------------- Core1：控制环 ----------------
static void control_task(void* param)
{
    (void)param;
    TickType_t next_wake = xTaskGetTickCount();
    const TickType_t period_ticks = pdMS_TO_TICKS(cfg::kCtrlPeriodMs);

    ControlSnapshot snap{};
    uint32_t cycle_count = 0;
    uint32_t hz_window_ms = millis();
    uint8_t  last_mode = 0;
    uint32_t last_reset_seq = 0;
    float    pos_ref = 0.0f; // 位置设定点(m)；跟速时积分，v≈0/故障时锁当前
    float    vref_smooth = 0.0f; // 斜率限幅后的速度目标：软起步/软停，驱动 pos_ref 积分与前馈
    float    yaw = 0.0f;     // 轮式里程计航向 (rad)
    float    yaw_ref = 0.0f;
    bool     yaw_hold = true;

    for (;;) {
        vTaskDelayUntil(&next_wake, period_ticks);

        const uint32_t t0_us = micros();

        for (int i = 0; i < 2; i++) {
            sensors[i].update(t0_us);
        }
        ahrs.update(cfg::kCtrlDt);

        // 必须在 ahrs.update() 之后取 now：update 内部 stamp=millis()，
        // 若 now 取在 I2C 之前且跨了 ms 边界，无符号 (now-stamp) 会下溢假报 IMU_LOST
        const uint32_t now_ms = millis();
        const CommandInput cmd = fetchCommand();

        const float v_l = sensors[cfg::kLeft].speed();
        const float v_r = sensors[cfg::kRight].speed();
        yaw += (v_r - v_l) / cfg::kWheelDistanceM * cfg::kCtrlDt;
        yaw = wrapPi(yaw);

        BalanceState x{};
        x.pitch = ahrs.pitch();
        x.pitch_rate = ahrs.pitchRate();
        x.pos = 0.5f * (sensors[cfg::kLeft].position() + sensors[cfg::kRight].position());
        x.vel = 0.5f * (v_l + v_r);

        // 模式切换或收到复位请求：清积分与故障锁存，位置/航向零点锁到当前
        if (cmd.mode != last_mode || cmd.reset_seq != last_reset_seq) {
            last_mode = cmd.mode;
            last_reset_seq = cmd.reset_seq;
            balance.reset();
            safety.clearFault();
            pos_ref = x.pos;
            vref_smooth = 0.0f;
            yaw_ref = yaw;
            yaw_hold = true;
        }
        safety.setLatch(cmd.mode == 1); // 开环模式保持阶段2的自恢复语义

        balance.setGains({cmd.gains[0], cmd.gains[1], cmd.gains[2],
                          cmd.gains[3], cmd.gains[4]});

        snap.fault = safety.evaluate(x.pitch, ahrs.ok(), now_ms,
                                     ahrs.stampMs(), cmd.stamp_ms);

        // 平衡且无故障：有速度指令时积分 pos_ref（匀速时位置误差不累积）；
        // v≈0 时锁到当前位姿，清掉遥控积分，避免松杆后仍被旧目标拽着走。
        // 故障/非平衡：清控制器积分并把零点锁到当前。
        const bool balancing = (cmd.mode == 1) && (snap.fault == Safety::kOk);
        if (balancing) {
            // 速度目标斜率限幅：软起步 + 软停。停止时 vref 斜坡衰减而非阶跃到 0，
            // pos_ref 跟着滑行减速的 vref 走，避免松杆瞬间锁死位置造成的回拉振荡。
            const float dv = cfg::kVelSlewMps2 * cfg::kCtrlDt;
            if (vref_smooth < cmd.linear_x) {
                vref_smooth = fminf(vref_smooth + dv, cmd.linear_x);
            } else if (vref_smooth > cmd.linear_x) {
                vref_smooth = fmaxf(vref_smooth - dv, cmd.linear_x);
            }

            if (fabsf(vref_smooth) < 1e-3f && fabsf(cmd.linear_x) < 1e-3f) {
                pos_ref = x.pos; // 完全停稳才锁零点、清遥控积分
            } else {
                pos_ref += vref_smooth * cfg::kCtrlDt;
                // anti-windup：位置误差饱和后停止累积，防 pos_ref 跑飞导致爆冲/过冲
                const float kpos = cmd.gains[2];
                if (kpos > 1e-6f) {
                    const float max_dev = cfg::kPosTermLimit / kpos;
                    const float dev = pos_ref - x.pos;
                    if (dev >  max_dev) pos_ref = x.pos + max_dev;
                    if (dev < -max_dev) pos_ref = x.pos - max_dev;
                }
            }
        } else {
            balance.reset();
            pos_ref = x.pos;
            vref_smooth = 0.0f;
            yaw_ref = yaw;
            yaw_hold = true;
        }

        BalanceState ref{};
        // 速度→倾角前馈：给定速度就主动前倾，顶过电机死区（docs/stage4_vel_pitch_feedforward.md）
        // 限幅相对 trim，防手滑大 vref 把参考角顶到接近摔倒角。
        ref.pitch = cmd.pitch_ref_rad
                  + clampAbs(cmd.k_vff * vref_smooth, cfg::kFfPitchLimitRad);
        ref.vel   = vref_smooth; // 斜率限幅后的速度目标：既是速度反馈参考，也驱动 pos_ref 积分
        ref.pos   = pos_ref;
        balance.setRef(ref);

        float balance_u = 0.0f;
        if (cmd.mode == 1) {
            balance_u = balance.update(x, cfg::kCtrlDt);
        }

        // 偏航差速：az≈0 heading hold；az≠0 只跟角速度，松杆锁新航向
        float u_yaw = 0.0f;
        const float yaw_rate = ahrs.yawRate();
        if (balancing) {
            float omega_ref = 0.0f;
            if (fabsf(cmd.angular_z) < cfg::kYawCmdEps) {
                if (!yaw_hold) {
                    yaw_ref = yaw;
                    yaw_hold = true;
                }
            } else {
                yaw_hold = false;
                yaw_ref = yaw;
                omega_ref = cmd.angular_z;
            }
            const float e_yaw = wrapPi(yaw_ref - yaw);
            u_yaw = cmd.k_yaw * e_yaw + cmd.k_yaw_rate * (omega_ref - yaw_rate);
            u_yaw = clampAbs(u_yaw, cfg::kMaxEffort);
        }

        for (int i = 0; i < 2; i++) {
            float desired = cmd.test_effort[i];
            if (cmd.mode == 1) {
                // 正 u_yaw → 右轮更快 → 逆时针（与 Kinematics 一致）
                desired = (i == cfg::kLeft) ? (balance_u - u_yaw) : (balance_u + u_yaw);
            }
            snap.effort[i] = safety.limit(i, desired, cfg::kCtrlDt);
            actuators[i].applyRawPwm(snap.effort[i]);
            snap.wheel_pos_m[i] = sensors[i].position();
            snap.wheel_vel_mps[i] = sensors[i].speed();
            snap.wheel_ticks[i] = sensors[i].ticks();
        }

        snap.stamp_ms = now_ms;
        snap.mode = cmd.mode;
        snap.pitch_rad = x.pitch;
        snap.pitch_rate_rps = x.pitch_rate;
        snap.pitch_ref_rad = ref.pitch;
        snap.pitch_acc_rad = ahrs.pitchAcc();
        snap.acc_g[0] = ahrs.accX();
        snap.acc_g[1] = ahrs.accY();
        snap.acc_g[2] = ahrs.accZ();
        snap.yaw_rad = yaw;
        snap.yaw_ref_rad = yaw_ref;
        snap.yaw_rate_rps = yaw_rate;
        snap.u_yaw = u_yaw;
        snap.imu_ok = ahrs.ok();
        memcpy(snap.terms, balance.terms(), sizeof(snap.terms));
        if ((micros() - t0_us) > cfg::kCtrlPeriodMs * 1000) {
            snap.overrun_count++; // 单周期算不完，控制律太重或 I2C 阻塞
        }

        cycle_count++;
        if (now_ms - hz_window_ms >= 1000) {
            snap.ctrl_hz = cycle_count;
            cycle_count = 0;
            hz_window_ms = now_ms;
        }
        publishSnapshot(snap);
    }
}

// ---------------- Core0：串口调参 + 遥测 ----------------
// micro-ROS 在同核的 microros_task：收到 /cmd_vel 就写 ros_*；
// 有新鲜 ROS 速度时覆盖串口 v；超时清零，stamp 用 ros 时刻 → 断链触发 CMD_TIMEOUT。
static bool link_up = true; // 串口侧保活：true 时无 ROS 帧也刷新 stamp（方便只调平衡）
static float test_effort[2] = {0.0f, 0.0f};
static uint8_t mode = 0;
static uint32_t reset_seq = 0;
static float pitch_ref_rad = cfg::kPitchTrimDeg * 0.0174532925f;
static float linear_x = 0.0f;     // 串口 v 写入
static float angular_z = 0.0f;    // 串口 a 写入
static uint32_t telemetry_ms = cfg::kTelemetryMs;
static float gains[5] = {cfg::kGainPitch, cfg::kGainPitchRate, cfg::kGainPos,
                         cfg::kGainVel, cfg::kGainIntegPitch};
static float k_yaw = cfg::kGainYaw;
static float k_yaw_rate = cfg::kGainYawRate;
static float k_vff = cfg::kGainVelToPitch; // 速度→倾角前馈, rad/(m/s)；串口 g 输入 deg/(m/s)

// micro-ROS → comm（同核写、读；float/u32 对齐写对 ESP32 足够）
static volatile float    ros_linear_x = 0.0f;
static volatile float    ros_angular_z = 0.0f;
static volatile uint32_t ros_cmd_stamp_ms = 0;
static volatile bool     ros_ready = false;

static float clampLinear(float v)
{
    if (v > cfg::kMaxLinearMps) return cfg::kMaxLinearMps;
    if (v < -cfg::kMaxLinearMps) return -cfg::kMaxLinearMps;
    return v;
}

static float clampAngular(float w)
{
    if (w > cfg::kMaxAngularRps) return cfg::kMaxAngularRps;
    if (w < -cfg::kMaxAngularRps) return -cfg::kMaxAngularRps;
    return w;
}

static void handleSerial()
{
    if (!Serial.available()) {
        return;
    }
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) {
        return;
    }

    const float value = line.substring(1).toFloat();
    switch (line[0]) {
    case 'm': mode = (line.substring(1).toInt() != 0) ? 1 : 0; break;
    case 'p': gains[0] = value; break;
    case 'd': gains[1] = value; break;
    case 'i': gains[4] = value; break;
    case 'w': gains[3] = value; break;
    case 'y': gains[2] = value; break;
    case 'z': k_yaw = value; break;
    case 'n': k_yaw_rate = value; break;
    case 'g': k_vff = value * 0.0174532925f; break; // 输入 deg/(m/s)，存 rad/(m/s)
    case 'v':
        linear_x = clampLinear(value);
        break;
    case 'a':
        angular_z = clampAngular(value);
        break;
    case 't': pitch_ref_rad = value * 0.0174532925f; break;
    case 'f': telemetry_ms = (uint32_t)(value < 20.0f ? 20.0f : value); break;
    case 'r': reset_seq++; break;
    case 'e':
        if (line.length() > 1 && line[1] == 'l') {
            test_effort[cfg::kLeft] = line.substring(2).toFloat();
        } else if (line.length() > 1 && line[1] == 'r') {
            test_effort[cfg::kRight] = line.substring(2).toFloat();
        } else {
            test_effort[cfg::kLeft] = test_effort[cfg::kRight] = value;
        }
        break;
    case 's':
        test_effort[0] = test_effort[1] = 0.0f;
        linear_x = 0.0f;
        angular_z = 0.0f;
        ros_linear_x = 0.0f;
        ros_angular_z = 0.0f;
        mode = 0;
        break;
    case 'x': link_up = false; break;
    case 'o': link_up = true; break;
    default:
        Serial.printf("unknown cmd: %s\n", line.c_str());
        return;
    }
    Serial.printf("cmd ok: %s -> m=%u kp=%.1f kd=%.3f ki=%.3f kvel=%.1f kpos=%.1f "
                  "kyaw=%.1f kn=%.1f vff=%.1fdeg/mps trim=%.2fdeg vref=%.3f aref=%.3f "
                  "eL=%.1f eR=%.1f link=%d ros=%d\n",
                  line.c_str(), mode, gains[0], gains[1], gains[4], gains[3], gains[2],
                  k_yaw, k_yaw_rate, k_vff * 57.2957795f,
                  pitch_ref_rad * 57.2957795f, linear_x, angular_z,
                  test_effort[cfg::kLeft], test_effort[cfg::kRight],
                  link_up, (int)ros_ready);
}

static void twist_callback(const void* msgin)
{
    const geometry_msgs__msg__Twist* twist =
        static_cast<const geometry_msgs__msg__Twist*>(msgin);
    ros_linear_x = clampLinear(static_cast<float>(twist->linear.x));
    ros_angular_z = clampAngular(static_cast<float>(twist->angular.z));
    ros_cmd_stamp_ms = millis();
    // 节流打印：确认解析是否进回调（agent 有会话 ≠ 一定进了 twist_callback）
    static uint32_t last_print_ms = 0;
    const uint32_t now = ros_cmd_stamp_ms;
    if ((int32_t)(now - last_print_ms) >= 200) {
        last_print_ms = now;
        Serial.printf("cmd_vel: lx=%.3f az=%.3f\n",
                      (float)ros_linear_x, (float)ros_angular_z);
    }
}

static void microros_task(void* param)
{
    (void)param;

    char wifi_ssid[32];
    char wifi_password[64];
    strncpy(wifi_ssid, cfg::kWifiSsid, sizeof(wifi_ssid) - 1);
    wifi_ssid[sizeof(wifi_ssid) - 1] = '\0';
    strncpy(wifi_password, cfg::kWifiPassword, sizeof(wifi_password) - 1);
    wifi_password[sizeof(wifi_password) - 1] = '\0';

    IPAddress agent_ip;
    if (!agent_ip.fromString(cfg::kAgentIp)) {
        Serial.printf("microros: bad agent ip %s\n", cfg::kAgentIp);
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    Serial.printf("microros: wifi SSID=%s agent=%s:%u\n",
                  wifi_ssid, cfg::kAgentIp, cfg::kAgentPort);
    set_microros_wifi_transports(wifi_ssid, wifi_password, agent_ip, cfg::kAgentPort);
    vTaskDelay(pdMS_TO_TICKS(2000));

    while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
        Serial.println("microros: ping agent failed, retry...");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    Serial.println("microros: ping agent ok");

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;
    rclc_executor_t executor;
    rcl_subscription_t cmd_vel_sub;
    geometry_msgs__msg__Twist cmd_vel_msg;

    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK ||
        rclc_node_init_default(&node, cfg::kRosNodeName, "", &support) != RCL_RET_OK ||
        rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK ||
        rclc_subscription_init_best_effort(
            &cmd_vel_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            cfg::kCmdVelTopic) != RCL_RET_OK ||
        rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg,
                                       &twist_callback, ON_NEW_DATA) != RCL_RET_OK) {
        Serial.println("microros: init failed, serial-only mode");
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ros_ready = true;
    Serial.printf("microros: subscribe %s ready\n", cfg::kCmdVelTopic);

    for (;;) {
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50)));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void comm_task(void* param)
{
    (void)param;
    uint32_t next_telemetry_ms = 0;
    uint8_t last_fault = Safety::kOk;

    for (;;) {
        handleSerial();

        const ControlSnapshot live = fetchSnapshot();
        if (live.fault != Safety::kOk && mode == 0 &&
            (test_effort[0] != 0.0f || test_effort[1] != 0.0f)) {
            test_effort[0] = test_effort[1] = 0.0f;
            if (last_fault == Safety::kOk) {
                Serial.printf("fault 0x%02X: cleared pending effort, re-send e/el/er to move\n",
                              live.fault);
            }
        }
        if (live.fault != Safety::kOk && last_fault == Safety::kOk && mode == 1) {
            Serial.printf("fault 0x%02X latched: send r to re-enable\n", live.fault);
        }
        last_fault = live.fault;

        const uint32_t now_ms = millis();
        const uint32_t ros_stamp = ros_cmd_stamp_ms;
        const bool ros_fresh =
            ros_ready && (ros_stamp != 0) &&
            ((int32_t)(now_ms - ros_stamp) <= (int32_t)cfg::kCmdTimeoutMs);

        // 速度：有新鲜 /cmd_vel 用 ROS，否则用串口 v；ROS 超时只清速度，不充当断链
        float cmd_linear = linear_x;
        float cmd_angular = angular_z;
        if (ros_fresh) {
            cmd_linear = ros_linear_x;
            cmd_angular = ros_angular_z;
        } else if (ros_ready && ros_stamp != 0 &&
                   (int32_t)(now_ms - ros_stamp) > (int32_t)cfg::kCmdTimeoutMs) {
            ros_linear_x = 0.0f;
            ros_angular_z = 0.0f;
        }

        // stamp 保活：
        //   link_up（默认 true）→ 始终 now，方便串口调平衡；停发 cmd_vel 只归零速度，不 CMD_TIMEOUT
        //   发过 x 后 → 仅靠新鲜 cmd_vel 保活，agent/teleop 断了才 CMD_TIMEOUT
        uint32_t cmd_stamp = 0;
        if (link_up) {
            cmd_stamp = now_ms;
        } else if (ros_fresh) {
            cmd_stamp = ros_stamp;
        }

        if (cmd_stamp != 0) {
            CommandInput cmd{};
            cmd.stamp_ms = cmd_stamp;
            cmd.test_effort[0] = test_effort[0];
            cmd.test_effort[1] = test_effort[1];
            cmd.linear_x = cmd_linear;
            cmd.angular_z = cmd_angular;
            cmd.mode = mode;
            cmd.pitch_ref_rad = pitch_ref_rad;
            cmd.reset_seq = reset_seq;
            cmd.k_yaw = k_yaw;
            cmd.k_yaw_rate = k_yaw_rate;
            cmd.k_vff = k_vff;
            for (int i = 0; i < 5; i++) {
                cmd.gains[i] = gains[i];
            }
            publishCommand(cmd);
        }

        if ((int32_t)(now_ms - next_telemetry_ms) >= 0) {
            next_telemetry_ms = now_ms + telemetry_ms;
            const ControlSnapshot s = live;
            Serial.printf("m=%u hz=%u ovr=%u fault=0x%02X%s%s%s | pitch=%.2f acc=%.2f ref=%.2f deg "
                          "rate=%.2f ax=%.2f ay=%.2f az=%.2f | yaw=%.1f/%.1f deg wz=%.2f uy=%.1f | "
                          "u pit=%.1f rate=%.1f pos=%.1f vel=%.1f int=%.1f | "
                          "vref=%.3f aref=%.3f%s v=%.3f/%.3f x=%.3f/%.3f ticks=%ld/%ld "
                          "eff=%.1f/%.1f%%\n",
                          s.mode, s.ctrl_hz, s.overrun_count, s.fault,
                          (s.fault & Safety::kFall) ? " FALL" : "",
                          (s.fault & Safety::kImuLost) ? " IMU_LOST" : "",
                          (s.fault & Safety::kCmdTimeout) ? " CMD_TIMEOUT" : "",
                          s.pitch_rad * 57.2957795f, s.pitch_acc_rad * 57.2957795f,
                          s.pitch_ref_rad * 57.2957795f,
                          s.pitch_rate_rps, s.acc_g[0], s.acc_g[1], s.acc_g[2],
                          s.yaw_rad * 57.2957795f, s.yaw_ref_rad * 57.2957795f,
                          s.yaw_rate_rps, s.u_yaw,
                          s.terms[BalanceController::kTermPitch],
                          s.terms[BalanceController::kTermPitchRate],
                          s.terms[BalanceController::kTermPos],
                          s.terms[BalanceController::kTermVel],
                          s.terms[BalanceController::kTermInteg],
                          cmd_linear, cmd_angular, ros_fresh ? "(ros)" : "",
                          s.wheel_vel_mps[cfg::kLeft], s.wheel_vel_mps[cfg::kRight],
                          s.wheel_pos_m[cfg::kLeft], s.wheel_pos_m[cfg::kRight],
                          (long)s.wheel_ticks[cfg::kLeft], (long)s.wheel_ticks[cfg::kRight],
                          s.effort[cfg::kLeft], s.effort[cfg::kRight]);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void setup()
{
    Serial.begin(115200);
    delay(200);

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
            .deadband = cfg::kMotorDeadband[i],
            .cmd_eps = cfg::kMotorCmdEps,
        });
        actuators[i].stop();
    }

    safety.init({
        .max_effort = cfg::kMaxEffort,
        .max_slew = cfg::kMaxEffortSlew,
        .fall_angle_rad = cfg::kFallAngleRad,
        .fall_hold_ms = cfg::kFallHoldMs,
        .imu_timeout_ms = cfg::kImuTimeoutMs,
        .cmd_timeout_ms = cfg::kCmdTimeoutMs,
    });
    balance.setLimits(cfg::kMaxEffort, cfg::kIntegTermLimit);

    Serial.println("IMU calibrating, keep still...");
    const bool imu_ready = ahrs.init({
        .sda = cfg::kImuSda,
        .scl = cfg::kImuScl,
        .i2c_hz = cfg::kImuI2cHz,
        .pitch_sign = cfg::kPitchSign,
        .yaw_sign = cfg::kYawSign,
        .gyro_coef = cfg::kPitchGyroCoef,
    });
    Serial.printf("IMU %s (SDA=%d SCL=%d)\n", imu_ready ? "ok" : "FAIL",
                  cfg::kImuSda, cfg::kImuScl);

    // 上电静态角度快照：纯加速度计，不含陀螺积分。保持当前倾斜姿势看哪个平面在动。
    // pitchXZ 绕 Y 轴（当前控制用的量）；rollYZ 绕 X 轴。两者哪个≈实际倾角即为俯仰平面。
    if (imu_ready) {
        const float ax = ahrs.accX(), ay = ahrs.accY(), az = ahrs.accZ();
        const float pitch_xz = -atan2f(ax, sqrtf(az * az + ay * ay)) * 57.2957795f;
        const float roll_yz  =  atan2f(ay, sqrtf(az * az + ax * ax)) * 57.2957795f;
        Serial.printf("IMU static acc: ax=%.3f ay=%.3f az=%.3f g | pitchXZ(gyroY)=%.2f "
                      "rollYZ(gyroX)=%.2f deg\n", ax, ay, az, pitch_xz, roll_yz);
    }

    xTaskCreatePinnedToCore(control_task, "control_task", 4096, NULL, 5, NULL, cfg::kCtrlCore);
    xTaskCreatePinnedToCore(comm_task, "comm_task", 4096, NULL, 2, NULL, cfg::kCommCore);
    xTaskCreatePinnedToCore(microros_task, "microros_task", 16384, NULL, 1, NULL, cfg::kCommCore);

    Serial.println("up: serial m/p/d/w/y/z/n/t/v/a/e/s/r/x/o | wifi /cmd_vel -> v/a");
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}
