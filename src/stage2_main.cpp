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
// 文本协议透传：/fishbot/cmd ← 同串口指令；/fishbot/log → 同串口遥测/应答
//
// 串口指令（USB 与 /fishbot/cmd 共用）：
//   m 0|1     切模式          p/d/i/w/y 增益     t <deg> trim    r 复位
//   v <mps>   线速度目标      a <rad/s> 角速度    z/n 航向增益
//   e/el/er   开环            s 停机
//   x / o     模拟断链 / 恢复（仅影响串口侧 stamp 保活）
//   f <ms>    遥测周期
// ============================================================

#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>
#include <rmw_microros/rmw_microros.h>

#include "Ahrs.h"
#include "BalanceController.h"
#include "CurrentSensor.h"
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
static CurrentSensor current_sensors[2];
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
    uint8_t  last_mode = 1; // 与默认 mode 一致，避免上电首帧被当成「切到平衡」
    uint32_t last_reset_seq = 0;
    bool     armed = false; // 上电/摔倒后需 r 才允许平衡出力，防未扶正猛推
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
            current_sensors[i].update();
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
            const bool got_reset = (cmd.reset_seq != last_reset_seq);
            last_mode = cmd.mode;
            last_reset_seq = cmd.reset_seq;
            balance.reset();
            safety.clearFault();
            pos_ref = x.pos;
            vref_smooth = 0.0f;
            yaw_ref = yaw;
            yaw_hold = true;
            // 仅 r 上电武装；上电默认 m=1 但未 armed，避免倾角误差直接满占空比
            if (got_reset) {
                armed = true;
            }
        }
        safety.setLatch(cmd.mode == 1); // 开环模式保持阶段2的自恢复语义

        balance.setGains({cmd.gains[0], cmd.gains[1], cmd.gains[2],
                          cmd.gains[3], cmd.gains[4]});

        snap.fault = safety.evaluate(x.pitch, ahrs.ok(), now_ms,
                                     ahrs.stampMs(), cmd.stamp_ms);

        const bool hard_fault =
            (snap.fault & Safety::kHardFaultMask) != 0;
        if (hard_fault) {
            armed = false; // 摔倒/IMU 丢失后必须再发 r
        }
        const bool cmd_lost =
            (snap.fault & Safety::kCmdTimeout) != 0;
        // 断链：速度/角速度目标清零，就地平衡；不因 CMD_TIMEOUT 退出平衡环
        const float cmd_linear_x = cmd_lost ? 0.0f : cmd.linear_x;
        const float cmd_angular_z = cmd_lost ? 0.0f : cmd.angular_z;

        // 平衡且已武装且无硬故障
        const bool balancing = (cmd.mode == 1) && armed && !hard_fault;
        if (balancing) {
            // 速度目标斜率限幅：软起步 + 软停。停止时 vref 斜坡衰减而非阶跃到 0，
            // pos_ref 跟着滑行减速的 vref 走，避免松杆瞬间锁死位置造成的回拉振荡。
            const float dv = cfg::kVelSlewMps2 * cfg::kCtrlDt;
            if (vref_smooth < cmd_linear_x) {
                vref_smooth = fminf(vref_smooth + dv, cmd_linear_x);
            } else if (vref_smooth > cmd_linear_x) {
                vref_smooth = fmaxf(vref_smooth - dv, cmd_linear_x);
            }

            if (fabsf(vref_smooth) < 1e-3f && fabsf(cmd_linear_x) < 1e-3f) {
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
        if (balancing) {
            balance_u = balance.update(x, cfg::kCtrlDt);
        }

        // 偏航差速：az≈0 heading hold；az≠0 只跟角速度，松杆锁新航向
        float u_yaw = 0.0f;
        const float yaw_rate = ahrs.yawRate();
        if (balancing) {
            float omega_ref = 0.0f;
            if (fabsf(cmd_angular_z) < cfg::kYawCmdEps) {
                if (!yaw_hold) {
                    yaw_ref = yaw;
                    yaw_hold = true;
                }
            } else {
                yaw_hold = false;
                yaw_ref = yaw;
                omega_ref = cmd_angular_z;
            }
            const float e_yaw = wrapPi(yaw_ref - yaw);
            u_yaw = cmd.k_yaw * e_yaw + cmd.k_yaw_rate * (omega_ref - yaw_rate);
            u_yaw = clampAbs(u_yaw, cfg::kMaxEffort);
        }

        for (int i = 0; i < 2; i++) {
            float desired = 0.0f;
            if (cmd.mode == 1) {
                // 正 u_yaw → 右轮更快 → 逆时针（与 Kinematics 一致）
                desired = (i == cfg::kLeft) ? (balance_u - u_yaw) : (balance_u + u_yaw);
            } else if (!cmd_lost) {
                // 开环：断链时清零，避免最后一次 e 指令继续驱动
                desired = cmd.test_effort[i];
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
        snap.current_a[cfg::kLeft] = current_sensors[cfg::kLeft].current();
        snap.current_a[cfg::kRight] = current_sensors[cfg::kRight].current();
        snap.current_raw_a[cfg::kLeft] = current_sensors[cfg::kLeft].currentRaw();
        snap.current_raw_a[cfg::kRight] = current_sensors[cfg::kRight].currentRaw();
        snap.imu_ok = ahrs.ok();
        snap.armed = armed;
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
// /fishbot/cmd 与 USB 串口共用 applyCommandLine；遥测/应答经 emitLog 镜像到 /fishbot/log。
static bool link_up = true; // 串口侧保活：true 时无 ROS 帧也刷新 stamp（方便只调平衡）
static float test_effort[2] = {0.0f, 0.0f};
static uint8_t mode = 1; // 上电默认平衡；扶正后再上电，倒地会故障锁存，发 r 清
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

// 文本日志 → /fishbot/log（comm 写、microros 发；覆盖未发出的旧行，保最新）
static constexpr size_t kLogCap = 512;
static char wifi_log_buf[kLogCap];
static volatile bool wifi_log_pending = false;

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

static void emitLog(const char* line)
{
    Serial.print(line);
    strncpy(wifi_log_buf, line, kLogCap - 1);
    wifi_log_buf[kLogCap - 1] = '\0';
    wifi_log_pending = true;
}

static void applyCommandLine(const char* raw)
{
    String line(raw);
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
    default: {
        char buf[96];
        snprintf(buf, sizeof(buf), "unknown cmd: %s\n", line.c_str());
        emitLog(buf);
        return;
    }
    }
    char buf[320];
    snprintf(buf, sizeof(buf),
             "cmd ok: %s -> m=%u kp=%.1f kd=%.3f ki=%.3f kvel=%.1f kpos=%.1f "
             "kyaw=%.1f kn=%.1f vff=%.1fdeg/mps trim=%.2fdeg vref=%.3f aref=%.3f "
             "eL=%.1f eR=%.1f link=%d ros=%d\n",
             line.c_str(), mode, gains[0], gains[1], gains[4], gains[3], gains[2],
             k_yaw, k_yaw_rate, k_vff * 57.2957795f,
             pitch_ref_rad * 57.2957795f, linear_x, angular_z,
             test_effort[cfg::kLeft], test_effort[cfg::kRight],
             link_up, (int)ros_ready);
    emitLog(buf);
}

static void handleSerial()
{
    if (!Serial.available()) {
        return;
    }
    String line = Serial.readStringUntil('\n');
    applyCommandLine(line.c_str());
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
        char buf[64];
        snprintf(buf, sizeof(buf), "cmd_vel: lx=%.3f az=%.3f\n",
                 (float)ros_linear_x, (float)ros_angular_z);
        emitLog(buf);
    }
}

static void serial_cmd_callback(const void* msgin)
{
    const std_msgs__msg__String* msg =
        static_cast<const std_msgs__msg__String*>(msgin);
    if (msg->data.data == nullptr || msg->data.size == 0) {
        return;
    }
    // micro-ROS String 未必带尾 '\0'，拷到本地再解析
    char line[96];
    const size_t n = msg->data.size < sizeof(line) - 1 ? msg->data.size : sizeof(line) - 1;
    memcpy(line, msg->data.data, n);
    line[n] = '\0';
    applyCommandLine(line);
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
    rcl_subscription_t serial_cmd_sub;
    rcl_publisher_t serial_log_pub;
    geometry_msgs__msg__Twist cmd_vel_msg;
    std_msgs__msg__String serial_cmd_msg;
    std_msgs__msg__String serial_log_msg;

    serial_cmd_msg.data.data = static_cast<char*>(malloc(96));
    serial_cmd_msg.data.size = 0;
    serial_cmd_msg.data.capacity = 96;
    serial_log_msg.data.data = static_cast<char*>(malloc(kLogCap));
    serial_log_msg.data.size = 0;
    serial_log_msg.data.capacity = kLogCap;
    if (serial_cmd_msg.data.data == nullptr || serial_log_msg.data.data == nullptr) {
        Serial.println("microros: string alloc failed, serial-only mode");
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK ||
        rclc_node_init_default(&node, cfg::kRosNodeName, "", &support) != RCL_RET_OK ||
        rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK ||
        rclc_subscription_init_best_effort(
            &cmd_vel_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            cfg::kCmdVelTopic) != RCL_RET_OK ||
        rclc_subscription_init_best_effort(
            &serial_cmd_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            cfg::kSerialCmdTopic) != RCL_RET_OK ||
        rclc_publisher_init_best_effort(
            &serial_log_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            cfg::kSerialLogTopic) != RCL_RET_OK ||
        rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg,
                                       &twist_callback, ON_NEW_DATA) != RCL_RET_OK ||
        rclc_executor_add_subscription(&executor, &serial_cmd_sub, &serial_cmd_msg,
                                       &serial_cmd_callback, ON_NEW_DATA) != RCL_RET_OK) {
        Serial.println("microros: init failed, serial-only mode");
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ros_ready = true;
    Serial.printf("microros: %s + %s/%s ready\n",
                  cfg::kCmdVelTopic, cfg::kSerialCmdTopic, cfg::kSerialLogTopic);

    for (;;) {
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50)));
        if (wifi_log_pending) {
            const size_t n = strnlen(wifi_log_buf, kLogCap - 1);
            memcpy(serial_log_msg.data.data, wifi_log_buf, n);
            serial_log_msg.data.data[n] = '\0';
            serial_log_msg.data.size = n;
            wifi_log_pending = false;
            RCSOFTCHECK(rcl_publish(&serial_log_pub, &serial_log_msg, NULL));
        }
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
            // 开环：硬故障或断链都清掉挂起的 e 指令（控制环也会在 cmd_lost 时输出 0）
            if ((live.fault & (Safety::kHardFaultMask | Safety::kCmdTimeout)) != 0) {
                test_effort[0] = test_effort[1] = 0.0f;
                if (last_fault == Safety::kOk) {
                    char buf[96];
                    snprintf(buf, sizeof(buf),
                             "fault 0x%02X: cleared pending effort, re-send e/el/er to move\n",
                             live.fault);
                    emitLog(buf);
                }
            }
        }
        const uint8_t hard_now = live.fault & Safety::kHardFaultMask;
        const uint8_t hard_was = last_fault & Safety::kHardFaultMask;
        if (hard_now != 0 && hard_was == 0 && mode == 1) {
            char buf[80];
            snprintf(buf, sizeof(buf),
                     "fault 0x%02X: motors off, hold upright near trim then send r\n",
                     live.fault);
            emitLog(buf);
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
            char line[kLogCap];
            if (s.mode == 0) {
                // mode 0：开环台架，突出电流与占空比，方便 ACS712 / e 联调
                snprintf(line, sizeof(line),
                         "m=0 hz=%u ovr=%u fault=0x%02X%s%s%s | "
                         "iL=%.3fA iR=%.3fA rawL=%.3f rawR=%.3f "
                         "eff=%.1f/%.1f%% v=%.3f/%.3f ticks=%ld/%ld\n",
                         s.ctrl_hz, s.overrun_count, s.fault,
                         (s.fault & Safety::kFall) ? " FALL" : "",
                         (s.fault & Safety::kImuLost) ? " IMU_LOST" : "",
                         (s.fault & Safety::kCmdTimeout) ? " CMD_TIMEOUT" : "",
                         s.current_a[cfg::kLeft], s.current_a[cfg::kRight],
                         s.current_raw_a[cfg::kLeft], s.current_raw_a[cfg::kRight],
                         s.effort[cfg::kLeft], s.effort[cfg::kRight],
                         s.wheel_vel_mps[cfg::kLeft], s.wheel_vel_mps[cfg::kRight],
                         (long)s.wheel_ticks[cfg::kLeft], (long)s.wheel_ticks[cfg::kRight]);
            } else {
                snprintf(line, sizeof(line),
                         "m=%u arm=%u hz=%u ovr=%u fault=0x%02X%s%s%s | pitch=%.2f acc=%.2f ref=%.2f deg "
                         "rate=%.2f ax=%.2f ay=%.2f az=%.2f | yaw=%.1f/%.1f deg wz=%.2f uy=%.1f | "
                         "u pit=%.1f rate=%.1f pos=%.1f vel=%.1f int=%.1f | "
                         "vref=%.3f aref=%.3f%s v=%.3f/%.3f x=%.3f/%.3f ticks=%ld/%ld "
                         "eff=%.1f/%.1f%% iL=%.3fA iR=%.3fA\n",
                         s.mode, (unsigned)s.armed, s.ctrl_hz, s.overrun_count, s.fault,
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
                         s.effort[cfg::kLeft], s.effort[cfg::kRight],
                         s.current_a[cfg::kLeft], s.current_a[cfg::kRight]);
            }
            emitLog(line);
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

    for (int i = 0; i < 2; i++) {
        current_sensors[i].init({
            .pin = cfg::kCurrentAdcGpio[i],
            .sensitivity_v_per_a = cfg::kCurrentSensVPerA[i],
            .sign = cfg::kCurrentSign[i],
            .lpf_alpha = cfg::kCurrentLpfAlpha[i],
            .zero_samples = cfg::kCurrentZeroSamples[i],
        });
        // 电机已 stop：采零点电压；接线后勿在有负载电流时上电校零
        current_sensors[i].calibrateZero();
        Serial.printf("CurrentSensor[%d] GPIO%d zero=%.3fV sens=%.0fmV/A sign=%.0f\n",
                      i, cfg::kCurrentAdcGpio[i], current_sensors[i].zeroVoltage(),
                      cfg::kCurrentSensVPerA[i] * 1000.0f, cfg::kCurrentSign[i]);
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

    Serial.println("up: default m=1 arm=0 | hold upright near trim, send r to arm balance");
    Serial.println("hint: m 0 then e/el/er to read ACS712 iL/iR");
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}
