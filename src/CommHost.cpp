#include "CommHost.h"

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

#include "BalanceController.h"
#include "Safety.h"
#include "config.h"
#include "shared_state.h"

#define RCSOFTCHECK(fn)                                                      \
    {                                                                        \
        rcl_ret_t rc = (fn);                                                 \
        if (rc != RCL_RET_OK) {                                              \
            Serial.printf("RCL soft error %d at %d: %s\n", (int)rc, __LINE__, #fn); \
        }                                                                    \
    }

static bool link_up = true; // 串口侧保活：true 时无 ROS 帧也刷新 stamp（方便只调平衡）
static float test_effort[2] = {0.0f, 0.0f};
static float test_current[2] = {0.0f, 0.0f};
static uint8_t use_current[2] = {0, 0};
static uint32_t current_zero_seq = 0;
#ifdef STAGE5_FIRMWARE
static uint8_t mode = 3; // stage5 只跑 LQR；s 仍切 0 停机
#else
static uint8_t mode = cfg::kBootMode; // 倒地锁存后发 r 清；未 armed 仍不出力
#endif
static uint32_t reset_seq = 0;
static uint32_t calib_seq = 0;
static float pitch_ref_rad = cfg::kPitchTrimDeg * 0.0174532925f;
static float linear_x = 0.0f;     // 串口 v 写入
static float angular_z = 0.0f;    // 串口 a 写入
static uint32_t telemetry_ms = cfg::kTelemetryMs;
static float gains[5] = {cfg::kGainPitch, cfg::kGainPitchRate, cfg::kGainPos,
                         cfg::kGainVel, cfg::kGainIntegPitch};
static float lqr_gains[4] = {cfg::kLqrPitch, cfg::kLqrPitchRate, cfg::kLqrPos,
                             cfg::kLqrVel};
#ifdef STAGE5_FIRMWARE
static float k_yaw = cfg::kGainYawNm;
static float k_yaw_rate = cfg::kGainWheelSync;
#else
static float k_yaw = cfg::kGainYaw;
static float k_yaw_rate = cfg::kGainYawRate;
#endif
static float k_yaw_integ = cfg::kGainYawIntegNm; // stage5 航向积分 ki（串口 j）
static float k_vff = cfg::kGainVelToPitch; // 速度→倾角前馈, rad/(m/s)；串口 g 输入 deg/(m/s)

static volatile float    ros_linear_x = 0.0f;
static volatile float    ros_angular_z = 0.0f;
static volatile uint32_t ros_cmd_stamp_ms = 0;
static volatile bool     ros_ready = false;

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
    case 'm': {
        int m = line.substring(1).toInt();
        if (m < 0) {
            m = 0;
        } else if (m > 3) {
            m = 3;
        }
        mode = (uint8_t)m;
        break;
    }
    case 'k': {
        float a = 0, b = 0, c = 0, d = 0;
        const char* rest = line.c_str() + 1;
        if (sscanf(rest, "%f %f %f %f", &a, &b, &c, &d) != 4 &&
            sscanf(rest, "%f,%f,%f,%f", &a, &b, &c, &d) != 4) {
            emitLog("k needs 4 numbers: k kth komega ks kv\n");
            return;
        }
        lqr_gains[0] = a;
        lqr_gains[1] = b;
        lqr_gains[2] = c;
        lqr_gains[3] = d;
        break;
    }
    case 'p':
#ifdef STAGE5_FIRMWARE
        lqr_gains[0] = value;
#else
        if (mode == 3) {
            lqr_gains[0] = value;
        } else {
            gains[0] = value;
        }
#endif
        break;
    case 'd':
#ifdef STAGE5_FIRMWARE
        lqr_gains[1] = value;
#else
        if (mode == 3) {
            lqr_gains[1] = value;
        } else {
            gains[1] = value;
        }
#endif
        break;
    case 'i':
        if (mode != 3) {
            gains[4] = value;
        }
        break;
    case 'w':
#ifdef STAGE5_FIRMWARE
        lqr_gains[3] = value;
#else
        if (mode == 3) {
            lqr_gains[3] = value;
        } else {
            gains[3] = value;
        }
#endif
        break;
    case 'y':
#ifdef STAGE5_FIRMWARE
        lqr_gains[2] = value;
#else
        if (mode == 3) {
            lqr_gains[2] = value;
        } else {
            gains[2] = value;
        }
#endif
        break;
    case 'z': k_yaw = value; break;
    case 'n': k_yaw_rate = value; break;
    case 'j': k_yaw_integ = value; break; // stage5 航向积分 ki
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
    case 'b': // 死区自标定：架空(轮子悬空)后触发，逐轮正反斜坡找起转门槛
        mode = 0;
        calib_seq++;
        break;
    case 'e':
        if (line.length() > 1 && line[1] == 'l') {
            test_effort[cfg::kLeft] = line.substring(2).toFloat();
            use_current[cfg::kLeft] = 0;
        } else if (line.length() > 1 && line[1] == 'r') {
            test_effort[cfg::kRight] = line.substring(2).toFloat();
            use_current[cfg::kRight] = 0;
        } else {
            test_effort[cfg::kLeft] = test_effort[cfg::kRight] = value;
            use_current[cfg::kLeft] = use_current[cfg::kRight] = 0;
        }
        break;
    case 'c':
        if (line.length() > 1 && line[1] == 'z') {
            test_effort[0] = test_effort[1] = 0.0f;
            test_current[0] = test_current[1] = 0.0f;
            use_current[0] = use_current[1] = 0;
            current_zero_seq++;
        } else if (line.length() > 1 && line[1] == 'l') {
            test_current[cfg::kLeft] = line.substring(2).toFloat();
            use_current[cfg::kLeft] = 1;
            test_effort[cfg::kLeft] = 0.0f;
        } else if (line.length() > 1 && line[1] == 'r') {
            test_current[cfg::kRight] = line.substring(2).toFloat();
            use_current[cfg::kRight] = 1;
            test_effort[cfg::kRight] = 0.0f;
        } else {
            test_current[cfg::kLeft] = test_current[cfg::kRight] = value;
            use_current[cfg::kLeft] = use_current[cfg::kRight] = 1;
            test_effort[cfg::kLeft] = test_effort[cfg::kRight] = 0.0f;
        }
        break;
    case 's':
        test_effort[0] = test_effort[1] = 0.0f;
        test_current[0] = test_current[1] = 0.0f;
        use_current[0] = use_current[1] = 0;
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
#ifdef STAGE5_FIRMWARE
    snprintf(buf, sizeof(buf),
             "cmd ok: %s -> m=%u lqr kth=%.6g kom=%.6g ks=%.6g kv=%.6g "
             "kz=%.4g kn=%.4g kj=%.4g trim=%.2fdeg vref=%.3f aref=%.3f link=%d\n",
             line.c_str(), mode, lqr_gains[0], lqr_gains[1], lqr_gains[2], lqr_gains[3],
             k_yaw, k_yaw_rate, k_yaw_integ, pitch_ref_rad * 57.2957795f, linear_x, angular_z, link_up);
#else
    if (mode == 3) {
        snprintf(buf, sizeof(buf),
                 "cmd ok: %s -> m=3 lqr kth=%.6g kom=%.6g ks=%.6g kv=%.6g "
                 "trim=%.2fdeg vref=%.3f aref=%.3f link=%d\n",
                 line.c_str(), lqr_gains[0], lqr_gains[1], lqr_gains[2], lqr_gains[3],
                 pitch_ref_rad * 57.2957795f, linear_x, angular_z, link_up);
    } else {
        snprintf(buf, sizeof(buf),
                 "cmd ok: %s -> m=%u kp=%.1f kd=%.3f ki=%.3f kvel=%.1f kpos=%.1f "
                 "kyaw=%.1f kn=%.1f vff=%.1fdeg/mps trim=%.2fdeg vref=%.3f aref=%.3f "
                 "eL=%.1f eR=%.1f cL=%.2fA cR=%.2fA cur=%u/%u link=%d ros=%d\n",
                 line.c_str(), mode, gains[0], gains[1], gains[4], gains[3], gains[2],
                 k_yaw, k_yaw_rate, k_vff * 57.2957795f,
                 pitch_ref_rad * 57.2957795f, linear_x, angular_z,
                 test_effort[cfg::kLeft], test_effort[cfg::kRight],
                 test_current[cfg::kLeft], test_current[cfg::kRight],
                 use_current[cfg::kLeft], use_current[cfg::kRight],
                 link_up, (int)ros_ready);
    }
#endif
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
            (test_effort[0] != 0.0f || test_effort[1] != 0.0f ||
             test_current[0] != 0.0f || test_current[1] != 0.0f)) {
            if ((live.fault & (Safety::kHardFaultMask | Safety::kCmdTimeout)) != 0) {
                test_effort[0] = test_effort[1] = 0.0f;
                test_current[0] = test_current[1] = 0.0f;
                use_current[0] = use_current[1] = 0;
                if (last_fault == Safety::kOk) {
                    char buf[96];
                    snprintf(buf, sizeof(buf),
                             "fault 0x%02X: cleared pending effort, re-send e or c to move\n",
                             live.fault);
                    emitLog(buf);
                }
            }
        }
        const uint8_t hard_now = live.fault & Safety::kHardFaultMask;
        const uint8_t hard_was = last_fault & Safety::kHardFaultMask;
        if (hard_now != 0 && hard_was == 0 && mode != 0) {
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
            cmd.test_current[0] = test_current[0];
            cmd.test_current[1] = test_current[1];
            cmd.use_current[0] = use_current[0];
            cmd.use_current[1] = use_current[1];
            cmd.current_zero_seq = current_zero_seq;
            cmd.linear_x = cmd_linear;
            cmd.angular_z = cmd_angular;
            cmd.mode = mode;
            cmd.pitch_ref_rad = pitch_ref_rad;
            cmd.reset_seq = reset_seq;
            cmd.calib_seq = calib_seq;
            cmd.k_yaw = k_yaw;
            cmd.k_yaw_rate = k_yaw_rate;
            cmd.k_yaw_integ = k_yaw_integ;
            cmd.k_vff = k_vff;
            cmd.lqr_gains[0] = lqr_gains[0];
            cmd.lqr_gains[1] = lqr_gains[1];
            cmd.lqr_gains[2] = lqr_gains[2];
            cmd.lqr_gains[3] = lqr_gains[3];
            if (mode == 3) {
                cmd.gains[0] = lqr_gains[0];
                cmd.gains[1] = lqr_gains[1];
                cmd.gains[2] = lqr_gains[2];
                cmd.gains[3] = lqr_gains[3];
                cmd.gains[4] = 0.0f;
            } else {
                for (int i = 0; i < 5; i++) {
                    cmd.gains[i] = gains[i];
                }
            }
            publishCommand(cmd);
        }

        if ((int32_t)(now_ms - next_telemetry_ms) >= 0) {
            next_telemetry_ms = now_ms + telemetry_ms;
            const ControlSnapshot s = live;
            char line[kLogCap];
            if (s.mode == 0) {
                snprintf(line, sizeof(line),
                         "m=0 hz=%u ovr=%u fault=0x%02X%s%s%s | "
                         "iL=%.3fA iR=%.3fA iref=%.3f/%.3f "
                         "eff=%.1f/%.1f%% v=%.3f/%.3f ticks=%ld/%ld\n",
                         s.ctrl_hz, s.overrun_count, s.fault,
                         (s.fault & Safety::kFall) ? " FALL" : "",
                         (s.fault & Safety::kImuLost) ? " IMU_LOST" : "",
                         (s.fault & Safety::kCmdTimeout) ? " CMD_TIMEOUT" : "",
                         s.current_a[cfg::kLeft], s.current_a[cfg::kRight],
                         s.i_ref_a[cfg::kLeft], s.i_ref_a[cfg::kRight],
                         s.effort[cfg::kLeft], s.effort[cfg::kRight],
                         s.wheel_vel_mps[cfg::kLeft], s.wheel_vel_mps[cfg::kRight],
                         (long)s.wheel_ticks[cfg::kLeft], (long)s.wheel_ticks[cfg::kRight]);
            } else {
                snprintf(line, sizeof(line),
                         "m=%u arm=%u hold=%u hN=%u hz=%u ovr=%u fault=0x%02X%s%s%s | pitch=%.2f acc=%.2f ref=%.2f deg "
                         "rate=%.2f ax=%.2f ay=%.2f az=%.2f | yaw=%.1f/%.1f deg wz=%.2f uy=%.1f ui=%.3f | "
                         "u pit=%.3f rate=%.3f pos=%.3f vel=%.3f int=%.3f | "
                         "tau=%.3f/%.3fNm vref=%.3f aref=%.3f%s v=%.3f/%.3f vdc=%.3f x=%.3f/%.3f ticks=%ld/%ld "
                         "eff=%.1f/%.1f%% iref=%.3f/%.3f iL=%.3fA iR=%.3fA\n",
                         s.mode, (unsigned)s.armed, (unsigned)s.hold, (unsigned)s.hold_n, s.ctrl_hz, s.overrun_count, s.fault,
                         (s.fault & Safety::kFall) ? " FALL" : "",
                         (s.fault & Safety::kImuLost) ? " IMU_LOST" : "",
                         (s.fault & Safety::kCmdTimeout) ? " CMD_TIMEOUT" : "",
                         s.pitch_rad * 57.2957795f, s.pitch_acc_rad * 57.2957795f,
                         s.pitch_ref_rad * 57.2957795f,
                         s.pitch_rate_rps, s.acc_g[0], s.acc_g[1], s.acc_g[2],
                         s.yaw_rad * 57.2957795f, s.yaw_ref_rad * 57.2957795f,
                         s.yaw_rate_rps, s.u_yaw, s.yaw_integ_term,
                         s.terms[BalanceController::kTermPitch],
                         s.terms[BalanceController::kTermPitchRate],
                         s.terms[BalanceController::kTermPos],
                         s.terms[BalanceController::kTermVel],
                         s.terms[BalanceController::kTermInteg],
                         s.tau_nm[cfg::kLeft], s.tau_nm[cfg::kRight],
                         cmd_linear, cmd_angular, ros_fresh ? "(ros)" : "",
                         s.wheel_vel_mps[cfg::kLeft], s.wheel_vel_mps[cfg::kRight],
                         s.v_dc_mps,
                         s.wheel_pos_m[cfg::kLeft], s.wheel_pos_m[cfg::kRight],
                         (long)s.wheel_ticks[cfg::kLeft], (long)s.wheel_ticks[cfg::kRight],
                         s.effort[cfg::kLeft], s.effort[cfg::kRight],
                         s.i_ref_a[cfg::kLeft], s.i_ref_a[cfg::kRight],
                         s.current_a[cfg::kLeft], s.current_a[cfg::kRight]);
            }
            emitLog(line);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void commHostStart()
{
    xTaskCreatePinnedToCore(comm_task, "comm_task", 4096, NULL, 2, NULL, cfg::kCommCore);
    if (cfg::kEnableWifi) {
        xTaskCreatePinnedToCore(microros_task, "microros_task", 16384, NULL, 1, NULL, cfg::kCommCore);
    } else {
        WiFi.disconnect(true, true);
        WiFi.mode(WIFI_OFF);
        Serial.println("wifi: off (serial only)");
    }
}
