// ============================================================
// 阶段2~4：频带分离 + 安全层 + 姿态/速度反馈 + 偏航差速
//
// 频带分离：
//   Core1  control_task  200Hz：采样 → 姿态 → 控制器 → 安全层 → 执行
//   Core0  CommHost      串口调参 + 遥测 + WiFi micro-ROS（见 CommHost.h）
//   控制与通信只经 shared_state.h 快照交换。
//
// 速度/角速度：串口 v/a，或 ROS /cmd_vel（有新帧时覆盖串口；超时清零）
// WiFi 参数：include/config.h 的 kWifi* / kAgent*
// 文本协议透传：/fishbot/cmd ← 同串口指令；/fishbot/log → 同串口遥测/应答
//
// 串口指令（USB 与 /fishbot/cmd 共用，解析在 CommHost）：
//   m 0|1|2|3  开环 / 平衡PWM / 平衡电流(手调%) / 平衡电流(LQR N·m)
//   k kθ kω ks kv   m 3 增益 (N·m/状态)，空格分隔；上电默认 kLqr*，不写 flash
//   p/d/y/w     m 1/2 为 %；m 3 时改对应 LQR 一项
//   v <mps>   线速度目标      a <rad/s> 角速度    z/n 航向增益
//   e/el/er   开环 PWM (%)     c/cl/cr 开环 I_ref (A)    cz 停转校零
//   s 停机
//   x / o     模拟断链 / 恢复（仅影响串口侧 stamp 保活）
//   f <ms>    遥测周期
// ============================================================

#include <Arduino.h>
#include <string.h>

#include "Ahrs.h"
#include "BalanceController.h"
#include "CommHost.h"
#include "CurrentSensor.h"
#include "HoldPolicy.h"
#include "Safety.h"
#include "WheelActuator.h"
#include "WheelSensor.h"
#include "config.h"
#include "shared_state.h"

static Esp32McpwmMotor motor_driver;
static WheelActuator actuators[2];
static WheelSensor sensors[2];
static CurrentSensor current_sensors[2];
static Ahrs ahrs;
static Safety safety;
static BalanceController balance;
static HoldPolicy hold;

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
    uint8_t  last_mode = cfg::kBootMode; // 与 CommHost 上电 mode 一致，避免首帧当成切模式
    uint32_t last_reset_seq = 0;
    uint32_t last_zero_seq = 0;
    bool     armed = false; // 上电/摔倒后需 r 才允许平衡出力，防未扶正猛推
    float    pos_ref = 0.0f; // 位置参考：r 时锁当前；仅 |v_cmd|>0 时按 v_cmd 积分
    float    yaw = 0.0f;     // 轮式里程计航向 (rad)
    float    yaw_ref = 0.0f;
    bool     yaw_hold = true;
    float    v_dc = 0.0f;    // 均速慢直流分量（遥测 vdc=），判真稳态爬行

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
            actuators[cfg::kLeft].resetCurrentLoop();
            actuators[cfg::kRight].resetCurrentLoop();
            hold.reset();
            pos_ref = x.pos;
            yaw_ref = yaw;
            yaw_hold = true;
            // 仅 r 上电武装；平衡模式未 armed 时不出力，避免倾角误差直接满力矩
            if (got_reset) {
                armed = true;
            }
        }
        safety.setLatch(cmd.mode != 0); // 开环不锁存；平衡摔倒后需 r

        const bool lqr = (cmd.mode == 3);
        if (lqr) {
            // balance_u = 两轮力矩之和；单轮上限 kMaxTorque → 总和限幅 2×
            // K 来自串口 k/p/d/y/w，上电默认 config.h 的 kLqr*
            balance.setGains({cmd.gains[0], cmd.gains[1], cmd.gains[2],
                              cmd.gains[3], 0.0f});
            balance.setLimits(2.0f * cfg::kMaxTorque, 0.0f);
            safety.setEffortLimits(cfg::kMaxTorque, cfg::kLqrMaxSlew);
        } else {
            balance.setGains({cmd.gains[0], cmd.gains[1], cmd.gains[2],
                              cmd.gains[3], cmd.gains[4]});
            balance.setLimits(cfg::kMaxEffort, cfg::kIntegTermLimit);
            safety.setEffortLimits(cfg::kMaxEffort, cfg::kMaxEffortSlew);
        }

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

        // 参考生成（不是反馈、不是软停）：v_cmd 直接进 ref；站立锁 s_ref。
        const bool balancing =
            (cmd.mode == 1 || cmd.mode == 2 || cmd.mode == 3) && armed && !hard_fault;
        const float v_cmd = cmd_linear_x;
        if (balancing) {
            if (fabsf(v_cmd) >= 1e-3f) {
                pos_ref += v_cmd * cfg::kCtrlDt;
                const float kpos = cmd.gains[2];
                if (kpos > 1e-6f) {
                    const float pos_lim =
                        lqr ? cfg::kLqrPosTermLimit : cfg::kPosTermLimit;
                    const float max_dev = pos_lim / kpos;
                    const float dev = pos_ref - x.pos;
                    if (dev >  max_dev) pos_ref = x.pos + max_dev;
                    if (dev < -max_dev) pos_ref = x.pos - max_dev;
                }
            }
        } else {
            balance.reset();
            hold.reset();
            pos_ref = x.pos;
            yaw_ref = yaw;
            yaw_hold = true;
        }
        if (balancing) {
            v_dc += cfg::kVdcLpfAlpha * (x.vel - v_dc);
        } else {
            v_dc = 0.0f;
        }

        BalanceState ref{};
        ref.pitch = cmd.pitch_ref_rad
                  + clampAbs(cmd.k_vff * v_cmd, cfg::kFfPitchLimitRad);
        ref.vel   = v_cmd;
        ref.pos   = pos_ref;
        balance.setRef(ref);

        float balance_u = 0.0f;
        if (balancing) {
            balance_u = balance.update(x, cfg::kCtrlDt);
        }

        hold.update({
            .e_theta = x.pitch - ref.pitch,
            .omega = x.pitch_rate,
            .vel = x.vel,
            .e_pos = x.pos - pos_ref,
            .tau_half = lqr ? 0.5f * fabsf(balance_u) : 0.0f,
            .v_cmd = v_cmd,
            .w_cmd = cmd_angular_z,
            .allow = balancing,
        });
        if (hold.justEntered()) {
            pos_ref = x.pos;
        }

        // 偏航差速（不进 LQR 状态）：补左右摩擦不齐。z/n 口径是 %；m 3 乘 kPctToTorque 成 N·m。
        // 有转向指令：跟 ω_z。松杆：锁 r 时航向 + 陀螺阻尼（z=0 则只阻尼）。
        float u_yaw = 0.0f;
        const float yaw_rate = ahrs.yawRate();
        if (balancing) {
            const bool turn_cmd = fabsf(cmd_angular_z) >= cfg::kYawCmdEps;
            float u_yaw_pct = 0.0f;
            if (turn_cmd) {
                yaw_hold = false;
                yaw_ref = yaw;
                u_yaw_pct = cmd.k_yaw_rate * (cmd_angular_z - yaw_rate);
            } else if (fabsf(cmd.k_yaw) > 1e-6f) {
                if (!yaw_hold) {
                    yaw_ref = yaw;
                    yaw_hold = true;
                }
                const float e_yaw = wrapPi(yaw_ref - yaw);
                u_yaw_pct = cmd.k_yaw * e_yaw + cmd.k_yaw_rate * (0.0f - yaw_rate);
            } else {
                yaw_hold = false;
                yaw_ref = yaw;
                u_yaw_pct = cmd.k_yaw_rate * (0.0f - yaw_rate);
            }
            if (lqr) {
                u_yaw = clampAbs(u_yaw_pct * cfg::kPctToTorque, cfg::kMaxTorque);
            } else {
                u_yaw = clampAbs(u_yaw_pct, cfg::kMaxEffort);
            }
        }

        if (cmd.current_zero_seq != last_zero_seq) {
            last_zero_seq = cmd.current_zero_seq;
            for (int i = 0; i < 2; i++) {
                actuators[i].stop();
                actuators[i].resetCurrentLoop();
                current_sensors[i].calibrateZeroFast();
            }
        }

        for (int i = 0; i < 2; i++) {
            float desired = 0.0f;
            float i_ref = 0.0f;
            float tau_nm = 0.0f;
            if (cmd.mode == 1 || cmd.mode == 2) {
                desired = (i == cfg::kLeft) ? (balance_u - u_yaw) : (balance_u + u_yaw);
                const float limited = safety.limit(i, desired, cfg::kCtrlDt);
                if (hold.holding()) {
                    if (cmd.mode == 2) {
                        tau_nm = clampAbs(limited * cfg::kPctToTorque, cfg::kMaxTorque);
                    }
                    actuators[i].stop();
                    snap.effort[i] = actuators[i].lastDuty();
                } else if (cmd.mode == 2) {
                    tau_nm = clampAbs(limited * cfg::kPctToTorque, cfg::kMaxTorque);
                    i_ref = hard_fault ? 0.0f : tau_nm / cfg::kTorquePerAmp[i];
                    actuators[i].applyTorque(hard_fault ? 0.0f : tau_nm,
                                             current_sensors[i].current(),
                                             cfg::kCtrlDt);
                    snap.effort[i] = actuators[i].lastDuty();
                } else {
                    actuators[i].applyRawPwm(limited);
                    snap.effort[i] = limited;
                }
            } else if (cmd.mode == 3) {
                // balance_u = 两轮力矩之和；每轮一半。I_ref = τ / Kt（applyTorque 内再除一次 Kt）
                tau_nm = 0.5f * balance_u;
                tau_nm = (i == cfg::kLeft) ? (tau_nm - u_yaw) : (tau_nm + u_yaw);
                tau_nm = safety.limit(i, tau_nm, cfg::kCtrlDt);
                if (hold.holding()) {
                    actuators[i].applyTorque(0.0f, current_sensors[i].current(),
                                             cfg::kCtrlDt);
                    snap.effort[i] = actuators[i].lastDuty();
                } else {
                    i_ref = hard_fault ? 0.0f : tau_nm / cfg::kTorquePerAmp[i];
                    actuators[i].applyTorque(hard_fault ? 0.0f : tau_nm,
                                             current_sensors[i].current(),
                                             cfg::kCtrlDt);
                    snap.effort[i] = actuators[i].lastDuty();
                }
            } else if (!cmd_lost && cmd.use_current[i]) {
                i_ref = hard_fault ? 0.0f : cmd.test_current[i];
                actuators[i].applyCurrent(i_ref, current_sensors[i].current(), cfg::kCtrlDt);
                snap.effort[i] = actuators[i].lastDuty();
            } else if (!cmd_lost) {
                desired = cmd.test_effort[i];
                snap.effort[i] = safety.limit(i, desired, cfg::kCtrlDt);
                actuators[i].applyRawPwm(snap.effort[i]);
                actuators[i].resetCurrentLoop();
            } else {
                actuators[i].stop();
                actuators[i].resetCurrentLoop();
                snap.effort[i] = 0.0f;
            }
            snap.tau_nm[i] = hard_fault ? 0.0f : tau_nm;
            snap.i_ref_a[i] = i_ref;
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
        snap.v_dc_mps = v_dc;
        snap.current_a[cfg::kLeft] = current_sensors[cfg::kLeft].current();
        snap.current_a[cfg::kRight] = current_sensors[cfg::kRight].current();
        snap.current_raw_a[cfg::kLeft] = current_sensors[cfg::kLeft].currentRaw();
        snap.current_raw_a[cfg::kRight] = current_sensors[cfg::kRight].currentRaw();
        snap.imu_ok = ahrs.ok();
        snap.armed = armed;
        snap.hold = hold.holding() ? 1 : 0;
        snap.hold_n = hold.confirmCount();
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
            .tau_eps = 0.0f,
            .i_max = cfg::kCurrentMaxA,
            .kt = cfg::kTorquePerAmp[i],
            .kp = cfg::kCurrentKp,
            .ki = cfg::kCurrentKi,
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
    hold.setLimits(HoldPolicy::robotLimits(false));

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
    commHostStart();

    Serial.println("up: default m=1 arm=0 | hold upright near trim, send r to arm balance");
    Serial.println("hint: m 0; e/el/er=PWM%; c/cl/cr=I_ref A; cz=zero");
    Serial.println("hint: m 3 LQR: k kth komega ks kv  (N·m); p/d/y/w tweak one");
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}
