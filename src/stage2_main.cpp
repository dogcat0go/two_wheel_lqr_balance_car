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
    uint32_t last_zero_seq = 0;
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
            actuators[cfg::kLeft].resetCurrentLoop();
            actuators[cfg::kRight].resetCurrentLoop();
            pos_ref = x.pos;
            vref_smooth = 0.0f;
            yaw_ref = yaw;
            yaw_hold = true;
            // 仅 r 上电武装；上电默认 m=1 但未 armed，避免倾角误差直接满占空比
            if (got_reset) {
                armed = true;
            }
        }
        safety.setLatch(cmd.mode != 0); // 开环不锁存；平衡摔倒后需 r

        const bool lqr = (cmd.mode == 3);
        if (lqr) {
            balance.setGains({cfg::kLqrPitch, cfg::kLqrPitchRate, cfg::kLqrPos,
                              cfg::kLqrVel, 0.0f});
            balance.setLimits(cfg::kMaxTorque, cfg::kIntegTermLimit);
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

        // 平衡且已武装且无硬故障
        const bool balancing =
            (cmd.mode == 1 || cmd.mode == 2 || cmd.mode == 3) && armed && !hard_fault;
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
                const float kpos = lqr ? cfg::kLqrPos : cmd.gains[2];
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

        // 偏航差速：无转向指令时默认 u_yaw=0。
        // m 1 且 z≠0 才做轮速航向锁；m 2/3 默认 u_yaw=0（m 3 的 n 仍是 %，不能当 N·m）。
        float u_yaw = 0.0f;
        const float yaw_rate = ahrs.yawRate();
        if (balancing) {
            const bool turn_cmd = fabsf(cmd_angular_z) >= cfg::kYawCmdEps;
            if (turn_cmd && cmd.mode != 3) {
                yaw_hold = false;
                yaw_ref = yaw;
                u_yaw = cmd.k_yaw_rate * (cmd_angular_z - yaw_rate);
            } else if (cmd.mode == 1 && fabsf(cmd.k_yaw) > 1e-6f) {
                if (!yaw_hold) {
                    yaw_ref = yaw;
                    yaw_hold = true;
                }
                const float e_yaw = wrapPi(yaw_ref - yaw);
                u_yaw = cmd.k_yaw * e_yaw + cmd.k_yaw_rate * (0.0f - yaw_rate);
            } else {
                yaw_hold = false;
                yaw_ref = yaw;
            }
            u_yaw = clampAbs(u_yaw, cfg::kMaxEffort);
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
            if (cmd.mode == 1 || cmd.mode == 2) {
                desired = (i == cfg::kLeft) ? (balance_u - u_yaw) : (balance_u + u_yaw);
                const float limited = safety.limit(i, desired, cfg::kCtrlDt);
                if (cmd.mode == 2) {
                    float tau = limited * cfg::kPctToTorque;
                    tau = clampAbs(tau, cfg::kMaxTorque);
                    i_ref = hard_fault ? 0.0f : tau / cfg::kTorquePerAmp[i];
                    actuators[i].applyTorque(hard_fault ? 0.0f : tau,
                                             current_sensors[i].current(),
                                             cfg::kCtrlDt);
                    snap.effort[i] = actuators[i].lastDuty();
                } else {
                    actuators[i].applyRawPwm(limited);
                    snap.effort[i] = limited;
                }
            } else if (cmd.mode == 3) {
                // balance_u = 两轮力矩之和；每轮一半。u_yaw 本模式为 0。
                float tau = 0.5f * balance_u;
                tau = (i == cfg::kLeft) ? (tau - u_yaw) : (tau + u_yaw);
                tau = safety.limit(i, tau, cfg::kCtrlDt);
                i_ref = hard_fault ? 0.0f : tau / cfg::kTorquePerAmp[i];
                actuators[i].applyTorque(hard_fault ? 0.0f : tau,
                                         current_sensors[i].current(),
                                         cfg::kCtrlDt);
                snap.effort[i] = actuators[i].lastDuty();
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
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}
