// ============================================================
// 阶段5：LQR 力矩内环（与 stage2 分固件，共用 CommHost）
//
//   Core1  control_task  200Hz
//   Core0  CommHost      串口；只经 shared_state 快照交换（禁止互相直接调对象）
//
// 控制框架（与偏航分层图同一套）：
//
//   [采样] IMU + 编码器
//        → ψ += (v_r-v_l)/L·dt     轮式航向（偏航 P 用）
//        → x = {θ, ω, s, ṡ}        均值状态（LQR 用）
//
//   [LQR 共模]  不进偏航
//        u_sum = K(x - x_ref)
//        x_ref.pitch = trim（kTrimObsEnable 时再加 TrimObserver 偏置）；vel = v_cmd；pos 锁定
//        观测关：ref=t。开：前倾降 ref、后倾升 ref；HOLD 连续 10 窗站住才 ref=pitch
//
//   [偏航差速]  力矩，叠在左右轮上
//        轮速差 P：u_sync = n·(ω_cmd·L + v_l − v_r)   （串口 n = k_sync）
//        有转向 |ω_cmd|：只 u_sync，ψ_ref ← ψ
//        松杆且 z≠0：u_yaw = z·sat(wrap(ψ_ref-ψ)) + j·∫误差 + u_sync（j 航向 I，补摩擦差）
//        z=0：只留 u_sync
//        单轮卡住：停积 ψ，避免假航向
//
//   [HOLD] 车辆级 in-position（HoldPolicy）：与门+确认进，或门当拍出；两轮同时 PWM=0
//        LQR 仍每拍计算（唤醒判据）；航向 I 在 HOLD 内冻结
//
//   [执行]
//        左 τ = u_sum/2 - u_yaw ；右 τ = u_sum/2 + u_yaw
//        HOLD 时两轮 τ_cmd=0，刚进入时 pos_ref←x
//        TRACK：Karnopp 门控死区前馈（粘着且小 τ 不补；滑动 sign(v)，破粘着 sign(τ)）
//        否则 Safety → 电流环 → PWM
//
//   [快照] 本拍填 ControlSnapshot，publishSnapshot 自旋锁拷给 Core0
//
// 串口：k kθ kω ks kv（N·m）；z 航向 P；n 轮速差 k_sync；t trim；v/a；r 探轮后武装；s 停机
// ============================================================

#include <Arduino.h>
#include <string.h>

#include "Ahrs.h"
#include "BalanceController.h"
#include "CommHost.h"
#include "CurrentSensor.h"
#include "DeadbandCalibrator.h"
#include "HoldPolicy.h"
#include "TrimObserver.h"
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
static TrimObserver trim_obs;
static DeadbandCalibrator calibrator;

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

static void control_task(void* param)
{
    (void)param;
    TickType_t next_wake = xTaskGetTickCount();
    const TickType_t period_ticks = pdMS_TO_TICKS(cfg::kCtrlPeriodMs);

    ControlSnapshot snap{}; // 本拍遥测缓存；末尾 publishSnapshot 加锁拷走
    uint32_t cycle_count = 0;
    uint32_t hz_window_ms = millis();
    uint8_t  last_mode = 0;
    uint32_t last_reset_seq = 0;
    uint32_t last_zero_seq = 0;
    uint32_t last_calib_seq = 0;
    bool     armed = false;
    float    pos_ref = 0.0f;   // LQR 位置参考：r / 退出平衡 / 刚进 HOLD 时锁当前 x
    float    yaw = 0.0f;       // 轮式 ψ
    float    yaw_ref = 0.0f;   // 松杆航向锁存
    float    yaw_integ = 0.0f; // 航向积分累加（补左右摩擦常值差）
    bool     yaw_hold = true;
    bool     db_ff = true;     // Karnopp：TRACK 死区前馈使能（回差）
    float    last_tau_half = 0.0f; // 上一拍 |u_sum|/2，交给 TrimObserver 力矩门

    for (;;) {
        vTaskDelayUntil(&next_wake, period_ticks);

        const uint32_t t0_us = micros();

        // ---- 采样：编码器 / 电流 / IMU（now 必须在 ahrs.update 之后取）----
        for (int i = 0; i < 2; i++) {
            sensors[i].update(t0_us);
            current_sensors[i].update();
        }
        ahrs.update(cfg::kCtrlDt);

        const uint32_t now_ms = millis();
        const CommandInput cmd = fetchCommand(); // Core0 指令快照（已加锁拷贝）

        // ---- 死区自标定：串口 b 触发，独占电机逐轮斜坡找起转门槛（见 DeadbandCalibrator）----
        if (cmd.calib_seq != last_calib_seq) {
            last_calib_seq = cmd.calib_seq;
            calibrator.start();
        } else if (cmd.mode == 0) {
            calibrator.abortPrearm();
        }
        if (calibrator.active()) {
            calibrator.update(ahrs.pitch() - cmd.pitch_ref_rad, ahrs.pitchRate());
            snap.stamp_ms = now_ms;
            snap.mode = calibrator.snapMode();
            snap.wheel_ticks[cfg::kLeft] = sensors[cfg::kLeft].ticks();
            snap.wheel_ticks[cfg::kRight] = sensors[cfg::kRight].ticks();
            snap.effort[cfg::kLeft] = actuators[cfg::kLeft].lastDuty();
            snap.effort[cfg::kRight] = actuators[cfg::kRight].lastDuty();
            snap.current_a[cfg::kLeft] = current_sensors[cfg::kLeft].current();
            snap.current_a[cfg::kRight] = current_sensors[cfg::kRight].current();
            if (calibrator.active()) {
                publishSnapshot(snap);
                continue;
            }
        }

        const float v_l = sensors[cfg::kLeft].speed();
        const float v_r = sensors[cfg::kRight].speed();
        const bool one_stuck =
            (fabsf(v_l) < 0.02f && fabsf(v_r) > 0.05f) ||
            (fabsf(v_r) < 0.02f && fabsf(v_l) > 0.05f);
        if (!one_stuck) {
            yaw += (v_r - v_l) / cfg::kWheelDistanceM * cfg::kCtrlDt;
            yaw = wrapPi(yaw);
        }

        BalanceState x{};
        x.pitch = ahrs.pitch();
        x.pitch_rate = ahrs.pitchRate();
        x.pos = 0.5f * (sensors[cfg::kLeft].position() + sensors[cfg::kRight].position());
        x.vel = 0.5f * (v_l + v_r);

        // ---- 锁存：r / 切模式时清积分、锁 s_ref 与 ψ_ref ----
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
            yaw_integ = 0.0f;
            yaw_hold = true;
            if (got_reset) {
                armed = false;
                calibrator.requestArm(x.pitch - cmd.pitch_ref_rad, x.pitch_rate);
            }
        }
        if (calibrator.takePrearmPass()) {
            armed = true;
        }
        safety.setLatch(cmd.mode != 0);

        // ---- LQR 共模：K 来自串口快照；ki=0；u_sum 限幅 2×单轮力矩 ----
        balance.setGains({cmd.lqr_gains[0], cmd.lqr_gains[1], cmd.lqr_gains[2],
                          cmd.lqr_gains[3], 0.0f});
        if (cmd.mode != 0) {
            balance.setLimits(2.0f * cfg::kMaxTorque, 0.0f);
            safety.setEffortLimits(cfg::kMaxTorque, cfg::kLqrMaxSlew);
        } else {
            safety.setEffortLimits(cfg::kMaxEffort, cfg::kMaxEffortSlew);
        }

        snap.fault = safety.evaluate(x.pitch, ahrs.ok(), now_ms,
                                     ahrs.stampMs(), cmd.stamp_ms);

        const bool hard_fault =
            (snap.fault & Safety::kHardFaultMask) != 0;
        if (hard_fault) {
            armed = false;
        }
        const bool cmd_lost =
            (snap.fault & Safety::kCmdTimeout) != 0;
        const float v_cmd = cmd_lost ? 0.0f : cmd.linear_x;
        const float w_cmd = cmd_lost ? 0.0f : cmd.angular_z;

        const bool balancing = (cmd.mode != 0) && armed && !hard_fault;
        if (!balancing) {
            balance.reset();
            hold.reset();
            pos_ref = x.pos;
            yaw_ref = yaw;
            yaw_integ = 0.0f;
            yaw_hold = true;
            last_tau_half = 0.0f;
            db_ff = true;
        }

        const float trim_bias = trim_obs.update({
            .balancing = balancing,
            .holding = hold.holding(),
            .vel = x.vel,
            .pitch = x.pitch,
            .omega = x.pitch_rate,
            .v_cmd = v_cmd,
            .pitch_cmd = cmd.pitch_ref_rad,
            .tau_half = last_tau_half,
        });

        // x_ref：trim；观测开时再加偏置；位置参考保持锁存值
        BalanceState ref{};
        ref.pitch = cmd.pitch_ref_rad + trim_bias;
        ref.vel   = v_cmd;
        ref.pos   = pos_ref;
        balance.setRef(ref);

        float balance_u = 0.0f; // 两轮力矩之和
        if (balancing) {
            balance_u = balance.update(x, cfg::kCtrlDt);
            last_tau_half = (hold.holding() || trim_obs.coast())
                ? 0.0f : 0.5f * fabsf(balance_u);
        }

        hold.update({
            .e_theta = x.pitch - ref.pitch,
            .omega = x.pitch_rate,
            .vel = x.vel,
            .e_pos = x.pos - pos_ref,
            .tau_half = 0.5f * fabsf(balance_u),
            .v_cmd = v_cmd,
            .w_cmd = w_cmd,
            .allow = balancing,
        });
        if (hold.justEntered()) {
            pos_ref = x.pos; // 停在这里；避免 k_s 继续拽回 r 的原点
        }

        // ---- 偏航差速：不进 LQR；P=航向误差，I=航向积分(补摩擦差)，D=轮速差 k_sync（串口 n）----
        const float yaw_rate = ahrs.yawRate();
        float u_yaw = 0.0f;
        float u_i = 0.0f;
        if (balancing) {
            const bool turn_cmd = fabsf(w_cmd) >= cfg::kYawCmdEps;
            float u_z = 0.0f;
            if (turn_cmd) {
                yaw_hold = false;
                yaw_ref = yaw;
                yaw_integ = 0.0f; // 转向时不累积
            } else if (fabsf(cmd.k_yaw) > 1e-6f) {
                if (!yaw_hold) {
                    yaw_ref = yaw;
                    yaw_integ = 0.0f; // 刚进 hold，从零积
                    yaw_hold = true;
                }
                const float e_yaw = clampAbs(wrapPi(yaw_ref - yaw), cfg::kYawErrLimitRad);
                u_z = cmd.k_yaw * e_yaw;
                // 航向积分：补左右摩擦常值差；单轮卡死冻结防假积分；限幅 anti-windup
                if (!one_stuck && cmd.k_yaw_integ > 1e-9f && !hold.holding()) {
                    yaw_integ += e_yaw * cfg::kCtrlDt;
                    yaw_integ = clampAbs(yaw_integ, cfg::kYawIntegTermLimit / cmd.k_yaw_integ);
                }
                u_i = clampAbs(cmd.k_yaw_integ * yaw_integ, cfg::kYawIntegTermLimit);
            } else {
                yaw_hold = false;
                yaw_ref = yaw;
                yaw_integ = 0.0f;
            }
            const float w_ref = turn_cmd ? w_cmd : 0.0f;
            const float u_sync = cmd.k_yaw_rate *
                (w_ref * cfg::kWheelDistanceM + v_l - v_r);
            u_yaw = clampAbs(u_z + u_i + u_sync, cfg::kMaxTorque);
        }

        // ---- 执行：共模/2 ± 差速 → Safety → Karnopp 死区门 → 电流环 ----
        if (cmd.current_zero_seq != last_zero_seq) {
            last_zero_seq = cmd.current_zero_seq;
            for (int i = 0; i < 2; i++) {
                actuators[i].stop();
                actuators[i].resetCurrentLoop();
                current_sensors[i].calibrateZeroFast();
            }
        }

        float tau_cmd[2] = {0.0f, 0.0f};
        float v_abs = 0.0f;
        if (balancing) {
            for (int i = 0; i < 2; i++) {
                float tau_nm = 0.5f * balance_u;
                tau_nm = (i == cfg::kLeft) ? (tau_nm - u_yaw) : (tau_nm + u_yaw);
                tau_cmd[i] = safety.limit(i, tau_nm, cfg::kCtrlDt);
            }
            if (!hold.holding() && !trim_obs.coast()) {
                v_abs = 0.5f * (fabsf(v_l) + fabsf(v_r));
                const float tau_abs = fmaxf(fabsf(tau_cmd[0]), fabsf(tau_cmd[1]));
                if (db_ff) {
                    if (v_abs < cfg::kHoldVelIn && tau_abs < cfg::kTorqueEps) {
                        db_ff = false;
                    }
                } else if (v_abs > cfg::kHoldVelOut ||
                           tau_abs > cfg::kTorqueEps * 1.5f) {
                    db_ff = true;
                }
            }
        }

        for (int i = 0; i < 2; i++) {
            float i_ref = 0.0f;
            const float tau_nm = tau_cmd[i];
            if (balancing) {
                if (hold.holding() || trim_obs.coast()) {
                    i_ref = 0.0f;
                    actuators[i].applyTorque(0.0f, current_sensors[i].current(),
                                             cfg::kCtrlDt, false);
                } else {
                    const float ff_dir = hard_fault ? 0.0f
                        : (v_abs >= cfg::kHoldVelIn
                               ? ((i == cfg::kLeft) ? v_l : v_r)
                               : tau_nm);
                    i_ref = hard_fault ? 0.0f : tau_nm / cfg::kTorquePerAmp[i];
                    actuators[i].applyTorque(hard_fault ? 0.0f : tau_nm,
                                             current_sensors[i].current(),
                                             cfg::kCtrlDt, db_ff && !hard_fault,
                                             ff_dir);
                }
                snap.effort[i] = actuators[i].lastDuty();
            } else if (cmd.mode == 0 && !cmd_lost && cmd.use_current[i]) {
                i_ref = hard_fault ? 0.0f : cmd.test_current[i];
                actuators[i].applyCurrent(i_ref, current_sensors[i].current(),
                                          cfg::kCtrlDt);
                snap.effort[i] = actuators[i].lastDuty();
            } else if (cmd.mode == 0 && !cmd_lost) {
                snap.effort[i] = safety.limit(i, cmd.test_effort[i], cfg::kCtrlDt);
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

        // ---- 遥测快照：填完后 publishSnapshot 临界区拷给 CommHost ----
        snap.stamp_ms = now_ms;
        snap.mode = balancing ? 3 : cmd.mode;
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
        snap.yaw_integ_term = u_i;
        snap.v_dc_mps = trim_obs.vDc();
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
            snap.overrun_count++;
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
            .tau_eps = 0.0f, // 分轮休息关掉；车辆 HOLD 由 HoldPolicy
            .i_max = cfg::kCurrentMaxA,
            .kt = cfg::kTorquePerAmp[i],
            .kp = cfg::kCurrentKp,
            .ki = cfg::kCurrentKi,
        });
        actuators[i].stop();
    }

    // 死区标定器：装配 + 加载 NVS 死区(串口 b 写入)，无记录回退 config.h 默认
    calibrator.init(actuators, sensors, current_sensors, {
        .ramp_pct_per_tick = cfg::kCalibRampPctPerTick,
        .duty_max_pct = cfg::kCalibDutyMaxPct,
        .ticks_thresh = cfg::kCalibTicksThresh,
        .settle_ticks = cfg::kCalibSettleTicks,
        .scale = cfg::kCalibScale,
        .probe_i_a = cfg::kPrearmProbeA,
        .probe_hold_ticks = cfg::kPrearmHoldTicks,
        .probe_settle_ticks = cfg::kPrearmSettleTicks,
        .probe_enable = cfg::kPrearmEnable,
        .probe_abort_pitch_rad = cfg::kPrearmAbortPitchDeg * 0.0174532925f,
        .probe_abort_omega = cfg::kPrearmAbortOmega,
        .dt_s = cfg::kCtrlDt,
    });
    calibrator.loadStored(cfg::kMotorDeadband[cfg::kLeft], cfg::kMotorDeadband[cfg::kRight]);

    for (int i = 0; i < 2; i++) {
        current_sensors[i].init({
            .pin = cfg::kCurrentAdcGpio[i],
            .sensitivity_v_per_a = cfg::kCurrentSensVPerA[i],
            .sign = cfg::kCurrentSign[i],
            .lpf_alpha = cfg::kCurrentLpfAlpha[i],
            .zero_samples = cfg::kCurrentZeroSamples[i],
        });
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
    balance.setLimits(2.0f * cfg::kMaxTorque, 0.0f);
    hold.setLimits(HoldPolicy::robotLimits(true));
    trim_obs.setParams({
        .enable = cfg::kTrimObsEnable,
        .vdc_alpha = cfg::kVdcLpfAlpha,
        .period_ticks = cfg::kTrimObsPeriodTicks,
        .enter_ticks = cfg::kHoldEnterTicks,
        .step0_rad = cfg::kTrimObsStep0Deg * 0.0174532925f,
        .step_min_rad = cfg::kTrimObsStepMinDeg * 0.0174532925f,
        .fall_rad = cfg::kTrimObsFallDeg * 0.0174532925f,
        .hold_snap_n = cfg::kTrimObsHoldSnapN,
        .vel_max = cfg::kHoldVelIn,
        .tau_max = cfg::kTorqueEps,
        .omega_max = cfg::kHoldOmegaIn,
        .alpha_max = cfg::kTrimObsAlphaMax,
        .alpha_lpf = cfg::kTrimObsAlphaLpf,
        .ctrl_hz = (float)cfg::kCtrlHz,
        .v_cmd_eps = cfg::kHoldVCmdEps,
        .limit_rad = cfg::kTrimObsLimitDeg * 0.0174532925f,
    });

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

    if (imu_ready) {
        const float ax = ahrs.accX(), ay = ahrs.accY(), az = ahrs.accZ();
        const float pitch_xz = -atan2f(ax, sqrtf(az * az + ay * ay)) * 57.2957795f;
        const float roll_yz  =  atan2f(ay, sqrtf(az * az + ax * ax)) * 57.2957795f;
        Serial.printf("IMU static acc: ax=%.3f ay=%.3f az=%.3f g | pitchXZ(gyroY)=%.2f "
                      "rollYZ(gyroX)=%.2f deg\n", ax, ay, az, pitch_xz, roll_yz);
    }

    xTaskCreatePinnedToCore(control_task, "control_task", 4096, NULL, 5, NULL, cfg::kCtrlCore);
    commHostStart();

    Serial.println("up: stage5 LQR | arm=0 | hold upright near trim, send r");
    Serial.println("hint: k kth komega ks kv (N.m); z heading P; n wheel-sync k_sync; j heading I");
    Serial.println("hint: s=stop; after s send m 3 then r (arm); b=deadband calib");
    Serial.printf("deadband: L=%.2f%% R=%.2f%% (NVS or default)\n",
                  actuators[cfg::kLeft].deadband(), actuators[cfg::kRight].deadband());
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}
