#include "Safety.h"

#include <math.h>

void Safety::setLatch(bool enabled)
{
    if (!enabled) {
        latched_ = kOk;
    }
    latch_enabled_ = enabled;
}

void Safety::clearFault()
{
    latched_ = kOk;
    fall_since_ms_ = 0;
    last_out_[0] = 0.0f;
    last_out_[1] = 0.0f;
}

void Safety::setEffortLimits(float max_effort, float max_slew)
{
    limits_.max_effort = max_effort;
    limits_.max_slew = max_slew;
}

uint8_t Safety::evaluate(float pitch_rad, bool imu_ok, uint32_t now_ms,
                         uint32_t imu_stamp_ms, uint32_t cmd_stamp_ms)
{
    fault_ = kOk;

    // 用有符号年龄：stamp 略新于 now（跨 ms 边界 / 跨核刷新）时为负，不能当超时。
    // 无符号相减会下溢成 ~4e9，假触发 IMU_LOST / CMD_TIMEOUT。
    const int32_t imu_age_ms = (int32_t)(now_ms - imu_stamp_ms);
    const int32_t cmd_age_ms = (int32_t)(now_ms - cmd_stamp_ms);

    if (!imu_ok || imu_age_ms > (int32_t)limits_.imu_timeout_ms) {
        fault_ |= kImuLost;
    }

    // 连续超角才判摔倒，避免磕碰/加速度毛刺单拍误触发
    if (fabsf(pitch_rad) > limits_.fall_angle_rad) {
        if (fall_since_ms_ == 0) {
            fall_since_ms_ = now_ms;
        } else if ((int32_t)(now_ms - fall_since_ms_) >= (int32_t)limits_.fall_hold_ms) {
            fault_ |= kFall;
        }
    } else {
        fall_since_ms_ = 0;
    }

    // stamp==0：尚未收到过指令（上电竞态），不算断链；有过指令后再按年龄判超时
    if (cmd_stamp_ms != 0 &&
        cmd_age_ms > (int32_t)limits_.cmd_timeout_ms) {
        fault_ |= kCmdTimeout;
    }

    // 只锁存硬故障；CMD_TIMEOUT 为软故障，指令恢复后自动清除
    if (latch_enabled_) {
        latched_ |= static_cast<uint8_t>(fault_ & kHardFaultMask);
        fault_ |= latched_;
    }
    return fault_;
}

float Safety::limit(int wheel, float desired, float dt_s)
{
    // 仅摔倒 / IMU 丢失才切输出；CMD_TIMEOUT 仍放行，由上层把速度目标清零保平衡
    if ((fault_ & kHardFaultMask) != 0) {
        last_out_[wheel] = 0.0f;
        return 0.0f;
    }

    if (desired > limits_.max_effort) {
        desired = limits_.max_effort;
    } else if (desired < -limits_.max_effort) {
        desired = -limits_.max_effort;
    }

    const float step = limits_.max_slew * dt_s;
    const float delta = desired - last_out_[wheel];
    if (delta > step) {
        desired = last_out_[wheel] + step;
    } else if (delta < -step) {
        desired = last_out_[wheel] - step;
    }

    last_out_[wheel] = desired;
    return desired;
}
