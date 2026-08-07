#include "Ahrs.h"

#include <Wire.h>
#include <math.h>

namespace {
constexpr float kDegToRad = 0.0174532925f;
constexpr float kAccMinG  = 0.1f; // 三轴绝对值之和低于此值视为总线无数据
} // namespace

bool Ahrs::init(const Params& params)
{
    params_ = params;

    Wire.begin(params_.sda, params_.scl);
    Wire.setClock(params_.i2c_hz);

    if (imu_.begin() != 0) {
        ok_ = false;
        return false;
    }
    // 只标陀螺零偏。加速度若一并 calcOffsets，会把「标定瞬间的重力分量」吃进
    // acc offset，导致 pitch 零点跟着上电姿势跑，表现为每次都要重拧 trim。
    // 见 docs/imu_boot_calibration.md
    imu_.calcOffsets(true, false);
    imu_.fetchData();

    // 平衡自由度绕 Y 轴：俯仰在 XZ 平面，pitch 用 ax、角速度用 gyroY。
    // 用加速度计直接给初值，省掉互补滤波的收敛过程
    pitch_acc_rad_ = params_.pitch_sign *
                     -atan2f(imu_.getAccX(), sqrtf(imu_.getAccZ() * imu_.getAccZ() +
                                                   imu_.getAccY() * imu_.getAccY()));
    pitch_rad_ = pitch_acc_rad_;
    pitch_rate_rps_ = 0.0f;
    yaw_rate_rps_ = 0.0f;
    acc_g_[0] = imu_.getAccX();
    acc_g_[1] = imu_.getAccY();
    acc_g_[2] = imu_.getAccZ();
    stamp_ms_ = millis();
    ok_ = true;
    return true;
}

void Ahrs::update(float dt_s)
{
    if (!ok_) {
        return;
    }

    imu_.fetchData();

    const float ax = imu_.getAccX();
    const float ay = imu_.getAccY();
    const float az = imu_.getAccZ();
    acc_g_[0] = ax;
    acc_g_[1] = ay;
    acc_g_[2] = az;
    if (fabsf(ax) + fabsf(ay) + fabsf(az) < kAccMinG) {
        ok_ = false; // I2C 掉线/传感器挂死，stamp 停更，安全层会判超时
        return;
    }

    pitch_acc_rad_ = params_.pitch_sign * -atan2f(ax, sqrtf(az * az + ay * ay));
    pitch_rate_rps_ = params_.pitch_sign * imu_.getGyroY() * kDegToRad;
    yaw_rate_rps_ = params_.yaw_sign * imu_.getGyroZ() * kDegToRad;

    // 互补滤波：陀螺积分给高频，加速度计给低频基准
    pitch_rad_ = params_.gyro_coef * (pitch_rad_ + pitch_rate_rps_ * dt_s) +
                 (1.0f - params_.gyro_coef) * pitch_acc_rad_;
    stamp_ms_ = millis();
}
