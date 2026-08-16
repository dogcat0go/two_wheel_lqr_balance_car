/*
 * @Author: LCOIT dogcat.let@gmail.com
 * @Date: 2026-08-05 14:29:42
 * @LastEditors: LCOIT dogcat.let@gmail.com
 * @LastEditTime: 2026-08-15 18:04:28
 * @FilePath: /fishbot_esp32_mt_example/lib/BalanceControl/BalanceController.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "BalanceController.h"

#include <math.h>

namespace {
inline float clampAbs(float v, float lim)
{
    if (v > lim) return lim;
    if (v < -lim) return -lim;
    return v;
}
} // namespace

void BalanceController::setLimits(float out_abs, float integ_abs)
{
    out_abs_ = out_abs;
    integ_abs_ = integ_abs;
}

float BalanceController::update(const BalanceState& x, float dt_s)
{
    const float e_pitch = x.pitch - ref_.pitch;

    // 如果偏差为5°时，弧度=0.0872665，k_pitch=500时，KTerm = 43.63325
    terms_[kTermPitch]     = gains_.k_pitch * e_pitch;
    terms_[kTermPitchRate] = gains_.k_pitch_rate * (x.pitch_rate - ref_.pitch_rate);
    terms_[kTermPos]       = gains_.k_pos * (x.pos - ref_.pos);
    terms_[kTermVel]       = gains_.k_vel * (x.vel - ref_.vel);

    float u = terms_[kTermPitch] + terms_[kTermPitchRate] +
              terms_[kTermPos] + terms_[kTermVel];

    if (gains_.ki_pitch != 0.0f && dt_s > 0.0f) {
        // 条件积分：已顶到饱和且误差还在推同方向时冻结，防 windup
        const float u_probe = u + gains_.ki_pitch * integ_;
        const bool sat_hi = (u_probe >= out_abs_) && (e_pitch > 0.0f);
        const bool sat_lo = (u_probe <= -out_abs_) && (e_pitch < 0.0f);
        if (!sat_hi && !sat_lo) {
            integ_ += e_pitch * dt_s;
        }
    } else {
        integ_ = 0.0f;
    }

    terms_[kTermInteg] = clampAbs(gains_.ki_pitch * integ_, integ_abs_);
    u += terms_[kTermInteg];

    return clampAbs(u, out_abs_);
}

void BalanceController::reset()
{
    integ_ = 0.0f;
    for (int i = 0; i < kTermCount; i++) {
        terms_[i] = 0.0f;
    }
}
