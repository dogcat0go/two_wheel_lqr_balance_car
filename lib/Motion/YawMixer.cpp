#include "YawMixer.h"

#include <math.h>

namespace {
float wrapPi(float a)
{
    return atan2f(sinf(a), cosf(a));
}

float clampf(float v, float lim)
{
    if (v > lim) {
        return lim;
    }
    if (v < -lim) {
        return -lim;
    }
    return v;
}
} // namespace

void YawMixer::reset(float yaw)
{
    yaw_ref_ = yaw;
    yaw_integ_ = 0.0f;
    yaw_hold_ = true;
}

YawMixer::Output YawMixer::update(const Sample& s)
{
    if (!s.active) {
        reset(s.yaw);
        return {0.0f, 0.0f, yaw_ref_, false};
    }

    const bool turning = fabsf(s.w_cmd) >= p_.w_eps;
    float u_z = 0.0f;
    float u_i = 0.0f;
    if (turning) {
        yaw_hold_ = false;
        yaw_ref_ = s.yaw;
        yaw_integ_ = 0.0f;
    } else if (fabsf(s.k_yaw) > 1e-6f) {
        if (!yaw_hold_) {
            yaw_ref_ = s.yaw;
            yaw_integ_ = 0.0f;
            yaw_hold_ = true;
        }
        const float e_yaw = clampf(wrapPi(yaw_ref_ - s.yaw), p_.yaw_err_lim);
        u_z = s.k_yaw * e_yaw;
        if (!s.one_stuck && s.k_integ > 1e-9f && !s.holding) {
            yaw_integ_ += e_yaw * p_.dt;
            yaw_integ_ = clampf(yaw_integ_, p_.integ_lim / s.k_integ);
        }
        u_i = clampf(s.k_integ * yaw_integ_, p_.integ_lim);
    } else {
        yaw_hold_ = false;
        yaw_ref_ = s.yaw;
        yaw_integ_ = 0.0f;
    }

    const float w_ref = turning ? s.w_ref : 0.0f;
    // one_stuck：单轮卡住/倒下时轮速差是伪信号，同步项会劫持共模救车力矩
    const float u_sync = s.one_stuck ? 0.0f
        : s.k_sync * (w_ref * p_.track + s.v_l - s.v_r);
    return {clampf(u_z + u_i + u_sync, p_.tau_max), u_i, yaw_ref_, turning};
}
