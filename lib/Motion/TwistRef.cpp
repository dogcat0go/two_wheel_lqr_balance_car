#include "TwistRef.h"

#include <math.h>

namespace {
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

float approach(float x, float target, float rate, float dt)
{
    const float step = rate * dt;
    const float d = target - x;
    if (d > step) {
        return x + step;
    }
    if (d < -step) {
        return x - step;
    }
    return target;
}
} // namespace

void TwistRef::reset(float pos)
{
    v_ = 0.0f;
    w_ = 0.0f;
    pos_ref_ = pos;
}

TwistRef::Output TwistRef::update(const Sample& s)
{
    if (!s.active) {
        reset(s.pos);
        return {0.0f, 0.0f, pos_ref_, 0.0f};
    }

    v_ = approach(v_, clampf(s.v_cmd, p_.v_max), p_.v_slew, p_.dt);
    w_ = approach(w_, clampf(s.w_cmd, p_.w_max), p_.w_slew, p_.dt);

    if (fabsf(v_) >= p_.v_eps) {
        pos_ref_ += v_ * p_.dt;
        if (s.k_pos > 1e-6f && p_.pos_term_limit > 0.0f) {
            const float max_dev = p_.pos_term_limit / s.k_pos;
            const float dev = pos_ref_ - s.pos;
            if (dev > max_dev) {
                pos_ref_ = s.pos + max_dev;
            } else if (dev < -max_dev) {
                pos_ref_ = s.pos - max_dev;
            }
        }
    }

    return {v_, w_, pos_ref_, clampf(s.k_ff * v_, p_.ff_limit)};
}
