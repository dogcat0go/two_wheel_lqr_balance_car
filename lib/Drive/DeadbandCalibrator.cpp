#include "DeadbandCalibrator.h"

#include <Arduino.h>
#include <Preferences.h>
#include <math.h>

namespace {
constexpr char kNvsNamespace[] = "motorcal";
}

void DeadbandCalibrator::init(WheelActuator* actuators, WheelSensor* sensors,
                              CurrentSensor* currents, const Params& params)
{
    act_ = actuators;
    sen_ = sensors;
    cur_ = currents;
    p_ = params;
}

void DeadbandCalibrator::loadStored(float default_l, float default_r)
{
    // 分方向键 dbLf/dbLr/dbRf/dbRr；无记录回退旧单值键 dbL/dbR，再回退 config 默认
    Preferences prefs;
    prefs.begin(kNvsNamespace, true);
    const float dbL = prefs.getFloat("dbL", default_l);
    const float dbR = prefs.getFloat("dbR", default_r);
    float db[4] = {prefs.getFloat("dbLf", dbL), prefs.getFloat("dbLr", dbL),
                   prefs.getFloat("dbRf", dbR), prefs.getFloat("dbRr", dbR)};
    prefs.end();
    const float sane_max = 0.6f * p_.duty_max_pct;
    static const char* kName[4] = {"Lf", "Lr", "Rf", "Rr"};
    for (int i = 0; i < 4; i++) {
        const float fb = (i < 2) ? default_l : default_r;
        if (!(db[i] >= 0.0f && db[i] < sane_max)) {
            Serial.printf("deadband: stored %s=%.2f%% out of range, fallback %.2f%%\n",
                          kName[i], db[i], fb);
            db[i] = fb;
        }
    }
    act_[0].setDeadbands(db[0], db[1]);
    act_[1].setDeadbands(db[2], db[3]);
    Serial.printf("deadband: L=%.2f/%.2f%% R=%.2f/%.2f%% fwd/rev (NVS or default)\n",
                  db[0], db[1], db[2], db[3]);
}

bool DeadbandCalibrator::ticksMoved(int wheel, float dir) const
{
    const int32_t d = sen_[wheel].ticks() - start_ticks_[wheel];
    return (dir > 0.0f ? d : -d) >= p_.ticks_thresh;
}

void DeadbandCalibrator::stopBoth()
{
    act_[0].stop();
    act_[1].stop();
}

void DeadbandCalibrator::start()
{
    kind_ = kCalib;
    phase_ = 0;
    duty_ = 0.0f;
    settle_ = p_.settle_ticks;
    capped_any_ = false;
    prearm_pass_ = false;
    start_ticks_[0] = sen_[0].ticks();
    start_ticks_[1] = sen_[1].ticks();
    stopBoth();
    Serial.println("calib: start deadband ramp; LIFT robot, wheels free-spinning");
}

void DeadbandCalibrator::requestArm(float e_pitch, float omega)
{
    (void)omega;
    if (!p_.probe_enable) {
        kind_ = kIdle;
        prearm_pass_ = true;
        Serial.println("prearm: disabled, arming");
        return;
    }
    if (fabsf(e_pitch) > p_.probe_abort_pitch_rad) {
        Serial.println("prearm: skipped, pitch too far from trim");
        return;
    }
    startPrearm();
}

void DeadbandCalibrator::startPrearm()
{
    kind_ = kPrearm;
    phase_ = 0;
    hold_ = 0;
    settle_ = p_.probe_settle_ticks;
    prearm_pass_ = false;
    start_ticks_[0] = sen_[0].ticks();
    start_ticks_[1] = sen_[1].ticks();
    act_[0].resetCurrentLoop();
    act_[1].resetCurrentLoop();
    stopBoth();
    Serial.printf("prearm: probe %.2fA both wheels; hold upright, allow a few mm roll\n",
                  p_.probe_i_a);
}

void DeadbandCalibrator::abortPrearm()
{
    if (kind_ != kPrearm) {
        return;
    }
    stopBoth();
    act_[0].resetCurrentLoop();
    act_[1].resetCurrentLoop();
    kind_ = kIdle;
    prearm_pass_ = false;
    Serial.println("prearm: aborted");
}

bool DeadbandCalibrator::takePrearmPass()
{
    if (!prearm_pass_) {
        return false;
    }
    prearm_pass_ = false;
    return true;
}

void DeadbandCalibrator::finishPrearm(bool ok)
{
    stopBoth();
    act_[0].resetCurrentLoop();
    act_[1].resetCurrentLoop();
    kind_ = kIdle;
    if (ok) {
        prearm_pass_ = true;
        Serial.println("prearm: ok, arming");
    } else {
        prearm_pass_ = false;
        Serial.printf("prearm: FAIL L dTicks=%ld R dTicks=%ld, not armed\n",
                      (long)(sen_[0].ticks() - start_ticks_[0]),
                      (long)(sen_[1].ticks() - start_ticks_[1]));
    }
}

void DeadbandCalibrator::update(float e_pitch, float omega)
{
    if (kind_ == kIdle) {
        return;
    }

    if (kind_ == kPrearm) {
        if (fabsf(e_pitch) > p_.probe_abort_pitch_rad ||
            fabsf(omega) > p_.probe_abort_omega) {
            abortPrearm();
            return;
        }
        if (settle_ > 0) {
            settle_--;
            start_ticks_[0] = sen_[0].ticks();
            start_ticks_[1] = sen_[1].ticks();
            hold_ = 0;
            stopBoth();
            return;
        }
        hold_++;
        act_[0].applyCurrent(p_.probe_i_a, cur_[0].current(), p_.dt_s);
        act_[1].applyCurrent(p_.probe_i_a, cur_[1].current(), p_.dt_s);
        if (ticksMoved(0, 1.0f) && ticksMoved(1, 1.0f)) {
            Serial.printf("prearm: ok dTicks L=%ld R=%ld\n",
                          (long)(sen_[0].ticks() - start_ticks_[0]),
                          (long)(sen_[1].ticks() - start_ticks_[1]));
            finishPrearm(true);
            return;
        }
        if (hold_ >= p_.probe_hold_ticks) {
            finishPrearm(false);
        }
        return;
    }

    const int   w   = phase_ / 2;
    const float dir = (phase_ % 2 == 0) ? 1.0f : -1.0f;

    if (settle_ > 0) {
        settle_--;
        start_ticks_[w] = sen_[w].ticks();
        stopBoth();
        return;
    }

    duty_ += p_.ramp_pct_per_tick;
    const bool moved = ticksMoved(w, dir);
    const bool capped = duty_ >= p_.duty_max_pct;
    if (!moved && !capped) {
        act_[w].writePwmRaw(dir * duty_);
        act_[1 - w].stop();
        return;
    }

    meas_[w][phase_ % 2] = duty_;
    if (capped) {
        capped_any_ = true;
    }
    Serial.printf("calib: wheel%d dir%+d start=%.1f%%%s\n",
                  w, (int)dir, duty_, capped ? " (CAPPED!)" : "");
    stopBoth();
    phase_++;
    duty_ = 0.0f;
    settle_ = p_.settle_ticks;
    if (phase_ < 4) {
        return;
    }

    if (capped_any_) {
        Serial.println("calib: FAILED (a wheel never moved / CAPPED); NVS & deadband kept");
        kind_ = kIdle;
        return;
    }

    // 分方向存：meas_[w][0]=正向门槛、[1]=反向门槛（实测正反可差 2 倍，平均必欠补粘的方向）
    const float dbLf = p_.scale * meas_[0][0];
    const float dbLr = p_.scale * meas_[0][1];
    const float dbRf = p_.scale * meas_[1][0];
    const float dbRr = p_.scale * meas_[1][1];
    act_[0].setDeadbands(dbLf, dbLr);
    act_[1].setDeadbands(dbRf, dbRr);
    Preferences prefs;
    prefs.begin(kNvsNamespace, false);
    prefs.putFloat("dbLf", dbLf);
    prefs.putFloat("dbLr", dbLr);
    prefs.putFloat("dbRf", dbRf);
    prefs.putFloat("dbRr", dbRr);
    // 旧单值键同步写均值，回滚旧固件时仍可用
    prefs.putFloat("dbL", 0.5f * (dbLf + dbLr));
    prefs.putFloat("dbR", 0.5f * (dbRf + dbRr));
    prefs.end();
    Serial.printf("calib: done dbL=%.2f/%.2f%% dbR=%.2f/%.2f%% fwd/rev saved to NVS\n",
                  dbLf, dbLr, dbRf, dbRr);
    kind_ = kIdle;
}
