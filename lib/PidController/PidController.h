/*
 * @Author: LCOIT dogcat.let@gmail.com
 * @Date: 2026-05-14 23:32:26
 * @LastEditors: LCOIT dogcat.let@gmail.com
 * @LastEditTime: 2026-05-18 04:59:27
 * @FilePath: /fishbot_esp32_mt_example/lib/PidController/PidController.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _PID_CONTROLLER_H_
#define _PID_CONTROLLER_H_

class PidController {
public:
    PidController() = default;
    PidController(float kp, float ki, float kd);

    // current: 当前测量值; dt_s: 距上次调用的时间间隔(秒)
    float update(float current, float dt_s);

    void update_target(float target);
    void update_pid(float kp, float ki, float kd);
    void out_limit(float out_min, float out_max);
    void integral_limit(float i_min, float i_max);
    void reset(); // 只清状态，不清参数

private:
    float target_ = 0.0f;
    float kp_ = 0.0f, ki_ = 0.0f, kd_ = 0.0f;

    float prev_error_ = 0.0f;
    float sum_error_  = 0.0f;

    float intergral_min_ = -2500.0f;
    float intergral_max_ =  2500.0f;
    float out_min_ = -100.0f;
    float out_max_ =  100.0f;
};

#endif