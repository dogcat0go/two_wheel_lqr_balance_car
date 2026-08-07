#include "Esp32McpwmMotor.h"

/*
MOTORID PCNTID(ID/3+ID)      IOA((ID%3)*2)     IOB(ID*2+1)    TIMER
0       MCPWM_UNIT_0          0                1              0
1       MCPWM_UNIT_0          2                3
2       MCPWM_UNIT_0          4                5
3       MCPWM_UNIT_1          0                1
4       MCPWM_UNIT_1          2                3
5       MCPWM_UNIT_1          4                5
*/

// 1 = AT8236 慢衰减（低速启动力矩通常更好）；0 = 快衰减（原厂逻辑）
#ifndef MCPWM_MOTOR_SLOW_DECAY
#define MCPWM_MOTOR_SLOW_DECAY 1
#endif

void Esp32McpwmMotor::attachMotor(uint8_t id, uint8_t gpioIn1, uint8_t gpioIn2)
{
    mcpwm_unit_t mcpwm_num = MCPWM_UNIT_0;
    mcpwm_io_signals_t io_signal_a = mcpwm_io_signals_t((id % 3) * 2);
    mcpwm_io_signals_t io_signal_b = mcpwm_io_signals_t((id % 3) * 2 + 1);
    if (id / 3 == 1)
    {
        mcpwm_num = MCPWM_UNIT_1;
    }
    mcpwm_timer_t mcpwm_timer = MCPWM_TIMER_0;
    if (id % 3 == 1)
    {
        mcpwm_timer = MCPWM_TIMER_1;
    }
    else if (id % 3 == 2)
    {
        mcpwm_timer = MCPWM_TIMER_2;
    }

    mcpwm_gpio_init(mcpwm_num, io_signal_a, gpioIn1);
    mcpwm_gpio_init(mcpwm_num, io_signal_b, gpioIn2);
    this->mMotorAttached[id] = true;

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 20000; // AT8236 常用 10kHz
    pwm_config.cmpr_a = 0;
    pwm_config.cmpr_b = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(mcpwm_num, mcpwm_timer, &pwm_config);
}

void Esp32McpwmMotor::stopMotor(int8_t motorId)
{
    if (motorId < 0)
    {
        updateMotorSpeed(0, 0);
        updateMotorSpeed(1, 0);
        return;
    }
    updateMotorSpeed(motorId, 0);
}

void Esp32McpwmMotor::updateMotorSpeed(int8_t id, int16_t pwmValue)
{
    mcpwm_unit_t mcpwm_num = MCPWM_UNIT_0;
    if (id / 3 == 1)
    {
        mcpwm_num = MCPWM_UNIT_1;
    }
    mcpwm_timer_t mcpwm_timer = MCPWM_TIMER_0;
    if (id % 3 == 1)
    {
        mcpwm_timer = MCPWM_TIMER_1;
    }
    else if (id % 3 == 2)
    {
        mcpwm_timer = MCPWM_TIMER_2;
    }

    if (pwmValue > 100) pwmValue = 100;
    if (pwmValue < -100) pwmValue = -100;

    if (pwmValue == 0)
    {
        // IN1=IN2=0 → AT8236 滑行/休眠
        mcpwm_set_signal_low(mcpwm_num, mcpwm_timer, MCPWM_OPR_A);
        mcpwm_set_signal_low(mcpwm_num, mcpwm_timer, MCPWM_OPR_B);
        return;
    }

#if MCPWM_MOTOR_SLOW_DECAY
    // 慢衰减（AT8236）:
    //   正转: IN1=1, IN2=PWM → PWM 高=刹车, 低=驱动 ⇒ 驱动占比 = 100 - duty
    //   反转: IN2=1, IN1=PWM → 同上
    // 对外仍保持 pwmValue = 期望驱动力度(0~100)
    const int16_t drive = (pwmValue > 0) ? pwmValue : -pwmValue;
    const int16_t brake_duty = 100 - drive;

    if (pwmValue > 0)
    {
        mcpwm_set_signal_high(mcpwm_num, mcpwm_timer, MCPWM_OPR_A);
        mcpwm_set_duty(mcpwm_num, mcpwm_timer, MCPWM_OPR_B, brake_duty);
        mcpwm_set_duty_type(mcpwm_num, mcpwm_timer, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    }
    else
    {
        mcpwm_set_signal_high(mcpwm_num, mcpwm_timer, MCPWM_OPR_B);
        mcpwm_set_duty(mcpwm_num, mcpwm_timer, MCPWM_OPR_A, brake_duty);
        mcpwm_set_duty_type(mcpwm_num, mcpwm_timer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    }
#else
    // 快衰减（原逻辑）: 正转 IN1=PWM,IN2=0；反转 IN1=0,IN2=PWM
    if (pwmValue > 0)
    {
        mcpwm_set_signal_low(mcpwm_num, mcpwm_timer, MCPWM_OPR_B);
        mcpwm_set_duty(mcpwm_num, mcpwm_timer, MCPWM_OPR_A, pwmValue);
        mcpwm_set_duty_type(mcpwm_num, mcpwm_timer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    }
    else
    {
        mcpwm_set_signal_low(mcpwm_num, mcpwm_timer, MCPWM_OPR_A);
        mcpwm_set_duty(mcpwm_num, mcpwm_timer, MCPWM_OPR_B, -pwmValue);
        mcpwm_set_duty_type(mcpwm_num, mcpwm_timer, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    }
#endif
}
