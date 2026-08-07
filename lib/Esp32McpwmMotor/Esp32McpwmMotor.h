#ifndef __ESP32MCPWMMOTOR_H__
#define __ESP32MCPWMMOTOR_H__

#include "Arduino.h"
#include <stdio.h>

#include "esp_system.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

class Esp32McpwmMotor
{
private:
    int16_t speeds[4]{0, 0};
    bool mMotorAttached[4]{false, false, false, false};

public:
    Esp32McpwmMotor() = default;
    ~Esp32McpwmMotor() = default;
    void attachMotor(uint8_t id, uint8_t gpioIn1, uint8_t gpioIn2);
    void stopMotor(int8_t motorId = -1);
    // pwmValue: 车体/接口约定的驱动力度 -100~100（占空比%）
    void updateMotorSpeed(int8_t id, int16_t pwmValue);
};

#endif // __ESP32MCPWMMOTOR_H__
