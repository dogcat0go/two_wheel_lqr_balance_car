#pragma once

#include <stdint.h>

// 安全层：控制器输出与执行器之间的最后一道闸。
// 阶段2 必须先于任何控制算法测通——调 PID/LQR 时炸机的代价全靠它兜。
// 用法（每个控制周期，顺序固定）：
//   evaluate(...)  先判全局故障
//   limit(wheel,...) 再逐轮限幅
// 故障态下输出立即置 0（不走斜率），恢复正常后从 0 开始按斜率爬。
class Safety {
public:
    struct Limits {
        float    max_effort;     // 输出饱和，单位同执行器（当前为占空比 %）
        float    max_slew;       // 斜率限幅，单位/秒
        float    fall_angle_rad; // pitch 超过此值判摔倒
        uint32_t fall_hold_ms;   // 连续超角多久才确认摔倒（抗毛刺）
        uint32_t imu_timeout_ms; // 姿态数据过期
        uint32_t cmd_timeout_ms; // 通信断链
    };

    enum Fault : uint8_t {
        kOk         = 0,
        kFall       = 1 << 0,
        kImuLost    = 1 << 1, // 传感器无效或数据过期
        kCmdTimeout = 1 << 2,
    };

    void init(const Limits& limits) { limits_ = limits; }

    // 故障锁存：平衡模式必须开——车摔了不能因为被扶正就自动恢复输出，
    // 要人为 clearFault() 才解锁。关闭时同时清掉已锁存的位。
    void setLatch(bool enabled);
    void clearFault();

    // 返回故障位掩码；imu_stamp_ms / cmd_stamp_ms 为对应数据的最后更新时刻
    uint8_t evaluate(float pitch_rad, bool imu_ok, uint32_t now_ms,
                     uint32_t imu_stamp_ms, uint32_t cmd_stamp_ms);

    // 限幅后的输出。故障时直接返回 0 并清掉斜率状态
    float limit(int wheel, float desired, float dt_s);

    uint8_t fault() const { return fault_; }

private:
    Limits   limits_{};
    uint8_t  fault_ = kOk;
    uint8_t  latched_ = kOk;
    bool     latch_enabled_ = false;
    float    last_out_[2] = {0.0f, 0.0f};
    uint32_t fall_since_ms_ = 0; // 0 = 当前未超角；否则为首次超角时刻
};
