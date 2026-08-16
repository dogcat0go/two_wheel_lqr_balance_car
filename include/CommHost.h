#pragma once

// Core0 通信宿主：USB 串口 + micro-ROS（/cmd_vel、/fishbot/cmd、/fishbot/log）+ 遥测。
// 与控制环只经 shared_state 交换；阶段 2 PID 与后续 LQR 共用本模块。
// 在 setup() 硬件初始化之后调用 commHostStart()。

void commHostStart();
