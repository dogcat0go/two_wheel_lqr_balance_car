#pragma once

// Core0 通信宿主：USB 串口 + micro-ROS（/cmd_vel、/fishbot/cmd、/fishbot/log）+ 遥测。
// 与控制环只经 shared_state 交换；stage2 与 stage5 共用本模块。
// STAGE5_FIRMWARE：默认 m=3，p/d/y/w 只改 lqr_gains；k 始终写 lqr_gains。
// 在 setup() 硬件初始化之后调用 commHostStart()。

void commHostStart();
