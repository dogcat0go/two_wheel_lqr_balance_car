---
title: Sim2Real 迁移方案与验收清单
level: 1
order: 6
sections: true
---

# Sim2Real 迁移方案与验收清单

> TWIP 平衡车 · 仿真 → 实车（ESP32 + WiFi）移植
> 对应实现：`balance_controller_node.py` / `control_backend.py` / `cascade_pid_backend.py` / `lqr_backend.py` / `safety_limiter.py` / `attitude_estimator.py` / `slope_dyn_obs_node.py` / `yaw_loop.py`
> 实车基础工程：`/home/lcoit/ros2_projects/fishbot_res/fishbot_esp32_mt_example`（ESP32 + PlatformIO + micro-ROS/WiFi，已有电机/编码器/差速运动学/轮速 PID，缺姿态与平衡）
> 配套文档：[动力学清单](./dynamics_kinematics_checklist.md) · [斜坡前馈](./lqr_slope_feedforward.md) · [KF](./slope_kf_simple.md) · [α̂ 死区低通](./alpha_lpf_deadband.md)

本文档回答两个问题：**移植方案怎么分阶段做**、**为何 PID 应该比 LQR 先上实车**。仿真里已验证稳定的一整套逻辑（停车不倒退、坡度估计、偏航保持）不能整体搬到实车上直接跑，必须按"先隔离硬件问题、再排模型问题"的顺序拆解。

---

## 〇、先答问题：PID 优先调试是否更快验证电机/电源参数

**结论：是，且不是"退而求其次"，而是专门用来隔离硬件问题的工具。**

LQR 的反馈增益 `K`（`lqr_gain_design.py`）是从线性化物理模型（`M, m, l, I, MGR, dt`）反算出来的。这些参数任意一项有偏差——电机死区、实际减速比、电池带载压降导致的有效力矩打折——LQR 给出的就是"错模型下的最优解"，出问题时无法区分是**模型错**还是**硬件没调好**。

PID（`cascade_pid_backend.py`）几乎不依赖物理参数,增益是经验拧出来的,对模型误差和执行器非线性的容忍度天然更高。上实车第一步用 PID,只要能站起来,就说明:

- IMU 安装方向、姿态融合符号正确
- 编码器方向、计数与轮转向一致
- 力矩/PWM 映射、死区补偿基本对
- 安全限幅逻辑没写反
- 200Hz 控制周期没有严重抖动/丢帧

这些全部验证完,再切 LQR 时只需要关心"模型参数标得准不准",排查范围大幅收窄。

---

## 一、先定架构：控制环跑在哪，WiFi 起什么作用 {#architecture}

这是决定整个移植方案的第一个岔路口，必须先定，否则后续阶段会被推翻重做。

沿用已经验证过的"**频带分离**"思路（快环管平衡、慢环管坡补偿/航向，参见 [频带分离](./lqr_slope_feedforward.md#freq-separation)）——这里把"频带"换成"控制环 vs 通信":

- [ ] **平衡内环（IMU → 姿态估计 → PID/LQR → 输出力矩，200Hz）必须闭环在 ESP32 本地**，不能经 WiFi 走一圈再回来。WiFi/TCP-UDP 天然有几十到上百 ms 的延迟方差，5ms 周期的平衡环经不起这种抖动。
- [ ] **WiFi 只承担非实时职责**：下发 `/cmd_vel`、上传遥测（复用现有 `rosbag`/`plotjuggler` 分析习惯）、在线调参（`kp/ki/kd`、`slope_ff_gain`、`c` 等）。

两条技术路线二选一：

| 方案 | 做法 | 优点 | 风险 |
| --- | --- | --- | --- |
| micro-ROS on ESP32 | ESP32 本地跑 micro-ROS,200Hz 定时器内部完成闭环;`/imu/data`、`/joint_states`、`/wheel_effort_controller/commands` 话题结构与仿真一致 | 消息/命名习惯零迁移成本,文档思维模型不用换 | ESP32 上内存/实时性开销需验证 200Hz 是否稳 |
| 自定义轻量协议 | 纯 C/C++（Arduino/ESP-IDF）写定时器任务闭环,WiFi 只发 UDP/WebSocket 遥测帧 | 实时性、资源可控性最好,更贴近真实嵌入式项目 | 需自写协议和上位机转 bag/画图脚本 |

面试角度：能讲清楚"为什么控制环必须本地闭环、通信只做遥测"本身就是系统设计加分项，倾向推荐自定义轻量协议方案。

---

## 二、分阶段移植路线图 {#phases}

每阶段设过关标准，不满足不进入下一阶段。

### 阶段 0：裸机/裸电验证（不跑控制算法）

- [ ] 车轮离地，给固定 PWM，确认电机转向与"正力矩=前进"符号约定一致（对应仿真里的 `sign` 参数，实车大概率还要再翻一次）
- [ ] 编码器读数方向与轮子实际转向一致，累计计数无跳变/丢步（对应 `wheel_position` 的连续性）
- [ ] 测空载转速、堵转电流、电池带载压降——决定实车 `max_wheel_effort` 上限（大概率低于仿真的 0.5 Nm）

**过关标准：** PWM/转速方向/编码器计数三者对得上，带载不掉压保护。

### 阶段 1：姿态估计单独验证（不接控制）

- [ ] 仿真 `/imu/data` 是 Gazebo 直接给的"完美"姿态四元数；实车只有加速度计+陀螺仪原始数据，**必须先跑姿态融合算法**（Mahony/Madgwick 互补滤波，或现成 DMP）得到 `pitch`——这是仿真里被"作弊"掉的环节
- [ ] 手持车体倾斜/晃动，对照肉眼角度检查融合 `pitch` 是否合理、有无明显漂移或滞后
- [ ] 静止零偏标定

**过关标准：** 静止 `pitch` 稳定在 0 附近，缓慢倾斜跟手感一致，无明显延迟。

### 阶段 2：安全层先行（比控制算法更早上电）

- [ ] 把 `safety_limiter.py` 的逻辑（`max_wheel_effort` 饱和、`max_effort_slew` 限幅、`fall_angle_rad` 摔倒清零、`imu_timeout_sec` 去抖）原样搬到固件，且要在控制算法写完**之前**先测通
- [ ] 补一条仿真不需要的：**通信超时保护**——WiFi/`cmd_vel` 断了必须本地自动停车清零，不能依赖 PC 端超时判断（链路可能先断在 ESP32 侦测不到 PC 的方向）

**过关标准：** 故意断 WiFi、模拟 pitch 超限、力矩指令突变，都能在几十 ms 内安全清零输出。

**落地位置（`pio run -e stage2`）：** 阈值全在 `include/config.h`；限幅逻辑 `lib/Safety`；
姿态 `lib/Ahrs`（固定 dt 互补滤波）；频带分离骨架 `src/stage2_main.cpp`
（Core1 200Hz 控制环 / Core0 通信环，只经 `include/shared_state.h` 的两个快照交换数据）。
串口 `x` 模拟断链，`o` 恢复，`e <duty>` 给开环输出验证饱和与斜率。

**验收步骤与记录表：** [`docs/stage2_safety_bench.md`](stage2_safety_bench.md)（与当前代码阈值同步；含摔倒去抖 `kFallHoldMs`）。

### 阶段 3：PID 平衡环上车（先站起来）

- [ ] 只搬 `cascade_pid_backend.py` 最内层姿态环，目标速度固定为 0，不接 `cmd_vel`
- [ ] 手扶车体在平衡角附近松手，测试能否立住；再过渡到独立站立

**过关标准：** 平地独立站立 >10s，轻推能恢复。

**落地方式：** 扩展 `src/stage2_main.cpp`（串口 `m 1` 切平衡模式），不新建 stage3 文件——
安全层/快照/遥测/落盘全部复用。控制器新增 `lib/BalanceControl`，写成**状态反馈**
\(u=\sum k_i(x_i-x_{\mathrm{ref},i})+k_{i\theta}\!\int\)，\(x=[\theta,\dot\theta,s,\dot s]\)：
阶段 3 只填 \(k_\theta,k_{\dot\theta}\)，阶段 4 补 \(k_s,k_{\dot s}\)，阶段 5 直接换成 \(-K_{\mathrm{LQR}}\)，
控制环结构三阶段不动。\(\dot\theta\) 用 `Ahrs::pitchRate()` 实测，增益经串口在线拧。

**`lib/PidController` 不再使用**：D 项是内部差分、误差符号与平衡环相反，且它已不被任何
参与编译的 env 引用（仅归档 `test/back/main.cpp` 提及）；等阶段 4 确认速度环也走状态反馈后删除。

**调参步骤与记录表：** [`docs/stage3_balance_tuning.md`](stage3_balance_tuning.md)
（含符号验证 Step 0、`kMaxEffortSlew` 必须放大等前置改动）。
**增益初值与范围（阶段 3/4 合一）：** [`docs/stage34_gain_init_theory.md`](stage34_gain_init_theory.md)
**姿态环物理推导详解：** [`docs/balance_gain_theory.md`](balance_gain_theory.md)

### 阶段 4：速度环 + 位置环

- [ ] 接入 `motion_command.py` 的 `cmd_vel`，验证前进/停止是否复现仿真里"倒退/来回振荡"的问题；已有的排查思路（`x_ref` 锁存、`lqr_ref_accel_limit`）可直接复用，但**增益需重新拧**，不能直接搬仿真数值

**落地方式（已完成骨架）：** 不新建文件/任务/级联，在 `src/stage2_main.cpp` 现有 200Hz 单环里
把 `kGainVel`(\(k_{\dot s}\)) / `kGainPos`(\(k_s\)) 从 0 打开，`ref.vel` 由 `cmd.linear_x` 给，
`pos_ref` 每拍按速度指令积分（`x_ref` 锁存，避免匀速时位置项与速度项打架）。
串口新增 `w`/`y`/`v`。先只调 \(k_{\dot s}\) 消阶段3 单向漂移，再给速度目标。

**调参步骤与记录：** [`docs/stage4_velocity_loop.md`](stage4_velocity_loop.md)
（含速度反馈符号验证 Step 0、`kMPerTick` 必须先重标）。
**\(k_{\dot s}/k_s\) 初值与时标分离推导：** [`docs/stage34_gain_init_theory.md`](stage34_gain_init_theory.md#stage4)

**过关标准：** 阶跃 `cmd_vel` 能跟踪，停车不倒退、不长时间振荡。

### 阶段 5：切换到 LQR（此时才做模型参数标定）

- [ ] 重新实测 `M`（车体质量）、`m`（含轮）、`l`（质心高度）、`I`、`MGR` 等物理参数，重新跑 `lqr_gain_design.py` 算 `K`
- [ ] **`K` 已是离散 LQR 解**（`cont2discrete` ZOH 离散 + `solve_discrete_are`，非连续近似），但离散化用的 `dt` 必须与固件实测循环周期一致；若 ESP32 单次循环（AHRS + 控制 + 通信）实测不是设计时的 200Hz/5ms，必须按实测 `dt` 重新跑 `lqr_gain_design.py` 生成新 `K`，不能直接套用仿真时 200Hz 算出的增益
- [ ] 阶段 3/4 已验证硬件链路通畗，此步若不稳基本可断定是模型参数或线性化误差

**过关标准：** LQR 平衡表现不差于 PID（响应更快、稳态误差更小）；固件实测控制周期与 `lqr_gain_design.py` 里的 `dt` 一致。

### 阶段 6：坡度估计器重新标定

- [ ] 仿真标定的 `c=3.916` 是仿真摩擦模型拟合值，实车滚动摩擦、轴承阻力、电机内阻完全不同，**必须重新在实车平地上做一次最小二乘标定**（方法见 [c 标定](./lqr_slope_feedforward.md#c-calibration)，直接复用流程只换数据源）

**过关标准：** 平地上 `alpha_hat` 稳定在 0 附近，不随速度变化漂移。

### 阶段 7：偏航环（仿真里被"作弊"掉的另一个点）

- [ ] 仿真 IMU 给的 `yaw` 是全局精确值，可直接做 heading hold 锁参考。**实车若只用陀螺仪积分 yaw 会持续漂移**，heading hold 锁的参考本身会跑偏；要绝对航向需融合磁力计，但磁力计离电机/大电流线越近干扰越大，这是实车上坑最多的环节
- [ ] 建议：只保留"抑制短时间漂移"能力，不追求分钟级绝对航向锁定

**过关标准：** 直线行驶 5~10s 内航向不明显跑偏即可，不强求长时间。

### 阶段 8：整合复现仿真场景

- [ ] 按仿真里验证过的场景表逐条复现：平地启停、斜坡启停、斜坡巡航、`cmd_vel` 阶跃、断线安全保护——每条对着仿真时的 bag 图做对比

---

## 三、模块对照表 {#module-mapping}

| 仿真里的模块 | 实车对应改动 | 关键风险点 |
| --- | --- | --- |
| `attitude_estimator.py`（直接读 Gazebo 四元数） | 换成 ESP32 本地 AHRS（互补/Madgwick） | 仿真"零成本"拿到的姿态，实车要单独调 |
| `/joint_states`（Gazebo 关节状态） | 编码器计数 → 角速度/角位置换算 + LPF | 计数溢出、方向、齿轮比换算 |
| `/wheel_effort_controller/commands`（理想力矩执行器） | 力矩 → PWM/电流映射 | 死区、非线性、电池压降下力矩打折 |
| `safety_limiter.py` | 原样搬固件，加通信超时保护 | 必须比控制算法先测通 |
| `slope_dyn_obs_node.py` 的 `c` | 实车重新最小二乘标定 | 不能直接用仿真标定值 |
| `yaw_loop.py` heading hold | 降低绝对航向锁定预期 | 缺乏可靠绝对航向源 |
| LQR `K`（`lqr_gain_design.py`） | 用实测物理参数重新算 | 先用 PID 排除硬件问题再切 |

---

## 四、总验收

全部阶段过关后，应能独立回答：

1. 为什么控制环不能经 WiFi 闭环，通信应该承担什么职责
2. 为什么先上 PID 再切 LQR 能缩小排查范围
3. 仿真里哪两处（姿态来源、绝对航向）是被仿真器"作弊"掉的，实车必须额外补
4. `c` 和 LQR 物理参数为何都不能直接把仿真标定值搬到实车

---

## 五、与仓库文档的索引

| 阶段 | 主文档 |
| --- | --- |
| 阶段 5 LQR 重标 | `docs/LQR_explain.md`、[动力学清单 §4](./dynamics_kinematics_checklist.md) |
| 阶段 6 `c` 标定 | [lqr_slope_feedforward · 缺口 6](./lqr_slope_feedforward.md#c-calibration) |
| 阶段 7 频带分离思路 | [频带分离](./lqr_slope_feedforward.md#freq-separation) |
| 阶段 4 参考轨迹/停车倒退 | [动力学清单 §6](./dynamics_kinematics_checklist.md)、[lqr_slope_feedforward · 缺口 4](./lqr_slope_feedforward.md#field-fixes) |

---

## 六、落地到 `fishbot_esp32_mt_example` 的具体清单 {#fishbot-project}

这是当前实车基础工程的现状盘点与优化顺序，把上面通用阶段对应到这个仓库的具体文件/类。

### 现状（已具备，对应阶段 0 + 部分阶段 4）

- PlatformIO + ESP32 + Arduino，`micro-ROS` 走 WiFi transport
- 电机驱动 `Esp32McpwmMotor` + 编码器 `Esp32PcntEncoder`
- 单轮速度 PID（`lib/PidController`），闭环 20ms/50Hz，**本地 `loop()` 闭环，未经 WiFi**——已符合"控制环必须本地闭环"原则
- 差速运动学 + 里程计（`lib/Kinematics`）：`kinematic_inverse/forward`、`update_odometry`
- `/cmd_vel` 订阅、`/odom` 发布（20Hz），micro-ROS 单独跑在 `microros_task`

### 缺口（对应阶段 1~7，是接下来要做的核心）

- 无 IMU、无姿态融合 → 无 `pitch`，当前只是纯差速小车，不具备"平衡"能力
- 无安全层（摔倒/超时保护）
- 无姿态/LQR 平衡控制器，现有 PID 只是轮速跟踪，不是姿态环
- 无坡度估计、无偏航保持

### 优化顺序清单

- [ ] **核心绑定**：`microros_task` 现固定在 core 1（`xTaskCreatePinnedToCore(..., 1)`），Arduino `loop()` 默认也在 core 1，二者抢同一核；控制环提到 200Hz 前先把 `microros_task` 改绑 core 0，把控制环独占 core 1（频带分离在硬件上的对应：通信和控制不能抢同一执行资源）
- [ ] **`cmd_vel` 超时看门狗**：现在 `twist_callback` 收到目标后一直保持，WiFi 断线不会自动清零，先补上（对应阶段 2 的"通信超时保护"）
- [ ] **接 IMU + 单独验证姿态**：优先选带硬件 DMP/AHRS 输出的 IMU，省去自己在 ESP32 上写融合算法；先只发布 `pitch` 观察，不接控制（对应阶段 1）
- [ ] **摔倒保护**：`pitch` 超阈值直接清零 PWM，不经过任何 PID（对应阶段 2）
- [ ] **姿态环上电**：新增 `lib/BalanceControl` 状态反馈类（\(k\) 已含符号，PID/LQR 只是增益表不同），目标倾角固定为 trim，先不接 `cmd_vel`，验证能否独立站立（对应阶段 3）。不复用 `PidController`：其 D 为内部差分、误差符号相反，且已是死代码
- [ ] **控制周期提到 200Hz**：从现在 `motorSpeedControl()` 的 20ms 轮询改为硬件定时器中断触发，保证平衡环时序确定性
- [ ] **速度环接入**：优先直接在 `BalanceControl` 里填 \(k_{\dot s}\)（单环状态反馈，与阶段 5 LQR 同构）；若仍要级联，则由外层速度环写 `ref.pitch`，不新建 backend（对应阶段 4）
- [ ] **差速力矩混合**：复用 `Kinematics::kinematic_inverse` 的 `主量 ± 转向量/2` 数学形式做力矩混合，不新建 `WheelMixer` 类，两行内联即可
- [ ] **遥测补齐**：仿照 `odom_pub` 加一个 debug 话题（`pitch`/PWM/轮速），复用现有 `rosbag`/画图排障习惯
- [ ] 在线调参（话题改 `kp/ki/kd`）暂缓，先接受重新烧录，等姿态环大致能站住、需要频繁微调时再加，避免过早引入不必要的复杂度
