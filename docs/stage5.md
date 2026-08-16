阶段 5 的核心不是「再写一套控制器」，而是：**把手调四项状态反馈换成由模型算出的力矩增益，并把 `m 2` 的 \(u\) 从占空比 % 改成 N·m**。结构（`BalanceController` + `vref_smooth` + `kff` + 软停 + 偏航差速）应原样保留。

**现行部署步骤（已测 \(M,m,l\)，电流环可用）：** [`stage5_lqr_deploy.md`](stage5_lqr_deploy.md)。  
[`stage34_gain_init_theory.md`](stage34_gain_init_theory.md) 只解释手调 **%** 增益，其中 \(l=0.012\) 已过时；不要把 500/10/10/20 当 LQR 的 \(K\)。

下面按「先标定、再算 K、最后换增益」给出可执行流程。

---

## 0. 进阶段 5 前的门槛（你现在基本过关）

对照 [`Sim2Real_list.md`](Sim2Real_list.md) 阶段 3/4：

| 项 | 状态建议 |
| --- | --- |
| 平地站立 >10s、轻推能回 | 需自检 |
| `hz=200 ovr=0` 稳定 | 已有 |
| 给 `v` 能走、`g=10` 前馈可用 | 已有 |
| 软停后抖可接受 | 你说已好多 |
| IMU 上电标定不跟姿势跑、`trim` 稳定 | 已修 |
| 偏航短时走直 | 已有骨架 |

还没过也不要急着上 LQR 代码；阶段 5 出问题时应能断定是**模型/执行器**，不是「车根本站不稳」。

---

## 1. 总路线（两条执行器路径，先选一条）

| 路径 | 适用 | 主文档 |
| --- | --- | --- |
| **A. 电流传感（推荐，你正打算走）** | 加装电流传感器，\(\tau=K_t I\)，再上电流内环 | **[`stage5_current_torque_plan.md`](stage5_current_torque_plan.md)** · SOP **[`current_to_torque_calibration_sop.md`](current_to_torque_calibration_sop.md)** |
| B. 无电流电压前馈 | 暂不加传感，开环 τ→PWM | [`tau_pwm_calibration_sop.md`](tau_pwm_calibration_sop.md) |

**路径 A（有电流）顺序（2026-08-14：P0～P2 已过，细节 [`stage5_current_torque_plan.md`](stage5_current_torque_plan.md)）：**

```text
P0✅ 接线 → P1⚠ 电流（外接表跳过）→ P2✅ 挂重 Kt=0.23/0.22
    → P3 停转重校零 + m0 电流PI（C1）→ 平衡改 N·m → P4 物理参数 + 离散 LQR
```

**路径 B（无电流）顺序：**

```text
① 测物理参数 (M,m,l,…)
      ↓
② τ→PWM 标定 (R,Kt,Kb,dead,Vbatt)
      ↓
③ 确认固件 dt 与设计一致（200Hz）
      ↓
④ 离线算离散 LQR 的 K（仿真侧 lqr_gain_design.py）
      ↓
⑤ 固件：执行器改力矩接口 + 增益换成 −K，对比手调基线
```

**不要**第一步就改 `BalanceController` 去「算 LQR」——没有可靠力矩执行器（A 的电流环或 B 的电压前馈），算出的 `K` 单位和物理意义都对不上。

---

## 2. 步骤 ①：实车物理参数（为模型服务）

要重新测，**不要直接搬仿真数值**（清单里写得很死）：

| 符号 | 含义 | 怎么测（够用精度） |
| --- | --- | --- |
| \(M\) | 车体（不含轮）质量 | 台秤 |
| \(m\) | 两轮+电机转子等效 | 拆轮称 / 估算 |
| \(l\) | 质心到轮轴高度 | 吊线/翻转找平衡，或拆装称重估 COM；你现在 `kComHeightM=0.012` 只是估算 |
| \(r\) | 轮半径 | 已有 0.0375 m，可复核 |
| \(I\) | 车体绕质心惯量 | 摆振法或 CAD；没有就用点质量近似并在 Q/R 里留裕度 |
| \(MGR\) 等 | 与仿真脚本一致的那套符号 | 对照动力学清单 / 仿真 `lqr_gain_design.py` 输入表 |

**验收**：按可填表 [`stage5_physical_params_table.md`](stage5_physical_params_table.md) 落盘（电池/骨架/板质量与相对轮轴高度分开填）。  
**\(M\)：拆掉车轮、电机留在车上**（电机属车体；轮子进 \(m\)）。

本仓库**没有** `lqr_gain_design.py`（在仿真工程里）。阶段 5 算 `K` 仍用那边的脚本，只把输入改成实车实测。

---

## 3. 步骤 ②：力矩执行器（硬门槛；按路径分支）

### 路径 A — 电流传感（推荐）

主文档：**[`stage5_current_torque_plan.md`](stage5_current_torque_plan.md)**

```text
Gate H✅ → Gate I⚠（无外接表）→ Gate Kt-I✅ 挂重 0.23/0.22
  → 停转重校零 + m0 电流 PI（C1）→ 平衡输出改 N·m → LQR
```

电压前馈全套 SOP **降级为可选**（\(R/K_b\) 仅作电流环前馈/排障）。

### 路径 B — 无电流电压前馈

| 文档 | 用途 |
| --- | --- |
| [`tau_pwm_calibration_sop.md`](tau_pwm_calibration_sop.md) | 现场 SOP + Gate |
| [`motor_torque_pwm_calibration.md`](motor_torque_pwm_calibration.md) | 公式与原理 |

`applyTorque(τ)` → 电压公式出 PWM；死区合并进 \(D_{\mathrm{dead}}\)。

**两条路径共同结论**：LQR 的 \(u\) 必须是 N·m；直接把 \(K\) 当 PWM 用会白算。

---

## 4. 步骤 ③：确认离散化 `dt`

遥测已有 `hz=200`。算 `K` 时：

- `dt = 1/200 = 0.005 s`，与 `cfg::kCtrlDt` **必须一致**
- 若以后改成 100/250 Hz，必须**重跑** `lqr_gain_design.py`，不能沿用旧 `K`

---

## 5. 步骤 ④：离线设计 \(K\)（仿真侧脚本）

在仿真仓库跑离散 LQR（清单要求：`cont2discrete` ZOH + `solve_discrete_are`）：

1. 填入步骤 ① 的物理参数 + 步骤 ② 后「力矩单位」一致的执行器假设  
2. 选 \(Q,R\)：用手调基线对照（见 [`balance_gain_theory.md`](balance_gain_theory.md) / [`stage34_gain_init_theory.md`](stage34_gain_init_theory.md)）  
   - 手调：\(k_\theta\approx500,\;k_{\dot\theta}\approx10,\;k_s\approx10,\;k_{\dot s}\approx15\)（单位是 **%/状态**）  
   - 切到力矩后，**数值不能直接对比**，要先换算到 N·m 量纲，或先保证符号/相对权重合理  
3. 输出 \(K=[k_\theta,k_{\dot\theta},k_s,k_{\dot s}]\)（注意固件填的是 **\(-K\)**，因为控制器是 \(u=\sum k_i(x_i-x_{\mathrm{ref},i})\)）

求解器用 SciPy：`cont2discrete` ZOH + `solve_discrete_are`，**不要**在 ESP32 上解 Riccati。公式推导与填表注意见 [`lqr_balance_theory.html` §7.5.1](lqr_balance_theory.html) / [`lqr_study_qa.html` §9](lqr_study_qa.html)。状态顺序必须与脚本一致后再重排进固件。

**对照检查**：量纲统一后，\(-K[0]\) 与手调 \(k_\theta\) 应**同量级**；差一个数量级 → 先查 \(l\)、\(K_a\)/τ→PWM、单位，别先怪求解器。

保留现有参考成形，**不要**指望 LQR 代替它们：

- `kff`（速度→倾角）  
- `vref_smooth`（软停）  
- `pos_ref` 积分 + anti-windup  
- yaw 差速  

LQR 只换四项反馈增益。

---

## 6. 步骤 ⑤：固件接入与实车对比

建议小步：

1. **只接通 τ→PWM**，增益仍用手调（把手调输出从「%」改成「按同一比例当 τ」或先做一层标定比例）——确认站立不炸  
2. **再换成 \(-K_{\mathrm{LQR}}\)**，串口仍可临时覆盖增益做 A/B  
3. 对比指标（同一电量、同一地面）：  
   - 静止抖幅（`eff`/`pitch`）  
   - `v 0.2` 跟踪与停稳时间  
   - 轻推恢复时间  
4. **过关**：不差于当前手调；响应/稳态更好更好；`dt` 与设计一致  

出问题排查顺序（阶段 5 特有）：

```text
τ→PWM 常数 → l / M → Q/R → 再怀疑符号/状态定义
```

不要一抖就去滤 `pitch_rate`（已踩过坑）。

---

## 7. 阶段 5 **不做**的事（留给后面）

| 内容 | 阶段 |
| --- | --- |
| 坡度估计 / `c` 重标定 | 6 |
| 长时绝对航向（磁力计） | 7（现在轮式 yaw 短时够用） |
| 仿真场景全集复现 | 8 |

---

## 8. 建议你「下周」的执行清单（2026-08-14）

P0～P2 已过（\(K_t\) 见 SOP）。按 [`stage5_current_torque_plan.md`](stage5_current_torque_plan.md) §5 / §10：

1. 停转重校零（静止须 ±0.1 A）  
2. \(K_t\) 写入 `config.h`；`m 0` 做 200 Hz 电流 PI；架空 `er 0.4` 过 C1  
3. 再把平衡出口改 N·m（手调，先不换 LQR）  
4. 同期称 \(M,m\)、复核 \(l\)  
5. C3 能站后再离线算 \(K\)、A/B  

---

## 一句话结论

植物理已测、电流环 `m 2` 能站：下一刀是 **离线离散 LQR（N·m）+ `m 2` 去掉 `kPctToTorque`**，见 [`stage5_lqr_deploy.md`](stage5_lqr_deploy.md)。  
摩擦与斜坡前馈不做。阶段 3/4 的 `%` 增益文档只作手调对照，不参与填 \(K\)。