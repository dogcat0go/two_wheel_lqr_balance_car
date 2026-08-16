# 阶段 5：从实测 \(M,m,l\) 到 LQR 实车（当前基线）

> 本文覆盖 [`stage34_gain_init_theory.md`](stage34_gain_init_theory.md) 与旧 [`stage5.md`](stage5.md) 里对 \(K\) 的冲突。  
> **阶段 3/4 的 \(k\) 是 %**；**LQR 的 \(K\) 是 N·m**。不要把 500/10/10/20 直接填进力矩环。  
> 本次**不做**摩擦补偿、坡度/`c` 前馈。保留 trim、`kff`、软停、`pos_ref`、偏航差速。

---

## 0. 两套 \(K\) 差在哪

| | 阶段 3/4 手调 | 现在要部署的 LQR |
| --- | --- | --- |
| 植物理 | \(l\ddot\theta=g\theta-\ddot s\)（加速度输入） | 完整 \(\mathbf{M}_q\)，输入 \(\tau\) |
| 设计 | \(\omega_n,\zeta\) → \(k_a\) → 除以未精测的 \(K_a\) | 离散 LQR：\(Q,R\) → DARE → \(K\) |
| 固件 \(u\) | `%`；`m 2` 再乘 `kPctToTorque` | **N·m**，直接 `applyTorque` |
| 式子 | `BalanceController`：\(u=\sum k_i(x_i-x_{\mathrm{ref},i})\) | 同一式子；SciPy 若 \(u=-K_{\mathrm{sp}}x\)，固件填 **\(k=-K_{\mathrm{sp}}\)**（\(k_\theta>0\)） |

[`stage34_gain_init_theory.md`](stage34_gain_init_theory.md) 只负责手调 `%` 初值（且其中 \(l=0.012\) 已过时）。LQR 以本文 + [`stage5_physical_params_table.md`](stage5_physical_params_table.md) 为准。

---

## 1. 已冻结的植物理（2026-08-16）

| 符号 | 值 | 进哪 |
| --- | ---: | --- |
| \(M\) | 0.70 kg | 车体，去轮留电机 |
| \(m\) | 0.090 kg | 两轮合计，转子=0 |
| \(l\) | 0.0204 m | §2 加权 \(l_M\) |
| \(I\) | **0**（第一次） | 绕质心；分段估 \(5.63\times10^{-4}\) 作对照可选 |
| \(r\) | 0.0375 m | `kWheelRadiusM` |
| \(g\) | 9.81 | |
| \(K_t\) | 0.23 N·m/A | 左右暂同一 |
| \(\mathrm{d}t\) | 0.005 s | `kCtrlDt`，必须与脚本一致 |

连续质量矩阵（\(I=0\)）见参数表 §4。开环 \(a_{21}\approx 4.22\times10^3\)，\(\sqrt{a_{21}}\approx 65\,\mathrm{rad/s}\)。

\(\tau\)：**两轮轮轴力矩之和**（N·m）。左右各 `applyTorque(\(\tau/2 \mp \tau_{\mathrm{yaw}}\))`，或先 \(\tau_L=\tau_R=\tau/2\)（\(u_{\mathrm{yaw}}=0\)）。

---

## 2. 离线算 \(A,B,P,K\)（仿真机，不在 ESP32）

本仓库无 `lqr_gain_design.py`，在仿真工程。按下面核对脚本；对不上就改脚本输入/状态顺序，不要改植物理。

1. 用 \(\mathbf{M}_q\ddot q = [0;Mgl]\theta + [1/r;-1]\tau\) 得到连续 \(A,B\)，状态 **\([\theta,\dot\theta,s,\dot s]^\top\)**（与 `BalanceState` 一致）。  
   公式：[`lqr_balance_theory.html` §4.6](lqr_balance_theory.html)。
2. `cont2discrete(A, B, dt=0.005, method='zoh')` → \(A_d,B_d\)。
3. Bryson 起步（经验上限，不是称重）：例如 \(\bar\theta=5^\circ\approx0.087\,\mathrm{rad}\)，\(\bar\tau=0.15\,\mathrm{N\cdot m}\)（两轮合计、接近日常站立出力），\(\bar\omega,\bar s,\bar v\) 自定。  
   \(Q=\mathrm{diag}(1/\bar\theta^2,\;1/\bar\omega^2,\;1/\bar s^2,\;1/\bar v^2)\)，\(R=1/\bar\tau^2\)。第一次 \(\bar s\) 可大一点（少罚位置）以免再顶满 `pos=-25` 那种。
4. `P = solve_discrete_are(Ad, Bd, Q, R)`，  
   \(K_{\mathrm{sp}}=(R+B_d^\top P B_d)^{-1} B_d^\top P A_d\)（一行四列，对应 \(u=-K_{\mathrm{sp}}x\)）。
5. 检查：\(A_d-B_d K_{\mathrm{sp}}\) 特征值全部 \(|\lambda|<1\)。开环 \(A\) 应有一个正实根。

固件四项：

\[
k_\theta=-K_{\mathrm{sp}}[0],\;
k_{\dot\theta}=-K_{\mathrm{sp}}[1],\;
k_s=-K_{\mathrm{sp}}[2],\;
k_{\dot s}=-K_{\mathrm{sp}}[3]
\]

前倾正力矩，故 \(k_\theta\) 应为**正**。若四个里 \(k_\theta<0\)，先查脚本 \(\theta\)/\(\tau\) 符号，不要在车上翻。

量纲：N·m/rad、N·m/(rad/s)、N·m/m、N·m/(m/s)。**不要**和手调 500、10、10、20 比大小。

---

## 3. 固件怎么接（结合现有代码）

结构已经是 LQR 同构，只换单位和四个数字。

| 文件 | 现在 | LQR 部署时 |
| --- | --- | --- |
| `BalanceController.cpp` | \(u=\sum k(x-x_{\mathrm{ref}})\)，饱和 `out_abs` | **式子不动**；`setLimits` 改成 N·m |
| `config.h` | `kGain*` 单位 `%`；`kPctToTorque`；`kMaxEffort=100` | 新四项填 §2 的 \(k_*\)；`kMaxTorque` 当输出饱和；可删掉平衡路径上的 `kPctToTorque` |
| `stage2_main.cpp` `m 2` | `tau = limited * kPctToTorque` → `applyTorque` | `limited` 已是 N·m：`applyTorque(limited)`；左右再分 \(\tau/2\) 与 yaw |
| `Safety` | 按 `%` 限幅/斜率 | 限幅改 N·m（与 `kMaxTorque` 一档），斜率改 N·m/s |
| `WheelActuator::applyTorque` | \(I_{\mathrm{ref}}=\tau/K_t\)，电流 PI | **不动** |
| `m 1` | `applyRawPwm` | **保留**，手调 `%` 对照 |

仍保留、本次不改设计：`vref_smooth`、`pos_ref`、`kff`（`g`）、`t` trim、`m 2` 默认 `u_yaw=0`。`ki_pitch` 继续 0。不要加摩擦项、不要加坡度前馈。

偏航：若以后开 `a`，`u_yaw` 也必须是 N·m，不能再当 %。

串口 `p/d/y/w` 仍改四个 \(k\)，只是单位变成力矩。

建议小步：

1. 仿真算出 \(K_{\mathrm{sp}}\)，写入 `config.h` 的 `kLqr*`（固件 \(k=-K_{\mathrm{sp}}\)）。  
2. 串口 **`m 3`**：LQR + `applyTorque`，**不经** `kPctToTorque`。  
3. **`m 1` / `m 2` 原样**：手调 `%`（`p/d/y/w` 只改这两档）。`m 0` 开环。  
4. 不要改 `BalanceController` 公式。A/B 用 `m 2` vs `m 3`。

---

## 4. 你现在按这个做

1. 仿真侧用 §1 参数算出 \(K_{\mathrm{sp}}\) 和闭环特征值，记状态顺序。  
2. 把 \(k=-K_{\mathrm{sp}}\) 四项发回来（或自己写入 `config.h`）。  
3. 改 `stage2_main` `m 2`：去掉 `%→τ` 乘子，`setLimits(kMaxTorque, …)`，Safety 同步。  
4. 烧录：`m 2`，扶正，`r`，看 `hz=200 ovr=0`、`iref` 与 `iL/iR`、两边 `ticks`。  
5. 太肉：加大 \(Q_\theta\) 重算；太抖：加大 \(R\)。不要先滤 `pitch_rate`。  
6. `v` 仍靠现有 `kff`+速度项；位置项若再饱和，先 `y 0` 或减小 \(Q_s\)。

摩擦、斜坡前馈：阶段 6 再说。
