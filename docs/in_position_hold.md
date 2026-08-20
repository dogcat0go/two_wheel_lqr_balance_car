# In-position 保持：倒立摆实车停机框架

> 对象：本仓库 stage5（LQR → `applyTorque` → 电流 PI → 死区前馈）。  
> 目标：空地、`vref=0`、无外支撑时，能进入工业上的 **in-position / standstill**，而不是只靠 `|τ|<τ_eps` 一条带子。  
> 现状：`kTorqueEps` 已在 `WheelActuator::applyTorque` 入口；它是执行器钳位，**还不是**车辆级 in-position。

本车没有抱闸。工业质量在这里的含义是：**进保持有与门和确认时间，出保持有或门，动/停两套执行律，遥测可验收**。不是编码器永久零速。

---

## 0. 质量条与非目标

| 要达到 | 不追求 |
| --- | --- |
| `vref=0` 时进入 HOLD 后 `eff=0`，靠静摩擦钉住 | 无静摩擦（架空）仍能「停」 |
| 倾角、角速度、轮速、位置误差**同时**小才允许停 | `|K·e|` 小就可以停（对消也能进） |
| 任一通道超限立即唤醒 LQR（倒立摆开环不稳） | dither / 故意微振过静摩擦 |
| TRACK 时才允许死区前馈 | 休息区里仍 `compensateDeadband` |
| 验收：空地 30 s 内 `x` 峰峰值 ≤ 3 cm，HOLD 占空比可见 | 与有抱闸的机床同一套锁死指标 |

trim（`t`）是另一个环：机械平衡点必须先对准。In-position **不**代替拧 `t`，也不代替扰动观测器。

---

## 1. 分层：策略在车，钳位在轮

工业伺服把 **in-position 策略**和 **电流/PWM 钳位**拆开。本车应对齐：

```text
control_task (Core1, 200 Hz)
  LQR 照常算 u_sum、u_yaw          ← 即使 HOLD 也算，用作唤醒判据
  HoldPolicy：TRACK / CONFIRM / HOLD
  HOLD：两轮 τ_cmd = 0，冻结积分
  TRACK：τ_cmd = u_sum/2 ± u_yaw，可加摩擦前馈

WheelActuator
  |τ_cmd| 为 0 → PWM=0、清电流积分、不走死区前馈
  不再单独根据 |τ| 决定「车该不该停」（避免左右一轮停一轮转）
```

`kTorqueEps` 降级为 **一致性检查 / 执行器保险丝**（`|τ_cmd|` 已经是 0 时的保护），车辆进 HOLD 的决定权上收到 `HoldPolicy`。

左右必须同一拍进入/退出 HOLD。分轮 `torque_rest_` 会在 `u_yaw≠0` 时撕航向。

---

## 2. 状态机

四个状态，全部在 `control_task`，与 `armed` / `Safety` 并列。

```text
         未武装 / 摔倒 / 有速度指令 / 有转向指令
TRACK ◄──────────────────────────────────────────── HOLD
  │ 进入条件连续成立 N 拍                              │
  ▼                                                    │
CONFIRM ──条件断开──► TRACK                            │
  │ 满 N 拍                                            │
  ▼                                                    │
 HOLD ──退出条件任一成立──► TRACK（可带唤醒斜坡）
```

| 状态 | 电机 | LQR | 偏航积分 | 死区前馈 | `pos_ref` |
| --- | --- | --- | --- | --- | --- |
| TRACK | 跟 `τ_cmd` | 出力 | 可积 | **开**（见 §5） | `v_cmd=0` 时锁存 |
| CONFIRM | 仍跟 `τ_cmd` | 出力 | 可积 | 开 | 不变 |
| HOLD | **PWM=0** | 只算不出力 | **冻结** | **关** | 可选：锁成当前 `x`（见 §4） |

`CONFIRM` 不可省：倒立摆过零时 `|v|`、`|ω|` 会抖一两个控制拍，没有确认时间会 200 Hz 开关。

---

## 3. 进入 = 与门；退出 = 或门

这是和「只看 `τ_eps`」的本质差别。

记 \(e_\theta=\theta-\theta_{\mathrm{ref}}\)，\(e_s=s-s_{\mathrm{ref}}\)，\(v=0.5(v_L+v_R)\)。  
`v_cmd`、`w_cmd` 非零时 **禁止进入**，已在 HOLD 则立即退出（跟速度/转向优先）。

### 3.1 进入（全部成立，并保持 `kHoldEnterTicks`）

回差用「进窄出宽」：进入门槛严，退出门槛松，避免边界抖。

| 通道 | 进入 | 退出（任一超限即醒） | 初值（按本车 log / \(K\)） | 物理含义 |
| --- | --- | --- | --- | --- |
| 倾角 | \(\|e_\theta\|<\theta_{\mathrm{in}}\) | \(\|e_\theta\|>\theta_{\mathrm{out}}\) | 0.4° / 0.8° | 静摩擦能撑住的歪角；\(l=2\,\mathrm{cm}\) 很敏感 |
| 倾角速率 | \(\|\dot\theta\|<\omega_{\mathrm{in}}\) | \(\|\dot\theta\|>\omega_{\mathrm{out}}\) | 0.12 / 0.25 rad/s | 还在倒就不要停电机 |
| 轮速 | \(\|v\|<v_{\mathrm{in}}\) | \(\|v\|>v_{\mathrm{out}}\) | 0.02 / 0.04 m/s | 「人眼已停」 |
| 位置 | \(\|e_s\|<s_{\mathrm{in}}\) | \(\|e_s\|>s_{\mathrm{out}}\) | 3 cm / 6 cm | 禁止在偏了 20 cm 时假装该停 |
| 力矩（可选 AND） | \(\|u_{\mathrm{sum}}\|/2<\tau_{\mathrm{in}}\) | \(\|u_{\mathrm{sum}}\|/2>\tau_{\mathrm{out}}\) | 0.009 / 0.0135 N·m | 与现 `kTorqueEps` 对齐；防对消漏检可保留 |
| 指令 | \(\|v_{\mathrm{cmd}}\|<\varepsilon_v\) 且 \(\|w_{\mathrm{cmd}}\|<\varepsilon_w\) | 指令超出 | 已有 `kYawCmdEps`；线速度用 0.01 m/s | 跟速时不准 HOLD |
| 安全 | `armed` 且无 `HardFault` | 摔倒 / IMU 丢失 | 已有 Safety | HOLD 里倒立摆无主动力矩 |

确认时间：`kHoldEnterTicks = 20`（100 ms @ 200 Hz）。退出：**当拍立即** TRACK，不加确认（倒立摆来不及等）。

### 3.2 为什么必须有位置通道

\(k_s=0.147\,\mathrm{N\cdot m/m}\) 时，20 cm 只贡献 \(0.03\,\mathrm{N\cdot m}\)，可与 \(e_\theta\) 对消后仍 `|τ|<τ_eps`。  
位置与门把休息区从「\(K\cdot e\) 的一条带子」收成「原点附近的盒子」。这才是 in-position。

位置门槛不要按 Bryson `s_m=1 m` 来：那是 LQR 设计时「可接受上限」，不是停机窗。停机窗按 **静摩擦能锁住的位移**（本车先 3 cm）。

### 3.3 摔倒角是硬退出，不管 in-position

`kFallAngleRad` 已在 Safety。HOLD 期间 pitch 一旦接近摔倒阈值，必须当拍 PWM 允许 LQR 拉回——实际上此时 `|e_θ|` 早已超过 `θ_out`。不要在 HOLD 里屏蔽 Safety。

---

## 4. HOLD 内做什么、不做什么

**做：**

1. 两轮 `applyTorque(0)`（或 `stop()` + `resetCurrentLoop()`），保证死区前馈不跑。
2. 冻结 `yaw_integ`、电流 PI 积分（`resetCurrentLoop` 已清）。
3. LQR **继续算** `u_sum` 和各项 `terms[]`，供退出判据和遥测。不算就不知道该不该醒。
4. 遥测打标志 `hold=1`，`eff` 必须为 0。

**可选（第二期）：** 进入 HOLD 时把 `pos_ref ← x`。  
效果：在摩擦锁住的位置安家，不把「20 cm 外的原点」继续当目标。  
代价：不再回 `r` 时的原点。定点站立产品通常选这个；回原点巡线才锁死 `pos_ref`。本车 `vref=0` 站立建议 **进入 HOLD 时 rebase `pos_ref`**。

**不做：**

- 不在 HOLD 里积分 pitch（`ki` 已是 0，保持）。
- 不加 dither。
- 不按轮分别休息。

**唤醒（HOLD → TRACK）：** 当拍恢复 `τ_cmd=u_sum/2±u_yaw`。若冲击大，用已有 `kLqrMaxSlew` 限斜率即可，不必另做软起。

---

## 5. TRACK：摩擦前馈与休息拆开

工业组合是 **动则补摩擦，停则钳位**。本车现在是继电器死区，HOLD 未生效时站立也会抬到 ±10%。

TRACK 才允许 `compensateDeadband`。stage5 已按 Karnopp 门控（门槛复用 `kHoldVelIn/Out` 与 `kTorqueEps`，回差）：

| 条件 | 行为 |
| --- | --- |
| HOLD | PWM=0，无前馈 |
| TRACK 且 \|v\| ≥ v_karnopp | 按 `sign(v)` 补库仑（比 `sign(τ)` 少在零速翻） |
| TRACK 且 \|v\| < v_karnopp 且 \|τ_cmd\| ≥ τ_in | 按 `sign(τ_cmd)` 补，用于起步跨门槛 |
| TRACK 且 \|v\| < v_karnopp 且 \|τ_cmd\| < τ_in | **不补**，让与门去进 CONFIRM |

`v_karnopp` 可与 `v_out` 同一量级（0.04 m/s）。不要把落地静摩擦整数值写进 `kMotorDeadband`（已有标定文档约定：NVS 只存架空电气门槛）。

---

## 6. 和现有模块怎么接

| 模块 | 角色 |
| --- | --- |
| `BalanceController` | 公式不动；HOLD 时仍 `update`，输出可丢掉 |
| `Safety` | 优先于 HOLD；hard fault 直接 disarm |
| `kTorqueEps` / `applyTorque` | 执行器侧：`τ=0` 必停；非 0 跟电流环。车辆策略不再藏在这里 |
| `kMotorCmdEps` | 仅 `applyRawPwm`（m 0/1）；m 3 走 HoldPolicy |
| `pos_ref` | `r` 锁一次；进 HOLD rebase（§4） |
| 偏航 `z/j/n` | HOLD 冻结 I；TRACK 且 yaw_hold 才积 |
| `cmd_vel` / 串口 `v` `a` | 非零 → 禁止 HOLD |
| 前探轮 `r` | `armed` 之前无 HOLD |

串口不必新字母：阈值进 `config.h`。需要现场拧再加 `h` 命令，第一期不要。

遥测（验收用，少即是多）：现有行加 `hold=0|1` 和 `hN=`（CONFIRM 计数）。`eff=0` + `hold=1` 才能认定休息咬住。

---

## 7. 初值从哪来（本车）

由 `wifi_20260819_233459.log`（`t=2.3`，拿掉盒子后的自由段）和当前 \(K\)：

- 自由站立 `|e_θ|` rms ≈ 1°，`|v|` 经常 > 0.02：说明 **现在进不了** 上述与门，这是预期（先 trim、再 HOLD、再考虑加 `k_s`）。
- `θ_in=0.4°`：对应单轮 \(k_θ/2\times e_θ\approx 0.006\,\mathrm{N\cdot m}\)，低于 `kTorqueEps`。
- `s_in=3\,\mathrm{cm}`：\(k_s\times 0.03\approx 0.004\,\mathrm{N\cdot m}\)，位置项不再能单独对消出「假休息」。
- `kHoldEnterTicks=20`：100 ms，短于倾角互补滤波时间常数（~0.25 s），避免等滤波「看起来稳了」才停。

只改一档：先 `θ`/`ω`/`v` 与门 + 确认时间，**位置门先关**（`s_in=∞`）做对照；能停再打开 `s_in=3 cm`。一次只开一个门，才能归因。

---

## 8. 验收

空地、不靠盒子、`m 3`、`t` 已贴实测均值、`v 0`：

| ID | 判据 | 过关 |
| --- | --- | --- |
| H1 | 扶正 `r` 后 5 s 内出现 `hold=1` | 能进 |
| H2 | HOLD 期间 `eff_L=eff_R=0` | 钳位真发生 |
| H3 | HOLD 期间 `\|v\|<0.02` 且 `x` 峰峰值 ≤ 3 cm（≥10 s 窗） | 静摩擦锁住 |
| H4 | 轻推后 `hold=0`，LQR 拉回，能再次 HOLD | 或门唤醒 |
| H5 | `v 0.1` 期间 `hold` 恒 0 | 指令优先 |
| H6 | 故意靠盒子再拿开：拿开后允许短暂 TRACK，不得把「顶住时的假零速」当成 HOLD 验收 | 外约束无效 |

H3 做不到先查 trim，再查确认时间是否太短/太长，最后才动 `K`。

---

## 9. 落地顺序（与代码，不做未来抽象）

不新建 backend 类。一个 `bool hold` + 一个计数器即可。

1. **车辆级与门 + CONFIRM + 两轮同时 PWM=0**（策略从 `applyTorque` 的 `|τ|` 回差里搬上来）。遥测 `hold=`。过 H1/H2/H4/H5。  
2. **打开位置门 + 进 HOLD 时 rebase `pos_ref`**。过 H3。  
3. **TRACK 才 `compensateDeadband`，零速且小 τ 不补**（§5）。消灭 HOLD 之外的 ±10% 继电器。  
4. （可选）很弱的 trim 偏置观测 / 位置积分。这是常值扰动，不是 in-position 本身。

第 1 步未过之前不要加 `y`（`k_s`）：位置项大会把 `|τ|` 顶出窗，HOLD 更进不去。

---

## 10. 和「只 τ_eps」对照

| | 仅 `τ_eps` | 本框架 |
| --- | --- | --- |
| 判据 | `|K·e|` 一条带子 | 各状态盒子 + 力矩一致性 |
| 左右 | 分轮可能一停一转 | 整车同一状态 |
| 时间 | 当拍进/出 | 进入确认、退出立即 |
| 对消 | 20 cm 对消后可停 | 位置门拒绝 |
| 死区前馈 | 仍可能在临界点抬 PWM | HOLD 内禁止；TRACK 才补 |
| 指令 | 不看 `v_cmd` | 跟速强制 TRACK |

`τ_eps` 借了 LQR 的权重，但不是 in-position。工业质量来自 **与门、确认、整车钳位、动停分律**；本框架按这个接到现有 200 Hz 单环上，不改 LQR 公式、不加仿真级模块。
