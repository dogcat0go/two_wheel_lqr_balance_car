# 阶段 6：斜坡补偿（实车流程）

> 固件：`pio run -e stage5`（本阶段不新建 env）。  
> 控制律与缺口 1–6（仿真教训）：[`lqr_slope_feedforward.md`](lqr_slope_feedforward.md)  
> 一维 KF 公式：[`slope_kf_simple.md`](slope_kf_simple.md)  
> \(c\) 最小二乘推导：[`lqr_slope_feedforward.md` · 缺口 6](lqr_slope_feedforward.md#c-calibration)

**本阶段目标：** 平地 \(\hat\alpha\approx 0\) 且不随速度漂；小坡（先 3°）上 \(\tau_{\mathrm{ff}}\) 与 \(\theta_{\mathrm{eq}}\) 配对，LQR 不再和重力打架。  
**不做：** 笼统 DOB、把 \(\alpha\) 塞进 \(K\)、在线 RLS、一上来就上 \(z_{\mathrm{kin}}\)、把仿真 \(c=3.916\) 写入固件。

过关再下一步。S1 不过不写 KF；S2 的 \(c\) 不过不把 \(\hat\alpha\) 接到 \(\theta_{\mathrm{ref}}\)。

---

## 0. 进本阶段的门槛（S0）

停车 ±5 cm 冻结，不再为 3 cm 抠 HOLD。确认：

| 项 | 判据 |
| --- | --- |
| 站立 | 平地 `m 3`、扶正 `r`，可重复站住 |
| 给速 | `v` 能跟，不点头炸 |
| 停车 | `v 0` 后不长距离倒退、不振荡十几秒 |
| 周期 | 遥测 `hz=200` |

HOLD / TrimObserver 本阶段只当**平地**功能。坡上 \(\tau_{\mathrm{eq}}\neq 0\)，PWM=0 会放车下滑。

### 0.1 先补参考成形，再进 S1（当前缺口）

stage5 切 LQR 时把 stage2 的参考生成丢掉了：`cmd.k_vff` 仍下发，**控制环没用**；`pos_ref` 只在 `r` / 退出平衡 / 进 HOLD 时锁存，**给 `v` 时不积分**。于是：

- 设速度不会主动前倾 → 低速卡死区，或靠 \(k_{\dot s}\) 慢慢拱，`pitch_ref` 仍等于 trim
- \(k_s\neq 0\) 时位置项把车拽回 `r` 时的原点（「走不动 / 倒退」）
- 停车 `v` 阶跃归零，无 `vref_smooth` → 缺口 4 那种回拉

这和坡度补偿是同一条 `ref.pitch` / `pos_ref` 管道。\(\theta_{\mathrm{eq}}(\hat\alpha)\) 要叠在 \(k_{\mathrm{ff}}v\) 上面；S2 标 \(c\) 需要「给速就真的匀速走」。**不要先做 S1。**

从 stage2 搬回三件事（仍在 `stage5_main`，不改 \(K\)）：

| 项 | 做法 | 过关 |
| --- | --- | --- |
| \(k_{\mathrm{ff}}\) | `ref.pitch = trim + trim_bias + clamp(k_vff·v_smooth)` | `v 0.15` 时 `pitch_ref` 抬约 \(k_{\mathrm{ff}}v\)，车能走 |
| `pos_ref` | \(\lvert v_{\mathrm{smooth}}\rvert\) 大时 `pos_ref += v_smooth·dt`，带现有位置项 anti-windup | 匀速时 `u pos` 不封顶往回拉 |
| 软停 | `v_smooth` 斜率限幅逼近 `linear_x`；停稳后再锁 `pos_ref` | `v 0` 无明显倒退振荡 |

左右：**已经有**航向 P（`z`）、轮速差（`n`）、航向 I（`j`）。短时走直够标 \(c\)、够上 3° 板。不要等磁力计（阶段 7），也不要为「完美差速」挡住 S1。

**S0 过关（补全后）：** 平地 `v 0.15` 能跟；`v 0` 能停；松杆走直线数秒不明显拧。然后才 S1。

---

## 1. 控制律（只记这一条）

\(\alpha\) 是环境扰动，不是状态，不进反馈增益 \(K\)。

\[
\tau = \tau_{\mathrm{fb}} + \tau_{\mathrm{ff}},\quad
\tau_{\mathrm{fb}}=-K(\hat x-x_{\mathrm{ref}}),\quad
\tau_{\mathrm{ff}}=MGR\sin_{\mathrm{eff}}
\]

\[
\sin_{\mathrm{eff}}=k\cdot\mathrm{ramp}\cdot\sin\hat\alpha_f,\quad
\theta_{\mathrm{eq}}=K_{\mathrm{EQ}}\sin_{\mathrm{eff}}
\]

\(\hat\alpha_f\) 是死区 + 一阶低通之后的角（A0）。**两半必须共用同一个 \(\sin_{\mathrm{eff}}\)**：只加力不搬姿态，或 `gain`/ramp 只乘在一半上，都会和 LQR 打架。

现有 \(k_{\mathrm{ff}}v_{\mathrm{cmd}}\) 仍加在 `ref.pitch` 上，与 \(\theta_{\mathrm{eq}}\) 叠加；前者跟速度指令，后者跟坡。不要用 \(k_{\mathrm{ff}}\) 冒充坡补偿。

完整参考（S0.1 先接前三项，S4 再加最后一项）：

\[
\theta_{\mathrm{ref}}
= \underbrace{\theta_{\mathrm{trim}}}_{\texttt{t}}
+ \underbrace{\mathrm{bias}}_{\mathrm{TrimObs}}
+ \underbrace{\mathrm{sat}(k_{\mathrm{ff}}\,v_{\mathrm{smooth}})}_{\text{跟速度，平地巡航}}
+ \underbrace{\theta_{\mathrm{eq}}(\hat\alpha)}_{\text{跟坡，S4}}
\]

\(k_{\mathrm{ff}}\) 的含义不变：给速就主动前倾，顶过 NMP/死区，**不是**坡上平衡角，也不从 \(K\) 重算。初值仍用 `kGainVelToPitch`（10°/(m/s)），LQR + Karnopp 后可能偏大，串口 `g` 往下拧。必须用 \(v_{\mathrm{smooth}}\)，不要对 `linear_x` 阶跃。限幅仍 `kFfPitchLimitRad`（相对 trim；S4 后 \(\theta_{\mathrm{eq}}\) 另占额度，真坡试验别把两项顶满摔倒角）。

---

## 2. 实车数字（`config.h`，不用仿真表）

| 量 | 值 | 备注 |
| --- | ---: | --- |
| \(M\)（车体） | 0.70 kg | 去轮留电机 |
| \(m\)（两轮） | 0.135 kg | `kWheelMassKg` |
| \(M_{\mathrm{tot}}\) | 0.835 kg | \(M+m\) |
| \(l\) | 0.0204 m | 车体质心到轮轴 |
| \(r\) | 0.0375 m | |
| \(g\) | 9.81 | |
| \(MGR=M_{\mathrm{tot}}gr\) | \(\approx 0.307\) N·m | 总轮力矩 |
| \(K_{\mathrm{EQ}}=M_{\mathrm{tot}}r/(Ml)\) | \(\approx 2.19\) | 仿真车 \(\approx 1\) |

本车 \(l<r\)，故 \(\theta_{\mathrm{eq}}\approx 2.2\alpha\)。摔倒阈 \(\approx 30^\circ\)：5° 坡前倾约 11°，10° 约 22°。真坡从 **3° 板** 起。

\(\tau\) 已是两轮力矩之和。估计器必须用**上一拍实际下发的总力矩**（含前馈），不能只用 \(\tau_{\mathrm{fb}}\)。

---

## 3. 分环（禁止合成一个 DOB）

trim 零点、摩擦 \(c\)、坡度 \(\alpha\)、外力在低频残差里分不开。

```text
快环（已有）     LQR K：θ, θ̇, s, v
慢补偿（本阶段）  α̂ → A0 → τ_ff 与 θ_eq 配对
有门、更慢       TrimObserver 只修 θ_ref；仅平地、v_cmd=0、|α̂|<死区
离线一次         平地有速度时标 c
```

| 谁 | 何时动 | 何时冻 |
| --- | --- | --- |
| HOLD（PWM=0） | 平地、近零速 | \(\lvert\sin_{\mathrm{eff}}\rvert\) 超死区，或 \(v_{\mathrm{cmd}}\neq 0\) → 强制 TRACK |
| TrimObserver | 同上 | 上坡 / 给速 → 冻结，禁止 `ref=pitch` |
| 坡度 KF | S3 起只观测；S4 起入环 | 不把 trim 当状态 |
| 笼统 DOB | **本阶段不做** | — |

---

## 4. 流程

### S1 开环喂角（补偿开，估计关）

**目的：** 把「控制集成」和「估计精度」拆开。

1. 串口注入固定 \(\alpha_{\mathrm{inj}}\)（平地即可；或把车放到已知板子上但 \(\hat\alpha\) 仍用手填）。
2. 只实现 \(\tau_{\mathrm{ff}}+\theta_{\mathrm{eq}}\)，`gain` 先 0.5，两半共用 \(\sin_{\mathrm{eff}}\)。不写 KF。
3. 对照：故意只开 \(\tau_{\mathrm{ff}}\)、\(\theta_{\mathrm{ref}}\) 不动，应看到位置爬或姿态–位置拉锯。

**过关：** 两半都开时，车体停在 \(\approx K_{\mathrm{EQ}}\sin_{\mathrm{eff}}\)，\(\tau_{\mathrm{fb}}\to 0\)。

### S2 只记录 \(z_{\mathrm{dyn}}\)，离线标 \(c\)

**目的：** 堵住假坡度（缺口 6）。\(\hat\alpha\) 仍不上控制。

1. 平地、`v≠0`（不要用 HOLD/`τ=0` 段）。录 `τ`（两轮和）、`v`；\(\dot v\) 可事后差分+低通。
2. \(\tilde m\) 先用 \(M_{\mathrm{tot}}\)（无 \(I_w\) 时）。过原点：

\[
y_i=\tau/r-\tilde m\,\dot v,\quad x_i=v,\quad
c^\star=\frac{\sum x_i y_i}{\sum x_i^2}
\]

3. 同一 log 切两段，\(c\) 应接近。把 \(c^\star\) 写入 `config.h`。仿真 3.916 **作废**。

**过关：** 复算后平地 \(z_{\mathrm{dyn}}\) 均值不随 \(v\) 走。

### S3 KF 只出遥测

**目的：** 估计器单独验收。

1. 200 Hz、Core1，状态只有 \(\alpha\)。先只开 \(z_{\mathrm{dyn}}\)。
2. \(z_{\mathrm{kin}}\) 本步不做（实车加速度计在加减速时污染 \(\hat\theta\)）。
3. 平地起步对外发 \(\hat\alpha=0\) 数秒，再渐入。

**过关：** 平地静立与巡航 \(\hat\alpha\) 在死区里（目标 \(0\pm 1^\circ\)），不随速度漂。这是 Sim2Real「阶段 6」原过关句。

### S4 \(\hat\alpha\) 入环

1. A0：死区（初值可 0.015 rad）+ 低通 \(\tau\approx 0.5\,\mathrm{s}\) + 使能 ramp。
2. `gain` 0.5 → 1。KF 输入含前馈的总 \(\tau\)。
3. HOLD / Trim 按 §3 加门。

**过关：** 平地 \(\hat\alpha\approx 0\) 时行为退回 stage5；3° 板前倾到位、不持续爬、\(\tau_{\mathrm{fb}}\) 稳态小。

### S5 真坡，再考虑 \(z_{\mathrm{kin}}\)

3° → 5°。\(z_{\mathrm{kin}}\) 用自适应 \(R\)（\(\dot v\to 0\) 自动闭嘴）。急刹鼓包先复算 \(c\)，不要先加观测门控。

**过关：** 坡上停 5 s \(\hat\alpha\) 不塌回 0。

---

## 5. 固件落点（写代码时）

不新建坡度 PID，不改 `BalanceController` 的四项公式。

| 块 | 放哪 | 约束 |
| --- | --- | --- |
| 估计 | 与 LQR 同拍，Core1 | 输入上一拍 \(\tau_{\mathrm{sum}}\)、\(v\)、\(\dot v\) |
| A0 + 两半 | `stage5_main` 参考生成与力矩求和 | `ref.pitch += θ_eq`（再叠加 trim、\(k_{\mathrm{ff}}v\)）；`balance_u += MGR·sin_eff` |
| HOLD 门 | `HoldPolicy` 或调用处 | \(\lvert\sin_{\mathrm{eff}}\rvert\) 超死区 → TRACK |
| Trim 门 | `TrimObserver` 调用处 | 上坡冻结 |
| \(c\) | `config.h` 常数 | 不做 RLS |
| 遥测 | wifi log | S2 起：`z_dyn`、`alpha_hat`、`sin_eff`、`tau_ff`、`tau_fb` |

---

## 6. 失败先查这张表

| 现象 | 先查 |
| --- | --- |
| 平地一走 \(\hat\alpha\) 跟速度走 | \(c\) 未标或偏小 |
| 平地来回蹭 | A0 没接，或 \(c\) 错 |
| 上坡 LQR 出力很大、车往竖拉 | 只加了力没搬 \(\theta_{\mathrm{eq}}\)，或两半缩放不一致 |
| 上坡进 HOLD 往下滑 | HOLD 未按 \(\hat\alpha\) 禁止 |
| trim 上坡后被改飞 | TrimObserver 未冻结 |
| 急刹二次鼓包 | 先复算 \(c\)，不要先加 \(R_{\mathrm{dyn}}\) 门控 |

---

## 7. 本阶段不做

| 项 | 留给 |
| --- | --- |
| 笼统 DOB 兼修 trim 与坡 | 另开慢环且上坡冻结；本阶段不写 |
| \(\alpha\) 进状态反馈 | 永不 |
| 增益调度 \(K(\alpha)\) | 坡 \(\gtrsim 25^\circ\) 才考虑 |
| 磁力计绝对航向 | 阶段 7 |
| 停车从 5 cm 打到 3 cm | 已冻结 |

下一步：**先把 §0.1 的参考成形搬回 stage5**，过关后再 S1 串口注入 \(\alpha\)。不要先写 KF。
