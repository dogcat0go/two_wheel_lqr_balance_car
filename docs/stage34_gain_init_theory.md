---
title: 阶段 3/4 增益初值与范围
level: 1
order: 12
sections: true
---

# 阶段 3/4 增益初值与范围

> 回答：\(k_\theta,k_{\dot\theta}\)（阶段 3）和 \(k_{\dot s},k_s\)（阶段 4）第一次填多少、落在什么区间、凭什么。
>
> - 调参步骤：[`stage3_balance_tuning.md`](stage3_balance_tuning.md)、[`stage4_velocity_loop.md`](stage4_velocity_loop.md)
> - 物理方程推导（取矩 / 非惯性系）：[`balance_gain_theory.md` 附录 A](balance_gain_theory.md#derivation)
> - 实现：`lib/BalanceControl`，同一式子填不同增益

固件符号（与串口命令）：

| 文档符号 | `config.h` | 串口 | 单位 |
| --- | --- | --- | --- |
| \(k_\theta\) | `kGainPitch` | `p` | %/rad |
| \(k_{\dot\theta}\) | `kGainPitchRate` | `d` | %/(rad/s) |
| \(k_{\dot s}\) | `kGainVel` | `w` | %/(m/s) |
| \(k_s\) | `kGainPos` | `y` | %/m |

约定：\(k=\partial u/\partial x\) 已含符号；前倾要正输出，故 \(k_\theta>0\)，速度/位置项同样取正（非最小相位，见第 4 节）。

---

## 1. 一张表看完全套初值 {#summary}

控制律（refs 已减掉）：

\[
u = k_\theta\theta + k_{\dot\theta}\dot\theta + k_s s + k_{\dot s}v + k_{i\theta}\!\int\theta\,dt
\]

| 增益 | 角色 | 设计旋钮 | 本车建议起步 | 合理搜索范围 |
| --- | --- | --- | ---: | ---: |
| \(k_\theta\) | 扶正（硬下界 \(>g/K_a\)） | \(\omega_n=\eta\lambda\) | 200 → 实测可到 500 | 150 ~ 730 |
| \(k_{\dot\theta}\) | 俯仰阻尼 | \(T_d=k_{\dot\theta}/k_\theta\) | \(0.022\,k_\theta\) | 随 \(k_\theta\)；勿单独乱拧 |
| \(k_{\dot s}\) | 消漂移 / 速度跟踪 | \(\omega_s\) 或 \(\tau_v\) | 40 | 20 ~ 90 |
| \(k_s\) | 定点 | \(\omega_s,\zeta_s\)；先 0 | 0 → 5~10 | 0 ~ 20 |
| \(k_{i\theta}\) | 消 trim 残差 | 先不用 | 0 | 阶段 3/4 默认 0 |

本车几何：\(l\approx0.012\,\mathrm{m}\)，\(\lambda=\sqrt{g/l}\approx30\,\mathrm{rad/s}\)。设计点 \(\eta=\omega_n/\lambda=2\)，\(\zeta=0.8\)。

调参主线（把多维拧增益降成一维）：

1. 固定 \(T_d=0.022\)，只放大 \(k_\theta\)（阶段 3）
2. 固定 \(k_s=0\)，只放大 \(k_{\dot s}\)（阶段 4 Step 1）
3. 需要定点时再开 \(k_s\)，并保持 \(k_{\dot s}/k_s \approx 2\zeta_s/\omega_s\)（本车常取 \(\approx4\sim8\,\mathrm{s}\)）

---

## 2. 阶段 3：\(k_\theta,k_{\dot\theta}\) {#stage3}

### 2.1 植物理 + 控制律

小角度轮式倒立摆（推导见 [附录](balance_gain_theory.md#derivation)）：

\[
l\,\ddot\theta = g\,\theta - \ddot s
\]

加速度域 PD（设计选择，不是物理）：

\[
\ddot s = k_{a,\theta}\theta + k_{a,\dot\theta}\dot\theta
\]

代回并对上标准二阶 \(\ddot\theta+2\zeta\omega_n\dot\theta+\omega_n^2\theta=0\)：

\[
\boxed{\;k_{a,\theta}=g+l\omega_n^2,\qquad k_{a,\dot\theta}=2\zeta\omega_n l\;}
\]

硬约束：\(k_{a,\theta}>g\)，否则闭环仍不稳定（「\(K_p\) 太小软倒」）。

取 \(\omega_n=\eta\lambda=\eta\sqrt{g/l}\) 时更干净：

\[
k_{a,\theta}=g(1+\eta^2),\qquad
k_{a,\dot\theta}=2\zeta\eta\sqrt{gl}
\]

注意 \(k_{a,\theta}\) **与 \(l\) 无关**；\(l\) 估错只影响 \(k_{a,\dot\theta}\propto\sqrt{l}\)。

### 2.2 换成占空比

\[
K_a=\frac{\text{轮轴加速度}}{\text{占空比}}\quad[\mathrm{(m/s^2)/\%}]
\qquad\Rightarrow\qquad
k_\theta=\frac{k_{a,\theta}}{K_a},\quad
k_{\dot\theta}=\frac{k_{a,\dot\theta}}{K_a}
\]

\(K_a\) 未精确实测时，绝对值不确定，但比值稳：

\[
T_d=\frac{k_{\dot\theta}}{k_\theta}
=\frac{2\zeta\omega_n l}{g+l\omega_n^2}
=\frac{2\zeta\eta\sqrt{l/g}}{1+\eta^2}
\]

本车 \(\eta=2,\zeta=0.8,l=0.012\) → \(T_d\approx0.022\,\mathrm{s}\)。

### 2.3 本车数值与范围

| 估法 | \(K_a\) | \(k_\theta\)（\(\eta=2\)） |
| --- | ---: | ---: |
| \(\tau=0.15\,\mathrm{s}\)，\(v_{ss}/u=0.0101\) | 0.067 | ~730 |
| \(\tau=0.08\,\mathrm{s}\) | 0.126 | ~390 |
| 堵转力粗估 | ~0.17 | ~290 |

结论：

| 量 | 值 |
| --- | --- |
| \(k_\theta\) 起步 | **200**（偏保守），×1.5 上爬 |
| \(k_\theta\) 区间 | **150 ~ 730** |
| \(k_{\dot\theta}\) | **始终** \(=0.022\,k_\theta\)（例如 \(k_\theta=500\Rightarrow k_{\dot\theta}=11\)） |
| \(k_{i\theta}\) | 先 **0** |

细节、死区、延迟天花板见 [`balance_gain_theory.md`](balance_gain_theory.md) 第 3~9 节。

### 2.4 为什么阶段 3 必然单向漂移

姿态环 alone 在「摆 + 电机一阶滞后」三阶模型里，特征多项式常数项恒负——无论 \(k_\theta\) 多大，都有一个不稳定实根。物理图像：扶直靠轮子持续加速，没有 \(k_{\dot s}\) 就把 \(v\) 拉不住。

**阶段 3 验收允许单向慢漂**；消漂移是阶段 4 的事。别靠拧 \(k_\theta\)/\(k_{i\theta}\) 硬消。

---

## 3. 阶段 4：\(k_{\dot s},k_s\)——时标分离 {#stage4}

阶段 3 把俯仰快模态压住后，平移慢模态可以单独设计。下面用准稳态把四阶系统拆开。

### 3.1 加速度域全状态反馈

\[
\ddot s = k_{a,\theta}\theta + k_{a,\dot\theta}\dot\theta + k_{a,s}s + k_{a,\dot s}v
\]

俯仰已按 \(\omega_n=\eta\lambda\) 设计好，且 \(\omega_n\) 远快于平移。准稳态（\(\ddot\theta\approx\dot\theta\approx0\)）下植物理给出 \(\ddot s = g\theta\)，代回控制律：

\[
g\theta = k_{a,\theta}\theta + k_{a,s}s + k_{a,\dot s}v
\]

\[
\theta=\frac{k_{a,s}s+k_{a,\dot s}v}{g-k_{a,\theta}}
\]

因 \(k_{a,\theta}>g\)，令

\[
\alpha=\frac{g}{k_{a,\theta}-g}=\frac{g}{l\omega_n^2}=\frac{1}{\eta^2}>0
\]

则慢模态：

\[
\ddot s + \alpha\,k_{a,\dot s}\,\dot s + \alpha\,k_{a,s}\,s = 0
\]

对照 \(\ddot s+2\zeta_s\omega_s\dot s+\omega_s^2 s=0\)：

\[
\boxed{\;
\omega_s^2=\alpha\,k_{a,s},\qquad
2\zeta_s\omega_s=\alpha\,k_{a,\dot s}
\;}
\]

反解：

\[
\boxed{\;
k_{a,s}=\frac{\omega_s^2}{\alpha}=\eta^2\omega_s^2,\qquad
k_{a,\dot s}=\frac{2\zeta_s\omega_s}{\alpha}=\eta^2\cdot 2\zeta_s\omega_s
\;}
\]

占空比域（\(K_a\) 约掉进比值）：

\[
k_s=\frac{k_{a,s}}{K_a},\qquad
k_{\dot s}=\frac{k_{a,\dot s}}{K_a},\qquad
\frac{k_{\dot s}}{k_s}=\frac{2\zeta_s}{\omega_s}
\]

相对俯仰增益：

\[
\frac{k_{\dot s}}{k_\theta}
=\frac{\eta^2\cdot 2\zeta_s\omega_s}{g(1+\eta^2)}
\]

### 3.2 只开速度项（\(k_s=0\)）时

慢模态退化成一阶：

\[
\dot v + \alpha\,k_{a,\dot s}\,v = 0
\qquad\Rightarrow\qquad
\tau_v=\frac{1}{\alpha\,k_{a,\dot s}}=\frac{1}{\alpha K_a k_{\dot s}}
\]

\(\tau_v\) 就是「漂移被刹住」的时间尺度。本车 \(\eta=2\Rightarrow\alpha=1/4\)：

\[
\tau_v=\frac{4}{K_a\,k_{\dot s}}
\]

| \(K_a\) | \(k_{\dot s}=40\) 时的 \(\tau_v\) |
| ---: | ---: |
| 0.067 | ≈ 1.5 s |
| 0.10 | ≈ 1.0 s |
| 0.17 | ≈ 0.6 s |

起步 \(k_{\dot s}=40\) 对应约 1 s 量级收漂，合理。太小（\(\tau_v\gg\) 数秒）看起来仍在单向跑；太大则与俯仰耦合，低频前后晃。

### 3.3 trim 残差给出 \(k_{\dot s}\) 的下界直觉

稳态时 \(u\approx0\)、\(k_s=0\)：\(k_\theta\delta+k_{\dot s}v_{ss}\approx0\)，

\[
v_{ss}=\frac{k_\theta\,\delta}{k_{\dot s}}
\]

（开了速度环后，无界加速变成**有界恒速爬行**——这是环在工作，不是坏了。详见 [`stage4_velocity_loop.md`](stage4_velocity_loop.md#kvel-drift)。）

若希望 \(|v_{ss}|<v_{\max}\)，需要

\[
k_{\dot s} > \frac{k_\theta\,|\delta|}{v_{\max}}
\]

例：\(k_\theta=500\)，\(\delta=0.5°\approx0.009\,\mathrm{rad}\)，\(v_{\max}=0.05\,\mathrm{m/s}\) → \(k_{\dot s}>90\)。  
trim 调好后 \(\delta\) 更小，\(k_{\dot s}=40\) 往往够用。根治仍是修 trim 或加 \(k_s\)（\(s_{ss}=k_\theta\delta/k_s\)）。

### 3.4 位置环：选 \(\omega_s,\zeta_s\)

约束：\(\omega_s\ll\omega_n\)。本车 \(\omega_n\approx60\,\mathrm{rad/s}\)，取

\[
\omega_s = 0.3\sim 1.0\,\mathrm{rad/s},\qquad \zeta_s = 0.8\sim 1.0
\]

（再快容易和俯仰/死区缠在一起抖；再慢定点「拽回来」像在蠕动。）

比值（与 \(K_a\) 无关，优先信这个）：

\[
\frac{k_{\dot s}}{k_s}=\frac{2\zeta_s}{\omega_s}
\]

| \(\omega_s\) | \(\zeta_s\) | \(k_{\dot s}/k_s\) | 若 \(k_{\dot s}=40\) 则 \(k_s\) |
| ---: | ---: | ---: | ---: |
| 0.5 | 1.0 | 4.0 s | 10 |
| 0.5 | 0.8 | 3.2 s | 12.5 |
| 0.3 | 1.0 | 6.7 s | 6 |
| 1.0 | 1.0 | 2.0 s | 20 |

这与调参记录里「\(w=40,y=10\) → 近似 \(\zeta_s=1,\omega_s=0.5\)」一致。

绝对值仍乘 \(1/K_a\)，和阶段 3 一样不确定；所以：

- **先定比值** \(k_{\dot s}/k_s\)
- **只开 \(k_{\dot s}\)** 收到漂，再按比值加 \(k_s\)
- 抖 → 降 \(k_s\) 或升 \(k_{\dot s}\)（加慢模态阻尼）

### 3.5 本车阶段 4 建议范围

| 增益 | 起步 | 搜索范围 | 备注 |
| --- | ---: | ---: | --- |
| \(k_{\dot s}\) | **40** | **20 ~ 90** | `y 0`；×1.5 上爬；晃了就退 |
| \(k_s\) | **0** | **0 ~ 20** | 定点才开；从 5 起，勿一上来 10+ |
| 比值 \(k_{\dot s}/k_s\) | — | **4 ~ 8 s** | \(\zeta_s\approx1,\omega_s\approx0.3\sim0.5\) |

前置：`kMPerTick` 必须重标，否则速度反馈整体差一个比例，增益全拧歪。

---

## 4. 符号：为什么四个 \(k\) 都取正 {#signs}

| 项 | \(k>0\) 的含义 | 反了会怎样 |
| --- | --- | --- |
| \(k_\theta\) | 前倾 → 向前加速（朝倒的方向追） | 上电即朝反方向推倒 |
| \(k_{\dot\theta}\) | 前倒加快 → 多给前进 | 俯仰振荡加剧 |
| \(k_{\dot s}\) | 车已前冲 → **先再加速** 把车身扳后仰再刹（非最小相位） | 越大越飞 |
| \(k_s\) | 位置超前 → 先向前「绕回来」 | 定点发散或猛拽 |

阶段 3/4 的 Step 0 就是验这两类符号（俯仰、速度），不过绝不落地放大增益。

---

## 5. 和 LQR / 级联的关系 {#lqr}

| | 本文手调 | 级联（外环写 `ref.pitch`） | 阶段 5 LQR |
| --- | --- | --- | --- |
| 结构 | 单环四项状态反馈 | 外环 PD → 内环姿态 | \(u=-Kx\) 四项 |
| 慢环设计 | \(\omega_s,\zeta_s\) → \(k_s,k_{\dot s}\) | \(\theta_{\mathrm{cmd}}=-k_p's-k_d'v\)，再经 \(a\approx g\theta\) | \(Q,R\) 自动分配快慢极点 |
| 与本文关系 | — | 准稳态下 \(k_{a,*}\leftrightarrow g\cdot k'_*\) 可对照 | 手调 \(K\) 应与 \(-K_{\mathrm{LQR}}\) 同量级 |

阶段 3/4 拧出的数是阶段 5 的基线：差一个数量级先查单位 / \(K_a\) / \(l\)，别先怪求解器。

---

## 6. 快速上手清单 {#quickstart}

**阶段 3**

1. \(\lambda=\sqrt{g/l}\)，取 \(\eta=2,\zeta=0.8\) → \(T_d\approx0.022\)（本车）
2. \(k_\theta=200\) 起，\(k_{\dot\theta}=T_d\,k_\theta\)，每次 \(k_\theta\times1.5\)
3. 按 [`stage3_balance_tuning.md`](stage3_balance_tuning.md#steps) Step 0→5；允许单向慢漂

**阶段 4**

1. 确认 `kMPerTick`；Step 0 验 speed 与 effort 同号
2. `y 0`，`w 40` 起 ×1.5，收到单向漂移为止（目标 \(\tau_v\sim1\,\mathrm{s}\)）
3. 需要定点：按 \(k_s=k_{\dot s}/(4\sim8)\) 从 5 试起；抖则降 `y` 或升 `w`
4. 恒速小爬行 → 先 `t` 修 trim，再考虑加大 `w` / 开 `y`
5. 按 [`stage4_velocity_loop.md`](stage4_velocity_loop.md#steps) 验收，定稿写回 `config.h`
