---
title: 平衡增益的理论与初值估算
level: 1
order: 12
sections: true
---

# 平衡增益的理论与初值估算

> 回答一个问题：\(k_\theta\)、\(k_{\dot\theta}\) 第一次该填多少，凭什么（姿态环详解）。
> 阶段 3/4 增益初值与范围总览（含 \(k_{\dot s},k_s\)）见
> [`stage34_gain_init_theory.md`](stage34_gain_init_theory.md)。
> 配套：调参流程见 [`stage3_balance_tuning.md`](stage3_balance_tuning.md)，
> 实现见 `lib/BalanceControl/BalanceController.{h,cpp}`。
> 公式推导（牛顿定律 → 增益）见 [附录 A](#derivation)。

---

## 1. 为什么倒立摆必须"朝倒的方向追" {#why-positive}

小角度线性化的轮式倒立摆（\(\theta\) = 前倾角，\(s\) = 轮轴位移，\(l\) = 质心到轮轴高度）：

\[
l\,\ddot\theta = g\,\theta - \ddot s
\]

> 此式从哪来（对轮轴取矩 / 非惯性系、小角度）见 [附录 A](#derivation)。

直觉验证：车身往前倾（\(\theta>0\)）时重力把它继续往前拽，\(g\theta\) 项让 \(\ddot\theta>0\)，
这就是**不稳定极点**的来源；而轮轴向前加速（\(\ddot s>0\)）会把车身相对往后扳，所以带负号。

结论：要压住前倾，必须让轮子**向前**加速，即 \(k_\theta>0\)。
这和常规「误差负反馈」相反——常规 PID 写 `kp*(target-current)` 会给出反号，
所以 `BalanceController` 直接约定 \(k=\partial u/\partial x\)、符号自带，不再套误差 PID。

**这个不稳定极点的时间尺度**决定了你的控制环至少要多快：

\[
\lambda = \sqrt{g/l}
\]

| \(l\)（质心高度） | \(\lambda\) | 倒下的特征时间 \(1/\lambda\) |
| ---: | ---: | ---: |
| 0.06 m | 12.8 rad/s | 78 ms |
| 0.10 m | 9.9 rad/s | 101 ms |
| 0.15 m | 8.1 rad/s | 124 ms |

车越矮，倒得越快，越难控。200Hz（5ms）控制环相对 100ms 的倒塌时间有 20 倍余量，够用。

---

## 2. 用二阶系统指标反解增益 {#design}

先在**加速度域**设计（\(u_a\) 的单位是 m/s²，之后再换成占空比）：

\[
\ddot s = k_{a,\theta}\,\theta + k_{a,\dot\theta}\,\dot\theta
\]

> 这是人为选定的控制律，不是物理推导；与标准二阶式的对照见 [附录 A](#derivation)。

代回运动方程：

\[
l\ddot\theta + k_{a,\dot\theta}\dot\theta + (k_{a,\theta}-g)\theta = 0
\]

对照标准二阶式 \(\ddot\theta+2\zeta\omega_n\dot\theta+\omega_n^2\theta=0\)：

\[
\boxed{\;\omega_n=\sqrt{\frac{k_{a,\theta}-g}{l}},\qquad
2\zeta\omega_n=\frac{k_{a,\dot\theta}}{l}\;}
\]

反解出设计公式：

\[
\boxed{\;k_{a,\theta}=g+l\,\omega_n^2,\qquad k_{a,\dot\theta}=2\zeta\,\omega_n\,l\;}
\]

三条能直接用的结论：

1. **\(k_{a,\theta}\) 必须大于 \(g=9.81\)**，否则 \(\omega_n^2<0\)——闭环仍然不稳定。
   这是「\(K_p\) 太小车就是软倒」的数学原因，不是手感问题。
2. \(\omega_n\) 取 \(2\sim3\times\lambda\)。比如 \(l=0.10\)（\(\lambda=9.9\)），取 \(\omega_n=20\sim30\) rad/s。
3. \(\zeta\) 取 \(0.7\sim1.0\)。低于 0.5 会明显过冲来回晃，高于 1.2 反应发钝、追不回来。

### 数值示例（\(l=0.10\ \mathrm{m},\ \omega_n=20,\ \zeta=0.8\)）

\[
k_{a,\theta}=9.81+0.10\times400=49.8\ \mathrm{(m/s^2)/rad},\qquad
k_{a,\dot\theta}=2(0.8)(20)(0.10)=3.2\ \mathrm{(m/s^2)/(rad/s)}
\]

---

## 3. 加速度 → 占空比 {#to-duty}

固件里 \(u\) 是占空比（%），所以需要一个标度 \(K_a\)：

\[
K_a=\frac{\text{轮轴加速度}}{\text{占空比}}\quad[\mathrm{(m/s^2)/\%}]
\qquad\Rightarrow\qquad
k_\theta=\frac{k_{a,\theta}}{K_a},\quad k_{\dot\theta}=\frac{k_{a,\dot\theta}}{K_a}
\]

\(K_a\) 要实测，一步就能测（阶段 0 台架数据只给了稳态速度，不够）：

```bash
# 开环模式给一个阶跃，记录带时间戳的 v
echo 'm 0' > /tmp/fishbot_cmd
echo 'e 40' > /tmp/fishbot_cmd     # 车必须架空
```

从日志里取 \(v(t)\) 的**起始斜率**：\(K_a \approx \dfrac{\Delta v/\Delta t}{40}\)。
若只想估算，用一阶模型 \(K_a\approx \dfrac{v_{ss}/u}{\tau}\)，其中阶段 0 已测得
\(v_{ss}/u = 0.0101\ \mathrm{(m/s)/\%}\)，\(\tau\) 是起转时间常数（这类小减速电机通常 0.1~0.2 s）。

### 结果量级

取 \(\tau=0.15\ \mathrm{s}\Rightarrow K_a\approx0.067\ \mathrm{(m/s^2)/\%}\)：

\[
k_\theta\approx\frac{49.8}{0.067}\approx740\ \mathrm{\%/rad}\;(\approx13\ \%/\text{deg}),\qquad
k_{\dot\theta}\approx\frac{3.2}{0.067}\approx48\ \mathrm{\%/(rad/s)}
\]

物理意义检查：倾 2° 就给 26% 占空比，倾 4° 直接顶到 `kMaxEffort=60%`。量级合理。

| 参数 | 建议起步范围 |
| --- | ---: |
| \(k_\theta\) | 500 ~ 900 %/rad |
| \(k_{\dot\theta}\) | 30 ~ 60 %/(rad/s) |
| \(k_{i\theta}\) | 先 0 |

---

## 4. 最稳的做法：先定比值，再整体放大 {#ratio}

\(l\) 和 \(\tau\) 都是估的，绝对值不可靠；但两个增益的**比值**很稳，因为 \(K_a\) 约掉了：

\[
T_d=\frac{k_{\dot\theta}}{k_\theta}=\frac{k_{a,\dot\theta}}{k_{a,\theta}}
=\frac{2\zeta\omega_n l}{g+l\omega_n^2}
\]

代入不同假设：

| \(l\) | \(\omega_n\) | \(\zeta\) | \(T_d\) |
| ---: | ---: | ---: | ---: |
| 0.10 | 20 | 0.8 | 0.064 s |
| 0.10 | 15 | 0.8 | 0.074 s |
| 0.15 | 20 | 0.8 | 0.069 s |
| 0.10 | 25 | 1.0 | 0.070 s |

参数变化不小，\(T_d\) 却都落在 **0.06~0.08 s**。所以调参主线是：

1. 固定 \(T_d=0.07\)，即 \(k_{\dot\theta}=0.07\,k_\theta\)
2. 只调 \(k_\theta\) 一个旋钮，\(k_{\dot\theta}\) 跟着算
3. \(k_\theta\) 从 300 起，每次乘 1.5（300→450→675→1000…）
4. 出现高频抖动就说明 \(k_{\dot\theta}\) 撞到了噪声上限，退回并单独减小 \(k_{\dot\theta}\)

这样把二维搜索降成一维，比同时瞎拧两个数快得多。

---

## 5. 为什么调参流程要"先 Kd、离地" {#why-kd-first}

[`stage3_balance_tuning.md`](stage3_balance_tuning.md#steps) 的 Step 1 先调 \(k_{\dot\theta}\)，原因有三：

- \(k_{\dot\theta}\) 只依赖 \(\dot\theta\)，**与平衡角 trim 无关**，此时 trim 还没标定
- 它是唯一受传感器噪声硬限制的项——上限由 IMU 决定，不由物理决定，先摸到这个天花板，
  才知道 \(k_\theta=k_{\dot\theta}/T_d\) 能不能做到需要的 \(\omega_n\)
- 离地测不会摔车

如果 \(k_{\dot\theta}\) 的噪声上限对应的 \(k_\theta\) 撑不起 \(\omega_n>\lambda\)，
那问题在**姿态估计**（滤波、IMU 安装刚度、振动），继续拧增益没用。

---

## 6. 延迟与离散化的天花板 {#limits}

\(\omega_n\) 不能无限提高，闭环总延迟会吃掉相位裕度。经验判据：

\[
\omega_n \lesssim \frac{0.3}{T_{\text{delay}}}
\]

本工程的延迟来源：

| 来源 | 量级 |
| --- | ---: |
| 控制周期（200Hz，零阶保持等效 \(T/2\)） | 2.5 ms |
| I2C 读取 + 计算 | < 1 ms |
| 互补滤波（快速运动走陀螺，几乎无延迟） | ~0 |
| 电机电气/机械响应 \(\tau\) | 100~200 ms ← **主导项** |

所以真正的天花板是**电机响应**，不是控制环频率。这也解释了为什么
`kMaxEffortSlew` 必须放大到 3000 %/s：斜率限幅会额外叠一层人为延迟，
400 %/s 相当于给系统再加约 150ms 的滞后，直接吃光裕度。

关于互补滤波：`kPitchGyroCoef=0.98` 在 200Hz 下对应加速度计修正时间常数
\(\tau_{acc}=\dfrac{\Delta t\cdot c}{1-c}=\dfrac{0.005\times0.98}{0.02}\approx0.25\ \mathrm{s}\)。
快速姿态变化走陀螺通路（无延迟），慢漂移由加速度计以 0.25 s 时间常数纠正——
对平衡环是好事，不构成相位滞后。

---

## 7. 死区如何影响这套估算 {#deadzone}

阶段 0 实测静摩擦启动门槛约 20%（见 [`stage0_motor_bench.md`](stage0_motor_bench.md#deadzone)）。
按 \(k_\theta=740\ \%/\mathrm{rad}\)，20% 对应 \(0.027\ \mathrm{rad}=1.55°\)：
**车得歪到 1.5° 以上电机才真的动**。

后果是平衡点附近形成极限环——低频前后摆动，周期 0.5~2 s。判据：

- 摆动幅度大致就是死区对应的角度（1~2°），且**与 \(k_\theta\) 关系不大** → 是死区
- 幅度随 \(k_\theta\) 增大而增大、频率也升高 → 是增益过大

前者继续拧增益没用，要补库仑摩擦前馈（完整版留到阶段 5 的 τ→PWM 标定）。

---

## 8. 和 LQR 的关系 {#lqr}

阶段 5 的 LQR 求出的 \(K\) 满足 \(u=-Kx\)，\(x=[\theta,\dot\theta,s,\dot s]\)。
本文手算的 \((k_\theta,k_{\dot\theta})\) 就是 \((-K[0],-K[1])\) 的手工版本——
**结构完全一样，只是选参方式不同**：

| | 手调 PD | LQR |
| --- | --- | --- |
| 输入 | \(\omega_n,\zeta\)（想要的闭环极点） | \(Q,R\)（状态与控制的代价权重） |
| 依赖模型 | 只要 \(l\) 的粗略值 | 要 \(M,m,l,I\) 全套实测 |
| 状态数 | 2 | 4（含位置/速度） |

所以阶段 3 手调出的 \(\omega_n,\zeta\) 不是白干：它给出了阶段 5 选 \(Q/R\) 的合理落点，
也是校验 LQR 结果的基线（\(-K[0]\) 应与手调 \(k_\theta\) 同量级，差一个数量级就说明
物理参数或单位标度搞错了）。

---

## 9. 本车实测参数与结论 {#this-robot}

| 量 | 值 | 来源 |
| --- | ---: | --- |
| 轮半径 \(r\) | 0.0375 m | 实测 |
| 电机+轮质量 | 0.515 kg | 实测 |
| 整车质量 | 0.775 kg | 实测 |
| 整车最高处 | 0.10 m | 实测 |
| 上层质量（整车−电机轮） | 0.260 kg | 推算 |
| 整车质心离地 | ≈ 0.048 m | 推算 |
| **\(l\)（质心到轮轴）** | **≈ 0.012 m**（区间 0.006~0.016） | 推算 |
| \(\lambda=\sqrt{g/l}\) | ≈ 30 rad/s，\(1/\lambda\) ≈ 33 ms | 计算 |

**质心几乎贴在轮轴上**：电机+轮占总重 66% 且都位于轮轴高度，上层只有 0.26 kg。
三个后果：

1. \(\lambda\approx30\) rad/s，比本文第 1 节 \(l=0.10\) 的例子快 3 倍，属于"矮而快"的摆。
   200 Hz 控制环相对 33 ms 倒塌时间只有 6.6 拍余量（不是例子里的 20 倍），还能用但不宽裕。
2. \(T_d\) 从 0.07 降到 **0.022 s**（\(T_d\propto\sqrt{l}\)）。沿用 0.07 会阻尼过量、反应发钝。
3. **trim 灵敏度 \(\propto 1/l\)**：质心横向偏 1 mm 就相当于 \(\arctan(1/12)=4.8°\) 的平衡角偏差。
   Step 3 的真实 trim 可能有好几度，且 `t` 命令会非常敏感，建议按 0.2° 一档试。

按 \(\omega_n=2\lambda,\ \zeta=0.8\)：

\[
k_{a,\theta}=g(1+\eta^2)=49.1\ \mathrm{(m/s^2)/rad},\qquad
k_{a,\dot\theta}=2\zeta\eta\sqrt{gl}=1.10\ \mathrm{(m/s^2)/(rad/s)}
\]

注意 \(k_{a,\theta}=g(1+\eta^2)\) **与 \(l\) 无关**——只要按 \(\omega_n=\eta\lambda\) 选极点，
\(l\) 的估计误差完全不影响 \(k_\theta\)，只让 \(k_{\dot\theta}\) 按 \(\sqrt{l}\) 变。
最不确定的参数恰好影响最小，所以 \(l\) 只是粗估也没关系。

### \(k_\theta\) 绝对值仍不确定 {#ka-unknown}

换算要除以 \(K_a\)（占空比→加速度），而 \(K_a\) 没实测，两条路估出的量级差很远：

| 估法 | 假设 | \(K_a\) | \(k_\theta\) |
| --- | --- | ---: | ---: |
| 一阶时间常数 | \(\tau=0.15\ \mathrm{s}\)，\(v_{ss}/u=0.0101\) | 0.067 | 730 |
| 一阶时间常数 | \(\tau=0.08\ \mathrm{s}\) | 0.126 | 389 |
| 堵转力估 | \(K_t=K_e=0.442\)，\(I_{stall}=1.5\mathrm{A}\)，计入折算转子惯量 | ~0.17 | ~290 |

所以 \(k_\theta\in[150,730]\)，`config.h` 取偏保守的 **200** 起步，靠 `p` 命令按 ×1.5 往上爬。
想收窄这个区间就补测阶段 0 遗留的**堵转电流**（见
[`stage0_motor_bench.md`](stage0_motor_bench.md#stall)）：
\(K_e=\dfrac{V}{\omega_{noload}}=\dfrac{11.8}{1.01/0.0375}=0.442\ \mathrm{V\cdot s/rad}\)，
则 \(K_a=\dfrac{2K_e I_{stall}/r}{100\,m_{eff}}\)。

### 姿态环单独无法渐近稳定 {#pitch-only}

把「摆 + 电机一阶滞后」写成 \([\theta,\dot\theta,v]\) 三阶模型，特征多项式的常数项恒为
\(c_0=-g/(l\tau)<0\)——\(k_\theta\) 在行列式里**精确抵消**，所以无论增益取多少都有一个正实根。

物理含义：姿态环能把车"扶直"，但扶直的代价是轮子持续朝一个方向加速，
没有速度反馈就没有任何东西把 \(v\) 拉回来。这正是所有平衡车教程都要"直立环 + 速度环"的原因。

对阶段 3 的实际影响：**车能站住但会持续朝一个方向跑**，这是预期行为，不是没调好。
阶段 3 的验收标准（站立 >10 s）就是在这个前提下定的；真正的渐近稳定要等阶段 4
填 \(k_{\dot s}\)。别试图靠拧 \(k_\theta\)/\(k_{i\theta}\) 消掉漂移。

### 最高性价比的机械改动 {#raise-com}

\(l\) 从 12 mm 提到 30 mm（把电池挪到顶板），\(\lambda\) 从 30 降到 18 rad/s，
倒塌时间从 33 ms 拉到 56 ms，trim 灵敏度降到 1/2.5。
调参难度的下降幅度远超任何增益技巧——**先调不动就先改机械**。

---

## 10. 快速上手清单 {#quickstart}

1. 量 \(l\)（质心到轮轴，粗测即可），算 \(\lambda=\sqrt{g/l}\)，定 \(\omega_n=2\lambda\)、\(\zeta=0.8\)
2. \(k_{a,\theta}=g(1+\eta^2)\)、\(k_{a,\dot\theta}=2\zeta\eta\sqrt{gl}\)，\(\eta=\omega_n/\lambda\)
3. 测 \(K_a\)（`e 40` 阶跃取起始斜率，先 `f 20` 把遥测提到 50Hz），换算成 %/rad
4. **本车已算好**（见第 9 节）：\(k_\theta=200\) 起步、\(T_d=0.022\)、\(k_{i\theta}=0\)
5. 固定 \(T_d=k_{\dot\theta}/k_\theta=0.022\)，只放大 \(k_\theta\)
6. 按 [`stage3_balance_tuning.md`](stage3_balance_tuning.md#steps) Step 0→5 走，
   **Step 0 符号验证不通过绝不落地**

---

## 附录 A. 公式从哪来 {#derivation}

> 前提：牛顿第二定律、转动方程 \(I\ddot\theta=\sum\tau\)、微积分。
> 本节先汇总整条链，再把「取矩 / 非惯性系」「控制律」「标准二阶式」拆开讲。

### A.1 汇总：整条理论链在干什么

整篇文档只做一件事：**根据物理模型，算出平衡环第一次该填的两个增益**。

| 步骤 | 式子 | 角色 |
| --- | --- | --- |
| ① 物理 | \(l\ddot\theta = g\theta - \ddot s\) | 车怎么倒、轮子加速怎么扳回来 |
| ② 控制 | \(\ddot s = k_{a,\theta}\theta + k_{a,\dot\theta}\dot\theta\) | 根据姿态开轮子（人为规定） |
| ③ 闭环 | \(\ddot\theta + \cdots\dot\theta + \cdots\theta = 0\) | ①+② 代进去，变成二阶方程 |
| ④ 对照 | \(\omega_n,\zeta\) | 给二阶方程起名字，方便设计「多快、多晃」 |
| ⑤ 反解 | \(k_{a,\theta}=g+l\omega_n^2\) 等 | 由想要的响应算出增益 |
| ⑥ 换单位 | \(k_\theta = k_{a,\theta}/K_a\) | 加速度换成占空比（正文第 3 节） |

```text
重力让 θ 变大          轮轴加速让 θ 变小
     │                        │
     ▼                        ▼
  l θ̈ = gθ − s̈     ←── 物理（取矩 / 非惯性系）
              ▲
              │ 你规定 s̈ = kθ·θ + kω·θ̇
              │
     θ̈ + (… )θ̇ + (… )θ = 0   ←── 闭环二阶 ODE
              │
              │ 对照标准形式，命名为 ωn、ζ
              ▼
     由想要的 ωn、ζ 反解 k    ←── 正文第 2 节的初值估算
```

---

### A.2 模型与符号

把车身简化成：轮轴上方一根无质量杆，顶端一个质点 \(m\)。

- \(\theta\)：相对竖直的前倾角（弧度）。\(\theta=0\) 直立，\(\theta>0\) 往前倒
- \(s\)：轮轴在地面上的水平位置
- \(l\)：质心到轮轴的距离
- \(g\)：重力加速度

质心在地面固定坐标系里的位置（\(z\) 向上）：

\[
x = s + l\sin\theta,\qquad z = l\cos\theta
\]

直立时 \(x=s\)、\(z=l\)。对轮轴、质点模型的转动惯量：\(I = ml^{2}\)。
所用转动知识就一条：

\[
I\,\ddot\theta = \sum \tau
\]

---

### A.3 对轮轴取矩 / 非惯性系（物理方程怎么来）

两种写法结果一样。下面用非惯性系（通常更好懂），并说明为什么要对轮轴取矩。

#### 生活直觉

站在公交车里，车突然往前加速：你会感觉被**往后甩**。
倒立摆也一样：轮轴突然往前加速 \(\ddot s>0\)，质心相对轮轴会感觉有一个**向后的假想力**。
车已往前倾时，这个向后的力会产生**把车扳回直立**的力矩——所以要「朝倒的方向追」。

#### 跟着轮轴平移的非惯性系

建一个坐标系：**原点钉在轮轴上，只随轮轴左右平移，不跟着车身转**。
这个坐标系有加速度 \(\ddot s\)，不是惯性系，牛顿定律要补一项**惯性力（假想力）**：

\[
\vec F_{\text{假想}} = -m\,\ddot s\,\hat x
\]

即：轮轴往前加速时，质点上多一个向后的力 \(-m\ddot s\)。

系里还有真正的力：

- 重力：竖直向下，\(mg\)
- 杆/轴承的约束力：作用在轮轴处

**为什么对轮轴取矩？**
约束力作用点就在轮轴，力臂为 0，力矩为 0，写力矩方程时可以不管它。这正是「对轮轴取矩」的好处。

二维力矩（符号约定：\(\tau>0\) 使 \(\theta\) 增大，即更往前倒）：

\[
\tau = z\,F_x - x\,F_z
\]

质心相对轮轴：\(x_{\text{相对}}=l\sin\theta\)，\(z=l\cos\theta\)。

**重力的力矩**（\(F_x=0\)，\(F_z=-mg\)）：

\[
\tau_g = (l\cos\theta)(0) - (l\sin\theta)(-mg) = mgl\sin\theta
\]

\(\theta>0\) 时 \(\tau_g>0\)：重力让它继续往前倒——不稳定的来源。

**假想力的力矩**（\(F_x=-m\ddot s\)，\(F_z=0\)）：

\[
\tau_{\text{假}} = (l\cos\theta)(-m\ddot s) - (l\sin\theta)(0) = -ml\ddot s\cos\theta
\]

\(\ddot s>0\) 时这项为负：把 \(\theta\) 往回扳。

转动方程：

\[
ml^{2}\,\ddot\theta = mgl\sin\theta - ml\,\ddot s\cos\theta
\]

两边除以 \(ml\)：

\[
l\,\ddot\theta = g\sin\theta - \ddot s\cos\theta
\]

这就是完整非线性方程。在地面固定系里没有假想力，对轮轴写角动量或对质心用牛顿定律再取矩，最终得到同一式——文档写「或在非惯性系里……」只是换一种等价算法。

#### 小角度 → 正文第 1 节的式子

\(\theta\) 很小：\(\sin\theta\approx\theta\)，\(\cos\theta\approx 1\)：

\[
l\,\ddot\theta = g\,\theta - \ddot s
\]

读法：

- 右边第一项 \(g\theta\)：歪一点就继续加速倒
- 右边第二项 \(-\ddot s\)：轮轴往前加速能抵消

令 \(\ddot s=0\)：\(\ddot\theta=(g/l)\theta\)，解是指数发散，特征频率 \(\lambda=\sqrt{g/l}\)。

---

### A.4 控制律从哪来（不是物理）

物理告诉我们：能动手的是 \(\ddot s\)。于是规定（**设计选择**，不是从牛顿定律推出来的）：

\[
\ddot s = k_{a,\theta}\theta + k_{a,\dot\theta}\dot\theta
\]

含义：看倾角和倾角速度，线性决定轮轴加速度——加速度域上的 PD。

代回 \(l\ddot\theta = g\theta - \ddot s\)：

\[
l\ddot\theta + k_{a,\dot\theta}\dot\theta + (k_{a,\theta}-g)\theta = 0
\]

再除以 \(l\)：

\[
\ddot\theta + \frac{k_{a,\dot\theta}}{l}\dot\theta + \frac{k_{a,\theta}-g}{l}\theta = 0
\]

---

### A.5 「标准二阶式」是什么

凡是常系数线性二阶方程

\[
\ddot x + a\,\dot x + b\,x = 0
\]

控制里习惯改写成：

\[
\ddot x + 2\zeta\omega_n\dot x + \omega_n^2 x = 0
\]

其中 \(\omega_n^2=b\)，\(2\zeta\omega_n=a\)。这只是**换一种参数命名**：

- \(\omega_n\)：响应有多「快」（自然频率）
- \(\zeta\)：抖不抖、衰减快不快（阻尼比；\(\zeta<1\) 衰减振荡，\(=1\) 临界，\(>1\) 过阻尼）

对照闭环系数，立刻得到正文第 2 节的框：

\[
\omega_n=\sqrt{\frac{k_{a,\theta}-g}{l}},\qquad
2\zeta\omega_n=\frac{k_{a,\dot\theta}}{l}
\]

反解就是调参用的初值公式。必须 \(k_{a,\theta}>g\)，否则 \(\omega_n^2<0\)，闭环仍不稳定——\(K_p\) 太小会「软倒」的数学原因。

| 式子 | 来源 |
| --- | --- |
| \(l\ddot\theta = g\theta - \ddot s\) | 物理（小角度倒立摆） |
| \(\ddot s = k_{a,\theta}\theta + k_{a,\dot\theta}\dot\theta\) | 设计选择（PD 反馈） |
| \(\ddot\theta+2\zeta\omega_n\dot\theta+\omega_n^2\theta=0\) | 线性二阶 ODE 的通用模板 |
