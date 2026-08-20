---
title: α 的一维卡尔曼滤波：最小可用实现
level: 1
order: 1
---

# α 的一维卡尔曼滤波：最小可用实现

> TWIP 平衡车 · 简化实现卷 · θ 由 IMU 库直出

前提：$\hat{\theta}$ 与 $\dot{\theta}$ 由 IMU 的库函数（或 gz-sim 的 orientation 字段）直接给出，姿态估计不用自己做。剩下的任务被压缩成一件事：**用一维卡尔曼滤波估计 $\alpha$**。状态只有 1 个数，协方差也只有 1 个数，全部滤波公式共 5 行，代码不到 100 行。理论依据见理论卷 §4、§6–8，本卷只保留「怎么做」。

## 00 流水线：每个控制周期做四件事

1. **备料** — 读入 $\hat{\theta}$, $\dot{\theta}$, $f_x$, $f_z$, $v$, $\tau$，算出 $\dot{v}$（带限微分）
2. **观测 $z_{\mathrm{dyn}}$** — 力矩平衡反解，每拍都有效
3. **观测 $z_{\mathrm{kin}}$** — 世界系比力法，只在 $\dot{v}$ 大时有信息
4. **KF 更新** — 预测 + 两次串行量测更新，输出 $\hat{\alpha}$, $P$

一维 KF 的一个好处：多条观测**不需要拼矩阵**，逐条做标量更新即可（数学上与矩阵形式严格等价）。所以 ②③④ 就是三段各几行的代码。

> **先校一个数：** gz-sim 默认重力是 **9.8** 而非 9.81（你已在实测中确认 $f_z \approx 9.7996$）。本卷所有公式中的 $g$ 必须与仿真世界一致，否则 $z_{\mathrm{kin}}$ 带固定偏置。建议在 world 文件里显式写 `<gravity>0 0 -9.81</gravity>`，或节点参数用 9.8。

## 01 备料：五个输入量

| 量 | 来源 | 备注 |
| --- | --- | --- |
| $\hat{\theta}$ | IMU 库 / `orientation` 四元数取俯仰 | 确认符号：前倾为正（理论卷 §1.3） |
| $\dot{\theta}$ | 陀螺 `angular_velocity.y`（符号对齐后） | 只用于杆臂修正，要求不高 |
| $f_x$, $f_z$ | `linear_acceleration` | 静止直立 $f_z = +g$ 已确认 |
| $v$ | $v = r(\omega_L+\omega_R)/2$，来自 `/joint_states` | $r = 0.0325\,\mathrm{m}$；不要用 DiffDrive odom |
| $\tau$ | 控制器上一拍**实际下发**的两轮力矩之和 | 必须含前馈项（理论卷 §7.3） |

$\dot{v}$ 用带限微分（先差分、再一阶低通），时间常数 $\tau_d = 50\,\mathrm{ms}$：

$$
\dot{v}_{f,k} = \dot{v}_{f,k-1} + \frac{\Delta t}{\tau_d}\left(\frac{v_k - v_{k-1}}{\Delta t} - \dot{v}_{f,k-1}\right)
$$

## 02 两条观测：各三行公式

### 2.1 动力学观测 $z_{\mathrm{dyn}}$（稳态主力，每拍有效）

$$
\sin z_{\mathrm{dyn}} = \frac{\tau/r - \tilde{m}\,\dot{v} - c\,v}{M g},\quad \tilde{m} = M + I_w/r^2
$$

$M = 0.756\,\mathrm{kg}$，$r = 0.0325\,\mathrm{m}$。$c$ 先取 0，平地匀速实验后用 $c = \tau/(r v)$ 辨识（理论卷结论 7.1）。摆动耦合项按简化实现省略。

### 2.2 运动学观测 $z_{\mathrm{kin}}$（瞬态补充，$\dot{v}$ 大时才有信息）

杆臂修正 → 用 $\hat{\theta}$ 旋到世界系 → 取竖直残差：

$$
f_z^{c} = f_z + \ell\dot{\theta}^{2},\quad
f_x^{c} = f_x - \ell\ddot{\theta}\,;\quad
f_{Wz} = -f_x^{c}\sin\hat{\theta} + f_z^{c}\cos\hat{\theta}\,;\quad
\sin z_{\mathrm{kin}} = \frac{f_{Wz} - g}{\dot{v}}
$$

$\ell \approx 0.0345\,\mathrm{m}$，$\ddot{\theta}$ 用 $\dot{\theta}$ 差分近似即可（此项本来就小）。除以 $\dot{v}$ 的危险不用门限解决——交给下一节的自适应 $R$。

## 03 一维 KF：全部五行

### 3.1 滤波公式

状态 $x = \alpha$（标量），协方差 $P$（标量）。每个周期：

$$
\begin{aligned}
&\text{预测：}\quad \hat{\alpha}^{-} = \hat{\alpha},\quad P^{-} = P + q\cdot\Delta t \\
&\text{对每条观测 } z\text{（先 dyn 后 kin）串行执行：}\quad
K = \frac{P}{P + R},\quad
\hat{\alpha} \leftarrow \hat{\alpha} + K(z - \hat{\alpha}),\quad
P \leftarrow (1 - K)P
\end{aligned}
$$

预测什么都不做（随机游走：坡不会因为车动而变），只让不确定度 $P$ 随时间涨一点——涨多快由 $q$ 决定，它回答的问题是「环境坡度多快会变」。

### 3.2 灵魂所在：自适应 $R_{\mathrm{kin}}$

$$
R_{\mathrm{kin}} = \frac{\sigma_f^{2}}{\dot{v}^{2}} + \sigma_{\varepsilon}^{2}
\quad（\sigma_f\text{：加计噪声，}\sigma_{\varepsilon}\text{：}\hat{\theta}\text{ 的误差}）
$$

$\dot{v} \to 0$ 时 $R_{\mathrm{kin}} \to \infty$，$K \to 0$，这条观测自动失去话语权——理论卷命题 4.1 的「稳态不可观」就这样被编码进了协方差，**不需要任何 if 门限**。代码里唯一的守护是给 $R_{\mathrm{kin}}$ 设个上限（防浮点溢出）并把 $\mathrm{asin}$ 的自变量夹到 $[-1, 1]$。

### 3.3 参数速查（按你的车与仿真噪声配置给出）

| 参数 | 取值 | 怎么来的 / 怎么调 |
| --- | --- | --- |
| $q$（过程噪声密度） | $7.6\times 10^{-5}\,\mathrm{rad}^{2}/\mathrm{s}$ | 假设坡最快按 $0.5°/\mathrm{s}$ 变化：$q = (0.5°/\mathrm{s})^{2}$。$\hat{\alpha}$ 跟踪太慢就调大，太抖就调小 |
| $R_{\mathrm{dyn}}$ | $3\times 10^{-4}\,\mathrm{rad}^{2}$（$\approx(1°)^{2}$） | 模型误差主导：$M$、$r$ 各 1% 偏差 + 未建摩擦，合计按 $1°$ 记（理论卷式 14） |
| $\sigma_f$ | $0.05\,\mathrm{m}/\mathrm{s}^{2}$ | 与 IMU SDF 里的 stddev 一致（你实测的抖动量级） |
| $\sigma_{\varepsilon}$ | $0.01\,\mathrm{rad}$（$\approx 0.6°$） | IMU 库姿态的典型误差；你实测过静态残差 $\approx 0.6°$，正好用它 |
| $P$ 初值 | $(10°)^{2} \approx 0.03$ | 开机不知道坡度，给大点让前几拍快速收敛 |
| $\tau_d$（$\dot{v}$ 微分） | $0.05\,\mathrm{s}$ | 毛刺大调大，瞬态迟钝调小 |

## 04 完整节点（&lt;100 行）

```python
#!/usr/bin/env python3
# 一维 KF 斜坡估计：θ 由 IMU orientation 直出，只估 α
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32, Float64MultiArray

class SlopeKF1D(Node):
    def __init__(self):
        super().__init__('slope_kf_1d')
        # --- 物理参数 ---
        self.g  = 9.8        # 与 gazebo 世界一致！(实测 f_z=9.7996)
        self.r  = 0.0325
        self.M  = 0.756
        self.Iw = 2.0e-5
        self.l  = 0.0345
        self.c  = 0.0        # 平地匀速实验辨识后回填
        # --- KF 参数（§3.3 速查表）---
        self.q       = 7.6e-5
        self.R_dyn   = 3e-4
        self.sig_f   = 0.05
        self.sig_eps = 0.01
        self.tau_d   = 0.05
        # --- 状态 ---
        self.alpha, self.P = 0.0, 0.03
        self.theta = self.gy = self.gy_prev = 0.0
        self.v = self.v_prev = self.vdot = 0.0
        self.tau = 0.0
        self.t_js = self.t_imu = None

        self.create_subscription(Imu, 'imu/data', self.imu_cb, 100)
        self.create_subscription(JointState, 'joint_states', self.js_cb, 50)
        self.create_subscription(Float64MultiArray,
            'effort_controller/commands', lambda m: setattr(self, 'tau', sum(m.data)), 50)
        self.pub  = self.create_publisher(Float32, 'slope/alpha', 10)
        self.dbg  = self.create_publisher(Float64MultiArray, 'slope/debug', 10)

    def js_cb(self, msg):
        try:
            iL = msg.name.index('left_wheel_joint')
            iR = msg.name.index('right_wheel_joint')
        except ValueError:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t_js is None: self.t_js = t; return
        dt = t - self.t_js; self.t_js = t
        if not (0.0 < dt < 0.1): return
        self.v_prev, self.v = self.v, self.r * 0.5 * (msg.velocity[iL] + msg.velocity[iR])
        self.vdot += (dt / self.tau_d) * ((self.v - self.v_prev) / dt - self.vdot)

    def imu_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t_imu is None: self.t_imu = t; return
        dt = t - self.t_imu; self.t_imu = t
        if not (0.0 < dt < 0.05): return

        # ① 备料：θ 直接取自 orientation（俯仰，绕 y）
        q = msg.orientation
        self.theta = math.asin(max(-1, min(1, 2*(q.w*q.y - q.z*q.x))))
        #   注意符号自检：手动前扳车体，theta 应为正；若为负，此行取反
        self.gy_prev, self.gy = self.gy, msg.angular_velocity.y
        thdd = (self.gy - self.gy_prev) / dt
        fx, fz = msg.linear_acceleration.x, msg.linear_acceleration.z

        # ② 观测一：动力学（每拍有效）
        m_eff = self.M + self.Iw / self.r**2
        s_dyn = (self.tau / self.r - m_eff * self.vdot - self.c * self.v) / (self.M * self.g)
        z_dyn = math.asin(max(-0.6, min(0.6, s_dyn)))

        # ③ 观测二：运动学 + 自适应方差
        fxc = fx - self.l * thdd
        fzc = fz + self.l * self.gy**2
        fWz = -fxc * math.sin(self.theta) + fzc * math.cos(self.theta)
        vd  = self.vdot if abs(self.vdot) > 1e-4 else 1e-4   # 仅防除零
        z_kin = math.asin(max(-0.6, min(0.6, (fWz - self.g) / vd)))
        R_kin = min((self.sig_f / vd)**2 + self.sig_eps**2, 1e6)

        # ④ 一维 KF：预测 + 两次串行标量更新
        self.P += self.q * dt
        for z, R in ((z_dyn, self.R_dyn), (z_kin, R_kin)):
            K = self.P / (self.P + R)
            self.alpha += K * (z - self.alpha)
            self.P *= (1.0 - K)

        self.pub.publish(Float32(data=float(self.alpha)))
        d = Float64MultiArray()
        d.data = [self.theta, self.v, self.vdot, z_dyn, z_kin, self.alpha, self.P]
        self.dbg.publish(d)

def main():
    rclpy.init()
    rclpy.spin(SlopeKF1D())

if __name__ == '__main__':
    main()
```

与流水线的对应：`js_cb` = §1 备料，`imu_cb` 的 ①②③④ 与 §0 四格一一对应。调试话题 `/slope/debug` 里同时给出两条原始观测与 $P$，PlotJuggler 里把 $z_{\mathrm{dyn}}$、$z_{\mathrm{kin}}$、$\hat{\alpha}$ 三条画在一起，滤波器在「信谁」一目了然。

## 05 三个验收检查

1. **平地静止**：$\hat{\alpha}$ 收敛到 $0\pm 0.5°$，$P$ 收敛到 $R_{\mathrm{dyn}}$ 量级。若有固定偏差，先查 $g$ 是否与世界一致（9.8 vs 9.81 恰好差出约 $0.06°$——比它大的偏差多半是 $\tau$ 话题或 $\theta$ 符号问题）。
2. **平地加减速往返**：$z_{\mathrm{kin}}$ 曲线应围绕 0 剧烈但对称地摆动，$\hat{\alpha}$ 几乎不动（说明自适应 $R$ 在正确加权）；$z_{\mathrm{dyn}}$ 若随加减速起伏，说明 $\dot{v}$ 与 $\tau$ 的滤波延迟不匹配（对 $\tau$ 也做同 $\tau_d$ 的低通）。
3. **上坡→坡上停住 5 s**：$\hat{\alpha}$ 在 1~2 s 内爬到 $10°$ 附近，**停住期间不塌回**（动力学观测在撑着它）；把代码里 $z_{\mathrm{dyn}}$ 那条更新注释掉重跑，停住期间 $\hat{\alpha}$ 应缓慢滑向 0 且 $P$ 持续增大——亲眼看一次「不可观测」长什么样，比任何推导都记得牢。

> **之后往哪走：** 这套一维 KF + 库函数 $\hat{\theta}$ 的组合已经够支撑前馈 $\tau_{\mathrm{ff}} = M\cdot g\cdot\sin(\hat{\alpha})\cdot r$ 与平衡点修正 $\sin\theta_{\mathrm{eq}} = (Mr/m_b L)\cdot\sin\hat{\alpha}$。如果之后发现 $\hat{\theta}$ 的库函数在急加减速时被污染（理论卷式 6），再考虑升级到联合 EKF（理论卷 §8.3），把 $\theta$ 和 $\alpha$ 放进同一个滤波器。

---

简化实现卷 · 前提：$\hat{\theta}$ 由 IMU 库直出。参数：$r = 0.0325\,\mathrm{m}$，$M = 0.756\,\mathrm{kg}$，$\ell \approx 0.0345\,\mathrm{m}$，$g = 9.8$（与 gz-sim 默认一致，实测 $f_z = 9.7996$）。理论依据与推导见理论卷 §4（可观测性）、§6–7（两条观测）、§8.2（本 KF）。
