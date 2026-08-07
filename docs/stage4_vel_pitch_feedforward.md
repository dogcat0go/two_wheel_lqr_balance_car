# 阶段 4：速度→倾角前馈（低速走不动的解法）

> 对应实现：`src/stage2_main.cpp`（`ref.pitch = trim + kff·vref_smooth`、`vref_smooth` 斜率限幅）、`include/config.h`（`kGainVelToPitch`、`kVelSlewMps2`）  
> 关联：[`stage4_velocity_loop.md`](stage4_velocity_loop.md)、[`balance_jitter_tuning.md`](balance_jitter_tuning.md)、[`stage34_gain_init_theory.md`](stage34_gain_init_theory.md)

---

## 1. 要解决的现象

给 `v 0.2/0.3`，车几乎不动，或极慢蠕动（实测 ~0.01 m/s，目标 0.2，差 20 倍）。  
遥测特征（`terminals/18.txt`）：

```text
u pit=+25   pos=-25(anti-windup 封顶)   vel=-3.0   → 净 u≈-3%   eff≈±2%
pitch 被顶到 5~7°（远超 trim 3.23°），x/ticks 仅缓慢增长
```

---

## 2. 根因：位置项与姿态项互相抵消

当前是**单环状态反馈**：

\[
u = k_\theta(\theta-\theta_{ref}) + k_{\dot\theta}(\dot\theta-\dot\theta_{ref})
  + k_s(s-s_{ref}) + k_{\dot s}(\dot s-\dot s_{ref})
\]

低速给定 `vref>0` 时：

- `vel` 项 `k_{\dot s}(v-vref)` 恒为**负**（NMP，先反向），一直拖后腿  
- 车跟不上 `pos_ref` → `pos` 项 `k_s(s-pos_ref)` 也是**负**（把车往回拉）  
- 这两个负项抵消掉前倾产生的正 `u pit`，净输出卡在电机死区（~4~6%）内 → 轮子不转  

于是姿态环只能把 `pitch` 越顶越高来强行补偿，形成「僵持蠕动」。

**这跟控制率（都 200 Hz）无关，是结构问题**：`pos` 项是「拉回」负反馈，方向与「想前进」相反。换成 LQR（同样 `u=−Kx`、同样 200 Hz）也不会自动消除，且 LQR 线性模型不含死区这种非线性。

### 两难

| `pos` 项处理 | 结果 |
| --- | --- |
| 不限幅 | 积分涨到压倒一切 → 突然爆冲、过冲摔倒（`v 0.5`） |
| anti-windup 封顶 | 封顶值正好与 `u pit` 打平 → 僵持蠕动（现状） |

单靠 `pos` 项调不出低速平顺，因为它的角色是「校正」，不是「驱动」。

---

## 3. 解法：速度→倾角前馈（级联思想）

平衡车前进的物理本质是**先前倾、让姿态环把车往前推**。所以给 `vref` 时，直接把目标倾角抬一点：

\[
\boxed{\theta_{ref} = \theta_{trim} + k_{ff}\cdot v_{ref}}
\]

- 平衡点本身移到「前倾走」的位置，`u pit` 持续输出把车推过死区  
- **不再依赖 `pos` 项累积去抵消**；`pos`/`vel` 退居校正与阻尼  
- 这就是「速度外环写姿态内环参考」的级联最小实现——你想要的「位置/姿态时标分离」的正确落地方式（靠参考耦合，不是靠改采样率）

### 符号

本车已确认：**前倾 `pitch` 为正**（后倾为负），且 `k_θ>0`（前倾出正输出=前进）。  
要前进（`vref>0`）就要更前倾 → `θ_ref` 增大 → **`k_ff > 0`**。后退自动对称（`vref<0` → 后仰）。

### 与反馈项的分工

| 项 | 前馈引入后的角色 |
| --- | --- |
| 前馈 `kff·vref` | **主动驱动**：给定速度就前倾，快速顶过死区 |
| `vel` 项 | 速度阻尼 / 抑制超调 |
| `pos` 项 | 位置校正 + anti-windup 兜底（保留现有限幅） |
| pitch/pitch_rate | 姿态内环，照旧 |

---

## 4. 参数与调法

| 参数 | 位置 | 含义 | 单位 |
| --- | --- | --- | --- |
| `kGainVelToPitch` | `config.h` | \(k_{ff}\) | rad/(m/s) |
| 串口 `g <deg_per_mps>` | 运行时 | 每 1 m/s 前倾多少**度**（内部转 rad） | deg/(m/s) |

调参步骤：

1. 站稳、`v 0`，先确认平衡不受影响（`vref=0` 时前馈为 0，等价原行为）  
2. `g` 从小往上：先 `g 5`（5°/(m/s)）→ `v 0.2`，看 `v` 是否离开 0、`pitch_ref` 抬没抬  
3. 逐步加大 `g`，直到 `v 0.2/0.3` 能较快达到目标速度  
4. **过大症状**：给速瞬间猛前倾、冲一下甚至过冲 → 回调 `g`  
5. 定稿写回 `kGainVelToPitch`

经验起点：低速需前倾 2~3° 才顶过死区；**实车定稿 `g=10`（=10°/(m/s) = 0.1745 rad/(m/s)）**，已写回 `kGainVelToPitch`。

### 安全限幅

前馈后的 `θ_ref` 会 clamp 到 `±kFfPitchLimitRad`（默认 ~15°），防手滑给大 `vref` 把参考角顶到接近摔倒角。

---

## 5. 软停 / 软起步：速度目标斜率限幅

### 要解决的现象

松杆（`v→0`）后车身剧烈抖、收敛慢。根因不是前馈本身，而是：

1. `linear_x` 阶跃到 0 → `ref.pitch` 前馈瞬间归零、`pos_ref` **立刻锁到当前位**  
2. 车还带着前进速度和前倾姿态，冲过锁定点  
3. `pos` 项把它猛拽回来 → 往复振荡，再叠死区极限环 → 「停下来后抖」

### 解法

引入斜率限幅后的 `vref_smooth`，每拍以 `kVelSlewMps2 · dt` 逼近 `cmd.linear_x`，**三处都用它**：

| 用途 | 效果 |
| --- | --- |
| `pos_ref += vref_smooth · dt` | 停止时位置参考跟着滑行衰减，不急锁 |
| `ref.vel = vref_smooth` | 速度反馈目标平滑，无阶跃 |
| `ref.pitch` 前馈用 `vref_smooth` | 倾角目标斜坡抬/落，无猛前倾/猛回正 |

只有 `vref_smooth` 与指令都接近 0 后，才 `pos_ref = x.pos` 锁零点。故障/切模式时 `vref_smooth` 清零。

### 调参

| 参数 | 含义 | 当前值 |
| --- | --- | --- |
| `kVelSlewMps2` | 速度目标加速度限幅 (m/s²) | 1.0 |

- 停得太肉 / 滑行太远 → 调大（如 2.0）  
- 停得还急、抖 → 调小（如 0.5）  
- 例：`v=0.2`、`slew=1.0` → 约 0.2 s（40 拍@200Hz）滑行到停  

实车：软停上线后**停止后抖动明显改善**（与 [`balance_jitter_tuning.md`](balance_jitter_tuning.md) 的死区极限环是两类问题；软停治的是「松杆回拉振荡」）。

---

## 6. 风险与边界

- **前馈是开环**：只保证「倾过去」，稳态速度精度仍靠 `vel`/`pos` 反馈收尾  
- **过冲**：`kff` 过大仍会冲；`vref` 阶跃已由 §5 斜率限幅缓和  
- **死区仍在**：前馈让「顶过死区」变容易，但没消除死区本身；死区标定该做还得做  
- **上坡/负载**：前馈值是平地经验，坡道另需坡度前馈（阶段 6）

---

## 7. 与阶段 5 的关系

- 前馈+反馈+软停这套结构**保留**到阶段 5：LQR 只替换 4 个反馈增益为 `−K`，`kff` 与 `vref_smooth` 作为参考成形继续留着  
- 手调出的 `kff`、`kvel`、`kpos` 是 LQR 结果的对照基线  
- 真正切 LQR 前仍需 τ→PWM 标定与物理参数实测（见 [`motor_torque_pwm_calibration.md`](motor_torque_pwm_calibration.md)、[`Sim2Real_list.md`](Sim2Real_list.md) 阶段 5）

---

## 8. 代码锚点

| 文件 | 内容 |
| --- | --- |
| `include/config.h` | `kGainVelToPitch`、`kFfPitchLimitRad`、`kVelSlewMps2` |
| `include/shared_state.h` | `CommandInput.k_vff` |
| `src/stage2_main.cpp` | `vref_smooth` 斜率限幅；`ref.pitch/vel` 与 `pos_ref` 积分均用它；串口 `g` |
