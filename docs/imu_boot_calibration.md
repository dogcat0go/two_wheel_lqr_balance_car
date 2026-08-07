# 上电 IMU 标定与 trim：为何姿势会污染倾角

> 对应实现：`lib/Ahrs/Ahrs.cpp`（`calcOffsets(true, false)`）  
> 相关：[`stage3_balance_tuning.md`](stage3_balance_tuning.md)（机械 trim）、[`Sim2Real_list.md`](Sim2Real_list.md) 阶段 1 AHRS

---

## 1. 你观察到的现象

烧录/上电时若车体略有倾斜，随后平衡往往要重新拧串口 `t`（或改 `kPitchTrimDeg`）。  
容易误判为「机械平衡角每次都变了」。

多数情况下**机械 trim 没变**，变的是：**加速度零偏把「标定瞬间的重力」写进了传感器坐标系**，AHRS 的 pitch 零点跟着上电姿势跑，控制器只好用 `t` 去补这个虚假零点。

---

## 2. 两套「角度」不要混

| 名字 | 含义 | 谁决定 | 上电姿势该不该影响 |
| --- | --- | --- | --- |
| **机械 trim** \(\theta^\*\) | 质心在轮轴正上方时，车体相对竖直的几何角 | 配重、电池位置、IMU 安装角 | **否**（硬件不动就不变） |
| **AHRS pitch** \(\hat\theta\) | 固件估计的倾角（rad） | 加速度计重力方向 + 陀螺积分 + 互补滤波 | 标定方式错误时 **会** |

控制器用的是：

\[
u \supset k_\theta\,(\hat\theta - \theta_{\mathrm{ref}}),\quad \theta_{\mathrm{ref}} = \texttt{trim}
\]

若 \(\hat\theta\) 的零点漂了 \(\delta\)，看起来就像 trim 要改 \(\delta\)。根因在估计，不在 \(k_\theta\)。

本车 \(l\) 很短，\(\delta\) 差零点几度就会明显单向爬行，所以「每次拧一点 t」特别显眼。

---

## 3. 加速度计如何给出 pitch

静止（或低频）时，比力主要是重力。对本车采用的公式（见 `Ahrs::update`）：

\[
\hat\theta_{\mathrm{acc}}
= s_\theta\cdot
\bigl(-\operatorname{atan2}(a_x,\sqrt{a_z^2+a_y^2})\bigr)
\]

其中 \(s_\theta=\) `kPitchSign`。  
**关键点：** \(\hat\theta_{\mathrm{acc}}\) 是「当前重力在 IMU 坐标系里的方向」，与「上次标定姿势」无关——**前提是 \(a\) 没有被错误地减掉姿势相关的 offset**。

互补滤波（`kPitchGyroCoef` ≈ 0.98）：

\[
\hat\theta
\leftarrow \alpha\,(\hat\theta + \dot\theta_{\mathrm{gyro}}\,\Delta t)
+ (1-\alpha)\,\hat\theta_{\mathrm{acc}}
\]

- 高频跟陀螺，低频被拉回 \(\hat\theta_{\mathrm{acc}}\)  
- 收敛后 \(\hat\theta \approx \hat\theta_{\mathrm{acc}}\)（慢变意义下）  
- 因此：**加速度计对重力的读法错了，整条 pitch 管道的零点都会错**

上电瞬间用 \(\hat\theta_{\mathrm{acc}}\) 赋初值，只是加快收敛；**稳态零点仍由加速度（及其 offset）决定**，不是由初值姿势「锁死」的。

---

## 4. 旧标定为何会「记住」上电倾角

`MPU6050_light::calcOffsets()` 默认 **陀螺 + 加速度** 一起标。加速度部分等价于：

\[
a_{\mathrm{offset}} \leftarrow \overline{a}_{\mathrm{calib}}
\qquad\Rightarrow\qquad
a_{\mathrm{used}} = a_{\mathrm{raw}} - a_{\mathrm{offset}}
\]

标定瞬间车体若相对「芯片理想水平/安装参考」有倾角 \(\phi\)，重力在 \(x\) 轴已有分量 \(g\sin\phi\)。  
库把它整段减进 offset 后：

- 在**标定姿势**下，\(a_{\mathrm{used}}\approx 0\) → \(\hat\theta_{\mathrm{acc}}\approx 0\)  
- 你扶到**真正的机械平衡角** \(\theta^\*\) 时，读数变成「相对标定姿势的差」，而不是相对竖直/安装基准的绝对倾角  

于是：

\[
\hat\theta\Big|_{\text{真直立}}
\approx \theta^\* - \phi_{\mathrm{boot}}
\quad(+\;\text{安装/缩放常数})
\]

每次上电 \(\phi_{\mathrm{boot}}\) 不同 → 同一机械姿态下 \(\hat\theta\) 不同 → 固定 `trim` 不够用 → 表现为「又要拧 t」。

陀螺零偏标定**需要**静止（平均角速度 → bias），与当时倾角无关：陀螺测的是角速度，不是重力方向。把「必须静止」和「必须水平」绑在一起，是把两种标定混为一谈了。

---

## 5. 修改后：轻微倾斜还会不会影响初始化？

固件改为：

```cpp
imu_.calcOffsets(true, false);  // 只标陀螺；加速度 offset 保持 0（或日后写死的厂标）
```

### 5.1 会改善什么

| 项目 | 只标陀螺之后 |
| --- | --- |
| 上电时车略倾 \(\phi_{\mathrm{boot}}\) | **不再**写入加速度 offset |
| 稳态 \(\hat\theta\) | 由当前重力方向决定，同一机械姿势 → 相近 pitch |
| 机械 `trim` | 可定稿为常数；不必每次烧录重拧 |

因此：**以后轻微上电倾斜，不应再系统性污染 pitch 的零点。**  
这正是你问的那句话的答案：**对，按这个改法，目的就是让上电微倾不再绑架倾角零点。**

### 5.2 仍会受影响的（预期内，不是回归）

1. **初值**仍用标定结束那一刻的 \(\hat\theta_{\mathrm{acc}}\)  
   - 若上电时倾得厉害，开头几秒 \(\hat\theta\) 从大角度往真值收敛  
   - 应用层应：标定期间尽量扶稳；**收敛后再 `m 1`**，不要在滤波还在爬的时候开环切平衡  

2. **标定期间必须静止**（陀螺 bias）  
   - 手抖 → 陀螺零偏不准 → \(\dot\theta\) 有常偏 → 积分慢漂，直到加速度拉回  
   - 这与「倾角大小」是另一类问题  

3. **加速度计本身的固定零偏 / 安装不正**  
   - 只标陀螺后，芯片固有 \(a\)-bias 仍在；它造成的是**固定**的 pitch 偏置  
   - 用一次机械 `trim`（或水平台一次性格定 acc offset 写死）吸收即可，**不应随上电姿势变**  

4. **运动加速度**  
   - 加减速时 \(\hat\theta_{\mathrm{acc}}\) 被线加速度污染；靠 \(\alpha\) 偏陀螺抑制  
   - 与上电标定无关  

5. **温度漂移**  
   - 陀螺 bias 随温度变；长时间运行可比上电再标一次陀螺，仍不要重标加速度姿势  

### 5.3 一句话对照

```text
旧：上电姿势 → 写进 acc offset → pitch 零点跟着跑 → 每次拧 t
新：上电姿势 → 只影响初值与短暂收敛 → 稳态 pitch 跟真姿态 → t 定稿为机械角
```

---

## 6. 和 trim / 积分 / LQR 的分工

| 手段 | 解决什么 | 解决不了什么 |
| --- | --- | --- |
| 只标陀螺 | 上电姿势污染 \(\hat\theta\) 零点 | 质心本来就不在几何零位 |
| `kPitchTrimDeg` / `t` | 机械 \(\theta^\*\)、安装角、固定 acc bias | AHRS 零点每次乱漂（应先修标定） |
| 小 `ki`（pitch 积分） | 慢变残差、温漂 | 错误的 acc 姿势标定；且遥控体验要防 windup |
| LQR | 在**正确状态估计**下优化增益 | 估角零点错时同样要错的 \(\theta_{\mathrm{ref}}\) |

建议顺序：修上电标定 → 同一姿势多次上电确认 `pitch` 复现 → 再定稿 `t` → 再谈阶段 5。

---

## 7. 实车验收（改完后做一次）

1. 车体故意前倾约 5°，上电等到 `IMU ok`  
2. 扶到你认为的机械直立，看遥测 `pitch`（度）  
3. 断电，改成后倾约 5° 再上电，再扶到同一机械直立  
4. **期望：** 两次直立时 `pitch` 相差应很小（典型亚度级；视噪声与扶姿重复性）  
5. **旧固件期望：** 两次直立 `pitch` 可能差出与上电倾角同量级的数，并伴随「又要拧 t」  

若验收仍差很多：查安装松动、`kPitchSign`、I2C 读数、是否仍调用了带加速度的 `calcOffsets()`。

---

## 8. 可选：水平台一次性格定加速度

若芯片 acc bias 明显，可在**已知水平（或已知安装角）**的夹具上：

1. `calcOffsets(false, true)` 或手写平均，得到固定 `accX/Y/Zoffset`  
2. 写入 NVS / `config.h`，上电只 `setAccOffsets(...)` + `calcGyroOffsets()`  

这与「每次上电按当前姿势标 acc」是反义词：前者是**绝对基准**，后者是**相对本次姿势的假零点**。

---

## 9. 代码锚点

| 文件 | 内容 |
| --- | --- |
| `lib/Ahrs/Ahrs.cpp` | `calcOffsets(true, false)`；pitch 初值与互补滤波 |
| `include/config.h` | `kPitchSign`、`kPitchGyroCoef`、`kPitchTrimDeg` |
| `src/stage2_main.cpp` | `ref.pitch = pitch_ref_rad`（串口 `t`） |
