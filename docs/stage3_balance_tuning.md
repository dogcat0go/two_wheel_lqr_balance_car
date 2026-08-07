---
title: 阶段 3 姿态环调参记录
level: 1
order: 11
sections: true
---

# 阶段 3 姿态环调参记录

> 目标：只用最内层姿态环站起来。速度目标恒 0，不接 `cmd_vel`。
> 过关标准（`docs/Sim2Real_list.md` 阶段 3）：平地独立站立 >10s，轻推能恢复。
>
> 落点：**扩展** `src/stage2_main.cpp`（不新建 stage3 文件）。控制器为
> `lib/BalanceControl`，写成**状态反馈**形式，使阶段 3（PID）/ 阶段 4（速度环）/
> 阶段 5（LQR）成为同一个类填不同增益，而不是三套 backend。
> 增益经串口在线改，掉电不保存，定稿后写回 `include/config.h`。
>
> 前置：阶段 2 验收全部通过（见 [`stage2_safety_bench.md`](stage2_safety_bench.md)）。
> 增益初值与范围见 [`stage34_gain_init_theory.md`](stage34_gain_init_theory.md)；
> 姿态环物理推导详解见 [`balance_gain_theory.md`](balance_gain_theory.md)。

---

## 控制律：统一状态反馈 {#law}

\[
u=\sum_i k_i\,(x_i-x_{\mathrm{ref},i})\;+\;k_{i\theta}\!\int(\theta-\theta_{\mathrm{ref}})\,dt,
\qquad x=[\theta,\;\dot\theta,\;s,\;\dot s]
\]

| 状态 | 来源 | 单位 |
| --- | --- | --- |
| \(\theta\) | `Ahrs::pitch()`，**正 = 车体前倾** | rad |
| \(\dot\theta\) | `Ahrs::pitchRate()`，陀螺实测 | rad/s |
| \(s\) | `WheelSensor::position()` 两轮均值 | m |
| \(\dot s\) | `WheelSensor::speed()` 两轮均值 | m/s |

\(u\) 当前是占空比（%），阶段 5 接 τ→PWM 后换成 N·m；两轮同量，阶段 3 无差速。

### 各阶段只是增益表不同 {#gain-map}

| 阶段 | \(k_\theta\) | \(k_{\dot\theta}\) | \(k_s\) | \(k_{\dot s}\) | \(k_{i\theta}\) |
| --- | --- | --- | --- | --- | --- |
| 3 姿态环 | \(K_p\) | \(K_d\) | 0 | 0 | 0 或极小 |
| 4 速度/位置环 | \(K_p\) | \(K_d\) | 可选 | \(K_v\) | 小 |
| 5 LQR | \(-K[0]\) | \(-K[1]\) | \(-K[2]\) | \(-K[3]\) | 0 |

阶段 4 若仍想走级联，让外层速度环把结果写进 `ref.pitch` 即可，本结构不排斥级联。

### 两个符号陷阱 {#sign}

**一、平衡环不是常规负反馈。** 倒立摆要「朝倒的方向追」——前倾（\(\theta>0\)）时轮子必须朝**前**转，
所以 \(k_\theta>0\)。本类约定 \(k\) 就是 \(\partial u/\partial x\)、**已含符号**；
而 LQR 的解是 \(u=-Kx\)，填进来要**取负**（见上表）。

**二、\(\dot\theta\) 用陀螺实测，不要用 \(\theta\) 差分。** 200Hz 下差分会把 pitch 噪声放大 200 倍。

这也是不复用 `lib/PidController` 的原因：它的 D 是内部差分（用不上），
且返回 `kp*(target-current)`，符号与平衡环相反，每次都要在外面取负。
它目前已不被任何参与编译的 env 引用（仅归档 `test/back/main.cpp` 提及），
阶段 3 不再引用；等阶段 4 确认速度环也走状态反馈后一并删除。

### 接口 {#api}

```cpp
struct BalanceState { float pitch, pitch_rate, pos, vel; }; // SI

class BalanceController {
public:
    struct Gains { float k_pitch, k_pitch_rate, k_pos, k_vel, ki_pitch; }; // 已含符号
    void  setGains(const Gains& g);
    void  setRef(const BalanceState& ref);   // 阶段3 只用 ref.pitch = trim
    void  setLimits(float out_abs, float integ_abs);
    float update(const BalanceState& x, float dt_s);
    void  reset();                            // 清积分，故障恢复时调
    const float* terms() const;               // [θ,θ̇,s,ṡ,∫] 分量，供遥测判振源
};
```

---

## 阶段 3 参数改动 {#params}

在 `include/config.h`，下列改动**已落地**：

| 参数 | 阶段 2 值 | 现值 | 已改 | 原因 |
| --- | ---: | ---: | :-: | --- |
| `kMaxEffortSlew` | 400 %/s | 12000 | [x] | **最关键**：本车倒塌特征时间仅 33 ms，3000 %/s 走完 −60→+60 要 40 ms，限幅自身就成主导滞后 |
| `kMaxEffort` | 100 % | 60 | [x] | 调参初期限幅防炸机，站住后再放开 |
| `kFallHoldMs` | 50 ms | 20 | [x] | 摔倒时更快切断，保护电机/减速箱 |
| `kWheelRadiusM` | 0.0325 | 0.0375 | [x] | 实测轮半径 3.75 cm |
| `kComHeightM` | — | 0.012 | [x] | 质心到轮轴，由质量分布推算，见 [理论第 9 节](balance_gain_theory.md#this-robot) |
| `kPitchTrimDeg` | — | 2.7（Step 3 实测，度） | [x] | 与串口 `t` 同单位；重心不在轮轴正上方，静止直立 pitch ≠ 0 |
| `kGainPitch` | — | 200 | [x] | \(k_\theta\) 起步值，见 Step 2 |
| `kGainPitchRate` | — | 4.4 | [x] | \(k_{\dot\theta}=0.022\,k_\theta\)，见 Step 1 |
| `kGainIntegPitch` | — | 0（在线拧） | [x] | \(k_{i\theta}\)，见 Step 4 |
| `kGainPos` / `kGainVel` | — | 0 | [x] | 阶段 3 不用，阶段 4 才填 |
| `kIntegTermLimit` | — | 15 % | [x] | 积分项限幅，防 windup |

**摔倒 latch 已实现**：`Safety::setLatch()`，仅在 `m 1` 平衡模式启用——故障后锁死输出，
扶正也不自动恢复，必须发 `r`。`m 0` 开环模式保持阶段 2 的自恢复语义。

增益初值怎么定见 [`balance_gain_theory.md`](balance_gain_theory.md#this-robot)。本车结论：
\(l\approx12\ \mathrm{mm}\)（质心几乎贴轮轴）→ \(\lambda\approx30\ \mathrm{rad/s}\)、
\(T_d=0.022\)，起步 \(k_\theta=200\)、\(k_{\dot\theta}=4.4\)。

> **预期行为**：姿态环单独无法渐近稳定（\(k_\theta\) 在特征多项式里精确抵消，
> 恒有一个正实根），所以车会站住但**持续朝一个方向跑**。这是阶段 3 的正常现象，
> 不要试图靠拧 \(k_\theta\)/\(k_{i\theta}\) 消掉，真正的解是阶段 4 的 \(k_{\dot s}\)。
> 推论见 [理论 §9](balance_gain_theory.md#pitch-only)。

---

## 串口命令 {#cli}

沿用 [`tools/stage2_serial_bridge.py`](../tools/stage2_serial_bridge.py)：**收发分终端 + 主机落盘**。
IDE 的 `device monitor` 不落盘、且会独占串口，调参前先关掉。

### 落盘流程（每步换一个文件）{#logging}

```bash
# 0. 烧录（仅改固件时）
pio run -e stage2 -t upload

# 1. 终端 A：只收遥测 + 落盘（本机串口 /dev/ttyUSB0）
#    不写 -l 时默认 logs/stage2_YYYYMMDD_HHMMSS.log
~/.platformio/penv/bin/python3 tools/stage2_serial_bridge.py \
  -p /dev/ttyUSB0 -l logs/stage3_step0.log

# 2. 终端 B：只发命令（另开，经 FIFO /tmp/fishbot_cmd，不抢串口）
echo 'm 1' > /tmp/fishbot_cmd
echo 'f 20' > /tmp/fishbot_cmd   # 判振荡时先压到 20ms
```

| 开关 | 作用 |
| --- | --- |
| `-p /dev/ttyUSB0` | 串口设备（脚本默认已是此口；换口时改这里） |
| `-l PATH` | 指定日志文件；建议按 Step 命名，如 `logs/stage3_step1.log` |
| （默认） | 不写 `-l` → `logs/stage2_时间戳.log` |
| `--no-log` | 只看终端，不写文件 |
| `-f /tmp/fishbot_cmd` | 发令 FIFO（与串口无关，别写成 ttyUSB0） |

日志约定（脚本已实现）：

- 每行带**主机时间戳**（毫秒）：`2026-08-06 10:54:12.345  m=1 hz=200 ...`
- 发出的命令记为 `>> m 1`，与 RX 同文件，事后能还原「何时发了什么」
- 终端 A 的屏幕打印**不加**时间戳，只文件里有
- 换 Step 时 **Ctrl+C 停掉桥 → 换 `-l` 路径再开**，避免多步混在一个文件里

填表时「日志文件」栏写相对路径即可（如 `logs/stage3_step1.log`）。

### 命令表

| 命令 | 作用 |
| --- | --- |
| `m 0` / `m 1` | 模式：0 = 阶段 2 开环，1 = 平衡模式 |
| `p <v>` | 设 \(k_\theta\)（`kGainPitch`） |
| `d <v>` | 设 \(k_{\dot\theta}\)（`kGainPitchRate`） |
| `i <v>` | 设 \(k_{i\theta}\)（`kGainIntegPitch`） |
| `t <deg>` | 设 \(\theta_{\mathrm{ref}}\)（度，内部转 rad） |
| `r` | 清故障 latch + 清积分，重新使能 |
| `s` | 立即停机（\(u=0\) 并回 `m 0`） |
| `e <duty>` | 开环占空比（仅 `m 0`，阶段 2 遗留） |
| `f <ms>` | 遥测周期，下限 20（115200 波特的发送时间下限） |
| `x` / `o` | 模拟断链 / 恢复 |

默认 200 ms 遥测只有 5 Hz 采样，**看不见任何振荡**。判振荡频率时先发 `f 20`（50 Hz 采样，
可分辨到 25 Hz），定完参数再发 `f 200` 回来。

阶段 4 再加设 \(k_{\dot s}\) / \(k_s\) 的命令，命令表同构扩展即可。

遥测格式（每 `kTelemetryMs` 一行）：

```text
m=1 hz=200 ovr=0 fault=0x00 | pitch=1.70 ref=0.00 deg rate=0.01 | u p=12.6 d=0.3 i=0.0 | effort=12.9/12.9% v=0.000/0.000
```

`u p=/d=/i=` 是 `terms()` 的分量，用来判断是哪一项在振荡。

---

## 调参步骤 {#steps}

每步做完填「现象」和「日志文件」，便于回溯。
**每步开始前按 [落盘流程](#logging) 开新 log**（`-l logs/stage3_stepN.log`），再发该步的串口命令。

### Step 0：符号验证（轮子离地，必做）{#step0}

```bash
# 终端 A
~/.platformio/penv/bin/python3 tools/stage2_serial_bridge.py \
  -p /dev/ttyUSB0 -l logs/stage3_step0.log
# 终端 B
echo 'm 1' > /tmp/fishbot_cmd
echo 'p 0' > /tmp/fishbot_cmd
echo 'i 0' > /tmp/fishbot_cmd
echo 'd 35' > /tmp/fishbot_cmd   # 只要能看见轮子跟手转即可，不必精确
```

支架架空。手动前后倾斜车体：

- [ ] 前倾（遥测 pitch 变正）时，轮子朝**车前进**方向转
- [ ] 后倾时轮子朝后转

| 项 | 结果 |
| --- | --- |
| 前倾 → 轮向 | |
| 日志文件 | `logs/stage3_step0.log` |

**不通过绝不要落地。** 先查 `kPitchSign`（pitch 正负）与 `kWheelDir`（+effort 是否 = 前进），
两者任一反了都会变成「加速倒下」。

### Step 1：只调 \(k_{\dot\theta}\)（仍然离地）{#step1}

目标：摸清陀螺噪声能撑住的阻尼上限，再取安全工作点。
为什么必须先 Kd、且离地，见 [理论](balance_gain_theory.md#why-kd-first)。

```bash
# 终端 A（新文件）
~/.platformio/penv/bin/python3 tools/stage2_serial_bridge.py \
  -p /dev/ttyUSB0 -l logs/stage3_step1.log
# 终端 B
echo 'm 1' > /tmp/fishbot_cmd
echo 'p 0' > /tmp/fishbot_cmd
echo 'i 0' > /tmp/fishbot_cmd
echo 'f 20' > /tmp/fishbot_cmd          # 必须：默认 200ms 看不见抖
echo 'd 4.4' > /tmp/fishbot_cmd         # 本车起步值（= 0.022 × 200）
```

**操作：** 支架架空，手握住车体缓慢前后摇晃（约 1～2 Hz），感受阻力；
同时看终端 / 日志里的 `rate=` 与 `u d=`。

**手感对照：**

| 手感 | 含义 | 下一步 |
| --- | --- | --- |
| 几乎没阻力，像空转 | \(k_{\dot\theta}\) 太小 | 按约 ×1.5 加大 `d` |
| 摇时明显「发粘 / 有阻尼」，松手后晃动能较快消掉 | 合适区 | 继续小幅加大，找起抖点 |
| 静止也嗡嗡抖，或 `rate`/`u d` 高频乱跳 | 已触噪声上限 | 记下当前 `d` 为 \(k_{\dot\theta,\mathrm{osc}}\) |

**建议加档顺序**（不必死跟，约 ×1.5）：

`4.4 → 6.6 → 10 → 15 → 22 → 33 → 50 …` 直到起抖。

**如何从日志确认「起抖」**（比纯手感可靠）：

1. 车体尽量静止扶住，发 `f 20`
2. 看连续多行：`rate` 与 `u d=` 是否在**无人为晃动**时仍大幅正负跳动
3. 真正的噪声振荡通常是十几～几十 Hz 的「嗡」，不是慢悠悠的前后摆
4. 刚出现这种嗡的那档 `d`，记为 \(k_{\dot\theta,\mathrm{osc}}\)

**取工作值：**

\[
k_{\dot\theta} = \frac{k_{\dot\theta,\mathrm{osc}}}{2}\sim\frac{k_{\dot\theta,\mathrm{osc}}}{3}
\]

偏保守取 `/3`（本车噪声大、安装软时）；手感干净可取 `/2`。
定完后发 `f 200` 把遥测压回去（省串口带宽）。

- [ ] 找到 \(k_{\dot\theta,\mathrm{osc}}\)
- [ ] 写下工作值，并确认静止时 `u d` 不再高频乱跳
- [ ] 日志已落盘，文件名填表

| 项 | 值 |
| --- | ---: |
| \(k_{\dot\theta,\mathrm{osc}}\)（起抖） | |
| \(k_{\dot\theta}\) 工作值 | |
| 日志文件 | `logs/stage3_step1.log` |

### Step 2：加 \(k_\theta\)（落地手扶）{#step2}

```bash
~/.platformio/penv/bin/python3 tools/stage2_serial_bridge.py \
  -p /dev/ttyUSB0 -l logs/stage3_step2.log
```

一手虚扶车顶。推荐固定比值 \(T_d=k_{\dot\theta}/k_\theta=0.022\)，
只放大 \(k_\theta\)（200→300→450→675→1000），\(k_{\dot\theta}\) 跟着算（`d` = 0.022×`p`），
把二维搜索降成一维。对照表：

| `p` | 200 | 300 | 450 | 675 | 1000 |
| --- | ---: | ---: | ---: | ---: | ---: |
| `d` | 4.4 | 6.6 | 9.9 | 14.9 | 22.0 |

例：`echo 'p 300' > /tmp/fishbot_cmd` 后立刻 `echo 'd 6.6' > /tmp/fishbot_cmd`。
判啸叫时再发一次 `f 20`。

- [ ] 太小：软倒，撑不住
- [ ] 太大：绕平衡点高频振荡/啸叫
- [ ] 取「能撑住又不啸叫」

| 项 | 值 / 现象 |
| --- | --- |
| \(k_\theta\) 起振值 | |
| \(k_\theta\) 工作值 | |
| 日志文件 | `logs/stage3_step2.log` |

### Step 3：调平衡角 \(\theta_{\mathrm{ref}}\) {#step3}

```bash
~/.platformio/penv/bin/python3 tools/stage2_serial_bridge.py \
  -p /dev/ttyUSB0 -l logs/stage3_step3.log
```

车持续往一个方向跑 = trim 偏了。用 `t <deg>` 试，直到左右漂移大致对称。

本车 \(l\approx12\ \mathrm{mm}\)，trim 灵敏度 \(\propto1/l\)：质心横向偏 1 mm 就等于 4.8° 的
平衡角偏差。所以真实 trim 可能有好几度，而且**按 0.2° 一档**试，不是 0.5°。
注意区分：trim 偏是「稳定地朝一个方向加速」，而 [§9 那个结构性漂移](balance_gain_theory.md#pitch-only)
即使 trim 完全正确也消不掉——后者要到阶段 4 才有解。

起点参考：阶段 2 静止遥测的 pitch（当时约 1.70°），但真平衡角需实测。

| 试验 | \(\theta_{\mathrm{ref}}\) (deg) | 漂移方向 | 日志文件 |
| ---: | ---: | --- | --- |
| 1 | | | `logs/stage3_step3.log` |
| 2 | | | |
| 3 | | | |

- [ ] 定稿 \(\theta_{\mathrm{ref}}\) = ______ deg

### Step 4：少量 \(k_{i\theta}\)（可选）{#step4}

```bash
~/.platformio/penv/bin/python3 tools/stage2_serial_bridge.py \
  -p /dev/ttyUSB0 -l logs/stage3_step4.log
```

仅用于消稳态偏差，从很小开始。偏大表现为低频前后晃动。

- [ ] 已用 `setLimits()` 给积分限幅（并在输出饱和时冻结积分，防 windup）

| 项 | 值 / 现象 |
| --- | --- |
| \(k_{i\theta}\) 工作值 | |
| 日志文件 | `logs/stage3_step4.log` |

### Step 5：独立站立验收 {#step5}

```bash
~/.platformio/penv/bin/python3 tools/stage2_serial_bridge.py \
  -p /dev/ttyUSB0 -l logs/stage3_step5_accept.log
```

- [ ] 松手独立站立 > 10s
- [ ] 轻推能恢复
- [ ] 全程 `hz=200`、`ovr` 不增长、无误触 `FALL`
- [ ] 定稿增益写回 `include/config.h`，并更新本文「定稿」栏

| 项 | 结果 |
| --- | --- |
| 最长站立时长 | |
| 轻推恢复 | |
| 日志文件 | `logs/stage3_step5_accept.log` |

---

## 增益试验记录 {#log}

| # | 日期 | \(k_\theta\) | \(k_{i\theta}\) | \(k_{\dot\theta}\) | \(\theta_{\mathrm{ref}}\)(deg) | 现象 | 日志文件 |
| ---: | --- | ---: | ---: | ---: | ---: | --- | --- |
| 1 | | | | | | | |
| 2 | | | | | | | |
| 3 | | | | | | | |
| 4 | | | | | | | |
| 5 | | | | | | | |

**定稿：** \(k_\theta\)=____ \(k_{i\theta}\)=____ \(k_{\dot\theta}\)=____ \(\theta_{\mathrm{ref}}\)=____ deg

---

## 现象 → 排查 {#troubleshoot}

| 现象 | 首查 |
| --- | --- |
| 一使能就加速倒下 | 符号（`kPitchSign` / `kWheelDir`），回 Step 0 |
| 高频抖动/啸叫 | \(k_{\dot\theta}\) 或 \(k_\theta\) 过大；或 \(\dot\theta\) 误用了 pitch 差分而非陀螺 |
| 低频前后摆动（周期 0.5~2s） | 大概率是阶段 0 那 ~20% 静摩擦启动门槛形成的极限环；拧增益消不掉就要提前补一点库仑摩擦前馈，而不是继续加 \(k_\theta\) |
| 反应「慢半拍」、总差一点追不回 | `kMaxEffortSlew` 仍太小，或 `kMaxEffort` 限得太死 |
| 持续单向跑 | 先调 \(\theta_{\mathrm{ref}}\)（Step 3）；调不掉是姿态环的结构性漂移，属正常，等阶段 4 |
| 怎么调都站不住、增益一大就振 | 大概率是 \(l\) 太小（12 mm）。**把电池挪到顶板**把 \(l\) 提到 30 mm，比任何增益技巧都有效 |
| 站住几秒后越晃越大 | \(k_{i\theta}\) 过大，或积分未限幅 |
| 偶发 `IMU_LOST` | 见 [`stage2_safety_bench.md`](stage2_safety_bench.md#checklist) 时间戳下溢那条 |
| 扶正后不动 | 平衡模式故障会锁存，需发 `r` 解锁（设计如此） |

---

## 备注 {#notes}

- 阶段 3 **不接** `cmd_vel`、不做差速、不做速度环——那是阶段 4（填 \(k_s\)/\(k_{\dot s}\)）。
- 阶段 5 切 LQR 时只换增益表并把 \(u\) 的物理量从占空比改成 N·m，控制环结构不动。
- 死区 / 库仑摩擦补偿的完整版留到阶段 5 的 τ→PWM 标定；阶段 3 只在出现极限环时补最小量。
- 在线调参掉电即失，每次定稿务必写回 `include/config.h` 并同步本文表格。
