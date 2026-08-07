---

## title: 阶段 2 安全层与频带分离验收
level: 1
order: 10
sections: true

# 阶段 2 安全层与频带分离验收

> 对应代码版本（以仓库当前文件为准）：
>
>
> | 文件                                                | 职责                              |
> | ------------------------------------------------- | ------------------------------- |
> | `src/stage2_main.cpp`                             | Core1 控制环 + Core0 串口通信骨架        |
> | `include/config.h`                                | 周期 / 安全阈值 / 方向符号                |
> | `include/shared_state.h` + `src/shared_state.cpp` | 跨核快照                            |
> | `lib/Safety/`                                     | 饱和 / 斜率 / 摔倒去抖 / IMU·通信超时       |
> | `lib/Ahrs/`                                       | 固定 dt 互补滤波 → pitch / pitch_rate |
> | `lib/Drive/`                                      | 轮速测量与 PWM 执行（方向在边界层收口）          |
>
>
> 本阶段**不含**平衡控制器；`desired` 直接取串口 `e <duty>`。
> 过关后再进阶段 3（姿态环）。对应清单：`docs/Sim2Real_list.md` → 阶段 2。

---

## 烧录与串口 {#flash}

```bash
pio run -e stage2 -t upload
```

IDE 自带的 `device monitor` 往往无法可靠输入，且收发挤在同一窗口。推荐**收发分终端**：

```bash
# 先停掉所有占用 /dev/ttyUSB0 的 monitor

# 终端 A：只看遥测（默认落盘到 logs/stage2_*.log）
~/.platformio/penv/bin/python3 tools/stage2_serial_bridge.py
# 指定文件:  ... stage2_serial_bridge.py -l logs/fall_test.log
# 不落盘:    ... stage2_serial_bridge.py --no-log

# 终端 B：只发命令（另开）
echo 'e 30' > /tmp/fishbot_cmd
echo 's'    > /tmp/fishbot_cmd
echo 'x'    > /tmp/fishbot_cmd
echo 'o'    > /tmp/fishbot_cmd
```

日志每行带主机时间戳，TX 记为 `>> e 30`，与 RX 遥测同一文件，便于对照验收步骤。

- 波特率 `115200`；默认口 `/dev/ttyUSB0`（可 `-p` / `-b` / `-f` 改）
- 上电后先看到 `IMU calibrating, keep still...`，标定期间车体必须静止
- 成功：`IMU ok ...` → `stage2 up: e <duty> / s / x=link down / o=link up`
- 失败：`IMU FAIL` 时控制环仍会跑，但 `fault` 会持续 `IMU_LOST`，先查 I2C（SDA=18 / SCL=19）

**安全：** 轮子会真转。架空车体，勿用手挡轮。

---

## 串口指令 {#cli}


| 命令         | 作用                                  |
| ---------- | ----------------------------------- |
| `e <duty>` | 开环输出目标（%），经安全层饱和 + 斜率后下发            |
| `s`        | 输出目标置 0（仍走斜率下降）                     |
| `x`        | 模拟通信断链：停止刷新 `CommandInput.stamp_ms` |
| `o`        | 恢复链路：重新刷新时间戳                        |


遥测约每 `kTelemetryMs`（200ms）一行，格式：

```text
hz=<ctrl_hz> ovr=<overrun> fault=0xXX [FALL] [IMU_LOST] [CMD_TIMEOUT] | pitch=..deg rate=..rad/s | v=L/R m/s effort=L/R%
```

---

## 当前阈值（`include/config.h`）{#thresholds}

改阈值只动这一处；下表与代码同步，改代码后请同步改这里。


| 符号               | 当前值      | 含义                          |
| ---------------- | -------- | --------------------------- |
| `kCtrlHz`        | 200      | 控制环名义频率                     |
| `kMaxEffort`     | 100      | 安全层输出饱和（%）                  |
| `kMaxEffortSlew` | 400      | 斜率限幅（%/s）；200Hz 下每拍最多 ±2%   |
| `kFallAngleRad`  | 0.52     | ≈30°，超角进入摔倒计时               |
| `kFallHoldMs`    | 50       | 连续超角 ≥50ms（约 10 拍）才置 `FALL` |
| `kImuTimeoutMs`  | 100      | IMU stamp 过期判 `IMU_LOST`    |
| `kCmdTimeoutMs`  | 500      | 指令 stamp 过期判 `CMD_TIMEOUT`  |
| `kPitchSign`     | +1       | 正 = 车体前倾；相反则翻符号             |
| `kWheelDir`      | {+1, +1} | 正 PWM = 车前进；阶段 0 已标定        |


故障位掩码（`Safety::Fault`）：`FALL=0x01`，`IMU_LOST=0x02`，`CMD_TIMEOUT=0x04`。可组合。

---

## 验收步骤 {#checklist}

每条通过打 `[x]`，并在「记录」栏填一次实测现象。

### 1. 控制环时序

- [ ] `hz` 稳定在 **200**（允许偶发 199/201）
- [ ] `ovr`（`overrun_count`）长时间不增长


| 项               | 结果  |
| --------------- | --- |
| 稳态 hz           |     |
| ovr 初值 → 5min 后 |     |


失败排查：I2C 阻塞、控制环里打日志、同核高优先级抢占。勿在控制环 `printf`。

若间歇出现 `IMU_LOST` 但 `pitch` 仍在缓慢变化、且未拔线：多半是 `now_ms` 取在 `ahrs.update()` 之前、I2C 跨越 ms 边界导致无符号 `(now-stamp)` 下溢。修法见 `Safety::evaluate` 有符号年龄 + `stage2_main` 先 update 再取 `now_ms`。

### 2. 斜率限幅（`e 30`）

发 `e 30`，盯 `effort`：

- [ ] **不跳变**到 30%，而是按约 `400 %/s` 爬升（0→30% 大约 75ms）
- [ ] 稳态 `effort ≈ 30`，且不超过 `kMaxEffort`
- [ ] 发 `s` 后 effort 按斜率回落至 0


| 项             | 结果  |
| ------------- | --- |
| 爬升是否平滑        |     |
| 稳态 effort L/R |     |


注：遥测 200ms 一帧，可能看不清中间台阶；肉眼看轮子是否「猛抽一下」即可。要精确可临时把 `kTelemetryMs` 改小，测完改回。

### 3. 摔倒保护（`FALL`）

先 `e 20` 让轮子转着，再**手动前倾超过约 30° 并保持 ≥50ms**：

- [ ] `fault` 出现 `FALL`（可与其它位 OR）
- [ ] `effort` **立刻归零**（不走斜率）
- [ ] 车体回到直立后，`FALL` 消失，`effort` 保持 0（不会自动爬回旧指令）；需再发 `e` 才从 0 按斜率爬


| 项                | 结果   |
| ---------------- | ---- |
| 触发时 pitch (deg)  |      |
| effort 是否瞬时清零    |      |
| 短暂晃一下（<50ms）是否误触 | 应不触发 |


### 4. 通信断链（`CMD_TIMEOUT`）

先 `e 20`，再发 `x`：

- [ ] 约 **500ms** 内 `fault` 出现 `CMD_TIMEOUT`
- [ ] `effort` 立刻归零
- [ ] 发 `o` 恢复后，可再 `e` 正常输出


| 项          | 结果  |
| ---------- | --- |
| x → 停车大致延迟 |     |
| o 后是否可再控   |     |


### 5. IMU 丢失（`IMU_LOST`）

先 `e 20`，再**拔 IMU 排线**（或断 SDA/SCL）：

- [ ] `fault` 出现 `IMU_LOST`
- [ ] `effort` 立刻归零
- [ ] 插回并复位后可恢复（本阶段不要求热插拔自恢复）


| 项         | 结果  |
| --------- | --- |
| 拔线后 fault |     |
| effort    |     |


### 6. 方向与符号确认（不通过则改 `config.h` 一处）

- [ ] 车体**前倾**时遥测 `pitch` 为正；若为负 → 翻 `kPitchSign`
- [ ] `e +30` 时两轮朝**车前进**方向转，且 `v` 为正；若不满足 → 翻对应轮的 `kWheelDir[i]`（阶段 0 逻辑）


| 项                  | 结果  | 是否改了 config |
| ------------------ | --- | ----------- |
| pitch 前倾符号         |     |             |
| +effort → 前进 / v>0 |     |             |


---

## 过关判定 {#pass}

全部勾选且记录栏无异常后，阶段 2 过关，可进阶段 3（姿态 PID，目标倾角 0）。

若只过了 1/2 而 3~5 失败：不要上平衡环——控制器再稳也挡不住断链/摔倒。

---

## 备注 {#notes} 

- `stage0_main.cpp` 现位于 `test/`，`[env:stage0]` 默认编不到；要重跑阶段 0 需先移回 `src/`。
- 死区 / 库仑摩擦补偿**不要**在本阶段塞进 `WheelActuator`；等阶段 5 的 τ→PWM 标定一起做。
- 故障态清零不走斜率；恢复后指令侧也会清掉挂起的 `test_effort`，需重新发 `e` 才会动——避免扶正后自动爬回旧占空比。

