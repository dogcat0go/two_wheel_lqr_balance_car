---
title: ROS 2 Runtime 与性能工程实验台
level: 1
order: 10
sections: true
---

# ROS 2 Runtime 与性能工程实验台

> **面向岗位：** runtime 架构 / 通信中间件 / 设备驱动 / 数据同步与日志系统；基于 ROS 2 的低延迟通信与调度；CPU·内存·IO 优化；内核态与用户态 crash 定位。工具面：perf、ftrace、valgrind；加分：eBPF、PREEMPT_RT。
>
> **手上资源：** 2D 激光雷达（USB/串口）、二轮差速底盘、ESP32 固件（本仓库）、**树莓派 4/5 作为机上主控**、一台 x86 Linux 主机（兼做对照组与虚拟机宿主）。强烈建议再添一个几十块的 8 通道 USB 逻辑分析仪，理由见 §1.2。

这套实验的目的不是"学会 ROS 2"，而是**制造出岗位描述里那些问题，然后用工具把它抓住**。面试官问的从来不是"你知道 perf 吗"，而是"你上次用 perf 定位了什么"。所以每个实验的产出物是一组前后对比数字，不是一段能跑的代码。

---

## 0. 三条纪律

1. **先基线，后注入。** 没有"改之前 p99 = X、改之后 p99 = Y"的实验等于没做。
2. **先可重复，后真实。** 在仿真或 rosbag 回放里确认现象与定位手段成立（不摔车、可无限次重跑），再上实车验证。实车才有真驱动、真 USB、真中断。
3. **一个实验一页纸。** 现象 → 假设 → 用什么工具看到了什么 → 改了什么 → 数字变化。这一页纸就是面试时的回答。

### 0.1 岗位描述 → 实验映射

| JD 条目 | 实验 |
| --- | --- |
| 通信中间件 | A1 QoS 不兼容 · A2 大消息分片丢包 · A3 零拷贝三档对比 · A4 发现风暴 · A5 网络劣化尾延迟 |
| runtime 架构 / 低延迟调度 | **§2.3 进程划分与节点组合** · B1 executor 队头阻塞 · B2 回调组切分 · B3 实时优先级与核隔离 · B4 缺页抖动 · B5 PREEMPT_RT 对比 |
| 设备驱动 | C1 串口延迟与 syscall 放大 · C2 USB 掉线重连 · C3 驱动线程模型 |
| 数据同步与日志系统 | **§2.2 三设备时间同步（硬件脉冲）** · D1 多源时间戳对齐 · D2 rosbag 记录引发 IO 抖动 · D3 日志写阻塞控制环 |
| CPU/内存/IO 优化 | A3 · B3 · B4 · D2 · E3 |
| 用户态 crash | E1 core dump 全流程 · E2 valgrind 与 sanitizer 分工 |
| 内核态 crash | E4 oops/panic 与 kdump 演练 |
| perf / ftrace / valgrind | 贯穿；主战场分别是 A3·B3、B1·B3、E2 |
| eBPF（加分，可延后） | F1 调度延迟 · F2 块层延迟 · F3 丢包点 · F4 自写探针（有 perf/ftrace 替代，见第 8 节） |
| PREEMPT_RT（加分） | B5 |

---

## 1. 台架与统一标尺

### 1.1 平台选型：被测对象要放在树莓派上，不是 x86 主机

**这是整套实验最重要的一个决定。** 在 8 核 x86 + NVMe 上，本文档里一多半的问题根本复现不出来——不是因为它们不存在，而是因为硬件余量把它们全盖住了。换成树莓派（4B/5，4 GB 起步）之后，这些问题会**在正常负载下自然发生**，不需要靠 `stress-ng` 硬造。

| 实验 | x86 桌面 | 树莓派 4/5 |
| --- | --- | --- |
| D2 rosbag IO 抖动 | NVMe 太快，要用 `fio` 打满才出得来 | SD 卡随手一录就出周期尖刺 |
| A2 大消息分片丢包 | 万兆/环回不丢 | 百兆/千兆 + 弱 CPU，点云一开就丢 |
| B1 executor 队头阻塞 | 慢回调要人为 sleep 才盖过定时器 | 真实的点云处理就足以盖过去 |
| B3 CPU 抢占与核隔离 | 16 核随便挤 | 4 核，隔掉 2 个就剩 2 个，取舍是真的 |
| B4 缺页与内存压力 | 32 GB 内存无感 | 4 GB，跑起 nav 栈就开始换页 |
| A3 零拷贝收益 | 序列化开销占比小 | 序列化直接吃掉可观 CPU，收益显著 |

除此之外还有两个"顺带"的好处：它是 **ARM**（目标公司的量产平台八成也是 ARM，x86 上调好的参数搬过去经常不成立），而且它让你**天然拥有两台机器**——跨机 DDS、真实网络发现、时钟同步这几类实验在单机上是做不了的（同机会走共享内存，把问题绕过去了）。

**但不要因此把 x86 也排进实验计划。** 本文档早先写的是"两台都用、把对比本身当成一个结论"，这个建议要收窄。x86 对照跑只能回答一个问题——"慢是平台造成的还是架构造成的"——而这个问题在 Pi 上用 `perf` 采一次火焰图就能回答，而且回答得更细：它直接指出时间花在哪个函数上，而不只是告诉你"换个平台会快"。用一次跨平台对比去换一个远不如剖析精确的答案，不划算。

反方向的成本更值得警惕：**在 x86 上"制造"瓶颈是在造赝品**。cgroup 掐内存、`stress-ng` 占 CPU、`tc netem` 加延迟、`fio` 打满盘——每一样都要调参，而且你无法确认造出来的瓶颈和真实瓶颈是同一个。Pi 上这些瓶颈是免费且真实的。

所以 x86 的定位是**开发机与分析机，不是实验平台**：写代码、编译、跑 valgrind 的重活、存 rosbag、画图，以及在 Pi 上跑 30 分钟浸泡测试时让你有地方继续干活（单板会成为串行瓶颈，这是保留工作站的主要实际理由）。它另外顺带满足两个需求：跨机 DDS 实验需要的第二个端点，以及 E4 内核 panic/kdump 必须的"可以随便崩的机器"。这些都不需要额外规划——你本来就有这台机器。

一次性的 x86 对照跑仍然值得做**一次**，但目的是叙事而不是定位：面试时能说"同一份代码 x86 p99 是 2 ms、Pi 上是 45 ms，差在这三处"是个好故事。做一次记下来，别让它变成每个实验的固定动作。

**代价要说清楚：** 直接上硬件的前提是**先把测量卫生做好**。真机上一个"变慢"背后往往同时站着降频、SD 卡、WiFi 重传、欠压、DVFS 好几个原因，不像 x86 那样能一个个隔离。§1.1 末尾的四个坑和 §1.5 的锁频基线不是可选项——跳过它们，硬件优先会从提速变成泥潭。

四档环境按这个思路排：

| 档 | 组成 | 用来做什么 | 注意 |
| --- | --- | --- | --- |
| **S** 可重复回放 | **rosbag2 回放，跑在 Pi 上** | 用真实数据做确定性重跑：改一版代码、放同一个包、比同一条曲线。可重复性是仿真的主要价值，回放全都给得了，而且数据是真的 | Gazebo 对性能实验价值很低（仿真时钟会掩盖真实抖动），列为可选，且只在 x86 上跑 |
| **H** 半实物 | 真雷达 + **树莓派**，底盘架空不落地 | 真驱动、真 USB、真 DDS，但不会摔车 | 主战场。C 组全部、A/B/D 组大部分在这一档 |
| **R** 实车 | 雷达 + 底盘 + 树莓派 + ESP32 闭环 | 真时序、真闭环耦合 | 只做已经在 S/H 验证过的实验 |
| **V** 虚拟机 | x86 上的 KVM/QEMU | 内核 panic、kdump、崩内核的驱动实验 | E4 必须在这一档，不要在 Pi 或主力机上做 |

#### 买哪一块：Pi 4 还是 Pi 5

选型原则是**能用软件制造的约束，就不要靠买硬件来制造；软件补不出来的能力，买的时候就必须有**。

按这条筛：内存小、核少、频率低、盘慢，全都能用 cgroup、`taskset`、慢盘、`mem=` 内核参数在强板子上模拟出来，所以不构成买弱板的理由。真正**事后补不了**的只有两样——片上硬件编解码器，和直连的 GPIO。

而硬件 H.264 编码器的分布是反直觉的：

| 平台 | 硬件 H.264 编码 | 备注 |
| --- | --- | --- |
| **Pi 4**（BCM2711） | **有** | 1080p30，以标准 V4L2 M2M 接口暴露，是 C4 的前提 |
| Pi 5（BCM2712） | 无 | 官方白皮书确认 legacy 编解码块整块移除，只保留自研 H.265 **解码**器；H.264 走 libx264 软编，1080p30 低延迟档约占 60~90% 单核 |
| Jetson Orin Nano | 无 | NVENC 被砍，官方文档明确要求改用软件编码；NVDEC 解码仍在 |
| Jetson Orin NX | 有 | NVENC 只在 NX 与 AGX 上 |

但**硬件编码器并不是 C4 那类驱动实验的前提**，这一点容易搞错。Pi 5 的相机链路是 `rp1-cfe` 驱动（已进上游内核文档），它用 V4L2 subdev API 注册 CSI-2 接收器与 PiSP 前端，用 Media Controller 组成完整媒体图，还额外暴露了统计量与配置缓冲区节点——**媒体拓扑、缓冲区所有权、DMA-BUF 零拷贝这些要练的东西，Pi 5 一样不缺，甚至更完整**。M2M 这一课也还在：Pi 5 保留了自研 H.265 硬件解码器，同样走 V4L2，用解码器替代编码器做 M2M 实验，缓冲区队列模型完全一样。

所以 Pi 4 真正独占的只剩"硬件编码器作为流水线中的一级"，而它换来的代价是**没有引出 PCIe**——这意味着后期想加 AI 加速器时是条死路（官方 AI HAT+ 依赖 Pi 5 的 PCIe 接口，Pi 4 只能退到 Coral USB）。

**结论：默认选 Pi 5（8 GB）。** 内存买大的，需要压力时用 cgroup 掐；PCIe 接 NVMe 当系统盘，正好把 SD 卡空出来当 §1.1 里"故意做慢的盘"；H.264 走软编，1080p30 低延迟档约占 60~90% 单核，剩下三个核够用，而且这份可调的 CPU 负载本身就是 B 组需要的自变量。

只有一种情况该选 Pi 4：明确不做视觉推理，且特别想练硬件编码器那一级。注意软编的代价不在 CPU 而在功耗——电池供电的机器人上，这个取舍要重新算。

**但如果手上已经有 Pi 4B 8 GB，直接用，别买 Pi 5。** 上面整段比较的唯一争点是硬件 H.264 编码器，而按 §9.1 砍掉 C4 之后这个争点就不存在了；何况编码器恰恰是 Pi 4 有、Pi 5 没有。8 GB 内存足够，要制造内存压力用 cgroup 掐即可。Pi 4 相对 Pi 5 的短板是没有引出 PCIe（后期加 AI 加速器时才成问题）和 CPU 更慢（A72 1.8 GHz 对 A76 2.4 GHz）——而后者对本文档而言**反倒是优点**，§1.1 开头那张表里的问题在更弱的板子上更容易自然发生。省下的钱按 §1.4 的建议给 USB SSD、散热外壳和逻辑分析仪。

不推荐 Jetson Orin Nano：想要的编码器它恰好没有；它的价值在 GPU/DLA 推理，而本文档对应的岗位方向一个字都没提推理；L4T 下游内核在 perf、BTF/eBPF、PREEMPT_RT 上的折腾成本远高于 Pi 上的 Ubuntu，时间会花在让工具跑起来而不是用工具解决问题。只有当目标平台明确就是 Jetson 时才值得上，且那时应直接选 Orin NX。

省下来的预算优先给：USB SSD（系统盘）、散热外壳、逻辑分析仪。

#### 装 Server 还是 Desktop：装 Server

对做性能测量的机器来说，桌面环境是**一整包不受控的自变量**，而 §1.5 的全部方法论都建立在基线稳定之上。具体会污染哪些实验：

| 桌面组件 | 污染什么 |
| --- | --- |
| `tracker-miner-fs` 文件索引 | 随机 IO 尖刺，**直接毁掉 D2 的 rosbag IO 抖动实验**——你会分不清尖刺是录包引起的还是索引引起的 |
| `gnome-shell` + 合成器 | 持续的后台 CPU 与周期性唤醒，抬高 B 组所有延迟分布的底噪 |
| `snapd` 后台刷新、`unattended-upgrades` | 不可预测的 CPU/IO/网络突发，浸泡测试里会变成"偶发异常"，浪费时间去追 |
| GPU 显存划分 | 从系统内存里切走一块，B4 的内存压力实验基线要重算 |

**GUI 工具放到 x86 工作站上跑，不要放在 Pi 上。** RViz2、rqt、PlotJuggler 直接订阅 Pi 上的话题就行——DDS 本来就是网络透明的。你用的是 2D 雷达，`LaserScan` 一帧才几 KB，跨网订阅的开销可以忽略（换成点云或图像就要重新掂量了）。这正好落在 §1.1 给 x86 划定的"分析机"角色里。

顺带：`ssh -X` 转发 RViz 能用但很卡，别指望它；偶尔要图形界面时装 VNC 按需起，比常驻桌面干净。

#### 装哪个版本：24.04 Server + Jazzy 原生

这里有个绕不开的三方冲突，**两个加分项都落在 24.04 那一边**：

| 需求 | 落在哪个版本 |
| --- | --- |
| ROS 2 Humble Tier 1（arm64 二进制包） | **22.04** |
| BTF / eBPF CO-RE（F 组） | **24.04**（`linux-raspi` 6.8.0-1009 起才带 `CONFIG_DEBUG_INFO_BTF`） |
| PREEMPT_RT 树莓派变体（B5） | **24.04**。Canonical 的实时内核变体列表里，22.04 只有 `generic` 和 `intel-iotg`，**树莓派变体是 24.04 才加的**；22.04 上要做 B5 只能自己给 5.15 raspi 内核打 RT 补丁重编，那就不是"一晚上能出数"了 |

**「24.04 + Humble」不存在原生形态。** `packages.ros.org` 没有 noble 对应的 humble 包（apt 源按发行代号分目录，noble 下只有 Jazzy 之后的）；源码编译则会撞上 Noble 的 Python 3.12——Humble 期的大量 `ament_python` 包依赖 3.12 已移除的 `distutils`。在 Pi 上编好几个小时再失败，概率不低。**所以选 Humble 就等于选容器。**

而换 Jazzy 的成本，对**本仓库**来说几乎是零：

| 要改什么 | 工作量 |
| --- | --- |
| 自己的 ROS 2 包 | **没有**。本仓库是纯 PlatformIO 固件，一个 `package.xml` 都没有 |
| `tools/fishbot_wifi_bridge.py` | 唯一用 rclpy 的文件，只用到 `init` / `Node` / `QoSProfile` / `spin` / `shutdown`，这几个从 Humble 到 Jazzy 没变。实际只有报错信息里那句 `/opt/ros/humble/setup.bash` 要改 |
| 固件 micro-ROS | `platformio.ini` 里 `board_microros_distro = humble` 改一行 |
| slam_toolbox / nav2 / robot_localization / micro_ros_agent | Jazzy 都有 arm64 二进制包 |

**结论：Ubuntu 24.04 Server（arm64+raspi 预装镜像）+ Jazzy 原生。** 全原生等于绕开容器的三个坑：漏掉 `--ipc=host` 会让 A3 零拷贝的数据**静默失真**（这是最危险的一个，因为它不报错）、设备透传、以及 cgroup v2 对 rtprio 的限制。RT 内核一条命令：

```bash
sudo pro attach                                  # 个人用途 5 台以内免费
sudo pro enable realtime-kernel --variant=raspi  # 千万别漏 --variant，装错变体会起不来
uname -rv                                        # 期望看到 ...-raspi-realtime ... PREEMPT_RT
```

**动手装系统之前先验证一件事：** micro_ros_platformio 的 jazzy 能不能编过。`platformio.ini` 里已经留着前车之鉴——注释写明默认的 kilted 会因 `rmw_test_fixture` 缺 rmw 而编译失败，所以才钉死 humble。jazzy 未必没有同类问题。改一行跑一次 `pio run` 就知道，代价极小，但它决定整个方案成不成立。

万一 jazzy 编不过，退路按代价从小到大：只把 micro_ros_agent 单独放进 Humble 容器、其余 Jazzy 原生（但要注意 Iron 之后引入的类型哈希，Humble 侧不带哈希，与 Jazzy 节点匹配可能出问题）；再不行就退回 22.04 + Humble 全原生，放弃两个加分项。

**不要往更新的版本跳。** 26.04 + Lyrical 虽然也有 raspi 实时内核变体，但发布没多久，nav2 / slam_toolbox / micro-ROS 的生态成熟度是未知数——时间会花在让工具跑起来上，这正是本节反对上 Jetson 的同一条理由。何况面试对口的是 Humble 和 Jazzy。另一个值得知道的时间点：**Humble 的支持到 2027 年 5 月截止**，Jazzy 到 2029 年 5 月。

#### 树莓派的四个坑（不先处理，测出来的数全是噪声）

1. **散热与"锁频"冲突。** §1.5 要求跑基线前锁频，但 Pi 裸板一跑满就降频，基线会一路漂。必须加金属外壳或风扇，并在**每次实验前后**记录：
   ```bash
   vcgencmd measure_temp && vcgencmd get_throttled   # 期望 throttled=0x0
   ```
   `get_throttled` 非 0 的那一轮数据直接作废。这条纪律本身在面试里就能讲——**测量环境不受控，优化结论就不成立**。
2. **发行版决定 eBPF 能不能用，而它和 ROS 版本互相打架。** F 组的 bcc/bpftrace 走 CO-RE，需要内核带 `CONFIG_DEBUG_INFO_BTF`。树莓派专用内核长期不带这个选项（Launchpad #2065829），**Ubuntu 的 `linux-raspi` 直到 24.04 的 6.8.0-1009 才补上**；22.04 的 5.15 raspi 内核没有，Raspberry Pi OS 的下游内核也没有。上手先验一句：
   ```bash
   ls /sys/kernel/btf/vmlinux && echo "BTF OK"
   ```
   冲突在于：ROS 2 Humble 的 Tier 1 平台是 Ubuntu **22.04** arm64，而带 BTF 的内核在 **24.04**。按上一节的结论走 **24.04 + Jazzy 原生**，这个冲突自然消失。

   只有在退回容器方案时才需要下面这段（留着因为它本身是好素材）：内核侧的可观测性不受容器影响，perf / ftrace / bpftrace 按 PID 观察进程，不关心它在哪个命名空间。容器按 `--net=host --ipc=host --pid=host` 起，再把雷达设备透进去；这三个参数加上之后基本只剩文件系统隔离，不会污染延迟测量。**`--ipc=host` 尤其不能少**，否则 Fast DDS 的共享内存传输用不了，A3 的零拷贝实验会直接失真，而且**不报错**——面试里能讲清这一条很加分。另外容器里给线程提实时优先级会撞上 cgroup v2 的 rt 带宽限制，要 `--cap-add=sys_nice --ulimit rtprio=99`；但 B5 不受影响，`cyclictest` 不是 ROS 程序，直接在宿主上跑。
3. **ARM 上的 perf 有两处不一样。** 硬件 PMU 事件（cycles、cache-misses）在部分内核配置下不可用，`perf stat` 会报 `<not supported>`——退回软件事件 `-e cpu-clock` 仍能采样出火焰图。另外 ARM64 常省略帧指针，调用栈会断，编译时加 `-fno-omit-frame-pointer`，或采样时用 `--call-graph dwarf`。
4. **别把系统盘和"故意做慢的盘"搞混。** 系统从 USB SSD 启动（快、不磨损），**把 SD 卡单独留给 D2 当录包目标**。这样"慢 IO"是一个你能开关的自变量，而不是拖慢一切的背景噪声。

### 1.2 谁是被测对象，谁是测量仪器

一个容易走错的方向是把 ESP32 也当成被测对象去测它的周期抖动。**不建议**，原因不是测不了，而是：

- **它测起来其实不难，本仓库已经测了一半。** `stage5_main.cpp` 的控制任务用 `vTaskDelayUntil` 锁相、`micros()` 计时，超过 5 ms 就累加 `overrun_count`，每秒统计 `ctrl_hz`，两者都从遥测的 `hz=` / `ovr=` 出来（`CommHost.cpp`）。缺的只是**分布**：现在只有"平均 200 Hz"和"超时了几次"，没有直方图。而 `hz=200` 完全可以是 3 ms / 7 ms 交替跑出来的——这正是 §1.3 说的"只报平均值会漏掉尾部"。
- **但它测出来会很无聊。** 控制任务 pin 在 Core 1、通信和 WiFi 全在 Core 0（`kCtrlCore=1` / `kCommCore=0`），优先级 5，`vTaskDelayUntil` 相位锁定。这套结构下唤醒抖动在几十微秒量级，稳得没有故事可讲。频带分离已经把问题解决掉了。
- **真正难的是归因，而不是测量。** ESP32 上没有 perf、没有 ftrace、没有 eBPF。抖动一旦发生，你看得见"这一拍晚了 2 ms"，但看不见"是谁抢了我"。而岗位描述里的 perf / ftrace / eBPF 全部只在 Linux 上有意义。

所以角色这样分：

> **树莓派是被测对象，ESP32 是测量仪器。**

ESP32 当仪器有一个 Linux 给不了的优势：**它的时钟不受被测系统影响**。跨机测延迟最麻烦的是两端时钟不同步，误差常常和被测量本身同一量级。用 ESP32 做外部参考就绕开了这个问题：

- 树莓派在 `cmd_vel` 发出的瞬间翻转一个 GPIO；
- ESP32 用边沿中断（或 MCPWM capture）打时间戳，和自己执行动作的时刻一起回传；
- 两个时刻在**同一个晶振**上测得，差值就是干净的端到端延迟，亚微秒分辨率，且不需要 NTP/PTP。

更省事的版本是买一个几十块的 8 通道 USB 逻辑分析仪（sigrok/PulseView）：Pi 的 GPIO、ESP32 的控制环脉冲、电机 PWM 三路一起抓。**软件零开销、不改被测系统、直接看到真实墙钟时序**——这是整套实验里性价比最高的一笔投入，也是唯一能给软件测量结果"校准"的手段。

这就带出一个很值得做的实验对：D1 用软件估计 ESP32 与主机的时钟偏移和 ppm 漂移，然后**用 GPIO 硬件测量去验证这个软件估计到底准到什么程度**。能拿出"我的软件时间同步方案，经硬件基准验证误差在 ±X µs"这句话，比单纯说"我做了时间同步"强一个档次。

如果仍然想给 ESP32 补上抖动分布（值得做，十几行的事）：在 `vTaskDelayUntil` 之后记录实际唤醒时刻与期望时刻的差，连同单拍执行耗时，各自打进一个固定分桶的小直方图，复用现有遥测发出来。注意两件事——**唤醒抖动和执行超时是两种不同的故障**（前者是"该醒没醒"，后者是"算不完"），现有的 `ovr` 只覆盖后者；另外长跑建议把 `micros()` 换成 `esp_timer_get_time()`，后者是 64 位微秒，不会像 32 位的 `micros()` 那样约 71 分钟回绕（现有代码用无符号相减，回绕是安全的，但直方图统计里更容易出错）。

### 1.3 统一标尺：一条链路，六个打点

所有实验共用同一根尺子，否则实验之间无法横向比较。

```
T0  雷达硬件出帧（驱动 read() 返回时刻，最接近的可观测点）
T1  驱动完成解包，构造出 LaserScan
T2  publisher 调用 publish() 返回
T3  下游订阅回调被唤醒并进入
T4  控制节点算完，cmd_vel publish()（同时翻转一个 GPIO，供硬件打点）
T5  ESP32 收到该指令并动作（边沿中断打时间戳，回传序号 + 本地时间）
```

`T4`/`T5` 这一段优先用 §1.2 的 GPIO 硬件方式取，它不依赖两机时钟同步；软件时间戳只作为交叉验证。

派生指标（每个实验都记这几个）：

| 指标 | 定义 | 说明 |
| --- | --- | --- |
| 端到端延迟 | `T5 - T0` | 用户能感知的那个数 |
| 中间件延迟 | `T3 - T2` | DDS 这一段花了多少，A 组主指标 |
| 排队延迟 | `T3` 减去消息可用时刻 | executor 有没有被堵，B 组主指标 |
| 驱动延迟 | `T1 - T0` | C 组主指标 |
| 周期抖动 | 控制定时器实际间隔的分布 | 记直方图，不记平均值 |
| 丢帧率 | 序号缺口 / 总帧数 | 区分"慢"和"丢"，两者根因完全不同 |

**统计口径固定为 p50 / p99 / max。** 只报平均值是新手标志——实时系统里所有事故都发生在尾部。样本量至少 1 万帧或 10 分钟，取大者。

两种取数方式都要会，面试常问区别：

- **侵入式**：消息里带 `seq` + 各段时间戳（走一个独立的 `/diag/latency` 话题，别污染业务消息）。优点是精确到自己想要的点，缺点是改了被测系统。
- **非侵入式**：`ros2_tracing`（LTTng）。`ros2 trace start s1` 采集，`tracetools_analysis` 出回调时长与发布订阅链路。优点是不改代码、能看到 executor 内部，缺点是要装 LTTng 且有采集开销。

### 1.4 一次装齐的工具

```bash
sudo apt install -y \
  linux-tools-common linux-tools-$(uname -r) \   # perf
  trace-cmd kernelshark \                        # ftrace 前端
  valgrind kcachegrind heaptrack \               # 内存 / 调用图
  bpfcc-tools bpftrace linux-headers-$(uname -r) \  # eBPF
  rt-tests stress-ng fio iproute2 sysstat \      # cyclictest / 负载 / IO / tc / iostat
  gdb systemd-coredump linux-image-$(uname -r)-dbgsym  # crash 定位
sudo apt install -y ros-$ROS_DISTRO-ros2trace ros-$ROS_DISTRO-tracetools-analysis
```

`perf` 需要放宽权限才能采内核栈：

```bash
sudo sysctl -w kernel.perf_event_paranoid=-1
sudo sysctl -w kernel.kptr_restrict=0
```

### 1.5 让"偶发"变成"可复现"：负载注入器

| 目标 | 手段 |
| --- | --- |
| CPU 抢占 | `stress-ng --cpu $(nproc) --cpu-load 80 --timeout 120s` |
| 内存压力 / 换页 | `stress-ng --vm 4 --vm-bytes 75% --vm-keep` |
| IO 压力 | `fio --name=bg --rw=randwrite --bs=4k --iodepth=32 --size=4G --numjobs=4 --time_based --runtime=120` |
| 网络劣化 | `sudo tc qdisc add dev <if> root netem delay 20ms 10ms distribution normal loss 1% reorder 2%` |
| 资源配额 | `systemd-run --scope -p CPUQuota=40% -p MemoryMax=512M -p IOWeight=10 <cmd>` |
| 缓存污染 | `stress-ng --cache 4 --cache-level 3` |

**跑基线前必须锁频，否则前后两组数字不可比：**

```bash
sudo cpupower frequency-set -g performance
sudo cpupower idle-set -D 0        # 禁用深 C-state，去掉唤醒延迟的随机性
```

这一条在面试里单独拿出来讲都是加分项——很多人测出来的"优化收益"其实是频率漂移。

树莓派上光设 governor 不够：降频是 VideoCore 固件按温度和电压强制做的，操作系统拦不住。Pi 上的等效纪律是**加散热 + 每轮实验前后查 `vcgencmd get_throttled`，非 0 就作废这组数据**（见 §1.1）。

---

## 2. 实际拓扑与由它决定的两道设计题

前面的实验是通用的。这一节把它们钉到你真实的三设备系统上——**这两道设计题不是练习，是不解决就跑不好的真问题**，而且它们各自能长出一串实验。

### 2.1 拓扑与每条链路的真实要求

```
  x86 工作站 ───────有线以太网───────┐
  开发·编译·RViz2·存包·画图          │   DDS 跨机
  （不参与实验，见 §1.1）             ▼
                            树莓派 4B（被测系统）
                            24.04 Server + Jazzy + PREEMPT_RT
                            micro_ros_agent ×2 → SLAM → 导航
                               ▲                    ▲
                      WiFi/UDP │                    │ WiFi/UDP
                     (部署期→串口)                (部署期→串口)
                               │                    │
       2D 雷达 ──UART──► ESP32-A                ESP32-B ──► 双电机驱动
                               ▲                    │       IMU
                               │                    │       200 Hz 本地平衡环
                               └── GPIO 1 Hz 脉冲 ───┘
                                   B 发 A 捕获，见 docs/esp32_time_sync.md
```

四台设备的角色是**不对称**的，这一点决定了后面所有安排：

| 设备 | 角色 | 关键约束 |
| --- | --- | --- |
| x86 工作站 | 开发机 + 分析机 | **不进实验计划**。它跑 GUI 和重编译，让 Pi 专心当被测对象 |
| 树莓派 4B | **被测系统** | 所有测量都在这里做；测量卫生（锁频、`get_throttled`）是前提 |
| ESP32-A | 雷达采集与发布 | 时间戳的**从**方，捕获 B 的脉冲 |
| ESP32-B | 电机 + IMU + 200 Hz 平衡环 | 时间戳的**主**方；它是唯一一个"断了会摔车"的设备 |

#### 调试期与部署期的差异

大部分连接在两个阶段是一样的，**变的只有三处**：

| | 调试期（底盘架空/台架） | 部署期（落地跑） |
| --- | --- | --- |
| **ESP32 ↔ Pi** | WiFi/UDP，维持现状 | **改 USB 串口** |
| **Pi 的网络** | **有线以太网**接路由器 | WiFi，或干脆不联网 |
| **工作站** | 同一局域网，跑 RViz2、收 rosbag | 不在场，最多 SSH 进去看一眼 |
| 日志 | 全量 rosbag 录到 SD 卡（D2 的实验对象） | 只留关键话题 |
| 逻辑分析仪 | 挂在同步脉冲线上 | 撤掉 |
| 供电 | 稳压电源 | 电池——**欠压会触发降频**，`get_throttled` 要继续查 |

调试期让 Pi 走**有线**很重要：这样 WiFi 变成一个你能主动劣化的自变量（A5 要的就是这个），而不是你的生命线。ESP32 挂在路由器的无线侧，Pi 挂有线侧，两边仍在同一个二层网里。

#### 为什么部署期要把 ESP32↔Pi 换成串口

看一眼上面的图就能发现一处荒谬：**ESP32-A 和树莓派拧在同一块底板上，相距几厘米，而 scan 数据要先发到空中、经路由器绕一圈再回来。** 这是当前架构里最大的一处结构缺陷。改成串口之后：

- WiFi 从关键数据通路上彻底消失，§2.4 里 MTU 分片、流历史溢出、尾延迟三个问题一起没了
- 链路延迟变成**可测的固定值**，而不是长尾分布
- 串口带宽够用：360 点 `LaserScan` 约 1.5~2 KB，10 Hz 即约 20 KB/s，461 kbaud 都绰绰有余
- 改动是 `platformio.ini` 里 `board_microros_transport` 一行，加两根 USB 线；代价是每个设备各要一个 agent 实例

**但调试期别急着改**——WiFi 链路正是 A2 和 A5 的实验对象，先在它上面把问题测出来。这个先后顺序本身就是最好的面试材料，正好套进 §10 的模板：现象是 WiFi 下 scan 丢帧率 x%、p99 y ms；定位是 MTU 分片加 best-effort；修复是换串口；数据是前后对照。**能讲"我为什么把无线换成有线"，比讲"我会配 QoS"有分量得多。**

换串口之后 C1（`latency_timer`、VMIN/VTIME、syscall 放大）就从"可选实验"变成"必须做的调优"——USB 转串口芯片的默认 latency timer 常常是 16 ms，不改的话你刚省下的 WiFi 延迟会原样还回去。

#### 两个阶段都不变的东西

**GPIO 同步脉冲线**和 **ESP32-B 的 200 Hz 本地平衡环**在两个阶段完全一致。这不是巧合：唯一"断了会摔车"的控制环不依赖任何一条外部链路，时间基准也不依赖上位机。**能在架构图上指出"哪些东西的正确性不依赖网络"，是这类岗位真正想考的判断。**

有一个容易被忽略但决定一切的事实：**平衡控制已经在 ESP32-B 本地闭环了**（200 Hz，`vTaskDelayUntil` 锁相，见 §1.2）。这意味着 WiFi 上跑的只有 `cmd_vel` 参考和上行遥测，没有任何一条链路是"关掉就摔车"的实时闭环。

于是两条上行链路的要求高度一致，而且和直觉相反：

| 链路 | 延迟要求 | 时间戳精度要求 |
| --- | --- | --- |
| ESP32-A → Pi（scan） | 松。几十 ms 无所谓，SLAM 不在乎数据什么时候到 | **严**。SLAM 在乎这帧数据**是什么时刻的** |
| ESP32-B → Pi（里程计） | 松。同上 | **严**。要和 scan 配准到同一时间轴 |
| Pi → ESP32-B（cmd_vel） | 中。几十 ms 可接受，因为内环自己稳得住 | 无所谓 |

> **结论：你的系统里，延迟不是主要矛盾，时间同步才是。** 这也解释了为什么你的直觉是对的——先做时间同步。传输延迟大但时间戳准，SLAM 照样出好图；传输延迟小但时间戳错，图必花。

### 2.2 设计题一：三设备时间同步

#### 先算清楚需要多准

误差主要来自旋转，不是平移。差速车原地转向时角速度可达 1 rad/s：

| 运动 | 时间戳误差 10 ms 造成的配准误差 |
| --- | --- |
| 平移 0.5 m/s | 5 mm |
| **旋转 1 rad/s** | **0.01 rad = 0.57°，在 3 m 处是 3 cm 横向误差** |

要把建图误差压到 1 cm 以内，时间戳误差需要 **< 3 ms**。取 **1 ms 作为设计目标**，留 3 倍余量。这个数字很重要——它决定了纯软件方案够不够。

#### 方案分级

| 方案 | 精度 | 评价 |
| --- | --- | --- |
| 各自 `millis()` 直接上报 | 完全不可用 | 三个晶振自由跑，几分钟就差出几十毫秒 |
| Pi 收到时用 `node->now()` 重新打戳 | 10~100 ms | **最常见的错误做法**。等于把 WiFi 的排队抖动当成了传感器时刻 |
| 软件 NTP 式四时间戳交换 | WiFi 上约 1~10 ms | 勉强够用，但 WiFi 往返延迟不对称且尾部很重，offset 估计有偏。必须用**最小值滤波**（取 RTT 最小的那些样本）而不是求平均 |
| **硬件脉冲 + 软件粗对齐** | **几十 µs** | 推荐。一根线换两个数量级 |

#### 先想清楚：精度要求只落在两块 MCU 之间

这一步能省掉一大半工作量。SLAM 拿 `t_scan` 时刻的 scan 去配 `t_scan` 时刻插值出来的位姿，**要求的是 scan 与 odom 相对对齐**。如果两者的时间戳都被平移了同一个量，配准结果完全不变——**共同的偏移是无害的，只有差分误差才伤人**。

于是精度要求可以拆成两级，难度差一个数量级：

| 关系 | 精度要求 | 怎么做 |
| --- | --- | --- |
| ESP32-A ↔ ESP32-B（差分） | **< 1 ms，这是硬要求** | 两块 MCU 之间拉一根线，硬件脉冲 |
| MCU 时基 → ROS 时间（共同偏移） | 松，几 ms 即可 | 软件估计就够，只要平滑、单调 |

**结论：硬件同步只发生在两块 MCU 之间，上位机完全不必参与捕获。** 这样上位机是树莓派还是别的板子、有没有现成的 `pps-gpio` 支持，都不再是约束。

#### 推荐方案

> 完整的可实施设计（接线、固件两段、上位机拟合、验证与失效处理）见 [`esp32_time_sync.md`](esp32_time_sync.md)。

1. **ESP32-B 产生 1 Hz 脉冲**（它已有最稳的时基和最规整的定时任务），记录发出时刻的 `esp_timer_get_time()`。
2. **ESP32-A 捕获同一根脉冲**，用 MCPWM 的 capture 单元硬件锁存计数器（比 GPIO 中断更准，没有 ISR 抖动）。
3. 两者每秒产生一组配对，用**递推最小二乘**在线估 offset 与 skew（ppm 级晶振漂移），把 A 的时间戳换算到 B 的时间轴。**至此差分精度已经解决。**
4. 上位机再用软件方式（NTP 式四时间戳交换 + 最小值滤波）把 MCU 时基映射到 ROS 时间。这一层只需要平滑单调，绝对精度不敏感。

如果上位机侧也想要硬件精度（例如以后在上位机上直接挂相机，要和 MCU 数据融合），树莓派可以用 `pps-gpio` 内核驱动接同一根脉冲：`config.txt` 加 `dtoverlay=pps-gpio,gpiopin=18` 得到 `/dev/pps0`，边沿时间戳在内核 ISR 里打，比用户态翻 GPIO 准一个量级。**但按上面的拆分，这一步是可选优化，不是前提。**

差分精度预算（两块 MCU 之间，这是真正要保证的那一条）：

| 环节 | 误差量级 |
| --- | --- |
| ESP32 侧 MCPWM 硬件捕获 | < 1 µs |
| 脉冲之间的晶振漂移（1 Hz × ±20 ppm） | < 20 µs |
| **合计** | **几十 µs，对 1 ms 目标有 20 倍以上余量** |

（可选的 Pi 侧 `pps-gpio` 内核捕获再加 1~10 µs，重载下变差，但不影响上面那条预算。）

#### 三件不能做的事

1. **不要在 Pi 收到消息时重新打时间戳。** 这会把 WiFi 排队延迟混进传感器时刻，而且抖动越大错得越多。时间戳必须在**采集瞬间**由采集它的那块 MCU 打。
2. **不要混用时钟源。** 一部分代码用受 `use_sim_time` 影响的 ROS 时间、一部分用 `steady_clock`，回放时 TF 立刻报 extrapolation。
3. **不要对 WiFi 往返时间求平均。** WiFi 的延迟分布是右偏重尾的，均值被尾部拉偏；NTP/PTP 都用最小值滤波，照抄。

#### 顺带把扫描去畸变做了

2D 雷达一圈要扫约 100 ms，**每条光束的时刻都不同**。车在动时，一帧 scan 其实是沿轨迹拉开的。既然时间戳已经准了，就该把 `LaserScan` 的 `time_increment` 填对，在 Pi 侧用里程计做去畸变。旋转时这一项的量级和上表算的一样——不做的话，前面辛苦同步出来的精度会被这里吃掉。

#### 验收

- 硬件脉冲测得的 offset 曲线，跑 30 分钟不发散，skew 估计收敛到一个稳定的 ppm 值；
- 用软件 NTP 式方案独立估一遍，和硬件基准比对，给出"软件方案误差 ±X µs"这个数——**这是 D1 实验最有价值的产出**；
- 原地旋转建图，地图墙面不出现重影。

### 2.3 设计题二：节点组合与进程划分

#### 收益到底从哪来

组合节点省的东西有三项，但在 4 核 ARM 上它们的权重和你想的可能不一样：

| 收益来源 | 在你的系统里的权重 |
| --- | --- |
| 省序列化与拷贝 | **小**。2D scan 才 1~3 KB，不是点云，省不下多少 |
| 省进程上下文切换 | 中 |
| **省 DDS participant 与它的线程** | **大**。每个独立进程是一个 participant，Fast DDS 每个 participant 要起若干接收/事件/发现线程，十个节点分十个进程就是几十上百个线程在 4 个核上抢 |

所以在小 ARM 板上，**组合的主要价值是压线程数和发现流量，不是省拷贝**。这个判断和大多数教程说的不一样，但它可以被测量证实（见下），也正是面试里能讲出深度的地方。顺带一提，participant 数量下降同时也缓解了 A4 的发现开销。

#### 建议的划分：按失效域切，不按消息大小切

| 进程 | 内容 | 理由 |
| --- | --- | --- |
| micro-ROS Agent | 单独进程（它是独立二进制，无法组合） | 两块 ESP32 用一个 agent 还是两个，本身就是个实验，见 §2.4 |
| **硬件桥接** | 两个桥接节点 + odom 计算 + TF 广播 | 硬件失效域。ESP32 掉线时只重启这个，不该带走地图 |
| **SLAM** | slam_toolbox 组件 + map_server | 计算重、偶发大内存分配，崩了不该带走驱动层 |
| **导航** | nav2 各 server 组合在一个容器（`use_composition:=True`） | costmap 与 controller 之间数据量最大频率最高，组合收益集中在这里 |

一个需要自己权衡的点：`/scan` 同时被 slam_toolbox 和 local costmap 订阅，它们现在分属两个进程，于是这帧数据要跨进程投递两次。把 SLAM 和导航合成一个容器能省掉，代价是**故障隔离没了**。

> 划分边界应该画在**失效域**上，而不是画在"哪些消息大"上。性能收益可以量化、可以再优化；故障隔离一旦丢掉，就是在用一次崩溃换几个百分点的 CPU。这句话本身就是"runtime 架构"这条职责想听的答案。

#### 怎么量化（不需要 eBPF）

```bash
ls /proc/<pid>/task | wc -l                        # 单进程线程数
ps -eLf | grep -c ros                              # 全系统 ROS 线程数
pidstat -w -p <pid> 1                              # 自愿/非自愿上下文切换
perf stat -e context-switches,cpu-migrations -a -- sleep 30
perf sched record -- sleep 20 && perf sched latency --sort max
```

基线（全独立进程）与组合后各测一组，记：总线程数、每秒上下文切换、调度延迟 max、总 CPU、以及 §1.3 那根标尺上的端到端 p99。

#### 验收

组合前后五项数字的对照表，外加一句能说清楚的结论：**这次收益主要来自哪一项**。如果测出来主要是线程数和上下文切换的功劳而不是省拷贝，那就照实说——能用数据推翻常见说法，比复述常见说法有价值。

### 2.4 你现在的链路里已经埋着的问题

把 A/B/C/D 接到实物上，不用人为注入，这几个问题已经在了：

| 问题 | 说明 | 对应实验 |
| --- | --- | --- |
| **LaserScan 超 MTU** | `custom_microros.meta` 里 `UCLIENT_CUSTOM_TRANSPORT_MTU=1024`，而 360 点的 `LaserScan` 光 `ranges` 就 1440 字节，带 `intensities` 更大。一帧被切成 3~4 个 UDP 片，**best-effort 下任一片丢失整帧就废** | **A2 的实物版**，比人为放大点云真实得多 |
| **流历史只有 4 槽** | `RMW_UXRCE_STREAM_HISTORY_INPUT/OUTPUT=4`，突发时会溢出丢帧 | A2 / A5 |
| **Agent 是共享瓶颈** | 两块 ESP32 若共用一个 agent 进程，它就是单点：一条链路的重传会拖累另一条。用两个 agent 隔离，代价是多一个进程 | **A4 + B1 的实物版**，且直接连到 §2.3 的进程划分 |
| **WiFi 尾延迟** | 不用 `tc netem` 造，真实 WiFi 自带重传和退避 | **A5 的实物版**；注意它影响的是控制指令，不影响建图质量（见 §2.1） |
| **两条串口/USB 设备** | ESP32 若改走 USB 串口，C1（latency_timer、VMIN/VTIME、syscall 放大）与 C2（掉线重连）直接适用 | C1 / C2 |

建议的推进顺序：**先 §2.2 时间同步**（不做它，后面所有测量的时间轴都不可信），**再 A2**（你的 scan 现在大概率在丢帧，先把链路修对），**然后 §2.3 进程划分**（有了可信的标尺才能量化收益），最后回到 B 组做调度深挖。

---

### 2.5 两个运行时：实验该落在哪一层

"runtime 架构"在你的系统里其实有**两个**，它们是不同的东西，各自对应 JD 的不同条目。把实验放错层是最容易走歪的地方：

| 层 | 运行时 | 能做的实验 | 对应 JD |
| --- | --- | --- | --- |
| ESP32 | FreeRTOS + rclc executor | 核绑定、任务优先级、共享状态临界区、spin 周期与阻塞 | 低延迟与调度 |
| 树莓派 | rclcpp executor + DDS | executor 类型、回调组、节点组合、intra-process、QoS | **runtime 架构、通信中间件** |

**两层都要做，但重心在 Pi 那一层。** JD 写的是"基于 ROS 2 建设低延迟通信与调度"，而 executor、组合、DDS 只存在于 Pi 上；只做 ESP32 那一层，最大的一块答不上来。

#### ESP32 侧：你的代码里已经有两个可测对象

这一层的好处是**不需要造问题，现成就有**，而且是你自己写的代码，比合成 demo 好讲得多。

**一、`microros_task` 里那句无条件的 `vTaskDelay(pdMS_TO_TICKS(10))`。**（`CommHost.cpp`）它给每一条下行 `cmd_vel` 加了一个 0~10 ms 的固定延迟，与网络无关，纯粹是任务结构造成的。这是最干净的一个实验：测 `cmd_vel` 从 Pi 发出到 `CommandInput.stamp_ms` 更新的分布，改掉这一句再测一遍。**注意别简单删掉**——去掉延迟会让这个低优先级任务空转吃满 Core0，正确做法是让它阻塞在数据上而不是阻塞在时钟上。这个取舍本身就是答案。

**二、`shared_state.cpp` 的 `portENTER_CRITICAL` 临界区。** `ControlSnapshot` 约 190 字节，200 Hz 下每拍要在关中断的临界区里整体拷贝，两个核都会进这把锁。绝对开销可能只有亚微秒级——**但它关的是中断**，而 `docs/esp32_time_sync.md` 的误差预算里，GPIO 捕获中断的抖动是按 1~2 µs 记的。临界区一旦压在脉冲边沿上，这笔预算就要重算。

所以这个实验的问题是"临界区最长有多久，会不会吃掉同步预算"，而**结论很可能是"不会，可以忽略"——那也是一个好结果**，因为你是测出来的而不是假设的。真要优化，方向是 seqlock 或双缓冲原子换指针，让读侧不关中断。

#### Pi 侧：用真实负载，不用合成负载

这一层有个现实障碍：你的节点全是现成的（micro_ros_agent、slam_toolbox、nav2），**没法往别人的节点里塞一个可控的慢回调**。

但不需要塞。**slam_toolbox 的 scan 回调在 Pi 4 上本来就要几十毫秒，它就是天然的"重回调"**，量级真实、无需伪造。你要加的只是一个**探针节点**：自己写的小节点，带一个固定频率定时器，它的实际周期分布就是被测指标。真实负载 + 可控测量，比 §9.2 那套纯合成的强。

于是 §9.2 的四种配置在实机上变成：探针节点与 slam_toolbox 同进程单线程 executor → 分回调组 → 拆进程 → 组合加 intra-process。**测量脚本完全复用。**

§9.2 那套纯合成脚手架的定位因此降为**预演**：在 Pi 装好之前先把测量和画图的脚本写对，等硬件就位直接换负载。

---

## 3. A 组：通信中间件与 DDS

### A1 QoS 不兼容：订阅者一帧都收不到

- **现象**：`ros2 topic hz /scan` 有数据，自己的节点回调却一次都不进。
- **复现**：雷达驱动用 `rclcpp::SensorDataQoS()`（BestEffort + KeepLast(5)），订阅端用默认 QoS（Reliable + KeepLast(10)）。反过来也做一组：发布 Volatile、订阅 TransientLocal。
- **定位**：`ros2 topic info /scan --verbose` 对比两端 QoS；打开 `RCUTILS_LOGGING_SEVERITY=DEBUG` 看 rmw 的 incompatible QoS 警告；代码里注册 `QOS_EVENT_REQUESTED_QOS_INCOMPATIBLE` 回调，把它变成显式故障而不是静默失效。
- **修复**：订阅端匹配 `SensorDataQoS`；把 QoS 收敛到一处配置（或用 QoS overrides 参数）而不是散落在各节点。
- **验收**：注册的 incompatible 事件回调能在 1 秒内打出具体的 policy 名；改后丢帧率 0。
- **延伸**：把 Deadline 设成 1.5 倍雷达周期、Liveliness 设成 automatic，制造一次"雷达线程卡死但进程还活着"，验证 deadline missed 回调能触发降级。这正是 JD 里"runtime 架构"想听的东西——**故障要能被观测到，而不是靠人看 rviz 发现**。

### A2 大消息分片与 socket 缓冲区：一压就丢

- **现象**：单路 scan 正常；把点云放大到 2 MB/帧、10 Hz 后，接收端周期性整帧丢失，且 CPU 不高。
- **复现**：发一个 `PointCloud2`（或人为把 scan 拼大），跨机或强制走 UDP（关掉共享内存传输）。
- **定位**：
  ```bash
  nstat -az | grep -i udp          # UdpRcvbufErrors / UdpInErrors 持续增长即为内核收包缓冲溢出
  ss -unmp | grep -A1 <pid>        # 看实际 rcv buffer 与积压
  ```
  一帧 2 MB 会被切成上千个 UDP 分片，**任何一片丢了整帧就废**，这是"CPU 不高但一直丢"的典型特征。
- **修复**：
  ```bash
  sudo sysctl -w net.core.rmem_max=16777216 net.core.rmem_default=16777216
  sudo sysctl -w net.core.wmem_max=16777216 net.ipv4.udp_mem="102400 873800 16777216"
  ```
  再在 DDS 侧把 `listenSocketBufferSize` / `sendSocketBufferSize` 调到同一量级（Fast DDS 用 XML profile，经 `FASTDDS_DEFAULT_PROFILES_FILE` 注入；旧版本环境变量名是 `FASTRTPS_DEFAULT_PROFILES_FILE`）。同机场景直接改用共享内存传输绕开分片。
- **验收**：`UdpRcvbufErrors` 增量归零；丢帧率从 X% 到 0；顺带记录 CPU 占用变化。
- **面试点**：能说清"内核 socket buffer"和"DDS 自己的 History/Resource Limits"是两层不同的队列，各自溢出的表现不一样。

### A3 同机零拷贝三档对比（CPU 优化主实验）

同一份雷达数据，三种部署跑同一段回放，测中间件延迟与 CPU：

| 档 | 做法 | 预期 |
| --- | --- | --- |
| 1 独立进程 | 驱动、处理、控制各一个进程，走网络回环 | 序列化 + 内核态往返，基线 |
| 2 同进程 + 共享内存传输 | 同上但让 DDS 走 SHM | 省掉内核网络栈，仍有一次序列化 |
| 3 组件化 + 进程内通信 | `ComposableNodeContainer` + `use_intra_process_comms=true`，且**用 `unique_ptr` 发布** | 真零拷贝，中间件延迟塌到微秒级 |

- **定位方法**：`perf record -F 997 -g -p <pid> -- sleep 30`，再出火焰图。档 1 里能明显看到 CDR 序列化与 `memcpy` 的宽条，档 3 里这些条消失。这是"用 perf 定位并优化 CPU"最好讲的一个故事。
- **坑（一定要踩一次）**：进程内通信只在用 `std::unique_ptr` 发布、且发布订阅在同一个容器进程内时才零拷贝；用 `const &` 发布会退化成拷贝。DDS 的 data-sharing（真正的 loaned message 零拷贝）要求定长的 plain 类型，**`PointCloud2` 这种变长消息不满足**——能讲清这条边界，比会喊"零拷贝"有用得多。
- **验收**：三档的 `T3 - T2` p99 与进程总 CPU，做成一张三行表。

### A4 发现风暴：节点一多就集体卡顿

- **现象**：节点数从 5 涨到 30，启动阶段网络流量与 CPU 出现周期性尖峰，已有节点的回调延迟被带崩。
- **复现**：脚本批量拉起 30 个空节点，每个订阅几个话题；同时监测控制环周期抖动。
- **定位**：`sudo tcpdump -i lo -n port 7400 or portrange 7410-7500` 看多播发现报文的量；`perf top` 看 rmw 线程占比。
- **修复**：改用 Fast DDS Discovery Server（`fastdds discovery -i 0 -l 127.0.0.1 -p 11811`，客户端设 `ROS_DISCOVERY_SERVER`）把 N×N 变成 N×1；或用 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` 限定范围（旧版本对应 `ROS_LOCALHOST_ONLY=1`）。
- **验收**：30 节点全启动耗时、启动期间控制环周期 max、发现流量三项对比。

### A5 网络劣化下的尾延迟

- **复现**：`tc netem` 注入 20 ms ± 10 ms 抖动 + 1% 丢包 + 乱序，跑遥控链路。
- **观察**：Reliable QoS 在丢包下会重传，**尾延迟被放大而不是丢帧**；BestEffort 则表现为丢帧但延迟稳定。把两者的 p99 与丢帧率画在一起。
- **结论要能讲出来**：控制指令这种"过期即无用"的数据用 BestEffort + 自带序号超时检测，比用 Reliable 让它排队重传更正确。这条直接呼应本仓库 `Sim2Real_list.md` 里"控制环必须本地闭环、通信只做遥测"的判断。

**本组常见追问：** Reliable 到底可靠在哪一层？History KeepLast 深度和 Reliable 的关系？跨机时 SHM 为什么失效？为什么大点云不能用 data-sharing？

---

## 4. B 组：Executor 与调度（低延迟的核心战场）

### B1 单线程 executor 的队头阻塞

- **现象**：接上雷达之后，200 Hz 控制定时器周期从 5 ms 抖到 40 ms，CPU 却只有 30%。
- **复现**：在 scan 回调里放一段 30 ms 的处理（或直接 `std::this_thread::sleep_for(30ms)`），同一个节点里挂一个 5 ms 定时器，用默认单线程 executor。
- **定位**：
  - 侵入式：定时器里记 `steady_clock` 相邻间隔，出直方图，能看到明显的 40 ms 峰。
  - 非侵入式：`ros2 trace` 采集后看 `callback_start` / `callback_end`，直接看到定时器回调被排在长回调后面。
  - 内核视角：`trace-cmd record -e sched_switch -e sched_wakeup -P <tid>`，用 KernelShark 看该线程"被唤醒到真正上 CPU"之间隔了多久——如果唤醒就跑，说明不是被内核抢占，而是**根本没被 executor 唤醒**，问题在用户态排队。这个区分是整组实验里最值钱的一句话。
- **修复三选一（要能说清各自代价）**：
  1. `MultiThreadedExecutor` + 把定时器放进独立的 `MutuallyExclusive` 回调组，慢回调放另一个组；
  2. 长处理挪出回调，丢给工作线程 + 无锁队列，回调只做搬运；
  3. 直接拆进程，用 A3 的零拷贝把跨进程代价补回来。
- **验收**：周期抖动 p99 与 max，修复前后各一组直方图。

### B2 回调组切分的正确姿势

在 B1 基础上做一组反例：把所有回调都塞进 `Reentrant` 组并开 8 线程。跑起来会更快，但共享状态开始出现竞态。用 **TSan**（`-fsanitize=thread`）或 valgrind 的 **helgrind/DRD** 把竞态抓出来，然后改回"按数据所有权划分 MutuallyExclusive 组"。

结论：**回调组不是并发开关，是数据竞争的边界声明。** 这句话在面试里比任何性能数字都好用。

### B3 实时优先级、核隔离与中断亲和

- **目标**：在 `stress-ng` 满载 CPU 的情况下，控制环周期 max 仍然可控。
- **步骤**：
  1. 基线：满载下测周期抖动（一般 max 会到几十毫秒）。
  2. 给控制线程 `SCHED_FIFO`：`sudo chrt -f 80 ros2 run ...`（需要在 `/etc/security/limits.conf` 放开 `rtprio`）。注意**只提控制线程**，别把整个进程包括 DDS 线程一起提到高优先级，那会制造新的优先级反转。
  3. 隔核：内核参数 `isolcpus=2,3 nohz_full=2,3 rcu_nocbs=2,3`，控制线程 `taskset -c 2`。
  4. 中断亲和：把雷达 USB 控制器的中断从隔离核上赶走，`cat /proc/interrupts` 找到号，`echo <hexmask> > /proc/irq/<N>/smp_affinity`。
- **定位**：`perf sched record -- sleep 20` 后 `perf sched latency --sort max` 直接给出每个线程的最大调度延迟；ftrace 的 `wakeup_rt` tracer 可以给出实时任务的唤醒延迟上界。
- **验收**：满载下周期 max 的四个数（基线 / +FIFO / +隔核 / +中断亲和），画成一张递降的表。

### B4 缺页与内存锁定：第一次跑总是慢

- **现象**：启动后前几秒周期抖动很大；或者内存压力一来周期就炸。
- **定位**：`perf stat -e page-faults,minor-faults,major-faults -p <pid> -- sleep 30`；`/proc/<pid>/status` 看 `VmHWM`。
- **修复**：`mlockall(MCL_CURRENT|MCL_FUTURE)`；启动时预热堆（预分配后 touch 一遍再释放到 pool）；关掉该进程的 THP；控制路径上禁止 `new`/`malloc`（消息用预分配池）。
- **验收**：major fault 归零、启动后前 5 秒的周期 max 与稳态一致。

### B5 PREEMPT_RT 前后对比（加分项主实验）

- **基线**：普通内核跑 `sudo cyclictest -m -S -p 90 -i 200 -h 400 -D 10m`，同时用 `stress-ng` + `fio` 满载。记 max 与直方图长尾。
- **换 RT 内核**：Ubuntu 可用 Pro 提供的 realtime 内核，或自行打 PREEMPT_RT 补丁编译。
- **复测**：同样负载同样命令，对比 max。典型结果是 max 从毫秒级降到几十微秒级。
- **再往上一层**：把 B3 的控制环搬到 RT 内核上跑，看端到端 p99 的改善**远小于** cyclictest 的改善——因为瓶颈这时已经在 DDS 和驱动，不在调度。**能说出这个"优化收益转移"的观察，比单纯报 cyclictest 数字有说服力得多。**

**本组常见追问：** SCHED_FIFO 和 SCHED_DEADLINE 怎么选？优先级反转如何避免（PI mutex）？为什么不能给 DDS 线程也设最高优先级？PREEMPT_RT 让什么变成了可抢占的？

---

## 5. C 组：设备驱动

### C1 串口延迟与 syscall 放大（性价比最高的一个实验）

- **现象**：雷达标称 10 Hz，但驱动测出来的 `T1 - T0` 有固定的十几毫秒延迟，且方差小得可疑——固定偏移通常是配置问题，不是负载问题。
- **两个真凶**：
  1. **FTDI 的 latency_timer 默认 16 ms**。如果雷达用的是 `ftdi_sio`：
     ```bash
     cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer   # 多半是 16
     echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
     ```
     固化成 udev 规则。注意 CP210x 等其他桥接芯片没有这个旋钮，得从 termios 和 URB 那边想办法。
  2. **termios 的 `VMIN`/`VTIME` 配错**，导致 `read()` 要么攒够字节才返回、要么每次超时等待。
- **另一条线是 syscall 放大**：很多驱动逐字节 `read()` 找帧头。
  ```bash
  strace -c -f -p <pid>                       # 看 read 调用次数占比
  sudo perf trace -p <pid> -s                 # 同样目的，开销更低
  ```
  改成一次读一大块进环形缓冲、在用户态做帧同步，syscall 次数可以掉两个数量级。
- **验收**：`T1 - T0` 的 p50/p99、每秒 `read()` 次数、驱动线程 CPU，三项前后对比。

### C2 USB 掉线与重连状态机

- **复现**：`sudo usbreset <bus:dev>`，或直接物理拔插，或用 `echo 0 > /sys/bus/usb/devices/<x>/authorized` 模拟。
- **要暴露的问题**：驱动 `read()` 返回 -1 后是不是死循环刷日志（顺带把 CPU 打满、把磁盘写爆）；`/dev/ttyUSB0` 重新枚举后编号变成 `ttyUSB1` 导致再也连不上；上层节点没收到任何状态变化，静默地拿着 5 分钟前的旧数据继续跑。
- **修复**：udev 按序列号做固定 symlink；驱动内做带指数退避的重连状态机；对外发布 `/diagnostics`；订阅端加数据年龄检查。
- **验收**：拔插 20 次全部自动恢复；恢复时间 p99；断连期间 CPU 不升高；上层在 1 个周期内感知到降级。
- **面试点**：这题考的不是 USB，是**驱动的失效语义**——"没有数据"和"数据很旧"必须是两个可区分的、可上报的状态。

### C3 驱动线程模型对比

同一个雷达，三种读法测 CPU 与延迟：忙轮询 `read()`、阻塞 `read()` 配独立线程、`epoll` 多路复用（雷达 + IMU + 底盘串口一起管）。得出的表能直接回答"你会怎么写一个多设备驱动节点"。

### C4 影像流水线：Media Controller 与 DMA-BUF 零拷贝

串口驱动只能练到"读字节"这一层，这个实验才练得到现代 Linux 媒体子系统的核心：**缓冲区所有权与跨设备零拷贝**。

先分清两个常被混为一谈的概念，它们不在同一层，不构成选项对比：

| | 是什么 | 位置 |
| --- | --- | --- |
| **CSI-2** | MIPI 定义的**物理链路**标准，规定差分通道与像素打包 | 硬件，与操作系统无关 |
| **V4L2** | Linux 内核给用户态的 **API**（`/dev/videoX` 上的 ioctl） | 内核与用户态的边界 |

USB 摄像头和 CSI 摄像头**都**通过 V4L2 访问，区别在驱动栈形态：UVC 模组把传感器、ISP 甚至编码器封在里面，Linux 只看到一个黑盒设备节点；CSI 模组通常只是裸传感器，去马赛克、AWB、缩放、编码由 SoC 上各自独立的硬件块完成，每块一个驱动，用 Media Controller 组成一张可查看、可重配的拓扑图。**驱动实验要的就是这张图**，所以选 CSI 而不是 UVC。

- **对象**：CSI 传感器 → CSI-2 接收器 → ISP → （编码器）。Pi 5 上是 `rp1-cfe` + PiSP，Pi 4 上是 `unicam` + `bcm2835-isp`；Pi 4 还多一级硬件 H.264 编码器 `bcm2835-codec`（M2M），Pi 5 上改用 H.265 硬解做 M2M 那一课。
- **要搞清的机制**：`VIDIOC_REQBUFS` / `QBUF` / `DQBUF` 的队列模型与谁持有缓冲区；`V4L2_MEMORY_MMAP` 与 `V4L2_MEMORY_DMABUF` 的区别；用 `media-ctl -p` 打印媒体控制器拓扑，看清 subdev 之间的链路。
- **实验主体**：对比两条管线，一条让每级都经 CPU 拷贝，一条用 DMA-BUF 把 fd 在设备间传递。测编码全程的 CPU 占用与内存带宽。
  ```bash
  v4l2-ctl --list-devices && v4l2-ctl -d /dev/video11 --list-formats-out
  media-ctl -p                       # 媒体拓扑
  perf stat -e cache-misses,bus-cycles -p <pid> -- sleep 20
  ```
- **和控制环的耦合**：编码器是独立硬件引擎，但它和 CPU **抢同一条内存带宽**。开编码后测控制环周期抖动——这是一类纯软件视角看不到的性能问题（CPU 利用率没涨，延迟却涨了），能讲出来很加分。
- **验收**：两条管线的 CPU 占用、编码延迟、以及编码开启前后控制环周期 p99。

---

## 6. D 组：数据同步与日志系统

### D1 多源时间戳对齐

> 三设备拓扑下的完整同步方案（精度预算、硬件脉冲、验收）见 **§2.2**，那是本实验在你实际系统上的落地版本。这里保留通用部分。

三个源：雷达 10 Hz（USB）、IMU 200 Hz（ESP32 上报）、轮速里程计 100 Hz。

- **要复现的问题**：
  1. **时间戳源混用**——一部分代码用 `node->now()`（受 `use_sim_time` 影响的 ROS 时间），一部分用 `steady_clock`，回放时 TF 立刻报 extrapolation。
  2. **ESP32 与主机时钟不同步**，且晶振有漂移。这是本仓库场景里最真实的一个同步问题。
  3. `message_filters` 的 `ApproximateTime` 队列太短，高负载下同步成功率暴跌，且失败是静默的。
- **做法**：ESP32 侧用 NTP 式四时间戳交换（主机发 t1，ESP32 记 t2/t3，主机收 t4）估计 offset，再用线性回归估 skew（ppm 级漂移），把设备时间戳换算到主机时间轴。跑 30 分钟看漂移是否被吃掉。
- **量化**：同步成功率、配对时间差分布、长时间运行的时钟偏移曲线。
- **拿硬件当裁判**：用 §1.2 的 GPIO 打点做一组独立测量，检验软件估计出来的 offset 到底准到多少微秒。**没有基准的同步方案是无法验收的**，这一步能把结论从"看起来对"变成"误差 ±X µs"。
- **加分做法**：如果有两台上位机，用 `ptp4l` + `phc2sys` 做一次硬件时间戳同步，对比 chrony 的软件同步精度（微秒 vs 毫秒量级）。

### D2 rosbag2 记录引发的 IO 抖动（IO 优化主实验）

- **现象**：一开始录包，控制环周期就出现周期性尖刺，间隔恰好和脏页回写周期吻合。
- **复现**：`ros2 bag record -a` 全量录制（含点云），同时监测周期抖动；用机械硬盘或 U 盘更容易复现。
- **定位**：
  ```bash
  iostat -x 1                                   # await / %util 尖峰
  sudo /usr/share/bcc/tools/biolatency -m 1 20  # 块层延迟直方图
  sudo /usr/share/bcc/tools/biosnoop            # 谁在写、单次多久
  cat /proc/meminfo | grep -i dirty             # 脏页水位
  ```
- **修复选项（逐个测收益）**：录包进程独立 cgroup 限 `IOWeight` + `ionice -c3`；`--max-cache-size` 加大做批量落盘；换 mcap 存储 + zstd 压缩（用 CPU 换 IO）；话题白名单而不是 `-a`；写到独立盘；调 `vm.dirty_background_ratio` 让回写更平滑。
- **验收**：控制环周期 max、块层延迟 p99、录包丢消息数，三项对照。
- **面试点**：能指出"日志系统的正确性目标是**不影响被观测系统**"，并给出 cgroup 隔离这个答案。

### D3 日志写阻塞控制环

- **复现**：在控制回调里加一条无节流的 `RCLCPP_INFO`，200 Hz 输出到终端，再把 stdout 重定向到慢速磁盘或一个满的管道。
- **定位**：`perf trace -e write -p <pid>` 看 `write()` 阻塞时长；或 `bpftrace -e 'tracepoint:syscalls:sys_enter_write /comm=="ctrl_node"/ { @[comm] = count(); }'`。
- **修复**：`RCLCPP_INFO_THROTTLE`；日志分级并把高频诊断改走独立话题；异步日志（生产者只入队，落盘另起线程）；确认 stdout 行缓冲在重定向后变成全缓冲带来的行为差异。
- **验收**：控制回调耗时 p99，以及"日志目标变慢时控制环是否还稳定"这个鲁棒性结论。

**本组常见追问：** 为什么不能在实时回调里做任何 IO？异步日志的队列满了怎么办（丢日志还是阻塞业务，怎么选）？

---

## 7. E 组：Crash 与内存定位

### E1 用户态 crash：从崩溃到栈帧的完整流程

- **制造**：在雷达驱动里埋一个越界写（比如帧长字段来自设备但没做上界检查，喂一段构造的坏帧）。**注意这正是真实驱动最常见的漏洞形态**，比 `int *p = nullptr` 有说服力。
- **流程**：
  ```bash
  ulimit -c unlimited
  cat /proc/sys/kernel/core_pattern           # 确认是 systemd-coredump 还是文件
  coredumpctl list && coredumpctl gdb <pid>   # 或 gdb <bin> <core>
  (gdb) bt full ; info registers ; thread apply all bt
  ```
- **必须演练的一环**：release 构建（`-O2`）没有符号时怎么办——用 `-g -O2` 编译后 `objcopy --only-keep-debug` 分离出 debuginfo，部署时不带、定位时加载。**这是"能定位线上 crash"和"只能定位本地 crash"的分界线。**
- **验收**：能从 core 还原出越界发生的具体行、以及那一帧的设备数据。

### E2 valgrind 与 sanitizer 的分工

| 工具 | 抓什么 | 代价 | 什么时候用 |
| --- | --- | --- | --- |
| valgrind memcheck | 越界、未初始化读、泄漏 | 20~50× 慢 | 离线回放，**绝不能**在实车实时环上跑 |
| ASan | 越界、UAF | 约 2× | 日常 CI 与仿真跑 |
| TSan | 数据竞争 | 5~15× | 专门验 B2 的回调组切分 |
| UBSan | 未定义行为 | 很小 | 长期挂着 |
| helgrind / DRD | 锁序、竞态 | 很慢 | 没法上 TSan 时的替补 |

```bash
ros2 run --prefix 'valgrind --tool=memcheck --leak-check=full --track-origins=yes --log-file=vg.%p.log' <pkg> <node>
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="-fsanitize=address -fno-omit-frame-pointer"
```

**要能主动说出的一句话**：valgrind 会让时序完全失真，所以它只能用来验"逻辑正确性"，不能用来验"实时性"；实时路径上的内存问题要靠 ASan + 回放数据集来抓。

### E3 内存增长与泄漏

- **复现**：让节点跑 8 小时，`while true; do ...; done` 反复重连雷达、反复创建订阅。
- **观察**：RSS 曲线（`pidstat -r -p <pid> 1` 采样后画图）。区分三种情况：真泄漏、glibc 堆碎片（RSS 不降但没泄漏，用 `malloc_trim` 验证）、缓存类正常增长。
- **工具**：`valgrind --tool=massif` + `ms_print`；`heaptrack` 更适合长跑（开销小、有火焰图）。
- **验收**：8 小时 RSS 斜率接近 0；如果不是泄漏而是碎片，要能拿出证据证明。

### E4 内核态：oops / panic 与 kdump 演练（在 V 档虚拟机做）

- **准备**：`sudo apt install kdump-tools`，`kdump-config show` 确认 crashkernel 内存已预留、`vmcore` 目录可写。
- **触发**：`echo c | sudo tee /proc/sysrq-trigger`（受控 panic）。
- **分析**：
  ```bash
  crash /usr/lib/debug/boot/vmlinux-$(uname -r) /var/crash/*/dump.*
  crash> bt ; ps ; log ; dmesg
  ```
- **更接近真实的一版**：写一个几十行的内核模块，在里面解引用空指针制造 oops，然后从 `dmesg` 的 RIP 与调用栈，用 `addr2line`/`faddr2line` 定位到模块源码行。
- **要能讲清的边界**：oops 与 panic 的区别、`panic_on_oops` 的取舍、为什么生产环境上宁可 panic + kdump 也不要带着损坏状态继续跑。

**本组常见追问：** core dump 在容器里怎么落盘？crash 发生在第三方 .so 里怎么办？内存越界为什么有时崩在完全无关的地方？

---

## 8. F 组：eBPF（加分项，可延后）

eBPF 的卖点是**不改代码、不重启、开销小**地看进生产系统。四个实验直接对应前面几组的定位难点。

> **可以先只理解原理、不动手。** 本组是加分项，且在树莓派上还要先解决内核 BTF 的问题（§1.1 坑 2）。下表给出每个 eBPF 用途的 perf/ftrace 替代方案——**主线实验一个都不会因为跳过 eBPF 而缺失**，只是定位时要多绕一步。等前面几组做完有余力再回头补。
>
> | eBPF 工具 | 用途 | 不用 eBPF 的替代 |
> | --- | --- | --- |
> | `runqlat` | 等 CPU 等了多久 | `perf sched record` + `perf sched latency --sort max` |
> | `offcputime` | 不在 CPU 上时卡在哪 | `perf record -e sched:sched_switch -g`，或 ftrace 的 `sched_switch` + `sched_wakeup` 配对 |
> | `biolatency` | 块层延迟分布 | `iostat -x 1` 看 `await`，粒度粗但够定位 |
> | `kfree_skb` 归因 | 丢包丢在哪一层 | `nstat -az` 的 `UdpRcvbufErrors` 等计数器 |
>
> 面试时的说法也很清楚：能说明白"`runqlat` 长尾和 `offcputime` 长尾分别指向什么根因"，比装过工具更重要。

### F1 "控制线程为什么没跑"

```bash
sudo /usr/share/bcc/tools/runqlat -p <pid> 10 1     # 调度延迟直方图：等 CPU 等了多久
sudo /usr/share/bcc/tools/offcputime -p <pid> -f 30 > off.stacks   # 不在 CPU 上时卡在哪个栈
```

`runqlat` 长尾说明是**被抢占/CPU 不够**（去 B3 加优先级、隔核）；`offcputime` 显示卡在锁或 `read()` 说明是**自己在等**（去 B1 拆回调、去 C1 修驱动）。**这两个图能把"卡顿"一刀切成两类根因，是这组最值得讲的价值。**

### F2 块层延迟归因（配合 D2）

```bash
sudo /usr/share/bcc/tools/biolatency -m 5 1
sudo /usr/share/bcc/tools/biosnoop | grep -i bag
```

### F3 丢包丢在哪一层（配合 A2）

```bash
sudo bpftrace -e 'tracepoint:skb:kfree_skb { @[kstack] = count(); }'
```

较新内核的 `kfree_skb` 带 `reason` 字段（如 `SKB_DROP_REASON_SOCKET_RCVBUFF`），能一步定位到"就是收缓冲满了"，与 A2 的 `nstat` 结论互相印证。

### F4 自写探针

用 `bpftrace` 写一个统计某进程 `write()` 按 fd 分布的单行，验证 D3 的日志放大；再写一个 uprobe 挂到驱动的解包函数上测函数级延迟分布（`funclatency`）。能现场写出一条 bpftrace 单行，比说"了解 eBPF"强太多。

---

## 9. 时间不够时的取舍

顺序不是按 JD 条目排的，是按**依赖关系**排的：前面的不做，后面的测量不可信。

| 序 | 做什么 | 为什么在这个位置 | 覆盖的 JD 条目 |
| --- | --- | --- | --- |
| 1 | **§2.2 三设备时间同步** | 时间轴不可信，后面所有实验的数字都不可信 | 数据同步、驱动、硬件时间戳 |
| 2 | **A2 大消息分片丢包** | 你的 scan 现在大概率就在丢帧（§2.4），先把链路修对 | 通信中间件、内核网络栈调优 |
| 3 | **B1 executor 队头阻塞** | 有了可信标尺，才能量化排队延迟 | runtime 架构、调度、ftrace |
| 4 | **§2.3 进程划分与组合** | 依赖 B1 的结论决定怎么切 | runtime 架构、CPU 优化 |
| 5 | **C1 串口延迟与 syscall 放大** | 独立，随时可做，投入小产出大 | 设备驱动、低延迟 |
| 6 | **D2 rosbag 引发 IO 抖动** | 独立 | 日志系统、IO 优化 |
| 7 | **E1 core dump 全流程** | 独立 | 用户态 crash 定位、符号化 |

加分项两个：**B5**（cyclictest 前后对比，一晚上能出数）和 **A3**（零拷贝三档对比，和第 4 项共享测量脚手架，顺手就做了）。

**F 组 eBPF 可以只理解原理不动手**，替代方案见第 8 节开头的对照表。E4 内核 crash 投入最大、离日常最远，放最后；但至少把流程走通一次，这样被问到时能说"我搭过 kdump、用 crash 看过 vmcore"，而不是背概念。

### 9.1 如果重点收在「性能调优 + runtime 架构」

这两项对应上表的第 1、3、4 项，加上 A3、B2、B3、B5。其余按下面处置：

| 实验 | 处置 | 理由 |
| --- | --- | --- |
| **C4 影像流水线 / DMA-BUF** | **砍掉** | 属于设备驱动方向，投入最大，且要额外买摄像头。**砍掉它之后 §1.1 里 Pi 4 与 Pi 5 的取舍随之失效**——硬件编码器是那场比较的唯一争点，不做视觉就按手头能拿到的买 |
| A4 发现风暴 | 砍掉 | 节点规模不够，造不出真实的发现风暴，只能靠脚本硬凑 |
| A5 网络劣化尾延迟 | **保留** | 你的 micro-ROS 走 WiFi，这一条不是假设而是现状（§2.4） |
| C1 / D2 / E1 | 保留 | 各自独立、投入小，分别覆盖驱动、IO、用户态 crash 三条 JD |
| E4 内核 kdump | 一次性演练 | x86 虚拟机上把流程走通一次即可，守住 JD 里"内核态 crash"这一条 |
| F 组 eBPF | 只理解原理 | 见第 8 节开头的对照表 |

砍掉 C4 之后，**剩下的实验全部只需要现有设备**（雷达、底盘、两块 ESP32）加一块 Linux 单板，不需要再采购。

### 9.2 结构性实验不必等硬件，现在就能开始

§1.1 说"被测对象要放树莓派"，这话有个前提没写清楚。把实验分成两类就清楚了：

| 类别 | 研究的是什么 | 换到 x86 上会怎样 | 代表 |
| --- | --- | --- | --- |
| **资源性** | 资源不够时会发生什么 | **现象直接消失**，只能靠 `stress-ng` 硬造 | B4 缺页、D2 IO 抖动、B3 核隔离取舍、A3 的收益幅度 |
| **结构性** | 排队顺序、回调调度、线程与进程划分、序列化路径 | **机制完全一致**，只有数值大小变 | B1、B2、A1、§2.3、A3 的机制部分 |

§1.1 那段针对的是资源性那一类。**而 runtime 架构几乎全在结构性这一边**——单线程 executor 里一个慢回调会阻塞定时器，这是 executor 的逻辑决定的，跟 CPU 多快无关；进程划分改变的是线程数与 DDS 参与者数量，这个账在哪台机器上都一样算。

所以 Pi 还没装好之前，**用 x86 上纯合成的节点就能把 B1 → B2 → §2.3 整条线跑完**，连 ESP32 和雷达都不用接。

> 但这只是**预演**。实机上有更好的做法：用真实的 slam_toolbox 当重回调、自己加一个探针节点当受害者，见 §2.5。下面这套脚手架的价值在于让你先把测量与画图脚本写对，等硬件就位直接换负载。

#### 一套脚手架覆盖三个实验

三个节点，数据全部合成：

- `fake_scan`：10 Hz 发一个 `LaserScan` 尺寸的消息（360 点）
- `heavy`：订阅它，每帧占用约 50 ms，模拟 scan matching
- `ctrl`：100 Hz 定时器，**它的实际周期分布就是被测指标**

四种配置各跑 5 分钟，**只改组织方式，业务代码一行不动**：

| 配置 | 预期结果 |
| --- | --- |
| 1. 单进程 + `SingleThreadedExecutor`，全用默认回调组 | `ctrl` 的周期尾部被 `heavy` 顶到 50 ms 以上——B1 的现象 |
| 2. 同上，`heavy` 挪进 `ReentrantCallbackGroup` + `MultiThreadedExecutor` | 尾部回落——B2 的解法 |
| 3. 拆成三个独立进程 | 尾部同样好，但线程数与上下文切换涨一截 |
| 4. 组合进单进程 + `use_intra_process_comms` | §2.3 的目标形态；和配置 3 对比才知道省下的是什么 |

每种配置记同一组数字（命令见 §2.3）：`ctrl` 周期的 p50/p99/max、线程数、每秒上下文切换、`perf sched latency --sort max`、总 CPU。回调级别的耗时用 `ros2_tracing` 取，比自己埋点干净。

**这套脚手架能整个搬到 Pi 上重跑。** 到时候把 `heavy` 换成真的 scan matching、`fake_scan` 换成 ESP32-A 的真数据，配置 1~4 一行都不用改——于是你顺手拿到一张 x86/Pi 对照表，正好就是 §1.1 说的"做一次就够"的那次对照。

#### 这样做会漏掉什么

- **数值不可搬运。** x86 上测出"组合省了 12% CPU"，到 Pi 上不成立，必须重测。只把 x86 的结论当作"机制成立"，别当作"收益是多少"。
- **DDS 走的是同机路径。** 默认会走共享内存，跨机发现、网络 QoS、A2 分片这些一个都碰不到，必须等硬件（或者至少两台机器）。
- **ARM 的弱内存序测不到。** x86 的 TSO 会掩盖数据竞争，无锁代码到 ARM 上必须再验一次。

---

## 10. 一页纸模板（每个实验做完就填）

```
【现象】   什么条件下、什么指标、坏到什么程度（带数字）
【假设】   当时怀疑的 2~3 个方向，以及为什么先查这个
【定位】   用了什么工具、看到了什么关键证据（贴图或贴那几行输出）
           —— 关键是要能说清「这个证据如何排除了另外两个假设」
【修复】   改了什么，为什么这么改，代价是什么
【数据】   前 / 后：p50、p99、max、丢帧率、CPU、内存
【边界】   这个修复在什么情况下会失效
```

最后一栏最容易被忽略，但**面试官最爱追问的就是它**。能主动说出"这个优化在跨机场景下不成立"或"这个参数在低端 CPU 上要重新标"，说明你是在做工程，不是在抄配置。
