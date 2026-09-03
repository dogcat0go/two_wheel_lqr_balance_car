#!/usr/bin/env python3
"""双轮足「预武装」场景下的 Executor 死锁演示（ROS 2 Humble / rclpy）。

故事：编排器 50 Hz 定时器想从 Observe 武装到 Balance，但必须先问
MCU 侧 /mcu/prearm_check（倾角是否足够小）。若把「问服务」写成
定时器里同步等待，而服务又由同一个 executor 来跑，就会自己等自己。

不依赖 Gazebo 也能跑（内部模拟 pitch 收敛）。仿真车开着时加 --use-imu，
会订 /imu/data，用真车体倾角当预武装条件。

用法：
  source /opt/ros/humble/setup.bash
  python3 arming_executor_demo.py --mode deadlock
  python3 arming_executor_demo.py --mode exclusive
  python3 arming_executor_demo.py --mode split
  python3 arming_executor_demo.py --mode async
  python3 arming_executor_demo.py --mode async --use-imu   # Gazebo 开着时

死锁两种（日志停在 “waiting prearm”，再也没有 [prearm]）：
  deadlock   SingleThreadedExecutor + 定时器里 future.result()
  exclusive  MultiThreadedExecutor，但定时器和服务在同一个互斥组

能跑通两种：
  split      多线程 + 两个互斥组；定时器里仍阻塞等待（能通，但控制拍被卡住，不进真环）
  async      定时器只发请求、下拍看 future；控制拍不阻塞（正确做法）
"""

from __future__ import annotations

import argparse
import math
import threading

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger


def pitch_from_imu(msg: Imu) -> float:
    q = msg.orientation
    s = 2.0 * (q.w * q.y - q.z * q.x)
    return math.asin(max(-1.0, min(1.0, s)))


class ArmingDemo(Node):
    def __init__(self, mode: str, use_imu: bool, pitch_limit: float):
        super().__init__("arming_executor_demo")
        self.mode = mode
        self.use_imu = use_imu
        self.pitch_limit = pitch_limit
        self.armed = False
        self._sim_pitch = 0.40
        self._imu_pitch = None
        self._future = None

        if mode in ("deadlock",):
            self.grp_ctrl = None
            self.grp_comm = None
        elif mode == "exclusive":
            g = MutuallyExclusiveCallbackGroup()
            self.grp_ctrl = g
            self.grp_comm = g
        elif mode == "split":
            self.grp_ctrl = MutuallyExclusiveCallbackGroup()
            self.grp_comm = MutuallyExclusiveCallbackGroup()
        else:  # async：控制组互斥，通信组可重入，避免服务响应卡控制拍
            self.grp_ctrl = MutuallyExclusiveCallbackGroup()
            self.grp_comm = ReentrantCallbackGroup()

        self.srv = self.create_service(
            Trigger, "/mcu/prearm_check", self.on_prearm, callback_group=self.grp_comm
        )
        self.cli = self.create_client(
            Trigger, "/mcu/prearm_check", callback_group=self.grp_comm
        )
        if use_imu:
            self.create_subscription(
                Imu, "/imu/data", self.on_imu, 20, callback_group=self.grp_comm
            )
        self.create_timer(0.05, self.on_timer, callback_group=self.grp_ctrl)
        self.get_logger().info(
            "mode={} use_imu={} pitch_limit={:.2f} rad. 等服务就绪后开始武装。".format(
                mode, use_imu, pitch_limit
            )
        )

    def current_pitch(self) -> float:
        if self.use_imu and self._imu_pitch is not None:
            return self._imu_pitch
        return self._sim_pitch

    def on_imu(self, msg: Imu):
        self._imu_pitch = pitch_from_imu(msg)

    def on_prearm(self, request, response):
        pitch = self.current_pitch()
        ok = abs(pitch) < self.pitch_limit
        response.success = ok
        response.message = "pitch={:.3f}".format(pitch)
        self.get_logger().info(
            "[prearm] pitch={:.3f} -> {}".format(pitch, "OK" if ok else "reject")
        )
        return response

    def on_timer(self):
        if self.armed:
            return
        if not self.use_imu:
            self._sim_pitch *= 0.92
        pitch = self.current_pitch()
        self.get_logger().info(
            "[timer] Observe, pitch={:.3f}, thread={}".format(
                pitch, threading.current_thread().name
            )
        )
        if not self.cli.service_is_ready():
            self.get_logger().warn("[timer] prearm 服务未就绪")
            return

        if self.mode == "async":
            self._async_arm()
        else:
            self._blocking_arm()

    def _blocking_arm(self):
        self.get_logger().warn(
            "[timer] 同步等待 /mcu/prearm_check …… 若再也没有 [prearm]，就是死锁（Ctrl-C 退出）"
        )
        future = self.cli.call_async(Trigger.Request())
        # 回调不返回，executor 就调度不到 on_prearm。
        # deadlock / exclusive 故意死等；split 有超时，正常应在几十毫秒内返回。
        try:
            if self.mode in ("deadlock", "exclusive"):
                resp = future.result()
            else:
                resp = future.result(timeout_sec=2.0)
        except Exception as exc:
            self.get_logger().error("[blocking] wait failed: {}".format(exc))
            return
        if resp is not None and resp.success:
            self.armed = True
            self.get_logger().warn("[blocking] ARMED -> Balance  ({})".format(resp.message))
        else:
            msg = "" if resp is None else resp.message
            self.get_logger().info("[blocking] 未通过预武装 ({})".format(msg))

    def _async_arm(self):
        if self._future is not None:
            if not self._future.done():
                return
            self._consume_future(self._future, tag="async")
            self._future = None
            return
        self.get_logger().info("[timer] 异步发出 prearm，本拍立即返回")
        self._future = self.cli.call_async(Trigger.Request())

    def _consume_future(self, future, tag: str):
        if not future.done():
            self.get_logger().error(
                "[{}] 2s 内无响应（死锁或服务没跑到）".format(tag)
            )
            return
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().error("[{}] exception: {}".format(tag, exc))
            return
        if resp is not None and resp.success:
            self.armed = True
            self.get_logger().warn(
                "[{}] ARMED -> Balance  ({})".format(tag, resp.message)
            )
        else:
            msg = "" if resp is None else resp.message
            self.get_logger().info("[{}] 未通过预武装 ({})".format(tag, msg))


def spin_node(args) -> None:
    rclpy.init()
    node = ArmingDemo(args.mode, args.use_imu, args.pitch_limit)
    if args.mode in ("deadlock", "async"):
        executor = SingleThreadedExecutor()
    else:
        executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    node.get_logger().info("executor={}".format(type(executor).__name__))
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def main():
    p = argparse.ArgumentParser()
    p.add_argument(
        "--mode",
        choices=("deadlock", "exclusive", "split", "async"),
        default="deadlock",
    )
    p.add_argument("--use-imu", action="store_true", help="订 /imu/data（Gazebo 开着时）")
    p.add_argument("--pitch-limit", type=float, default=0.15)
    spin_node(p.parse_args())


if __name__ == "__main__":
    main()
