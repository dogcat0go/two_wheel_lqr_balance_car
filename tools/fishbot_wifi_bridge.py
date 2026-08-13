#!/usr/bin/env python3
"""
stage2 WiFi 文本桥：经 micro-ROS agent 收 /fishbot/log、发 /fishbot/cmd。

用法（先起 agent，再烧录/上电 ESP）：
  source /opt/ros/humble/setup.bash
  ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

  # 终端 A：看遥测 + 落盘
  python3 tools/fishbot_wifi_bridge.py

  # 终端 B：发命令（与串口同一套协议）
  echo 'r' > /tmp/fishbot_wifi_cmd
  echo 'v 0.1' > /tmp/fishbot_wifi_cmd

也可用：
  ros2 topic echo /fishbot/log
  ros2 topic pub --once /fishbot/cmd std_msgs/msg/String "{data: 'r'}"
"""
from __future__ import print_function

import argparse
import os
import select
import stat
import sys
import threading
from datetime import datetime

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:
    sys.stderr.write(
        "need rclpy (source /opt/ros/humble/setup.bash first)\n"
    )
    sys.exit(1)

FIFO_DEFAULT = "/tmp/fishbot_wifi_cmd"
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)


def ensure_fifo(path):
    if os.path.exists(path):
        if not stat.S_ISFIFO(os.stat(path).st_mode):
            sys.stderr.write("%s exists and is not a fifo\n" % path)
            sys.exit(1)
        return
    os.mkfifo(path)


def default_log_path():
    logs_dir = os.path.join(PROJECT_ROOT, "logs")
    os.makedirs(logs_dir, exist_ok=True)
    name = "wifi_%s.log" % datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(logs_dir, name)


class LineLogger(object):
    def __init__(self, path):
        self.path = path
        self._fp = open(path, "a", encoding="utf-8") if path else None

    def _stamp(self):
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

    def write_rx(self, text):
        sys.stdout.write(text if text.endswith("\n") else text + "\n")
        sys.stdout.flush()
        if self._fp:
            self._fp.write("%s  %s\n" % (self._stamp(), text.rstrip("\r\n")))
            self._fp.flush()

    def write_tx(self, cmd_text):
        msg = ">> %s" % cmd_text
        sys.stderr.write(msg + "\n")
        if self._fp:
            self._fp.write("%s  %s\n" % (self._stamp(), msg))
            self._fp.flush()

    def close(self):
        if self._fp:
            self._fp.close()
            self._fp = None


class WifiBridge(Node):
    def __init__(self, logger, log_topic, cmd_topic):
        super().__init__("fishbot_wifi_bridge")
        self._logger = logger
        self._pub = self.create_publisher(String, cmd_topic, 10)
        self.create_subscription(String, log_topic, self._on_log, 10)

    def _on_log(self, msg):
        self._logger.write_rx(msg.data)

    def send_cmd(self, line):
        line = line.strip()
        if not line:
            return
        out = String()
        out.data = line
        self._pub.publish(out)
        self._logger.write_tx(line)


def fifo_loop(node, fifo_path, stop_event):
    ensure_fifo(fifo_path)
    fifo_fd = os.open(fifo_path, os.O_RDONLY | os.O_NONBLOCK)
    cmd_buf = b""
    try:
        while not stop_event.is_set():
            rlist, _, _ = select.select([fifo_fd], [], [], 0.2)
            if fifo_fd not in rlist:
                continue
            chunk = os.read(fifo_fd, 256)
            if not chunk:
                os.close(fifo_fd)
                fifo_fd = os.open(fifo_path, os.O_RDONLY | os.O_NONBLOCK)
                continue
            cmd_buf += chunk
            while b"\n" in cmd_buf:
                line, cmd_buf = cmd_buf.split(b"\n", 1)
                text = line.strip().decode("utf-8", errors="replace")
                if text:
                    node.send_cmd(text)
    finally:
        try:
            os.close(fifo_fd)
        except OSError:
            pass


def main():
    ap = argparse.ArgumentParser(description="ESP32 /fishbot/log + /fishbot/cmd bridge")
    ap.add_argument("--log-topic", default="/fishbot/log")
    ap.add_argument("--cmd-topic", default="/fishbot/cmd")
    ap.add_argument("-f", "--fifo", default=FIFO_DEFAULT)
    ap.add_argument("-l", "--log", default=None, help="log file (default logs/wifi_*.log)")
    ap.add_argument("--no-log", action="store_true")
    args = ap.parse_args()

    log_path = None
    if not args.no_log:
        log_path = args.log if args.log else default_log_path()
        parent = os.path.dirname(os.path.abspath(log_path))
        if parent:
            os.makedirs(parent, exist_ok=True)

    logger = LineLogger(log_path)
    rclpy.init()
    node = WifiBridge(logger, args.log_topic, args.cmd_topic)

    sys.stderr.write(
        "RX %s\nTX: echo 'r' > %s\n" % (args.log_topic, args.fifo)
    )
    if log_path:
        sys.stderr.write("LOG: %s\n" % log_path)
    sys.stderr.write("Ctrl+C quit\n")

    stop_event = threading.Event()
    t = threading.Thread(
        target=fifo_loop, args=(node, args.fifo, stop_event), daemon=True
    )
    t.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        sys.stderr.write("\nbye\n")
    finally:
        stop_event.set()
        node.destroy_node()
        rclpy.shutdown()
        logger.close()


if __name__ == "__main__":
    main()
