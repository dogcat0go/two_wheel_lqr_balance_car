#!/usr/bin/env python3
"""
stage2 串口：收发分终端 + 主机落盘。

  终端 A（只读遥测，串口默认 /dev/ttyUSB0）:
    ~/.platformio/penv/bin/python3 tools/stage2_serial_bridge.py -p /dev/ttyUSB0
  终端 B（只发命令，经 FIFO，不是串口）:
    echo 'e 30' > /tmp/fishbot_cmd

默认把 RX/TX 记到 logs/stage2_YYYYMMDD_HHMMSS.log（相对工程根目录），
每行带主机时间戳，便于事后对照验收。--no-log 关闭；-l PATH 指定文件。

先停掉 IDE 里的 device monitor，否则抢不到 /dev/ttyUSB0。
"""
from __future__ import print_function

import argparse
import os
import select
import stat
import sys
from datetime import datetime

try:
    import serial
except ImportError:
    sys.stderr.write(
        "need pyserial, run with:\n"
        "  ~/.platformio/penv/bin/python3 tools/stage2_serial_bridge.py\n"
    )
    sys.exit(1)

FIFO_DEFAULT = "/tmp/fishbot_cmd"
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
    name = "stage2_%s.log" % datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(logs_dir, name)


class LineLogger(object):
    """按行落盘，主机时间戳；缺换行的尾巴在 close 时冲掉。"""

    def __init__(self, path):
        self.path = path
        self._fp = open(path, "a", encoding="utf-8") if path else None
        self._rx_buf = ""

    def _stamp(self):
        # 毫秒：对照 200ms 遥测 / 故障边沿够用
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

    def write_rx(self, text):
        sys.stdout.write(text)
        sys.stdout.flush()
        if not self._fp:
            return
        self._rx_buf += text
        while "\n" in self._rx_buf:
            line, self._rx_buf = self._rx_buf.split("\n", 1)
            self._fp.write("%s  %s\n" % (self._stamp(), line.rstrip("\r")))
            self._fp.flush()

    def write_tx(self, cmd_text):
        # TX 也进同一文件，事后能还原「何时发了什么」
        msg = ">> %s" % cmd_text
        sys.stderr.write(msg + "\n")
        if self._fp:
            self._fp.write("%s  %s\n" % (self._stamp(), msg))
            self._fp.flush()

    def close(self):
        if self._fp and self._rx_buf:
            self._fp.write("%s  %s\n" % (self._stamp(), self._rx_buf.rstrip("\r")))
            self._rx_buf = ""
        if self._fp:
            self._fp.close()
            self._fp = None


def main():
    ap = argparse.ArgumentParser(description="ESP32 RX + FIFO TX + host log")
    ap.add_argument("-p", "--port", default="/dev/ttyUSB0")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("-f", "--fifo", default=FIFO_DEFAULT)
    ap.add_argument(
        "-l",
        "--log",
        default=None,
        help="log file path (default: logs/stage2_TIMESTAMP.log)",
    )
    ap.add_argument("--no-log", action="store_true", help="disable file logging")
    args = ap.parse_args()

    log_path = None
    if not args.no_log:
        log_path = args.log if args.log else default_log_path()
        parent = os.path.dirname(os.path.abspath(log_path))
        if parent:
            os.makedirs(parent, exist_ok=True)

    ensure_fifo(args.fifo)
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except serial.SerialException as e:
        sys.stderr.write(
            "open %s failed: %s\n(stop other monitor first)\n" % (args.port, e)
        )
        sys.exit(1)

    logger = LineLogger(log_path)
    sys.stderr.write(
        "RX on %s @ %d\nTX: echo 'e 30' > %s   (s / x / o)\n"
        % (args.port, args.baud, args.fifo)
    )
    if log_path:
        sys.stderr.write("LOG: %s\n" % log_path)
        logger._fp.write(
            "%s  # session start port=%s baud=%d\n"
            % (logger._stamp(), args.port, args.baud)
        )
        logger._fp.flush()
    else:
        sys.stderr.write("LOG: disabled\n")
    sys.stderr.write("Ctrl+C quit\n")

    fifo_fd = os.open(args.fifo, os.O_RDONLY | os.O_NONBLOCK)
    cmd_buf = b""

    try:
        while True:
            rlist, _, _ = select.select([ser, fifo_fd], [], [], 0.05)

            if ser in rlist or ser.in_waiting:
                data = ser.read(ser.in_waiting or 1)
                if data:
                    logger.write_rx(data.decode("utf-8", errors="replace"))

            if fifo_fd in rlist:
                chunk = os.read(fifo_fd, 256)
                if not chunk:
                    os.close(fifo_fd)
                    fifo_fd = os.open(args.fifo, os.O_RDONLY | os.O_NONBLOCK)
                    continue
                cmd_buf += chunk
                while b"\n" in cmd_buf:
                    line, cmd_buf = cmd_buf.split(b"\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    ser.write(line + b"\n")
                    ser.flush()
                    logger.write_tx(line.decode("utf-8", errors="replace"))
    except KeyboardInterrupt:
        sys.stderr.write("\nbye\n")
    finally:
        try:
            os.close(fifo_fd)
        except OSError:
            pass
        logger.close()
        ser.close()


if __name__ == "__main__":
    main()
