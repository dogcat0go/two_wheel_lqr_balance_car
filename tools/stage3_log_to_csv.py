#!/usr/bin/env python3
"""把 stage2_serial_bridge 落盘的遥测 log 转成 CSV，供 PlotJuggler 等打开。

  ~/.platformio/penv/bin/python3 tools/stage3_log_to_csv.py logs/stage3_step1.log
  # → logs/stage3_step1.csv

只解析遥测行（m=...）；session 头、>> TX、cmd ok、fault 提示行跳过。
"""
from __future__ import print_function

import argparse
import csv
import os
import re
import sys
from datetime import datetime

# 2026-08-06 11:07:48.139  m=1 hz=200 ovr=0 fault=0x00 | pitch=-4.90 ref=0.00 deg rate=0.34 | u p=-17.1 d=1.5 i=0.0 | effort=-15.6/-15.6% v=0.024/0.019
TELE_RE = re.compile(
    r"^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3})\s+"
    r"m=(\d+)\s+hz=(\d+)\s+ovr=(\d+)\s+fault=0x([0-9A-Fa-f]+)"
    r".*?\|\s*pitch=([-\d.]+)\s+ref=([-\d.]+)\s+deg\s+rate=([-\d.]+)"
    r".*?\|\s*u\s+p=([-\d.]+)\s+d=([-\d.]+)\s+i=([-\d.]+)"
    r".*?\|\s*effort=([-\d.]+)/([-\d.]+)%\s+v=([-\d.]+)/([-\d.]+)"
)

FIELDS = [
    "t_s",
    "stamp",
    "mode",
    "hz",
    "ovr",
    "fault",
    "pitch_deg",
    "ref_deg",
    "rate_rps",
    "u_p",
    "u_d",
    "u_i",
    "effort_l",
    "effort_r",
    "v_l",
    "v_r",
]


def parse_line(line):
    m = TELE_RE.match(line.rstrip("\n"))
    if not m:
        return None
    stamp = datetime.strptime(m.group(1), "%Y-%m-%d %H:%M:%S.%f")
    return {
        "stamp_dt": stamp,
        "stamp": m.group(1),
        "mode": int(m.group(2)),
        "hz": int(m.group(3)),
        "ovr": int(m.group(4)),
        "fault": int(m.group(5), 16),
        "pitch_deg": float(m.group(6)),
        "ref_deg": float(m.group(7)),
        "rate_rps": float(m.group(8)),
        "u_p": float(m.group(9)),
        "u_d": float(m.group(10)),
        "u_i": float(m.group(11)),
        "effort_l": float(m.group(12)),
        "effort_r": float(m.group(13)),
        "v_l": float(m.group(14)),
        "v_r": float(m.group(15)),
    }


def convert(in_path, out_path):
    rows = []
    with open(in_path, "r", encoding="utf-8", errors="replace") as fp:
        for line in fp:
            row = parse_line(line)
            if row:
                rows.append(row)

    if not rows:
        sys.stderr.write("no telemetry lines in %s\n" % in_path)
        return 1

    t0 = rows[0]["stamp_dt"]
    with open(out_path, "w", encoding="utf-8", newline="") as fp:
        w = csv.DictWriter(fp, fieldnames=FIELDS)
        w.writeheader()
        for row in rows:
            row["t_s"] = "%.3f" % (row["stamp_dt"] - t0).total_seconds()
            w.writerow({k: row[k] for k in FIELDS})

    sys.stderr.write("wrote %d rows → %s\n" % (len(rows), out_path))
    return 0


def main():
    ap = argparse.ArgumentParser(description="stage3 serial log → CSV")
    ap.add_argument("log", help="input .log from stage2_serial_bridge")
    ap.add_argument(
        "-o",
        "--output",
        default=None,
        help="output CSV (default: same path with .csv)",
    )
    args = ap.parse_args()

    in_path = args.log
    if not os.path.isfile(in_path):
        sys.stderr.write("not found: %s\n" % in_path)
        return 1

    out_path = args.output
    if not out_path:
        base, _ = os.path.splitext(in_path)
        out_path = base + ".csv"

    return convert(in_path, out_path)


if __name__ == "__main__":
    sys.exit(main())
