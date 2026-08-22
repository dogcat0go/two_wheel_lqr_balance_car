#!/usr/bin/env python3
"""遥测 log → CSV（PlotJuggler）。

  python3 tools/stage3_log_to_csv.py logs/wifi_20260820_162323.log
  python3 tools/stage3_log_to_csv.py logs/x.log -o logs/hold.csv

按 key=value 抽字段，'|' 只当分段、不当分隔符。兼容：
  - 现 WiFi/stage5：arm= hold= hN=；u pit=/rate=/pos=/vel=；tau=；eff=
  - mode=0 紧凑：iL/iR、craw=、iref=、eff=、v=、ticks=（L/R 用 / 分隔）
  - 旧 stage3：m= 后紧跟 hz=；u p=/d=/i=；effort=
重复 key（如两个 rate=）按出现顺序：先俯仰角速度，后控制项 u_rate。
"""
from __future__ import print_function

import argparse
import csv
import os
import re
import sys
from datetime import datetime

STAMP_RE = re.compile(r"^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3})\s+")
KV_RE = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)=([^\s|]+)")

FIELDS = [
    "t_s",
    "stamp",
    "mode",
    "arm",
    "hold",
    "hold_n",
    "hz",
    "ovr",
    "fault",
    "pitch_deg",
    "acc_deg",
    "ref_deg",
    "rate_rps",
    "ax",
    "ay",
    "az",
    "yaw_deg",
    "yaw_ref_deg",
    "wz",
    "u_yaw",
    "u_yaw_i",
    "u_pit",
    "u_rate",
    "u_pos",
    "u_vel",
    "u_int",
    "tau_l",
    "tau_r",
    "vref",
    "aref",
    "v_l",
    "v_r",
    "v_dc",
    "x_l",
    "x_r",
    "ticks_l",
    "ticks_r",
    "effort_l",
    "effort_r",
    "iref_l",
    "iref_r",
    "craw_l",
    "craw_r",
    "i_l",
    "i_r",
    "ainj_deg",
    "sin_eff",
    "teq_deg",
    "tff",
]


def _num(val):
    if val is None:
        return ""
    m = re.match(r"(-?\d+(?:\.\d+)?)", val)
    return m.group(1) if m else ""


def _pair(val):
    if val is None:
        return "", ""
    m = re.match(
        r"(-?\d+(?:\.\d+)?)/(-?\d+(?:\.\d+)?)",
        val,
    )
    if not m:
        return "", ""
    return m.group(1), m.group(2)


def parse_line(line):
    raw = line.rstrip("\n\r")
    if not raw.strip():
        return None
    sm = STAMP_RE.match(raw)
    if not sm:
        return None
    stamp = sm.group(1)
    body = raw[sm.end() :]
    if "m=" not in body:
        return None
    kvs = KV_RE.findall(body)
    if len(kvs) < 3:
        return None

    first = {}
    extra = {}
    for k, v in kvs:
        if k not in first:
            first[k] = v
        else:
            extra.setdefault(k, []).append(v)

    if "pitch" not in first and "hz" not in first:
        return None

    u_rate = extra["rate"][0] if extra.get("rate") else first.get("rate")
    # 旧 stage3：u p= d= i=；新：u pit= 且第二个 rate= 是 u_rate
    u_pit = first.get("pit", first.get("p"))
    if "pit" in first and extra.get("rate"):
        u_rate = extra["rate"][0]
    u_d = first.get("d", u_rate)
    u_int = first.get("int", first.get("i"))
    if "iL" in first or "iref" in first:
        u_int = first.get("int", "")

    eff = first.get("eff", first.get("effort"))
    el, er = _pair(eff)
    vl, vr = _pair(first.get("v"))
    xl, xr = _pair(first.get("x"))
    tl, tr = _pair(first.get("tau"))
    iref_l, iref_r = _pair(first.get("iref"))
    craw_l, craw_r = _pair(first.get("craw"))
    yaw, yaw_ref = _pair(first.get("yaw"))
    ticks_l, ticks_r = _pair(first.get("ticks"))

    stamp_dt = datetime.strptime(stamp, "%Y-%m-%d %H:%M:%S.%f")
    fault = first.get("fault", "")
    if fault.startswith("0x") or fault.startswith("0X"):
        fault = str(int(fault, 16))

    return {
        "stamp_dt": stamp_dt,
        "stamp": stamp,
        "mode": first.get("m", ""),
        "arm": first.get("arm", ""),
        "hold": first.get("hold", ""),
        "hold_n": first.get("hN", ""),
        "hz": first.get("hz", ""),
        "ovr": first.get("ovr", ""),
        "fault": fault,
        "pitch_deg": _num(first.get("pitch")),
        "acc_deg": _num(first.get("acc")),
        "ref_deg": _num(first.get("ref")),
        "rate_rps": _num(first.get("rate")),
        "ax": _num(first.get("ax")),
        "ay": _num(first.get("ay")),
        "az": _num(first.get("az")),
        "yaw_deg": yaw,
        "yaw_ref_deg": yaw_ref,
        "wz": _num(first.get("wz")),
        "u_yaw": _num(first.get("uy")),
        "u_yaw_i": _num(first.get("ui")),
        "u_pit": _num(u_pit),
        "u_rate": _num(u_d),
        "u_pos": _num(first.get("pos")),
        "u_vel": _num(first.get("vel")),
        "u_int": _num(u_int),
        "tau_l": tl,
        "tau_r": tr,
        "vref": _num(first.get("vref")),
        "aref": _num(first.get("aref")),
        "v_l": vl,
        "v_r": vr,
        "v_dc": _num(first.get("vdc")),
        "x_l": xl,
        "x_r": xr,
        "ticks_l": ticks_l,
        "ticks_r": ticks_r,
        "effort_l": el,
        "effort_r": er,
        "iref_l": iref_l,
        "iref_r": iref_r,
        "craw_l": craw_l,
        "craw_r": craw_r,
        "i_l": _num(first.get("iL")),
        "i_r": _num(first.get("iR")),
        "ainj_deg": _num(first.get("ainj")),
        "sin_eff": _num(first.get("seff")),
        "teq_deg": _num(first.get("teq")),
        "tff": _num(first.get("tff")),
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
    ap = argparse.ArgumentParser(description="telemetry log → CSV")
    ap.add_argument("log", help="input .log")
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
