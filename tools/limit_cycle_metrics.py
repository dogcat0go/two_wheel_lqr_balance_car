#!/usr/bin/env python3
"""极限环口径统计（docs/deadband_limit_cycle_playbook.md §1/§2/§4）。

  python3 tools/limit_cycle_metrics.py logs/e4.csv
  python3 tools/limit_cycle_metrics.py logs/wifi.log          # 修剪后的遥测 log
  python3 tools/limit_cycle_metrics.py logs/e4.csv --all     # 不做 20s/5s 裁切

默认稳态窗：武装后丢掉前 20s、结尾 5s（与 playbook 一致）。已修剪的 log 用 --all。
"""
from __future__ import print_function

import argparse
import csv
import math
import os
import sys

_TOOLS = os.path.dirname(os.path.abspath(__file__))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
from stage3_log_to_csv import parse_line  # noqa: E402


def _f(row, key, default=None):
    v = row.get(key, "")
    if v is None or v == "":
        return default
    try:
        return float(v)
    except ValueError:
        return default


def load_rows(path):
    ext = os.path.splitext(path)[1].lower()
    if ext == ".csv":
        with open(path, "r", encoding="utf-8", errors="replace") as fp:
            return list(csv.DictReader(fp))
    rows = []
    t0 = None
    with open(path, "r", encoding="utf-8", errors="replace") as fp:
        for line in fp:
            row = parse_line(line)
            if not row:
                continue
            if t0 is None:
                t0 = row["stamp_dt"]
            row["t_s"] = "%.3f" % (row["stamp_dt"] - t0).total_seconds()
            rows.append(row)
    return rows


def median(xs):
    if not xs:
        return None
    ys = sorted(xs)
    n = len(ys)
    mid = n // 2
    return ys[mid] if n % 2 else 0.5 * (ys[mid - 1] + ys[mid])


def mean_sd(xs):
    n = len(xs)
    if n == 0:
        return 0.0, 0.0
    m = sum(xs) / n
    var = sum((x - m) ** 2 for x in xs) / n
    return m, math.sqrt(var)


def xcorr_peak(a, b, maxlag):
    """互相关峰。返回 (lag_samples, r)。正 lag = b 滞后 a。"""
    n = len(a)
    ma = sum(a) / n
    mb = sum(b) / n
    sa = math.sqrt(sum((x - ma) ** 2 for x in a))
    sb = math.sqrt(sum((x - mb) ** 2 for x in b))
    if sa <= 0.0 or sb <= 0.0:
        return 0, 0.0
    best_lag, best_r = 0, -2.0
    for lag in range(-maxlag, maxlag + 1):
        s = 0.0
        for i in range(n):
            j = i + lag
            if 0 <= j < n:
                s += (a[i] - ma) * (b[j] - mb)
        r = s / (sa * sb)
        if r > best_r:
            best_lag, best_r = lag, r
    return best_lag, best_r


def stick_runs(v, eps, dt):
    runs, cur = [], 0
    for x in v:
        if abs(x) < eps:
            cur += 1
        elif cur:
            runs.append(cur)
            cur = 0
    if cur:
        runs.append(cur)
    if not runs:
        return 0, 0.0, 0.0
    return len(runs), sum(runs) * dt / len(runs), max(runs) * dt


def breakaway(v, tau, cur, eff, eps):
    pos, neg = [], []
    for i in range(1, len(v)):
        if abs(v[i - 1]) < eps and abs(v[i]) >= eps:
            rec = (abs(tau[i]), abs(cur[i]) if cur[i] is not None else None,
                   abs(eff[i]) if eff[i] is not None else None)
            (pos if v[i] > 0 else neg).append(rec)
    return pos, neg


def fmt_break(arr):
    if not arr:
        return "n=0"
    taus = [a[0] for a in arr]
    curs = [a[1] * 1e3 for a in arr if a[1] is not None]
    effs = [a[2] for a in arr if a[2] is not None]
    parts = ["n=%d τ中位=%.1f mN·m" % (len(arr), median(taus) * 1e3)]
    if curs:
        parts.append("I中位=%.0f mA" % median(curs))
    if effs:
        parts.append("eff中位=%.1f%%" % median(effs))
    return " ".join(parts)


def col(seg, key):
    return [_f(r, key, 0.0) for r in seg]


def analyze(rows, skip_head, skip_tail, v_eps, all_rows):
    if not rows:
        sys.stderr.write("no rows\n")
        return 1

    ts = [_f(r, "t_s") for r in rows]
    if any(t is None for t in ts):
        sys.stderr.write("missing t_s\n")
        return 1
    dts = [ts[i + 1] - ts[i] for i in range(len(ts) - 1) if ts[i + 1] > ts[i]]
    dt = median(dts) if dts else 0.05

    armed = [r for r in rows if _f(r, "arm", 1) == 1]
    if not armed:
        armed = rows
    t0 = _f(armed[0], "t_s")
    tend = _f(armed[-1], "t_s")
    if all_rows:
        seg = armed
    else:
        seg = [r for r in armed
               if _f(r, "t_s") >= t0 + skip_head and _f(r, "t_s") <= tend - skip_tail]
        if len(seg) < 20:
            seg = armed

    n = len(seg)
    span = _f(seg[-1], "t_s") - _f(seg[0], "t_s")
    v_l, v_r = col(seg, "v_l"), col(seg, "v_r")
    t_l, t_r = col(seg, "tau_l"), col(seg, "tau_r")
    i_l, i_r = col(seg, "i_l"), col(seg, "i_r")
    e_l, e_r = col(seg, "effort_l"), col(seg, "effort_r")
    pit, ref = col(seg, "pitch_deg"), col(seg, "ref_deg")
    vdc = col(seg, "v_dc")

    print("文件稳态: n=%d 时长=%.1fs dt=%.3fs (%.1fHz) 武装=%.1f..%.1fs" %
          (n, span, dt, 1.0 / dt if dt else 0, t0, tend))

    print("\n== 粘着占比 |v|<%g ==" % v_eps)
    for name, v in (("v_l", v_l), ("v_r", v_r)):
        frac = 100.0 * sum(1 for x in v if abs(x) < v_eps) / n
        nrun, avg, mx = stick_runs(v, v_eps, dt)
        print("  %s: %.0f%%  段数=%d 平均=%.2fs 最长=%.2fs" %
              (name, frac, nrun, avg, mx))

    print("\n== τ→v 相位（互相关峰；正 lag = v 滞后 τ）==")
    maxlag = max(1, int(round(3.0 / dt))) if dt else 20
    for name, tau, v in (("L", t_l, v_l), ("R", t_r, v_r)):
        zc = sum(1 for a, b in zip(v, v[1:]) if a * b < 0)
        period = (2.0 * span / zc) if zc else 0.0
        lag, r = xcorr_peak(tau, v, maxlag)
        lag_s = lag * dt
        deg = (360.0 * lag_s / period) if period > 1e-6 else 0.0
        kind = "死区/粘滑型" if abs(deg) < 30.0 else (
            "滞后型" if abs(deg) > 60.0 else "中间带，看波形")
        print("  %s: lag=%+.3fs (%.0f°) r=%.2f  v周期~%.1fs  → %s" %
              (name, lag_s, deg, r, period, kind))

    print("\n== 起转 τ 中位（|v| 跨过 %.4g）==" % v_eps)
    for name, v, tau, cur, eff in (
        ("L", v_l, t_l, i_l, e_l),
        ("R", v_r, t_r, i_r, e_r),
    ):
        pos, neg = breakaway(v, tau, cur, eff, v_eps)
        print("  %s正: %s" % (name, fmt_break(pos)))
        print("  %s反: %s" % (name, fmt_break(neg)))

    err = [p - q for p, q in zip(pit, ref)]
    em, esd = mean_sd(err)
    vm, vsd = mean_sd(vdc)
    print("\n== 其它口径 ==")
    print("  pitch−ref: mean=%+.2f° sd=%.2f  max=%+.1f min=%+.1f  >+2°:%d  <-2°:%d" %
          (em, esd, max(err), min(err),
           sum(1 for e in err if e > 2), sum(1 for e in err if e < -2)))
    print("  v_dc: mean=%+.4f sd=%.3f" % (vm, vsd))
    holds = sum(1 for r in armed if _f(r, "hold", 0) == 1)
    print("  HOLD 拍数: %d / 武装 %d" % (holds, len(armed)))
    return 0


def main():
    ap = argparse.ArgumentParser(description="粘着占比 + τ→v 相位 + 起转四格")
    ap.add_argument("path", help="修剪后的 .log 或 .csv")
    ap.add_argument("--all", action="store_true", help="整段算，不裁 20s/5s")
    ap.add_argument("--skip-head", type=float, default=20.0)
    ap.add_argument("--skip-tail", type=float, default=5.0)
    ap.add_argument("--v-eps", type=float, default=0.008,
                    help="粘着/起转门槛 m/s（默认 0.008）")
    args = ap.parse_args()
    if not os.path.isfile(args.path):
        sys.stderr.write("not found: %s\n" % args.path)
        return 1
    rows = load_rows(args.path)
    if not rows:
        sys.stderr.write("no telemetry in %s\n" % args.path)
        return 1
    return analyze(rows, args.skip_head, args.skip_tail, args.v_eps, args.all)


if __name__ == "__main__":
    sys.exit(main())
