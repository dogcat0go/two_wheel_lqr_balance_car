#!/usr/bin/env python3
"""遥测日志格式筛查（第一步）：按行分类 → 检查格式一致性 → 确认 key=value 可解析。

用法:
  python3 tools/tele_log_inspect.py path/to.log
  platformio device monitor ... | python3 tools/tele_log_inspect.py -
  # 只看某行号区间（含两端，1-based）:
  python3 tools/tele_log_inspect.py logs/x.log --from 10 --to 50

识别规则（启发式，不绑定某一 stage 的固定字段）:
  - 含 `m=` 且含 `|`，并至少解析出 3 个 key=value → 视为遥测候选行
  - 其余行记为 noise（cmd ok / microros / 截断等）
"""
from __future__ import print_function

import argparse
import collections
import re
import sys

# 可选主机时间戳前缀（stage2_serial_bridge 落盘）
STAMP_RE = re.compile(r"^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3})\s+")
# key=value；value 吃到空白或 '|' 为止（'|' 前后常有空格）
KV_RE = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)=([^\s|]+)")
# 裸单位词（如 deg），不当作 KV，但记入结构指纹
BARE_RE = re.compile(r"(?<![A-Za-z0-9_=.-])([A-Za-z_%]+)(?![A-Za-z0-9_=])")

TELE_MARKERS = ("m=", "|")


def strip_stamp(line):
    m = STAMP_RE.match(line)
    if not m:
        return None, line
    return m.group(1), line[m.end() :]


def classify_line(line):
    """返回 ('tele'|'noise', stamp, body)。"""
    raw = line.rstrip("\n\r")
    if not raw.strip():
        return "noise", None, raw
    stamp, body = strip_stamp(raw)
    body = body.strip()
    if not all(m in body for m in TELE_MARKERS):
        return "noise", stamp, body
    kvs = KV_RE.findall(body)
    if len(kvs) < 3:
        return "noise", stamp, body
    return "tele", stamp, body


def value_shape(val):
    """推断 value 形态，用于确认能否被分析。"""
    if re.fullmatch(r"0[xX][0-9A-Fa-f]+", val):
        return "hex"
    if re.fullmatch(r"-?\d+", val):
        return "int"
    if re.fullmatch(r"-?\d+\.\d+", val):
        return "float"
    # 带单位：0.175A / 70.0% / 12deg
    m = re.fullmatch(r"(-?\d+(?:\.\d+)?)([A-Za-z%]+)", val)
    if m:
        num, unit = m.group(1), m.group(2)
        kind = "float" if "." in num else "int"
        return "%s+unit(%s)" % (kind, unit)
    # 对：70.0/70.0% 或 0.887/0.878 或 ticks
    m = re.fullmatch(
        r"(-?\d+(?:\.\d+)?)/(-?\d+(?:\.\d+)?)([A-Za-z%]*)", val
    )
    if m:
        a, b, unit = m.group(1), m.group(2), m.group(3)
        kind = "float" if ("." in a or "." in b) else "int"
        if unit:
            return "%s_pair+unit(%s)" % (kind, unit)
        return "%s_pair" % kind
    return "unknown(%s)" % val[:32]


def schema_of(body):
    """结构指纹：有序 key 列表 + 中间裸词（如 deg）。"""
    keys = [k for k, _ in KV_RE.findall(body)]
    # 去掉已被 KV 覆盖的位置上的词，只保留像 'deg' 这种独立 token
    stripped = KV_RE.sub(" ", body)
    stripped = stripped.replace("|", " ")
    bares = [t for t in BARE_RE.findall(stripped) if t.lower() not in ("fall",)]
    # FALL / IMU_LOST / CMD_TIMEOUT 是故障标志，单独记
    flags = []
    for flag in ("FALL", "IMU_LOST", "CMD_TIMEOUT"):
        if re.search(r"\b%s\b" % flag, body):
            flags.append(flag)
    return tuple(keys), tuple(bares), tuple(flags)


def parse_kvs(body):
    return [(k, v, value_shape(v)) for k, v in KV_RE.findall(body)]


def read_lines(path, from_line, to_line):
    if path == "-":
        fp = sys.stdin
    else:
        fp = open(path, "r", encoding="utf-8", errors="replace")
    try:
        for i, line in enumerate(fp, 1):
            if from_line and i < from_line:
                continue
            if to_line and i > to_line:
                break
            yield i, line
    finally:
        if path != "-":
            fp.close()


def inspect(path, from_line=None, to_line=None, show_samples=3):
    total = 0
    tele = []
    noise = []
    schema_counter = collections.Counter()
    shape_by_key = collections.defaultdict(collections.Counter)
    incomplete = []  # 疑似遥测但被截断

    for lineno, line in read_lines(path, from_line, to_line):
        total += 1
        kind, stamp, body = classify_line(line)
        if kind == "noise":
            # 截断遥测：以 m= 开头但缺 | 或 KV 太少
            s = body.strip()
            if s.startswith("m=") and ("|" not in s or len(KV_RE.findall(s)) < 3):
                incomplete.append((lineno, body[:120]))
            noise.append((lineno, body[:100]))
            continue

        keys, bares, flags = schema_of(body)
        schema_counter[(keys, bares)] += 1
        parsed = parse_kvs(body)
        for k, v, shape in parsed:
            shape_by_key[k][shape] += 1
        tele.append(
            {
                "lineno": lineno,
                "stamp": stamp,
                "keys": keys,
                "bares": bares,
                "flags": flags,
                "parsed": parsed,
                "body": body,
            }
        )

    print("=== 行分类 ===")
    print("总行数: %d" % total)
    print("遥测行: %d" % len(tele))
    print("非遥测: %d" % len(noise))
    if incomplete:
        print("疑似截断/残缺遥测: %d" % len(incomplete))
        for ln, preview in incomplete[:5]:
            print("  L%-5d %s" % (ln, preview))

    if noise and len(noise) <= 20:
        print("\n--- 非遥测样例 ---")
        for ln, preview in noise[:10]:
            print("  L%-5d %s" % (ln, preview if preview else "(空行)"))
    elif noise:
        # 汇总噪声前缀
        prefixes = collections.Counter()
        for _, b in noise:
            prefixes[b.split(":", 1)[0][:40] if b else "(空行)"] += 1
        print("\n--- 非遥测前缀 Top ---")
        for p, c in prefixes.most_common(8):
            print("  %4d  %s" % (c, p))

    if not tele:
        print("\n未找到可解析遥测行。")
        return 1

    print("\n=== 格式一致性（key 顺序指纹）===")
    n_schema = len(schema_counter)
    dominant = schema_counter.most_common(1)[0]
    (dom_keys, dom_bares), dom_n = dominant
    print("不同结构数: %d" % n_schema)
    print(
        "主结构占比: %d/%d (%.1f%%)"
        % (dom_n, len(tele), 100.0 * dom_n / len(tele))
    )
    print("主结构 keys: %s" % " ".join(dom_keys))
    if dom_bares:
        print("主结构裸词: %s" % " ".join(dom_bares))

    if n_schema > 1:
        print("\n--- 各结构 ---")
        for (keys, bares), c in schema_counter.most_common():
            mark = " *" if (keys, bares) == (dom_keys, dom_bares) else ""
            extra = (" bare=[%s]" % " ".join(bares)) if bares else ""
            print("  %4d%s  %s%s" % (c, mark, " ".join(keys), extra))
        # 列出偏离主结构的行号
        print("\n--- 偏离主结构的行 ---")
        shown = 0
        for row in tele:
            if (row["keys"], row["bares"]) != (dom_keys, dom_bares):
                print(
                    "  L%-5d keys=%s"
                    % (row["lineno"], " ".join(row["keys"]))
                )
                shown += 1
                if shown >= 15:
                    print("  ...")
                    break
    else:
        print("结论: 全部遥测行结构一致。")

    print("\n=== key=value 可解析性 ===")
    all_ok = True
    # 按主结构 key 顺序输出
    key_order = list(dom_keys)
    for k in shape_by_key:
        if k not in key_order:
            key_order.append(k)
    print("%-10s %-22s %s" % ("key", "shape", "count / 样例"))
    print("-" * 60)
    for k in key_order:
        shapes = shape_by_key[k]
        shape, cnt = shapes.most_common(1)[0]
        sample = None
        for row in tele:
            for kk, vv, ss in row["parsed"]:
                if kk == k:
                    sample = vv
                    break
            if sample is not None:
                break
        multi = ""
        if len(shapes) > 1:
            all_ok = False
            multi = "  !!多形态: " + ", ".join(
                "%s=%d" % (s, c) for s, c in shapes.most_common()
            )
        if shape.startswith("unknown"):
            all_ok = False
        print(
            "%-10s %-22s %d  e.g. %s%s"
            % (k, shape, cnt, sample, multi)
        )

    print("\n=== 样例解析（前 %d 条遥测）===" % show_samples)
    for row in tele[:show_samples]:
        print("L%d:" % row["lineno"])
        parts = ["%s=%s" % (k, v) for k, v, _ in row["parsed"]]
        print("  " + " | ".join(parts))

    print("\n=== 结论 ===")
    if n_schema == 1 and all_ok:
        print(
            "格式一致，全部 key=value 可解析。后续可据此做统计/CSV。"
        )
        return 0
    if n_schema == 1 and not all_ok:
        print("结构一致，但部分 value 形态混杂或未知，需先约定解析规则。")
        return 2
    print(
        "存在多种遥测结构（常见于 mode 切换或固件改版）。"
        "建议按主结构筛选，或按 mode 分段分析。"
    )
    return 2


def main():
    ap = argparse.ArgumentParser(
        description="遥测日志格式筛查：行分类 / 结构一致性 / key=value 确认"
    )
    ap.add_argument("log", help="日志文件，或 - 表示 stdin")
    ap.add_argument("--from", dest="from_line", type=int, default=None)
    ap.add_argument("--to", dest="to_line", type=int, default=None)
    ap.add_argument(
        "-n",
        "--samples",
        type=int,
        default=3,
        help="打印解析样例条数（默认 3）",
    )
    args = ap.parse_args()
    if args.log != "-":
        try:
            open(args.log, "r").close()
        except OSError as e:
            sys.stderr.write("无法打开: %s (%s)\n" % (args.log, e))
            return 1
    return inspect(args.log, args.from_line, args.to_line, args.samples)


if __name__ == "__main__":
    sys.exit(main())
