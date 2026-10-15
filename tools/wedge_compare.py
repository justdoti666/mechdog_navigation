#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""wedge_compare.py — 离线重算 vs 节点日志 的对照工具 (R-8, 2026-10-12)

为什么有它 (背景, 见 2026-10-10 复核报告 R-8):
  节点「感知:」行与 tools/wedge_recompute 的输出是**逐字段比对**的, 但旧存证里两侧都没有
  "帧身份" ⇒ 只能靠"点云数在日志里唯一"**间接**论证"算的是同一帧"。核心新增帧指纹
  (core/frame_fingerprint.h) 后, 两侧行尾都带 `| 帧fp=<8位hex>` ⇒ 可以**直接**配对。
  本脚本把这个配对过程固化下来, 并且不再像旧脚本那样"多行时只取 sorted(...)[0]"。

配对优先级:
  1) 帧指纹 (推荐): 离线输出里的 `帧fp=` → 在节点日志里找**同一个指纹**的「感知:」行,
     找到后**整行逐字符比** (含 fp 字段)。同一内容被 3000ms 节流日志/多份日志拷贝**重复**
     打出不算多命中 —— 仅当出现 **≥2 种不同行** 才判失败 (无法证明同帧唯一)。
     (2026-10-15 R-8 收尾: 旧版"命中>1 即失败"被节流重复误伤, 已改为去重判定)
  2) 回落: 点云数唯一 (legacy 旧存证, 没有 fp 字段) —— 会明确标注这是**间接**论证。

★ 对齐前提 (2026-10-15 实测): 离线与节点"同帧同参数"才可比。回放灌帧**必须发布**
  /camera/depth/camera_info —— 否则节点用 FOV 反推兜底内参 (572.7/320.0), 与工装的
  570.3422047415297129/319.5 几何不同 ⇒ 平面拟合**假分歧** (同帧整行必然不同, 非工具缺陷)。

退出码: 0=全部一致; 1=有不一致/找不到/命中不唯一; 2=用法错。

用法:
  python3 wedge_compare.py --dir <证据目录>                  # 自动配对目录内 final_*.txt
  python3 wedge_compare.py --dir <目录> --pair tag:log[:点云数]   # 显式指定 (可重复)
  python3 wedge_compare.py --dir <目录> --md5 md5s_compare.txt    # 顺带校验 md5 清单
"""
import argparse
import hashlib
import os
import re
import sys

PERC_PREFIX = "感知:"
RE_FP = re.compile(r"帧fp=([0-9a-f]{8}|-{8})")
RE_POINTS = re.compile(r"点云=(\d+)")


def norm(s: str) -> str:
    return s.strip().replace("\r", "")


def strip_log_prefix(line: str) -> str:
    """从 `[INFO] [ts] [safety_node]: 感知: ...` 里取出 `感知: ...` 部分。"""
    if "]: " in line:
        return norm(line.split("]: ", 1)[1])
    return norm(line)


def read_perception_lines(path: str):
    """返回该日志里的「感知:」行 (排除计时段)。"""
    out = []
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            for ln in f:
                if PERC_PREFIX not in ln or "[计时段]" in ln:
                    continue
                seg = strip_log_prefix(ln)
                if seg.startswith(PERC_PREFIX):
                    out.append(seg)
    except OSError:
        pass
    return out


def fp_of(line: str):
    m = RE_FP.search(line)
    return m.group(1) if m else None


def points_of(line: str):
    m = RE_POINTS.search(line)
    return m.group(1) if m else None


def match_offline(off_line: str, logs):
    """在 logs = [(name, [perception lines])] 里给离线行找节点行。返回 (mode, hits)。"""
    fp = fp_of(off_line)
    if fp:
        hits = [(n, l) for n, ls in logs for l in ls if fp_of(l) == fp]
        return "fp", hits
    pts = points_of(off_line)
    if pts:
        hits = [(n, l) for n, ls in logs for l in ls if points_of(l) == pts]
        return "points", hits
    return "none", []


def verify_md5(manifest_path: str, base_dir: str):
    """按 md5 清单逐条校验 (语义同 md5sum -c)。返回 (ok, bad, missing)。"""
    ok = bad = missing = 0
    try:
        with open(manifest_path, encoding="utf-8", errors="replace") as f:
            for ln in f:
                ln = ln.strip()
                if not ln or ln.startswith("#"):
                    continue
                parts = ln.split(None, 1)
                if len(parts) != 2:
                    continue
                want, name = parts[0], parts[1].lstrip("*")
                p = os.path.join(base_dir, name)
                if not os.path.isfile(p):
                    missing += 1
                    continue
                h = hashlib.md5()
                with open(p, "rb") as g:
                    for chunk in iter(lambda: g.read(1 << 20), b""):
                        h.update(chunk)
                if h.hexdigest() == want:
                    ok += 1
                else:
                    bad += 1
                    print(f"  [md5 BAD] {name}: 清单 {want} != 实算 {h.hexdigest()}")
    except OSError as e:
        print(f"  [md5] 无法读取清单 {manifest_path}: {e}")
        return 0, 1, 0
    return ok, bad, missing


def main() -> int:
    ap = argparse.ArgumentParser(add_help=True)
    ap.add_argument("--dir", required=True, help="证据目录 (内含 final_*.txt 与节点日志)")
    ap.add_argument("--pair", action="append", default=[],
                    help="显式配对 tag:logfile[:点云数] (可重复; 省略则自动找 final_*.txt)")
    ap.add_argument("--md5", default=None, help="md5 清单文件名 (相对 --dir)")
    ap.add_argument("--offline-glob", default="final_", help="自动模式的离线输出前缀")
    args = ap.parse_args()

    ev = args.dir
    if not os.path.isdir(ev):
        print(f"用法错: 目录不存在 {ev}")
        return 2

    logs = []
    for name in sorted(os.listdir(ev)):
        if name.endswith(".log"):
            ls = read_perception_lines(os.path.join(ev, name))
            if ls:
                logs.append((name, ls))
    if not logs:
        print("用法错: 目录里没有含「感知:」行的 *.log")
        return 2
    print(f"节点日志: " + ", ".join(f"{n}({len(ls)} 行)" for n, ls in logs))

    if args.pair:
        cases = []
        for spec in args.pair:
            bits = spec.split(":")
            tag, logf = bits[0], bits[1]
            filt = bits[2] if len(bits) > 2 else None
            cases.append((tag, logf, filt))
    else:
        cases = []
        for name in sorted(os.listdir(ev)):
            if name.startswith(args.offline_glob) and name.endswith(".txt"):
                tag = name[len(args.offline_glob):-len(".txt")]
                cases.append((tag, None, None))
    if not cases:
        print("用法错: 没找到任何离线输出 (final_*.txt) 或 --pair")
        return 2

    bad = 0
    for tag, logf, filt in cases:
        off_path = os.path.join(ev, f"{args.offline_glob}{tag}.txt")
        # final_<tag>.txt 与 off_<tag>.txt 两种历史命名都接受
        if not os.path.isfile(off_path):
            alt = [f for f in sorted(os.listdir(ev))
                   if f.endswith(f"_{tag}.txt") or f == f"{tag}.txt"]
            if not alt:
                print(f"[{tag}] MISSING 离线输出")
                bad += 1
                continue
            off_path = os.path.join(ev, alt[0])
        with open(off_path, encoding="utf-8", errors="replace") as f:
            off_line = ""
            for ln in f:
                if PERC_PREFIX in ln:
                    off_line = norm(ln)
                    break
        if not off_line:
            print(f"[{tag}] MISSING 离线输出里没有「感知:」行")
            bad += 1
            continue

        pool = logs
        if logf:
            pool = [(n, ls) for n, ls in logs if n == logf]
            if not pool:
                ls = read_perception_lines(os.path.join(ev, logf))
                pool = [(logf, ls)]
        if filt:
            pool = [(n, [l for l in ls if points_of(l) == filt]) for n, ls in pool]

        mode, hits = match_offline(off_line, pool)
        off_fp = fp_of(off_line)
        if not hits:
            print(f"[{tag}] MISMATCH 找不到对应的节点行 "
                  f"(离线 fp={off_fp or 'n/a'}, 点云={points_of(off_line)})")
            bad += 1
            continue
        # 2026-10-15 (R-8 收尾): 按**去重后的不同行**判定 —— 同内容重复(节流/多拷贝)不算多命中
        uniq = []
        for n, l in hits:
            if all(l != u[1] for u in uniq):
                uniq.append((n, l))
        if len(uniq) > 1:
            print(f"[{tag}] MISMATCH 命中 {len(uniq)} 种**不同行** (共 {len(hits)} 次, 含节流重复) "
                  f"⇒ 无法证明同帧:")
            for n, l in uniq[:4]:
                print(f"    [{n}] {l}")
            bad += 1
            continue

        log_name, node_line = uniq[0]
        repeat_n = len(hits)
        if node_line == off_line:
            how = "按帧指纹" if mode == "fp" else "按点云数(legacy, 间接论证)"
            extra = f"; 重复 {repeat_n} 次(节流)" if repeat_n > 1 else ""
            print(f"[{tag}] OK 逐字符一致 ({how}; 日志={log_name}; 帧fp={off_fp or 'n/a'}{extra})")
        else:
            print(f"[{tag}] MISMATCH ({'帧指纹相同但整行不同' if mode == 'fp' else '点数相同但整行不同'})")
            print(f"    node: {node_line}")
            print(f"    off : {off_line}")
            bad += 1

    if args.md5:
        ok, mbad, missing = verify_md5(os.path.join(ev, args.md5), ev)
        print(f"[md5] 通过 {ok} / 不符 {mbad} / 缺件 {missing}  ({args.md5})")
        if mbad or missing:
            bad += 1

    print(f"=== {len(cases)} 个工况, {bad} 个不一致 ===")
    return 0 if bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
