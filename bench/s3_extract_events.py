#!/usr/bin/env python3
"""
S3 日志事件流提取 —— t_first 与 (t, cost) 收敛事件（stage_1_ompl_3.md §4.1 决定）。

输入:
    move_group 日志（tee 落盘的，含 [improved_solution] / [t_first_solution] 行）
    runner 的 CSV（含 t_request 墙钟列）

对齐方式:
    CSV 行按 t_request 排序后，日志行的时间戳落在哪个请求的时间窗内，
    就归属于哪一行。窗口上界取下一个请求的 t_request 与 t_request+max_window 的较小者
    （防止实验结束后手工发起的 Plan 被误记到最后一行头上）。
    日志可能混有多轮实验（如地基检查）——窗口之外的行计为 unassigned，只报告不使用。

输出（--out-prefix 控制）:
    <prefix>_events.csv  每个 improved_solution 事件一行（收敛曲线 F4 数据源）
    <prefix>_tfirst.csv  CSV 原列 + t_first/t_first_source/n_events/cost_first/cost_final/endline_status
                         （t_first 分布 F2 数据源）

t_first 口径（文档 §4.1）:
    RRTConnect          = planning_time（首解即返回；endline 恒 tfirst=-1）
    RRTstar/BiRRTStar   = 第一个 improved_solution 事件的 t（成功时必有事件）
    失败行              = -1, source=none
"""

import argparse
import bisect
import csv
import re
import statistics as st
from collections import defaultdict

RE_IMPROVED = re.compile(
    r"\[(\d+\.\d+)\]\s*\[ar4_my_planners\]: "
    r"\[improved_solution\] planner=(\S+) t=([\d.eE+-]+) cost=([\d.eE+-]+)")
RE_ENDLINE = re.compile(
    r"\[(\d+\.\d+)\]\s*\[ar4_my_planners\]: "
    r"\[t_first_solution\] planner=(\S+) tfirst=(-?[\d.eE+-]+) status=(.*)$")


def parse_log(path):
    """返回 (improved, endline) 两个列表，均按时间戳升序（日志文件本身有序）。"""
    improved, endline = [], []
    with open(path, errors="replace") as f:
        for line in f:
            if "improved_solution" in line:
                m = RE_IMPROVED.search(line)
                if m:
                    improved.append((float(m.group(1)), m.group(2),
                                     float(m.group(3)), float(m.group(4))))
            elif "t_first_solution" in line:
                m = RE_ENDLINE.search(line)
                if m:
                    endline.append((float(m.group(1)), m.group(2),
                                    float(m.group(3)), m.group(4).strip()))
    return improved, endline


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", default="/tmp/move_group_036.log")
    ap.add_argument("--csv", default="s3_phaseD_036.csv")
    ap.add_argument("--out-prefix", default="s3_phaseD_036")
    ap.add_argument("--max-window", type=float, default=35.0,
                    help="单个请求时间窗上限秒（runner 等待上限 timeout+30 再放宽）")
    args = ap.parse_args()

    rows = list(csv.DictReader(open(args.csv)))
    for r in rows:
        r["t_request_f"] = float(r["t_request"])
    rows.sort(key=lambda r: r["t_request_f"])
    times = [r["t_request_f"] for r in rows]

    # 每行累积: events=[(t,cost)...], endline=(tfirst,status) or None
    per_row_events = [[] for _ in rows]
    per_row_end = [None] * rows.__len__()
    un_before = un_after = un_improved = 0

    def window_owner(ts):
        i = bisect.bisect_right(times, ts) - 1
        if i < 0:
            return None
        cap = times[i] + args.max_window
        if i + 1 < len(times):
            cap = min(cap, times[i + 1])
        return i if ts <= cap else -1  # -1 = 窗口外（之后）

    improved, endline = parse_log(args.log)
    for ts, planner, t, cost in improved:
        i = window_owner(ts)
        if i is None:
            un_before += 1
        elif i == -1:
            un_improved += 1
        else:
            per_row_events[i].append((t, cost))
    for ts, planner, tfirst, status in endline:
        i = window_owner(ts)
        if i is None:
            un_before += 1
        elif i == -1:
            un_after += 1
        else:
            if per_row_end[i] is not None:
                print(f"[警告] 行 {i} 出现第二条 endline（时间窗过宽?）")
            per_row_end[i] = (planner, tfirst, status)

    # ---- 输出 1: 事件表
    with open(f"{args.out_prefix}_events.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["gap", "query_id", "planner", "rep", "t_rel", "cost", "t_wall"])
        n_ev = 0
        for r, evs in zip(rows, per_row_events):
            for t, cost in evs:
                w.writerow([r["gap"], r["query_id"], r["planner"], r["rep"],
                            f"{t:.6f}", f"{cost:.6f}", ""])
                n_ev += 1
    # t_wall 事件行暂时留空（对齐诊断用不上时省一次回填；需要时在此补 ts）

    # ---- 输出 2: 逐行 t_first 表
    fields = list(rows[0].keys())
    fields.remove("t_request_f")
    out_fields = fields + ["t_first", "t_first_source", "n_events",
                           "cost_first", "cost_final", "endline_status"]
    n_mismatch = 0
    with open(f"{args.out_prefix}_tfirst.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=out_fields)
        w.writeheader()
        for r, evs, end in zip(rows, per_row_events, per_row_end):
            ok = r["success"] == "True"
            evs.sort()
            if evs:
                t_first, src = f"{evs[0][0]:.6f}", "event"
                c_first, c_final = f"{evs[0][1]:.6f}", f"{evs[-1][1]:.6f}"
            elif ok and r["planner"].endswith("RRTConnect"):
                t_first, src = r["planning_time"], "planning_time"
                c_first = c_final = ""
            else:
                t_first, src = "-1", "none"
                c_first = c_final = ""
            end_status = end[2] if end else ""
            if (not evs) and ok and not r["planner"].endswith("RRTConnect"):
                n_mismatch += 1  # 成功的 anytime 算法应有事件
            out = {k: r[k] for k in fields}
            out.update(t_first=t_first, t_first_source=src, n_events=len(evs),
                       cost_first=c_first, cost_final=c_final,
                       endline_status=end_status)
            w.writerow(out)

    # ---- 汇总
    print(f"CSV 行 {len(rows)} | 事件 {n_ev} | endline {len(endline)}")
    print(f"窗口外: 请求前 {un_before}, 请求后/超窗 {un_after + un_improved}"
          f"（多为日志里混入的其他轮次，如地基检查——预期行为）")
    if n_mismatch:
        print(f"[警告] {n_mismatch} 行成功但无事件（anytime 算法不应如此，抽查！）")
    by = defaultdict(list)
    for r, evs in zip(rows, per_row_events):
        if r["success"] == "True" and evs:
            by[r["planner"]].append((evs[0][0], evs[-1][1], len(evs), evs[0][1]))
    for p, v in sorted(by.items()):
        tf = sorted(x[0] for x in v)
        imp = sorted(x[3] / x[1] if x[1] > 0 else 0 for x in v)
        print(f"{p:20s} n={len(v)}  t_first 中位={st.median(tf):.4f}s  "
              f"事件数中位={st.median([x[2] for x in v]):.0f}  "
              f"首解cost/最终cost 中位={st.median(imp):.2f}")


if __name__ == "__main__":
    main()
