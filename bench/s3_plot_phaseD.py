#!/usr/bin/env python3
"""
S3 Phase D 出图（F1–F4）。数据源: s3_phaseD_tfirst.csv / s3_phaseD_events.csv（s3_extract_events.py 产出）。

  F1 成功率 + Wilson 95% CI   F2 t_first 分布（成功样本）
  F3 path_length 分布          F4 (t,cost) 收敛曲线（cost 为 OMPL L1 口径）

图内文字用英文（matplotlib 默认字体无中文），解读见控制台输出。
"""

import csv
import math
import sys
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# gap 档位写在图标题里；默认空 = 沿用旧标题（gap=0.23）。
GAP_TAG = ""

PLANNERS = ["mine/RRTConnect", "mine/RRTstar", "mine/BiRRTStar"]
SHORT = {"mine/RRTConnect": "RRTConnect", "mine/RRTstar": "RRTstar",
         "mine/BiRRTStar": "BiRRTStar"}
COLOR = {"mine/RRTConnect": "#4878CF", "mine/RRTstar": "#EE854A",
         "mine/BiRRTStar": "#D65F5F"}


def wilson(k, n, z=1.96):
    if n == 0:
        return 0.0, 0.0, 0.0
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z / d * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return p, max(0.0, c - h), min(1.0, c + h)


def median(v):
    s = sorted(v)
    return s[len(s) // 2] if len(s) % 2 else (s[len(s) // 2 - 1] + s[len(s) // 2]) / 2


def pct(v, q):
    s = sorted(v)
    return s[min(len(s) - 1, int(round(q * (len(s) - 1))))]


def main():
    global GAP_TAG
    prefix = "s3_phaseD"
    args = sys.argv[1:]
    for i, a in enumerate(args):
        if a == "--prefix" and i + 1 < len(args):
            prefix = args[i + 1]
        elif a == "--gap" and i + 1 < len(args):
            GAP_TAG = args[i + 1]

    rows = list(csv.DictReader(open(f"{prefix}_tfirst.csv")))
    events = list(csv.DictReader(open(f"{prefix}_events.csv")))
    ok = [r for r in rows if r["success"] == "True"]
    print(f"行 {len(rows)}（成功 {len(ok)}），事件 {len(events)}")

    # ---------- F1 成功率
    fig, ax = plt.subplots(figsize=(5.2, 4))
    print("\nF1 成功率（Wilson 95% CI）:")
    for i, p in enumerate(PLANNERS):
        n = sum(1 for r in rows if r["planner"] == p)
        k = sum(1 for r in rows if r["planner"] == p and r["success"] == "True")
        pt, lo, hi = wilson(k, n)
        print(f"  {SHORT[p]:12s} {k}/{n} = {pt:.1%}  CI [{lo:.1%}, {hi:.1%}]")
        ax.bar(i, pt * 100, color=COLOR[p], width=0.55)
        ax.errorbar(i, pt * 100, yerr=[[100 * (pt - lo)], [100 * (hi - pt)]],
                    fmt="none", ecolor="black", capsize=4)
        ax.text(i, pt * 100 + 3, f"{pt:.0%}\n({k}/{n})", ha="center", fontsize=9)
    ax.set_xticks(range(3))
    ax.set_xticklabels([SHORT[p] for p in PLANNERS])
    ax.set_ylabel("Success rate (%)"), ax.set_ylim(0, 108)
    ax.set_title(f"F1  Success rate @ gap={GAP_TAG or '0.23'}, budget=2s (Wilson 95% CI)")
    fig.tight_layout(); fig.savefig(f"{prefix}_f1_success.png", dpi=150)

    # ---------- F2 t_first 分布
    fig, ax = plt.subplots(figsize=(5.2, 4))
    data, printout = [], []
    for p in PLANNERS:
        v = [float(r["t_first"]) for r in ok
             if r["planner"] == p and r["t_first_source"] != "none"]
        data.append(v)
        printout.append(f"  {SHORT[p]:12s} n={len(v)}  中位={median(v):.4f}s  "
                        f"p95={pct(v, 0.95):.3f}s")
    ax.boxplot(data, labels=[SHORT[p] for p in PLANNERS], showfliers=False)
    for i, v in enumerate(data, 1):
        ax.scatter([i] * len(v), v, s=2, alpha=0.06, color=COLOR[PLANNERS[i - 1]])
        ax.text(i + 0.28, median(v), f"{median(v):.3f}s", va="center", fontsize=9)
    ax.set_yscale("log")
    ax.set_ylabel("t_first (s, log)")
    ax.set_title("F2  Time to first solution (successful runs)")
    fig.tight_layout(); fig.savefig(f"{prefix}_f2_tfirst.png", dpi=150)
    print("\nF2 t_first:", *printout, sep="\n")

    # ---------- F3 path_length 分布
    fig, ax = plt.subplots(figsize=(5.2, 4))
    data, printout = [], []
    for p in PLANNERS:
        v = [float(r["path_length"]) for r in ok if r["planner"] == p]
        data.append(v)
        printout.append(f"  {SHORT[p]:12s} n={len(v)}  中位={median(v):.2f}  "
                        f"IQR [{pct(v, 0.25):.2f}, {pct(v, 0.75):.2f}]")
    ax.boxplot(data, labels=[SHORT[p] for p in PLANNERS], showfliers=False)
    for i, v in enumerate(data, 1):
        ax.text(i + 0.28, median(v), f"{median(v):.1f}", va="center", fontsize=9)
    ax.set_ylabel("path_length (rad, L2)")
    n_rrts = sum(1 for r in ok if r["planner"] == "mine/RRTstar")
    ax.set_title("F3  Final path length (successful runs)"
                 + (f"\nnote: RRTstar n={n_rrts} is a survivor-biased subset"
                    if n_rrts * 2 < len(ok) // 3 else ""))
    fig.tight_layout(); fig.savefig(f"{prefix}_f3_pathlen.png", dpi=150)
    print("\nF3 path_length:", *printout, sep="\n")

    # ---------- F4 收敛曲线（cost = OMPL objective，L1 口径）
    # 每行事件 -> (t, best-so-far) 阶梯；曲线 = 各时刻已解出行的中位 best cost + IQR 带
    byrun = defaultdict(list)
    for e in events:
        byrun[(e["planner"], e["query_id"], e["rep"])].append(
            (float(e["t_rel"]), float(e["cost"])))
    fig, ax = plt.subplots(figsize=(6.2, 4.2))
    print("\nF4 收敛（中位 best cost；括号内=该时刻已解出行数）:")
    cps = [0.05, 0.1, 0.25, 0.5, 1.0, 1.5, 2.0]
    import bisect as bi
    print("（口径：固定队列=最终成功的全部 run；未解出记 inf，"
          "故中位曲线在 >50% run 解出前不存在；曲线单调不升）")
    for p in ("mine/RRTstar", "mine/BiRRTStar"):
        runs = []
        for key, evs in byrun.items():
            if key[0] != p:
                continue
            evs.sort()
            best, tlist, clist = math.inf, [], []
            for t, c in evs:
                best = min(best, c)
                tlist.append(min(t, 2.05)); clist.append(best)
            runs.append((tlist, clist))

        def vals_at(g):
            """固定队列在 t=g 的 best cost（未解出 = inf）。"""
            out = []
            for tl, cl in runs:
                i = bi.bisect_right(tl, g) - 1
                out.append(cl[i] if i >= 0 else math.inf)
            return out

        grid = [g / 200 for g in range(1, 402)]  # 0.005..2.0
        med, lo25, hi75 = [], [], []
        for g in grid:
            v = vals_at(g)
            m, q1, q3 = median(v), pct(v, 0.25), pct(v, 0.75)
            med.append(m if math.isfinite(m) else math.nan)
            lo25.append(q1 if math.isfinite(q1) else math.nan)
            hi75.append(q3 if math.isfinite(q3) else math.nan)
        ax.plot(grid, med, color=COLOR[p], label=SHORT[p])
        ax.fill_between(grid, lo25, hi75, color=COLOR[p], alpha=0.15)
        pts = []
        for c in cps:
            v = vals_at(c)
            m = median(v)
            alive = sum(1 for x in v if math.isfinite(x))
            pts.append(f"{c}:{'-' if not math.isfinite(m) else f'{m:.2f}'}({alive})")
        print(f"  {SHORT[p]:12s} " + " ".join(pts))

    # RRTConnect 参考线: 用 BiRRTStar 行的 cost_final(L1)/path_length(L2) 比值估计
    ratios = [float(r["cost_final"]) / float(r["path_length"])
              for r in ok if r["planner"] == "mine/BiRRTStar" and r["cost_final"]]
    r_est = median(ratios)
    rc_len = [float(r["path_length"]) for r in ok if r["planner"] == "mine/RRTConnect"]
    ref = r_est * median(rc_len)
    ax.axhline(ref, color=COLOR["mine/RRTConnect"], ls="--",
               label=f"RRTConnect (est. L1 = {r_est:.2f} x L2 median)")
    print(f"  RRTConnect 参考线 L1_est = {ref:.2f}（L1/L2 比 {r_est:.2f}，估计值）")

    ax.set_xlabel("t since solve start (s)")
    ax.set_ylabel("best cost so far (L1, rad)")
    ax.set_xlim(0, 2.0)
    ax.set_title("F4  Anytime convergence (cost = OMPL L1 objective)")
    ax.legend()
    fig.tight_layout(); fig.savefig(f"{prefix}_f4_convergence.png", dpi=150)

    print(f"\n图已保存: {prefix}_f1_success.png / _f2_tfirst.png / "
          f"_f3_pathlen.png / _f4_convergence.png")


if __name__ == "__main__":
    sys.exit(main())
