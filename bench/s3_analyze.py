#!/usr/bin/env python3
"""
S3 结果分析 —— 出表 + 出图。

成功率一律带 Wilson 95% 置信区间：50 次里成功 44 次和 500 次里成功 440 次
是完全不同强度的证据，光报一个百分比会掩盖这个差别。

耗时报中位数和 p95，不报均值 —— 采样类算法的耗时分布是长尾的，
均值会被少数极慢的样本拖走，失去代表性。

用法:
    python3 s3_analyze.py                  # 出表
    python3 s3_analyze.py --plot           # 出表 + 存图 s3_curve.png
"""

import argparse
import csv
import math
from collections import defaultdict


def wilson(k, n, z=1.96):
    """成功率的 Wilson 95% 置信区间。n=0 时返回 (0,0,0)。"""
    if n == 0:
        return 0.0, 0.0, 0.0
    p = k / n
    d = 1 + z * z / n
    center = (p + z * z / (2 * n)) / d
    half = z / d * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return p, max(0.0, center - half), min(1.0, center + half)


def pct(sorted_vals, q):
    if not sorted_vals:
        return float("nan")
    i = min(len(sorted_vals) - 1, int(round(q * (len(sorted_vals) - 1))))
    return sorted_vals[i]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="s3_results.csv")
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()

    rows = defaultdict(list)
    codes = defaultdict(lambda: defaultdict(int))
    per_query = defaultdict(lambda: [0, 0])  # (gap, planner, qid) -> [成功, 总数]
    per_query_len = defaultdict(list)        # (gap, planner, qid) -> [成功样本的路径长]
    with open(args.csv) as f:
        for r in csv.DictReader(f):
            key = (float(r["gap"]), r["planner"])
            ok = r["success"] == "True"
            rows[key].append(
                (ok, float(r["planning_time"]), float(r["path_length"]))
            )
            qk = (float(r["gap"]), r["planner"], r["query_id"])
            pq = per_query[qk]
            pq[1] += 1
            if ok:
                pq[0] += 1
                per_query_len[qk].append(float(r["path_length"]))
            else:
                codes[key][r["error_code"]] += 1

    gaps = sorted({g for g, _ in rows}, reverse=True)
    planners = sorted({p for _, p in rows})

    print(f"{'gap':>6} {'planner':<26} {'n':>5} {'成功率':>8} {'95%CI':>16} "
          f"{'t_中位':>8} {'t_p95':>8} {'路径长':>8} {'失败t中位':>10}")
    print("-" * 108)

    stats = {}
    for g in gaps:
        for p in planners:
            data = rows.get((g, p), [])
            if not data:
                continue
            n = len(data)
            k = sum(1 for ok, _, _ in data if ok)
            rate, lo, hi = wilson(k, n)

            ts = sorted(t for ok, t, _ in data if ok)
            ls = sorted(L for ok, _, L in data if ok)
            # 失败样本的耗时：接近超时 = 真的搜不到；极短 = 请求被拒绝
            fs = sorted(t for ok, t, _ in data if not ok)
            stats[(g, p)] = (rate, lo, hi)

            print(f"{g:>6.3f} {p:<26} {n:>5} {rate:>7.1%} "
                  f"  [{lo:>5.1%},{hi:>6.1%}] "
                  f"{pct(ts, 0.5):>8.3f} {pct(ts, 0.95):>8.3f} {pct(ls, 0.5):>8.2f} "
                  f"{pct(fs, 0.5):>10.3f}")
        print()

    # 区分两种失败机制。判据来自 MoveIt 的一个实现细节:
    #   ModelBasedPlanningContext::solve() 只在【成功分支】给 res.planning_time
    #   赋值，搜索失败时该字段保持默认 0。
    # 于是:
    #   planning_time == 0 精确为零  -> OMPL 真的没搜到（算法难度，结论成立）
    #   planning_time  > 0 却报错    -> 路径找到了、时间也记了，被 response
    #                                   adapter (ValidateSolution) 否掉
    #
    # 注意别用"和超时值比大小"来判断 —— 那个判据对第一种机制完全失效，
    # 因为它的 planning_time 压根没被填。
    print("失败机制拆分:")
    print(f"{'gap':>6} {'planner':<26} {'搜索失败':>8} {'后处理拒绝':>10} "
          f"{'拒绝耗时中位':>12}")
    print("-" * 68)
    tot_search = tot_reject = 0
    for g in gaps:
        for p in planners:
            fs = [t for ok, t, _ in rows.get((g, p), []) if not ok]
            if not fs:
                continue
            search = sum(1 for t in fs if t == 0.0)
            rej = sorted(t for t in fs if t > 0.0)
            tot_search += search
            tot_reject += len(rej)
            rej_med = f"{pct(rej, 0.5):.3f}" if rej else "-"
            print(f"{g:>6.3f} {p:<26} {search:>8} {len(rej):>10} {rej_med:>12}")
        print()
    n_all = sum(len(v) for v in rows.values())
    print(f"  合计: 搜索失败 {tot_search}，后处理拒绝 {tot_reject} "
          f"({tot_reject / n_all:.2%} of {n_all})")
    print("  后处理拒绝是 longest_valid_segment_fraction 离散化的残留，"
          "占比需在 README 中披露\n")

    # 每个 query 的成功率分布。聚合成功率会掩盖双峰:
    # "36 个 query 各有 65% 成功率" 和 "24 个必成 + 12 个必败" 聚合后都是 65%,
    # 但含义完全不同 —— 后者说明难度集中在少数几个 query 上。
    print("每个 query 的成功率分布（检查双峰）:")
    print(f"{'gap':>6} {'planner':<26} {'全败':>6} {'部分':>6} {'全成':>6}")
    print("-" * 56)
    for g in gaps:
        for p in planners:
            never = sometimes = always = 0
            for (gg, pp, _), (k, n) in per_query.items():
                if gg != g or pp != p:
                    continue
                if k == 0:
                    never += 1
                elif k == n:
                    always += 1
                else:
                    sometimes += 1
            if never or sometimes or always:
                print(f"{g:>6.3f} {p:<26} {never:>6} {sometimes:>6} {always:>6}")
        print()

    # 配对路径长度比较 —— 主表里那一列不能直接横向比。
    #
    # 原因是幸存者偏差: gap=0.15 时 RRT 只成功 26%，那 26% 是树恰好朝目标
    # 长过去的"顺利"样本；真正要绕远路的难题被算作失败，不进路径长度统计。
    # 而 RRTConnect 98% 全做出来了，包含那些必然绕远的题。两列的【条件分布
    # 不同】，直接比等于拿 A 的简单题和 B 的全部题比。
    #
    # 唯一正确的比法是配对: 只取两个 planner 都成功过的 query，
    # 各自用其成功样本的中位数代表该 query，再逐题相比。
    if len(planners) == 2:
        pa, pb = planners
        print(f"配对路径长度（只取两者都成功的 query）:")
        print(f"{'gap':>6} {'配对数':>7} {'弃用':>6} {pa[:12]:>14} "
              f"{pb[:12]:>14} {'比值中位':>10}")
        print("-" * 66)
        for g in gaps:
            qids = {q for (gg, _, q) in per_query_len if gg == g}
            ratios, la, lb, dropped = [], [], [], 0
            for q in sorted(qids):
                va = per_query_len.get((g, pa, q), [])
                vb = per_query_len.get((g, pb, q), [])
                if not va or not vb:
                    dropped += 1  # 有一方一次都没成功，无法配对
                    continue
                ma, mb = pct(sorted(va), 0.5), pct(sorted(vb), 0.5)
                la.append(ma)
                lb.append(mb)
                ratios.append(ma / mb if mb else float("nan"))
            if not ratios:
                continue
            print(f"{g:>6.3f} {len(ratios):>7} {dropped:>6} "
                  f"{pct(sorted(la), 0.5):>14.2f} {pct(sorted(lb), 0.5):>14.2f} "
                  f"{pct(sorted(ratios), 0.5):>10.3f}")
        print(f"  比值 = {pa[:12]} / {pb[:12]}，>1 表示前者路径更长")
        print("  『弃用』随 gap 收窄而上升，正是幸存者偏差的直接证据 ——"
              "被弃掉的都是弱者解不动的难题\n")

    # 失败原因汇总
    bad = {c: sum(d.get(c, 0) for d in codes.values())
           for c in {c for d in codes.values() for c in d}}
    if bad:
        print("失败 error_code 汇总:", dict(sorted(bad.items(), key=lambda kv: -kv[1])))
        print("  -1 PLANNING_FAILED / -6 TIMED_OUT / 99999 FAILURE(MoveIt2 通用码)")
        print("  99999 太笼统，用上面的『失败机制拆分』表区分，判据是")
        print("  planning_time 是否精确为 0（MoveIt 只在成功分支给它赋值）")

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(7, 4.5))
        for p in planners:
            xs = [g for g in gaps if (g, p) in stats]
            ys = [stats[(g, p)][0] for g in xs]
            # max(0, ...): 成功率 100% 时 Wilson 上界算出来是 0.9999999996，
            # 减去 p=1.0 得到 -4e-10，matplotlib 会拒绝负的 yerr。
            lo = [max(0.0, stats[(g, p)][0] - stats[(g, p)][1]) for g in xs]
            hi = [max(0.0, stats[(g, p)][2] - stats[(g, p)][0]) for g in xs]
            ax.errorbar(xs, ys, yerr=[lo, hi], marker="o", capsize=3,
                        label=p.replace("kConfigDefault", ""))
        ax.set_xlabel("gap width w (m)")
        ax.set_ylabel("success rate")
        ax.set_title("S3 narrow passage: success rate vs gap width")
        ax.invert_xaxis()  # 左宽右窄 = 从易到难
        ax.set_ylim(-0.05, 1.05)
        ax.grid(alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig("s3_curve.png", dpi=150)
        print("\n图已存到 s3_curve.png")


if __name__ == "__main__":
    main()
