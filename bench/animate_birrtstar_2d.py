#!/usr/bin/env python3
"""把 birrtstar_2d 快照模式导出的 <scenario>_frames.txt 做成 GIF。

左：两棵树生长 → 汇合 → rewire 把路径拉直；右：c_best 随规划时间的收敛曲线。

用法：
    ./build/ar4_my_planners/birrtstar_2d narrow 2.0 /tmp 60
    python3 bench/animate_birrtstar_2d.py narrow /tmp --out media/birrtstar_2d.gif
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plot_birrtstar_trees import SCENARIOS  # 场景定义只维护一份（与 birrtstar_2d.cpp 一致）

import matplotlib.pyplot as plt
from matplotlib.animation import PillowWriter
from matplotlib.collections import LineCollection
from matplotlib.patches import Circle, Rectangle

TREE_COLORS = {1: "#2f6fd6", 2: "#d9534f"}  # 起点树蓝 / 目标树红，与 plot_birrtstar_trees.py 同色系
OBSTACLE = "#9aa0a6"

# 解析最优（路径贴着障碍角点），作为收敛曲线的参考线；circles 没有简单闭式解，不画
OPTIMA = {
    "free": math.hypot(90, 90),
    "obstacle": 2 * math.hypot(25, 65),                     # 经 (30,70) 或 (70,30)
    "narrow": math.hypot(47, 29) + math.hypot(43, 61),      # 经上墙左下角 (52,34)
}


def load_frames(path):
    frames, cur = [], None
    with open(path) as f:
        for line in f:
            p = line.split()
            if p[0] == "F":
                cur = {"t": float(p[2]), "cost": float(p[3]), "iters": int(p[4]),
                       "V": {}, "E": [], "P": []}
                frames.append(cur)
            elif p[0] == "V":
                cur["V"][int(p[1])] = (float(p[2]), float(p[3]))
            elif p[0] == "E":
                cur["E"].append((int(p[1]), int(p[2]), int(p[3])))
            elif p[0] == "P":
                cur["P"].append((float(p[1]), float(p[2])))
    if not frames:
        sys.exit(f"{path} 里没有帧 —— birrtstar_2d 第 4 个参数（snapshots）给了吗？")
    return frames


def main():
    ap = argparse.ArgumentParser(description="BiRRTStar 二维生长过程动图")
    ap.add_argument("scenario", choices=sorted(SCENARIOS))
    ap.add_argument("frames_dir", nargs="?", default="/tmp")
    ap.add_argument("--out", default=None, help="默认 <frames_dir>/<scenario>_birrtstar.gif")
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--final-hold", type=float, default=2.5, help="末帧定格秒数")
    ap.add_argument("--dpi", type=int, default=80, help="figsize 10x5 英寸 × dpi = 像素")
    args = ap.parse_args()

    sc = SCENARIOS[args.scenario]
    frames = load_frames(os.path.join(args.frames_dir, f"{args.scenario}_frames.txt"))
    out = args.out or os.path.join(args.frames_dir, f"{args.scenario}_birrtstar.gif")

    finite = [(fr["t"], fr["cost"]) for fr in frames if math.isfinite(fr["cost"])]
    t_lo, t_hi = frames[0]["t"], frames[-1]["t"]
    opt = OPTIMA.get(args.scenario)
    if finite:
        c_lo = min(c for _, c in finite + ([(0, opt)] if opt else []))
        c_hi = max(c for _, c in finite)
        pad = 0.06 * (c_hi - c_lo or 1.0)
        c_lo, c_hi = c_lo - pad, c_hi + pad

    fig, (ax, axc) = plt.subplots(1, 2, figsize=(10, 5), gridspec_kw={"width_ratios": [1, 1]})

    def draw(fr):
        ax.clear()
        for x0, y0, x1, y1 in sc["obstacles"]:
            ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, facecolor=OBSTACLE, edgecolor="none"))
        for cx, cy, r in sc.get("circles", []):
            ax.add_patch(Circle((cx, cy), r, facecolor=OBSTACLE, edgecolor="none"))
        for tag in (1, 2):
            segs = [(fr["V"][a], fr["V"][b]) for a, b, tg in fr["E"] if tg == tag]
            ax.add_collection(LineCollection(segs, colors=TREE_COLORS[tag], linewidths=0.5, alpha=0.75))
        if fr["P"]:
            xs, ys = zip(*fr["P"])
            ax.plot(xs, ys, color="#111111", linewidth=2.4, zorder=3)
        (sx, sy), (gx, gy) = sc["start"], sc["goal"]
        ax.plot(sx, sy, "o", color=TREE_COLORS[1], markersize=9, zorder=4)
        ax.plot(gx, gy, "o", color=TREE_COLORS[2], markersize=9, zorder=4)
        ax.set_xlim(0, 100)
        ax.set_ylim(0, 100)
        ax.set_aspect("equal")
        ax.set_xticks([])
        ax.set_yticks([])
        cost = f"{fr['cost']:.1f}" if math.isfinite(fr["cost"]) else "—"
        ax.set_title(f"t = {fr['t']:.3f} s    nodes {len(fr['V'])}    c_best {cost}", fontsize=11)

        axc.clear()
        seen = [(t, c) for t, c in finite if t <= fr["t"] + 1e-12]
        if opt:
            axc.axhline(opt, color="#888888", linestyle="--", linewidth=1)
            axc.text(t_hi, opt, f" optimum {opt:.1f}", va="bottom", ha="right", fontsize=9, color="#666666")
        if seen:
            ts, cs = zip(*seen)
            axc.step(ts, cs, where="post", color="#111111", linewidth=1.8)
            axc.plot(ts[-1], cs[-1], "o", color="#111111", markersize=5)
        axc.set_xscale("log")
        axc.set_xlim(t_lo, t_hi * 1.05)
        if finite:
            axc.set_ylim(c_lo, c_hi)
        axc.set_xlabel("planning time (s, log)")
        axc.set_ylabel("best path cost")
        axc.set_title("anytime convergence", fontsize=11)
        axc.grid(True, which="both", alpha=0.25)
        fig.suptitle(f"BiRRTStar — {args.scenario}   (blue: start tree, red: goal tree)", fontsize=12)
        fig.tight_layout()

    writer = PillowWriter(fps=args.fps)
    with writer.saving(fig, out, dpi=args.dpi):
        for fr in frames:
            draw(fr)
            writer.grab_frame()
        for _ in range(int(args.final_hold * args.fps)):  # 定格：重复末帧
            writer.grab_frame()
    print(f"saved {out}（{len(frames)} 帧 + 定格 {args.final_hold}s）")


if __name__ == "__main__":
    main()
