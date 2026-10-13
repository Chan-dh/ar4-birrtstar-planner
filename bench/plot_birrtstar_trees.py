#!/usr/bin/env python3
"""画出 birrtstar_2d 导出的两棵树与最终路径（stage_1_ompl_3.md §7.2）。

用法：python3 plot_birrtstar_trees.py <scenario> [trees_dir]
  scenario ∈ {free, obstacle, narrow, circles}，读 <trees_dir>/<scenario>_trees.txt
  输出 <trees_dir>/<scenario>_trees.png
"""
"""
./build/ar4_my_planners/birrtstar_2d narrow 2.0 /tmp
python3 bench/plot_birrtstar_trees.py narrow /tmp
"""
import sys
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle

# 场景定义与 birrtstar_2d.cpp 保持一致
SCENARIOS = {
    "free": {"start": (5, 5), "goal": (95, 95), "obstacles": []},
    "obstacle": {"start": (5, 5), "goal": (95, 95), "obstacles": [(30, 30, 70, 70)]},
    "narrow": {"start": (5, 5), "goal": (95, 95),
               "obstacles": [(48, 0, 52, 26), (48, 34, 52, 100)]},
    "circles": {"start": (5, 5), "goal": (95, 95), "obstacles": [],
                "circles": [(18, 18, 6), (24, 45, 16), (50, 60, 7), (70, 65, 10), (85, 80, 6)]},
}
        #  {{40.0, 20.0, 6.0}, {24.0, 45.0, 16.0}, {50.0, 60.0, 7.0}, {70.0, 65.0, 10.0}, {85.0, 80.0, 6.0}}},

def main():
    scenario = sys.argv[1] if len(sys.argv) > 1 else "obstacle"
    trees_dir = sys.argv[2] if len(sys.argv) > 2 else "/tmp"
    txt = os.path.join(trees_dir, f"{scenario}_trees.txt")
    out = os.path.join(trees_dir, f"{scenario}_trees.png")
    sc = SCENARIOS[scenario]

    vertices, edges, path = {}, [], []
    with open(txt) as f:
        for line in f:
            p = line.split()
            if p[0] == "V":
                vertices[int(p[1])] = (float(p[2]), float(p[3]), int(p[4]))
            elif p[0] == "E":
                edges.append((int(p[1]), int(p[2]), int(p[3])))
            elif p[0] == "P":
                path.append((float(p[1]), float(p[2])))

    fig, ax = plt.subplots(figsize=(8, 8))
    for x0, y0, x1, y1 in sc["obstacles"]:
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, facecolor="#9aa0a6", edgecolor="none"))
    for cx, cy, r in sc.get("circles", []):
        ax.add_patch(Circle((cx, cy), r, facecolor="#9aa0a6", edgecolor="none"))

    tree_colors = {1: "#0000ff", 2: "#ff0000"}  # tag 1 起点树纯蓝 / 2 目标树纯红
    tree_counts = {1: 0, 2: 0}
    for a, b, tag in edges:
        if tag not in (1, 2):
            continue
        (xa, ya, _), (xb, yb, _) = vertices[a], vertices[b]
        ax.plot([xa, xb], [ya, yb], color=tree_colors[tag], linewidth=0.4, zorder=1)
        tree_counts[tag] += 1

    if path:
        xs, ys = zip(*path)
        ax.plot(xs, ys, color="#000000", linewidth=2.2, zorder=3, label="solution path")

    sx, sy = sc["start"]
    gx, gy = sc["goal"]
    ax.plot(sx, sy, "*", color="#2ca02c", markersize=18, zorder=4, label="start")
    ax.plot(gx, gy, "*", color="#d62728", markersize=18, zorder=4, label="goal")

    # 手工图例：两棵树用纯线段句柄
    from matplotlib.lines import Line2D
    handles = [Line2D([], [], color=tree_colors[1], lw=2, label=f"start tree ({tree_counts[1]} edges)"),
               Line2D([], [], color=tree_colors[2], lw=2, label=f"goal tree ({tree_counts[2]} edges)")]
    if path:
        handles.append(Line2D([], [], color="#000000", lw=2.2, label=f"solution ({len(path)} waypoints)"))
    ax.legend(handles=handles, loc="upper left", fontsize=9, framealpha=0.9)

    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.set_aspect("equal")
    ax.set_title(f"BiRRTStar - {scenario}")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    print(f"saved {out}")


if __name__ == "__main__":
    main()
