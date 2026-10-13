#!/usr/bin/env python3
"""
S3 query 生成器 —— 为窄缝墙场景生成 start/goal 对。

方法论要点（每一条都是为了让 benchmark 站得住脚）：

1. 关节空间采样 + 碰撞检查，不做笛卡尔位姿 + IK。
   → start / goal 各自一定是合法可达构型，不存在 "IK 无解" 这种污染。

2. 强制 start 在近侧、goal 在远侧，绝不随机配对。
   → 远侧构型只占 ~5%，随机配对的话 90% 的 query 压根不用穿墙，
     墙就不再是变量，成功率曲线会是一条平线。

3. start / goal 池在【无墙】场景下采样一次，固定复用于所有 gap 档位。
   → 否则窄 gap 下根本采不到远侧样本（实测 gap=0.10 时远侧为 0）。

4. 顺序是 FK 先、碰撞检查后。
   → FK 便宜，先用它筛掉 ~95% 的近侧样本，只对候选做昂贵的 validity 调用。

注意：本脚本只产出【候选】query。还要用金标准 planner（长超时）预筛选出
     确定有解的子集，才能进入正式 benchmark —— 那一步在 runner 里做。

用法（先确保 move_group 在跑，且场景已清空）:
    python3 s3_scene.py --clear
    python3 s3_queries.py --samples 20000 --queries 60
"""

import argparse
import json
import math
import random

import rclpy

from s3_scene import (
    JOINT_LIMITS_DEG,
    JOINT_NAMES,
    WALL_X,
    S3Client,
)


def joint_dist(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


class QueryBuilder(S3Client):
    def build_pools(self, n_samples, group, tip, frame, seed, margin, near_rate):
        """在无墙场景下采样，分出近侧 / 远侧两个合法构型池。

        margin: 远侧构型的末端 x 必须 > WALL_X + margin，
                避免 goal 贴着墙面，给规划器留一点余地。
        """
        rng = random.Random(seed)
        limits = [(math.radians(lo), math.radians(hi)) for lo, hi in JOINT_LIMITS_DEG]

        near, far = [], []
        far_candidates = 0

        for i in range(n_samples):
            q = [rng.uniform(lo, hi) for lo, hi in limits]

            # FK 便宜，先判侧
            x = self.fk_x(q, tip, frame)
            if x is None:
                continue

            if x > WALL_X + margin:
                far_candidates += 1
                if self.is_valid(q, group):
                    far.append(q)
            elif x < WALL_X - margin:
                # 近侧只需要一部分，抽样以省下 validity 调用
                if rng.random() < near_rate and self.is_valid(q, group):
                    near.append(q)

            if (i + 1) % 2000 == 0:
                print(
                    f"  已采样 {i + 1}/{n_samples}  "
                    f"远侧候选={far_candidates}  远侧合法={len(far)}  近侧合法={len(near)}"
                )

        return near, far, far_candidates

    def feasible_under_gap(self, queries, gap, group, tip, frame):
        """在给定 gap 下，哪些 query 的两个端点仍然无碰撞。

        窄 gap 时大量 goal 会失效 —— 不是末端够不到，而是整条手臂被墙挡住。
        端点本身碰撞 = 该 query 无解，应当排除出统计，而不是算作算法失败。
        """
        self.publish_wall(gap, frame)
        ok = []
        for qi, q in enumerate(queries):
            if self.is_valid(q["start"], group) and self.is_valid(q["goal"], group):
                ok.append(qi)
        return ok


def main():
    ap = argparse.ArgumentParser(description="S3 query 生成器")
    ap.add_argument("--samples", type=int, default=20000, help="无墙场景下的采样总数")
    ap.add_argument(
        "--queries", type=int, default=0, help="候选 query 数上限，0 = 用满整个近侧池"
    )
    ap.add_argument(
        "--near-rate",
        type=float,
        default=0.05,
        help="近侧样本做碰撞检查的抽样率。调高可增大近侧池，代价是更多 validity 调用",
    )
    ap.add_argument(
        "--gaps",
        nargs="*",
        type=float,
        default=[0.45, 0.36, 0.29, 0.23, 0.18, 0.15],
        help="难度档位",
    )
    ap.add_argument("--margin", type=float, default=0.04, help="端点离墙面的最小余量 (m)")
    ap.add_argument("--min-dist", type=float, default=1.5, help="start/goal 关节空间最小距离")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--group", default="ar_manipulator")
    ap.add_argument("--tip", default="link_6")
    ap.add_argument("--frame", default="base_link")
    ap.add_argument("--out", default="s3_queries.json")
    args = ap.parse_args()

    rclpy.init()
    node = QueryBuilder()
    try:
        node.wait_for(node.apply_cli, "/apply_planning_scene")
        node.wait_for(node.valid_cli, "/check_state_validity")
        node.wait_for(node.fk_cli, "/compute_fk")

        # --- 步骤 1: 无墙场景下建池（顺便得到缺失的对照组数据）-----------
        print("清除场景，开始在【无墙】条件下采样...")
        node.clear_wall(args.frame)
        near, far, far_cand = node.build_pools(
            args.samples,
            args.group,
            args.tip,
            args.frame,
            args.seed,
            args.margin,
            args.near_rate,
        )

        far_ratio_no_wall = far_cand / args.samples
        print(
            f"\n[对照组] 无墙时远侧占比 = {far_ratio_no_wall:.2%}"
            f"  (远侧候选 {far_cand}/{args.samples})"
        )
        print(f"近侧合法池 = {len(near)}   远侧合法池 = {len(far)}")

        # --- 步骤 2: 配对成 query（强制 近侧 -> 远侧）---------------------
        # 候选数尽量拉满：最窄档只留存 ~7%，交集集合全靠候选基数撑起来。
        cap = args.queries if args.queries > 0 else min(len(near), len(far))
        rng = random.Random(args.seed + 1)
        rng.shuffle(near)
        rng.shuffle(far)

        queries, gi, rejected = [], 0, 0
        for s in near:
            if len(queries) >= cap or gi >= len(far):
                break
            g = far[gi]
            gi += 1
            if joint_dist(s, g) < args.min_dist:
                rejected += 1
                continue
            queries.append({"id": len(queries), "start": s, "goal": g})

        print(
            f"生成候选 query {len(queries)} 个（start 近侧 -> goal 远侧），"
            f"因 min_dist 剔除 {rejected} 个"
        )

        # --- 步骤 3: 逐档过滤端点可行性 ---------------------------------
        print("\n逐档检查端点可行性:")
        feasible = {}
        for gap in sorted(args.gaps, reverse=True):
            ok = node.feasible_under_gap(queries, gap, args.group, args.tip, args.frame)
            feasible[f"{gap:.3f}"] = ok
            print(
                f"  gap={gap:.3f}  端点可行 {len(ok):>4}/{len(queries)}  "
                f"({len(ok) / len(queries):.1%})"
            )

        node.clear_wall(args.frame)

        # --- 步骤 4: 取交集，作为跨全部档位的统一 query 集 ----------------
        # 自由区 {|y|<w/2} ∩ {0.25<z<0.55} 随 w 严格单调增，
        # 所以交集 == 最窄档的可行集。这里显式求交，兼作单调性自检。
        inter = set(range(len(queries)))
        for ids in feasible.values():
            inter &= set(ids)
        canonical = sorted(inter)

        narrowest = f"{min(args.gaps):.3f}"
        if len(canonical) != len(feasible[narrowest]):
            print(
                f"\n[警告] 交集 {len(canonical)} != 最窄档 {len(feasible[narrowest])}，"
                f"单调性被破坏，检查墙体几何"
            )

        print(f"\n>>> 统一 query 集（交集）= {len(canonical)} 个，用于全部 {len(args.gaps)} 档")
        if len(canonical) < 30:
            print("    偏少。加大 --samples 或 --near-rate，或把最窄档从 0.15 放宽到 0.18。")

        out = {
            "meta": {
                "wall_x": WALL_X,
                "joint_names": JOINT_NAMES,
                "group": args.group,
                "tip": args.tip,
                "frame": args.frame,
                "seed": args.seed,
                "samples": args.samples,
                "margin": args.margin,
                "min_dist": args.min_dist,
                "far_ratio_no_wall": round(far_ratio_no_wall, 4),
                "near_pool": len(near),
                "far_pool": len(far),
            },
            "gaps": sorted(args.gaps, reverse=True),
            "queries": queries,
            "endpoint_feasible_by_gap": feasible,
            # 正式 benchmark 只跑这一批，全部档位共用 —— 唯一变量是 gap
            "canonical_query_ids": canonical,
        }
        with open(args.out, "w") as f:
            json.dump(out, f, indent=2)
        print(f"\n已写入 {args.out}")
        print("下一步: 用金标准 planner（60s 超时）在最窄档预筛选出确定有解的子集。")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
