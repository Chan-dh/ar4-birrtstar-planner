#!/usr/bin/env python3
"""
S3 演示动画 —— README 首屏 GIF 的素材生成器。

同一道窄通道题，先播 RRTConnect、再播 BiRRTStar，末端（ee_link）轨迹各留一条彩线，
最后两条线叠在一起定格。机械臂姿态由本脚本按时间逐帧发布（DisplayRobotState），
所以机械臂动作和彩线严格同步，每次重放画面都完全一样，录坏了重来即可。

诚实性约定：
  - 选题：在 Phase D CSV 中，两个算法成功率都 ≥ --min-success 的题里，
    取"BiRRTStar/RRTConnect 路径长度中位数比值"最接近全体中位数的那一道 —— 不挑最好看的。
  - 选解：每个算法规划 --tries 次，取 path_length 最接近该题 CSV 中位数的一次。
  - 规划走 mine 管线（与跑分完全相同，不经简化），path_length 与 CSV 同口径（关节空间 L2，rad）。
  - 播放速度对两者相同（关节空间恒速 --speed rad/s），所以"更长的路径播得更久"是真实的。

用法（在 bench/ 目录下）:
    # 终端 1
    ros2 launch ./bench.launch.py use_rviz:=true rviz_config:=$PWD/demo.rviz
    # 终端 2
    python3 demo_viz.py --list          # 只看候选题（离线，不连 ROS）
    python3 demo_viz.py                 # 自动选题 → 规划 → 播放一遍
    python3 demo_viz.py --loop          # 循环播放：调视角、录屏都用这个

选中的两条路径缓存在 demo_paths.json，之后重放不再重新规划（--replan 强制重规划）。
"""

import argparse
import bisect
import csv
import json
import math
import os
import statistics
import sys
import time

import rclpy

from s3_runner import PLANNERS, Runner, load_queries, path_length
from s3_scene import JOINT_NAMES, WALL_X, WALL_Z_TOP

from geometry_msgs.msg import Point
from moveit_msgs.msg import DisplayRobotState, RobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

BASELINE = "mine/RRTConnect"
MINE = "mine/BiRRTStar"

# 橙 / 蓝：红绿色盲也能区分；浅色背景下文字同色即可读
COLORS = {
    BASELINE: (0.93, 0.49, 0.13),
    MINE: (0.16, 0.42, 0.85),
}
GREY = (0.35, 0.35, 0.38)
NS = "s3_demo"


# ---------------------------------------------------------------------------
# 选题（纯离线，读 CSV）
# ---------------------------------------------------------------------------


def candidates(csv_path, gap, min_success):
    """返回 (按"离全体中位比值的距离"排序的候选列表, 全体中位比值)。"""
    runs = {}  # (qid, planner) -> [path_length of successes], total
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            if abs(float(row["gap"]) - gap) > 1e-6 or row["planner"] not in (BASELINE, MINE):
                continue
            key = (int(row["query_id"]), row["planner"])
            ok, n = runs.get(key, ([], 0))
            if row["success"] == "True":
                ok.append(float(row["path_length"]))
            runs[key] = (ok, n + 1)

    out = []
    for qid in sorted({k[0] for k in runs}):
        a_ok, a_n = runs.get((qid, BASELINE), ([], 0))
        b_ok, b_n = runs.get((qid, MINE), ([], 0))
        if not a_n or not b_n:
            continue
        sa, sb = len(a_ok) / a_n, len(b_ok) / b_n
        if sa < min_success or sb < min_success:
            continue
        ma, mb = statistics.median(a_ok), statistics.median(b_ok)
        out.append({"qid": qid, "succ_a": sa, "succ_b": sb,
                    "med_a": ma, "med_b": mb, "ratio": mb / ma})
    if not out:
        sys.exit(f"gap={gap} 下没有两个算法成功率都 ≥ {min_success} 的题，换 --gap 或降低 --min-success")

    ratio_med = statistics.median(c["ratio"] for c in out)
    out.sort(key=lambda c: abs(c["ratio"] - ratio_med))
    return out, ratio_med


def print_candidates(cands, ratio_med, top=8):
    print(f"合格题 {len(cands)} 道，BiRRTStar/RRTConnect 路径中位比值的全体中位数 = "
          f"{ratio_med:.3f}（路径 {100 * (ratio_med - 1):+.0f}%）")
    print(f"{'qid':>4} {'succ RC':>8} {'succ BI':>8} {'med RC':>8} {'med BI':>8} {'ratio':>7}")
    for c in cands[:top]:
        print(f"{c['qid']:>4} {c['succ_a']:>8.0%} {c['succ_b']:>8.0%} "
              f"{c['med_a']:>8.2f} {c['med_b']:>8.2f} {c['ratio']:>7.3f}")


# ---------------------------------------------------------------------------
# 轨迹处理
# ---------------------------------------------------------------------------


def densify(points, step):
    """关节空间线性插值，使相邻点 L2 距离 ≤ step —— 末端彩线才画得出真实的弯。"""
    out = [list(points[0])]
    for a, b in zip(points[:-1], points[1:]):
        d = math.dist(a, b)
        n = max(1, math.ceil(d / step))
        for k in range(1, n + 1):
            s = k / n
            out.append([x + (y - x) * s for x, y in zip(a, b)])
    return out


def timestamps(points, speed):
    """关节空间恒速：t_i = 累计弧长 / speed。"""
    t = [0.0]
    for a, b in zip(points[:-1], points[1:]):
        t.append(t[-1] + math.dist(a, b) / speed)
    return t


def rgba(rgb, a=1.0):
    return ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=a)


# ---------------------------------------------------------------------------
# 节点
# ---------------------------------------------------------------------------


class Demo(Runner):
    def __init__(self, args):
        super().__init__()
        self.args = args
        self.state_pub = self.create_publisher(DisplayRobotState, "/s3_demo/robot_state", 1)
        self.marker_pub = self.create_publisher(MarkerArray, "/s3_demo/markers", 10)

    # -- 规划 ---------------------------------------------------------------
    def best_of(self, start, goal, planner, target):
        """规划 --tries 次，返回 path_length 最接近 target 的那条路点列表。"""
        sols = []
        for i in range(self.args.tries):
            res, _ = self.plan_raw(start, goal, planner, self.args.group, self.args.timeout)
            if res is None or res.motion_plan_response.error_code.val != 1:
                print(f"  {planner} #{i}: 失败")
                continue
            traj = res.motion_plan_response.trajectory
            L = path_length(traj)
            print(f"  {planner} #{i}: path_length = {L:.2f}")
            sols.append((L, [list(p.positions) for p in traj.joint_trajectory.points]))
        if not sols:
            sys.exit(f"{planner} 在 {self.args.tries} 次里一次都没成功")
        if target is None:  # 没有 CSV：退化为本轮的中位数
            target = statistics.median(L for L, _ in sols)
        L, pts = min(sols, key=lambda s: abs(s[0] - target))
        print(f"  → 选用 {L:.2f}（目标 {target:.2f}）")
        return {"path_length": L, "points": pts}

    # -- 发布 ---------------------------------------------------------------
    def show_state(self, q):
        msg = DisplayRobotState()
        msg.state = RobotState()
        msg.state.joint_state = JointState(name=JOINT_NAMES, position=list(q))
        msg.state.is_diff = True
        self.state_pub.publish(msg)

    def _marker(self, mid, mtype):
        m = Marker()
        m.header.frame_id = self.args.frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = NS
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def line(self, mid, pts, rgb):
        m = self._marker(mid, Marker.LINE_STRIP)
        if len(pts) < 2:
            m.action = Marker.DELETE
        m.scale.x = 0.008
        m.color = rgba(rgb)
        m.points = pts
        return m

    def sphere(self, mid, p, rgb):
        m = self._marker(mid, Marker.SPHERE)
        m.pose.position = p
        m.scale.x = m.scale.y = m.scale.z = 0.03
        m.color = rgba(rgb)
        return m

    def text(self, mid, xyz, s, rgb, size=0.045):
        m = self._marker(mid, Marker.TEXT_VIEW_FACING)
        m.pose.position = Point(x=xyz[0], y=xyz[1], z=xyz[2])
        m.scale.z = size
        m.color = rgba(rgb)
        m.text = s  # RViz 字体不含中文，只用 ASCII
        return m

    def publish(self, *markers):
        self.marker_pub.publish(MarkerArray(markers=list(markers)))

    def clear(self):
        m = Marker()
        m.ns = NS
        m.action = Marker.DELETEALL
        self.publish(m)

    # -- 播放 ---------------------------------------------------------------
    def play(self, track, mid):
        """逐帧推进机械臂并让彩线跟着长。"""
        q, ee, t = track["q"], track["ee"], track["t"]
        rgb = COLORS[track["planner"]]
        dt = 1.0 / self.args.fps
        t0 = time.monotonic()
        while True:
            now = min(time.monotonic() - t0, t[-1])
            i = bisect.bisect_right(t, now) - 1
            self.show_state(q[i])
            self.publish(self.line(mid, ee[: i + 1], rgb))
            if now >= t[-1]:
                return
            time.sleep(dt)

    def run_once(self, tracks, gap):
        a, b = tracks[BASELINE], tracks[MINE]
        La, Lb = a["path_length"], b["path_length"]
        # 标签放在墙上方、远离洞口，不挡机械臂
        x, y, z = WALL_X, -0.35, WALL_Z_TOP + 0.08

        self.clear()
        self.show_state(a["q"][0])
        self.publish(
            self.text(10, (x, y, z + 0.16), f"S3 narrow passage  gap = {gap:.2f} m", GREY, 0.05),
            self.sphere(20, a["ee"][0], GREY),
            self.sphere(21, a["ee"][-1], GREY),
            self.text(22, (a["ee"][0].x, a["ee"][0].y, a["ee"][0].z + 0.05), "start", GREY, 0.035),
            self.text(23, (a["ee"][-1].x, a["ee"][-1].y, a["ee"][-1].z + 0.05), "goal", GREY, 0.035),
        )
        time.sleep(self.args.hold)

        self.publish(self.text(11, (x, y, z + 0.08), "RRTConnect", COLORS[BASELINE]))
        self.play(a, 1)
        self.publish(self.text(11, (x, y, z + 0.08), f"RRTConnect   path {La:.1f} rad",
                               COLORS[BASELINE]))
        time.sleep(self.args.pause)

        self.show_state(b["q"][0])
        self.publish(self.text(12, (x, y, z), "BiRRTStar", COLORS[MINE]))
        time.sleep(0.4)
        self.play(b, 2)
        self.publish(self.text(12, (x, y, z),
                               f"BiRRTStar    path {Lb:.1f} rad  ({100 * (Lb / La - 1):+.0f}%)",
                               COLORS[MINE]))
        time.sleep(self.args.final_hold)

    # -- 预处理：插值 + 末端 FK -----------------------------------------------
    def make_track(self, planner, sol):
        q = densify(sol["points"], self.args.step)
        ee = []
        for qi in q:
            p = self.fk_point(qi, self.args.tip, self.args.frame)
            if p is None:
                sys.exit(f"FK 失败：检查 --tip {self.args.tip!r} 是否存在")
            ee.append(p)
        return {"planner": planner, "path_length": sol["path_length"],
                "q": q, "ee": ee, "t": timestamps(q, self.args.speed)}


# ---------------------------------------------------------------------------


def load_cache(path, gap, qid):
    if not os.path.exists(path):
        return None
    with open(path) as f:
        c = json.load(f)
    if abs(c.get("gap", -1) - gap) > 1e-6 or c.get("query_id") != qid:
        return None
    return c["solutions"]


def main():
    ap = argparse.ArgumentParser(description="S3 演示动画（RViz）")
    ap.add_argument("--gap", type=float, default=0.23)
    ap.add_argument("--query", type=int, default=None, help="指定题号；缺省时按 CSV 自动选代表性题")
    ap.add_argument("--csv", default="s3_phaseD.csv")
    ap.add_argument("--queries-file", default="s3_queries.json")
    ap.add_argument("--min-success", type=float, default=0.9)
    ap.add_argument("--list", action="store_true", help="只打印候选题后退出（不连 ROS）")
    ap.add_argument("--tries", type=int, default=10, help="每个算法规划几次，从中取代表性的一次")
    ap.add_argument("--timeout", type=float, default=2.0, help="与 Phase D 相同的预算")
    ap.add_argument("--cache", default="demo_paths.json")
    ap.add_argument("--replan", action="store_true", help="忽略缓存，重新规划")
    ap.add_argument("--speed", type=float, default=3.0, help="播放速度，关节空间 rad/s，两算法相同")
    ap.add_argument("--step", type=float, default=0.03, help="插值步长 rad（越小彩线越平滑）")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--hold", type=float, default=2.0, help="开场静止秒数")
    ap.add_argument("--pause", type=float, default=1.0, help="两段之间的停顿秒数")
    ap.add_argument("--final-hold", type=float, default=4.0, help="结尾定格秒数")
    ap.add_argument("--loop", action="store_true", help="循环播放直到 Ctrl-C")
    ap.add_argument("--group", default="ar_manipulator")
    ap.add_argument("--tip", default="ee_link", help="画轨迹的连杆（ee_link 在夹爪指尖）")
    ap.add_argument("--frame", default="base_link")
    args = ap.parse_args()

    # 1. 选题
    targets = {BASELINE: None, MINE: None}
    if os.path.exists(args.csv):
        cands, ratio_med = candidates(args.csv, args.gap, args.min_success)
        print_candidates(cands, ratio_med)
        if args.query is None:
            chosen = cands[0]
        else:
            chosen = next((c for c in cands if c["qid"] == args.query), None)
            if chosen is None:
                print(f"注意：题 {args.query} 不在合格候选里，按指定题继续，但它不具代表性")
        if chosen is not None:
            targets = {BASELINE: chosen["med_a"], MINE: chosen["med_b"]}
            args.query = chosen["qid"]
            print(f"\n选用题 {args.query}：该题中位比值 {chosen['ratio']:.3f}，"
                  f"全体中位数 {ratio_med:.3f}")
    elif args.query is None:
        sys.exit(f"找不到 {args.csv}，请用 --query 指定题号")
    if args.list:
        return

    _, qmap = load_queries(args.queries_file)
    q = qmap[args.query]

    rclpy.init()
    node = Demo(args)
    try:
        node.wait_for(node.plan_cli, "/plan_kinematic_path")
        node.wait_for(node.fk_cli, "/compute_fk")
        node.publish_wall(args.gap, args.frame)

        # 2. 规划（或读缓存）
        sols = None if args.replan else load_cache(args.cache, args.gap, args.query)
        if sols is None:
            sols = {}
            for p in (BASELINE, MINE):
                print(f"\n规划 {p}（{PLANNERS[p][1]}），{args.tries} 次：")
                sols[p] = node.best_of(q["start"], q["goal"], p, targets[p])
            with open(args.cache, "w") as f:
                json.dump({"gap": args.gap, "query_id": args.query, "solutions": sols}, f)
            print(f"已缓存到 {args.cache}")
        else:
            print(f"\n读取缓存 {args.cache}（--replan 可重新规划）")

        # 3. 预处理
        print("计算末端轨迹 ...")
        tracks = {p: node.make_track(p, sols[p]) for p in (BASELINE, MINE)}
        for p, tr in tracks.items():
            print(f"  {p}: path_length {tr['path_length']:.2f} rad，播放 {tr['t'][-1]:.1f} s")

        # 4. 播放
        time.sleep(0.5)  # 让 RViz 先完成订阅
        while True:
            node.run_once(tracks, args.gap)
            if not args.loop:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
