#!/usr/bin/env python3
"""
S3 benchmark runner —— 预筛选 + 正式实验。

两个模式:

  --prescreen   用金标准 planner（长超时）在【最窄档】筛出确定有解的 query。
                自由空间随 gap 单调增，所以最窄档有解 => 全部档位都有解。
                只需筛一次，省掉 5/6 的开销。

  （默认）      正式实验。全部档位共用同一批 SOLVABLE query，唯一变量是 gap。
                成功率的定义因此很干净:
                    "在已知有解的问题上，算法能在 T 秒内找到解的比例"

success 的定义（这是整个实验的因变量，务必看清）:
    error_code == SUCCESS，且 pipeline 只保留 ValidateSolution 这一个
    response adapter（见 bench.launch.py）。因此:
        success = "OMPL 找到了一条路径，且该路径通过独立的碰撞复检"
    而【不是】"整条 MoveIt 流水线跑通了"。两者的区别不是文字游戏 ——
    上一轮开着 AddTimeOptimalParameterization 和路径简化时，928 次失败
    全部是后处理拒绝，失败耗时 ≈ 成功耗时、且从不超时，测出来的根本
    不是规划能力。

已知限制（务必写进 README 的 Limitations）:
  1. collision_checks 拿不到。/plan_kinematic_path 不暴露 planner 内部计数，
     要插计数器必须自己写 planner 插件。故本轮只能记 planning_time ——
     请固定机器、固定负载跑完整轮，并在 README 注明 CPU 型号。
  2. 随机种子控制不了。MotionPlanRequest 没有 seed 字段，"5 个 seed" 实为
     5 次独立重复。统计上等价，但单次结果不可精确复现。

用法（先起 bench.launch.py，不要用 demo.launch.py —— 后者开着路径简化）:
    # 0) move_group 输出务必 tee 落盘 —— t_first 与 (t,cost) 收敛事件流
    #    都在日志里，分析期靠 CSV 的 t_request 墙钟列与之对齐：
    #    ros2 launch ./bench.launch.py 2>&1 | tee /tmp/move_group.log
    # 1) 预筛选（一次性）。换夹爪配置后必须重跑。
    python3 s3_runner.py --prescreen --max-queries 60

    # 2) 正式实验（可随时 Ctrl-C，重跑会自动续上；默认 mine 三算法）
    python3 s3_runner.py --reps 5 --timeout 2.0

    # 2b) 地基检查（一次性门禁）：mine 与 ompl 的 RRTConnect 对比
    python3 s3_runner.py --gaps 0.23 --planners mine/RRTConnect,ompl/RRTConnect \
                         --reps 40 --out s3_ground.csv

    # 2c) Phase D 定点实验
    python3 s3_runner.py --gaps 0.23 --reps 50 --out s3_phaseD.csv

    # 3) 无夹爪对照组 —— 换 launch 参数，并且换输出文件名
    #    注意: link_5/link_6 的碰撞网格在上游 URDF 里缺失（Link_5_Col.STL /
    #    Link_6_Col.STL 不存在），MoveIt 会把这两个连杆的碰撞体置空。
    #    所以摘掉夹爪不是"同一条臂减掉夹爪"，而是"link_4 之后什么碰撞体
    #    都没有" —— 末端整段变成幽灵，测的不再是"末端能不能钻过洞"。
    #    要跑这组对照必须如实这么描述，否则结论是错的。
    #    ros2 launch ./bench.launch.py include_gripper:=false
    python3 s3_runner.py --prescreen --solvable-file s3_solvable_nogrip.json
    python3 s3_runner.py --solvable-file s3_solvable_nogrip.json \
                         --out s3_results_nogrip.csv
"""

import argparse
import csv
import hashlib
import json
import math
import os
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from s3_scene import JOINT_LIMITS_DEG, JOINT_NAMES, S3Client

from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, RobotState
from moveit_msgs.srv import GetMotionPlan
from sensor_msgs.msg import JointState
from std_msgs.msg import String

# label → (pipeline_id, planner_id)。label 即 CSV 的 planner 列（自含 pipeline 信息）。
# mine 三项 = Phase D 正式实验默认集；ompl 两项仅地基检查用（--planners 显式指定）。
# 算法本体同一份（og::RRTConnect / og::RRTstar），ompl 位是 MoveIt 官方管线的"验壳基线"。
PLANNERS = {
    "mine/RRTConnect": ("mine", "RRTConnect"),
    "mine/RRTstar": ("mine", "RRTstar"),
    "mine/BiRRTStar": ("mine", "BiRRTStar"),
    "ompl/RRTConnect": ("ompl", "RRTConnectkConfigDefault"),
    "ompl/RRTstar": ("ompl", "RRTkConfigDefault"),
}
DEFAULT_PLANNERS = ["mine/RRTConnect", "mine/RRTstar", "mine/BiRRTStar"]
GOLDEN = "mine/RRTConnect"  # 预筛选金标准：与正式实验同一适配层

# moveit_msgs/MoveItErrorCodes。区分 "搜索失败(-1/-6)" 和 "请求被拒绝(其余)" 至关重要：
# 后者说明 request 压根没进 planner，是我们的 query 或约束有问题，不是算法难。
ERROR_NAMES = {
    1: "SUCCESS",
    -1: "PLANNING_FAILED",
    -2: "INVALID_MOTION_PLAN",
    -6: "TIMED_OUT",
    -11: "START_STATE_IN_COLLISION",
    -12: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
    -13: "GOAL_IN_COLLISION",
    -14: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    -15: "GOAL_CONSTRAINTS_VIOLATED",
    -16: "INVALID_GROUP_NAME",
    -17: "INVALID_GOAL_CONSTRAINTS",
    -18: "INVALID_ROBOT_STATE",
    -31: "NO_IK_SOLUTION",
}

CSV_FIELDS = [
    "gap",
    "query_id",
    "planner",
    "rep",
    "success",
    "planning_time",
    "path_length",
    "num_waypoints",
    "error_code",
    # 发请求时的墙钟（unix 秒）。分析期用它把 CSV 行与 move_group 日志里的
    # [t_first_solution] / [improved_solution] 行按时间窗对齐（stage_1_ompl_3.md §4.1 决定）。
    "t_request",
]


def path_length(traj):
    """关节空间路径长度：相邻路点 L2 距离累加。"""
    pts = traj.joint_trajectory.points
    if len(pts) < 2:
        return 0.0
    total = 0.0
    for a, b in zip(pts[:-1], pts[1:]):
        total += math.sqrt(sum((x - y) ** 2 for x, y in zip(a.positions, b.positions)))
    return total


class Runner(S3Client):
    def __init__(self):
        super().__init__()
        self.plan_cli = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self._urdf = None

    def urdf(self, timeout=10.0):
        """取回实际加载的 URDF（带缓存）。

        不看 launch 参数、不看用户声明 —— 直接读 move_group 。
        """
        if self._urdf is not None:
            return self._urdf
        got = []
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # latched，晚订阅也收得到
            history=HistoryPolicy.KEEP_LAST,
        )
        sub = self.create_subscription(
            String, "/robot_description", lambda m: got.append(m.data), qos
        )
        deadline = time.time() + timeout
        while not got and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        self.destroy_subscription(sub)
        if not got:
            raise RuntimeError(
                "收不到 /robot_description —— robot_state_publisher 起来了吗？"
            )
        self._urdf = got[0]
        return self._urdf

    def has_gripper(self):
        return "gripper_base_link" in self.urdf()

    def check_limits(self, tol=1e-4):
        """核对 s3_scene.JOINT_LIMITS_DEG 与实际加载的机器人是否一致。

        采样箱是硬编码的，机器人型号是 launch 参数 —— 两者天然会脱钩，
        而且脱钩了不会报错。mk3/mk4 限位恰好相同，正是这个巧合掩盖了问题；
        mk5 的 j1 窄 10°，换过去采样箱就比真实可行域宽。

        后果很隐蔽: 采出越限的 start/goal，CheckStartStateBounds 会静默
        clamp 或拒绝，表现为"莫名其妙的失败"，从成功率上看不出原因。
        所以开跑前硬核对一次，不一致直接中止。
        """
        root = ET.fromstring(self.urdf())
        actual = {}
        for j in root.iter("joint"):
            lim = j.find("limit")
            if lim is not None and lim.get("lower") is not None:
                actual[j.get("name")] = (
                    float(lim.get("lower")), float(lim.get("upper"))
                )

        bad = []
        for name, (lo_deg, hi_deg) in zip(JOINT_NAMES, JOINT_LIMITS_DEG):
            if name not in actual:
                bad.append(f"  {name}: URDF 里没有这个关节，或它没有 <limit>")
                continue
            have = actual[name]
            if (abs(math.radians(lo_deg) - have[0]) > tol
                    or abs(math.radians(hi_deg) - have[1]) > tol):
                bad.append(
                    f"  {name}: 硬编码 [{lo_deg:>6.1f}°,{hi_deg:>6.1f}°]   "
                    f"实际 [{math.degrees(have[0]):>6.1f}°,"
                    f"{math.degrees(have[1]):>6.1f}°]"
                )
        if bad:
            raise SystemExit(
                "[中止] s3_scene.JOINT_LIMITS_DEG 与实际加载的机器人不符:\n"
                + "\n".join(bad)
                + "\n\n       采样箱和真实可行域错位，query 不可信。\n"
                  "       要么 launch 换回 ar_model:=mk4，\n"
                  "       要么改 JOINT_LIMITS_DEG 并重跑 s3_queries.py。"
            )

    def plan(self, start, goal, planner, group, timeout, attempts=1):
        """planner 参数是 PLANNERS 的 label（如 "mine/BiRRTStar"）。"""
        if planner not in PLANNERS:
            raise KeyError(f"未知 planner label: {planner!r}，可选: {sorted(PLANNERS)}")
        pipeline_id, planner_id = PLANNERS[planner]

        req = GetMotionPlan.Request()
        mpr = MotionPlanRequest()
        mpr.group_name = group
        mpr.planner_id = planner_id
        mpr.pipeline_id = pipeline_id
        mpr.allowed_planning_time = float(timeout)
        mpr.num_planning_attempts = attempts
        # 显式指定，否则默认 0 会让 TOTG 每次打 warning 并回退到 1.0。
        # 值本身不影响规划，只影响时间参数化 —— 但要写死，实验才自解释。
        mpr.max_velocity_scaling_factor = 1.0
        mpr.max_acceleration_scaling_factor = 1.0

        js = JointState()
        js.name = JOINT_NAMES
        js.position = list(start)
        rs = RobotState()
        rs.joint_state = js
        rs.is_diff = True
        mpr.start_state = rs

        con = Constraints()
        for name, val in zip(JOINT_NAMES, goal):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(val)
            jc.tolerance_above = 1e-4
            jc.tolerance_below = 1e-4
            jc.weight = 1.0
            con.joint_constraints.append(jc)
        mpr.goal_constraints = [con]

        req.motion_plan_request = mpr

        t_req = time.time()  # 墙钟：与 move_group 日志时间戳对齐用
        fut = self.plan_cli.call_async(req)
        # 服务调用的等待上限要宽于 planner 自己的超时
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout + 30.0)
        res = fut.result()
        if res is None:
            return {"success": False, "planning_time": timeout, "path_length": 0.0,
                    "num_waypoints": 0, "error_code": "NO_RESPONSE",
                    "t_request": round(t_req, 3)}

        r = res.motion_plan_response
        ok = r.error_code.val == 1  # SUCCESS
        return {
            "success": ok,
            "planning_time": round(r.planning_time, 4),
            "path_length": round(path_length(r.trajectory), 4) if ok else 0.0,
            "num_waypoints": len(r.trajectory.joint_trajectory.points) if ok else 0,
            "error_code": r.error_code.val,
            "t_request": round(t_req, 3),
        }


def load_queries(path):
    with open(path) as f:
        data = json.load(f)
    qmap = {q["id"]: q for q in data["queries"]}
    return data, qmap


# ---------------------------------------------------------------------------
# 配置指纹 —— 断点续跑的安全锁
#
# 上一轮就栽在这里：改了机器人模型（摘夹爪）和 OMPL 配置之后直接续跑，
# done 集合只按 (gap, query_id, planner, rep) 去重，完全不知道配置变了，
# 结果一个 CSV 里混着两批不可比的数据。
# 现在把配置摘要写进 sidecar，不一致就拒绝续跑。
# ---------------------------------------------------------------------------


def fingerprint(has_gripper, planners, timeout, questions):
    """questions: [(start, goal), ...]，把实际跑的题目也纳入指纹。"""
    h = hashlib.sha1(json.dumps(questions, sort_keys=True).encode()).hexdigest()[:12]
    return {
        "has_gripper": has_gripper,
        "planners": sorted(planners),
        "timeout": timeout,
        "questions_sha1": h,
    }


def guard_fingerprint(meta_path, fp, out_path):
    """比对 sidecar；不存在就写入，不一致就报错退出。"""
    if not os.path.exists(meta_path):
        if os.path.exists(out_path):
            # 有结果却没指纹 = 上一轮（无保护）留下的，来源不明，不能续
            raise SystemExit(
                f"[中止] {out_path} 存在但没有配套的 {meta_path}。\n"
                f"       这是加指纹之前跑的数据，无法确认它用的什么配置。\n"
                f"       rm {out_path} 重跑，或 --out 换个文件名。"
            )
        with open(meta_path, "w") as f:
            json.dump(fp, f, indent=2)
        return

    with open(meta_path) as f:
        old = json.load(f)
    diff = [k for k in fp if old.get(k) != fp[k]]
    if not diff:
        return

    print("\n[中止] 已有结果的配置和当前不一致，续跑会污染数据:\n")
    for k in diff:
        print(f"  {k}: 已有={old.get(k)!r}  当前={fp[k]!r}")
    print(f"\n处理方式（二选一）:")
    print(f"  rm {out_path} {meta_path}        # 丢弃旧数据重跑")
    print(f"  --out 换个文件名                  # 两组配置各存一份，最后对比")
    raise SystemExit(1)


def prescreen(node, args):
    data, qmap = load_queries(args.queries_file)
    ids = data["canonical_query_ids"][: args.max_queries]
    narrowest = min(data["gaps"])
    grip = node.has_gripper()

    print(f"预筛选: {len(ids)} 个 query @ gap={narrowest:.3f}, "
          f"金标准={GOLDEN}, 超时={args.golden_timeout}s, 重试={args.golden_attempts}")
    print(f"机器人: {'含夹爪' if grip else '无夹爪'}（读自 /robot_description）")
    print("（最窄档有解 => 全部档位都有解）\n")

    node.publish_wall(narrowest, data["meta"]["frame"])

    solvable, t0 = [], time.time()
    codes = {}
    for i, qid in enumerate(ids):
        q = qmap[qid]
        r = node.plan(q["start"], q["goal"], GOLDEN, data["meta"]["group"],
                      args.golden_timeout, args.golden_attempts)
        if r["success"]:
            solvable.append(qid)
        else:
            codes[r["error_code"]] = codes.get(r["error_code"], 0) + 1
        print(f"  [{i + 1:>3}/{len(ids)}] id={qid:<5} "
            f"{'SOLVABLE' if r['success'] else 'UNKNOWN '} "
            f"t={r['planning_time']:>6.2f}s  code={r['error_code']:<4} "
            f"累计有解={len(solvable)}  耗时={time.time() - t0:.0f}s")

    node.clear_wall(data["meta"]["frame"])

    # 失败原因分布 —— 区分 "搜索失败" 和 "请求被拒绝" 的关键
    if codes:
        print("\n失败 error_code 分布:")
        for c, n in sorted(codes.items(), key=lambda kv: -kv[1]):
            print(f"  {c:>5} ({ERROR_NAMES.get(c, '?')}) × {n}")

    # has_gripper 必须记下来：夹爪改变碰撞几何，因而改变哪些 query 有解。
    # 换了夹爪设置还沿用旧的 solvable 集，等于用 A 的题考 B。
    out = {"gap_prescreened": narrowest, "golden": GOLDEN,
           "timeout": args.golden_timeout, "attempts": args.golden_attempts,
           "has_gripper": grip,
           "candidates": ids, "solvable": solvable}
    with open(args.solvable_file, "w") as f:
        json.dump(out, f, indent=2)

    print(f"\n>>> SOLVABLE = {len(solvable)}/{len(ids)} ({len(solvable) / len(ids):.1%})")
    print(f"已写入 {args.solvable_file}")
    if len(solvable) < 25:
        print("    偏少。补充 canonical 里剩余的 query，或把最窄档放宽到 0.18。")


def benchmark(node, args):
    data, qmap = load_queries(args.queries_file)
    with open(args.solvable_file) as f:
        sol = json.load(f)
    solvable = sol["solvable"]

    grip = node.has_gripper()
    if sol.get("has_gripper") != grip:
        raise SystemExit(
            f"[中止] solvable 集是在 {'含' if sol.get('has_gripper') else '无'}夹爪下筛的，"
            f"当前是{'含' if grip else '无'}夹爪。\n"
            f"       夹爪改变碰撞几何，题目的可解性也随之改变 —— 必须重跑 --prescreen。"
        )

    # --planners / --gaps 过滤：地基检查与 Phase D 都只跑子集
    active = ([p.strip() for p in args.planners.split(",") if p.strip()]
              if args.planners else DEFAULT_PLANNERS)
    unknown = [p for p in active if p not in PLANNERS]
    if unknown:
        raise SystemExit(f"[中止] 未知 planner: {unknown}，可选: {sorted(PLANNERS)}")
    if args.gaps:
        want = {float(g) for g in args.gaps.split(",")}
        gaps = [g for g in sorted(data["gaps"], reverse=True) if g in want]
        missing = want - set(gaps)
        if missing:
            raise SystemExit(f"[中止] 未知 gap 档位: {sorted(missing)}，可选: {data['gaps']}")
    else:
        gaps = sorted(data["gaps"], reverse=True)

    total = len(gaps) * len(solvable) * len(active) * args.reps
    print(f"正式实验: {len(gaps)} 档 × {len(solvable)} query × "
          f"{len(active)} planner × {args.reps} 次 = {total} 次规划")
    print(f"planner 位: {', '.join(active)}")
    print(f"机器人: {'含夹爪' if grip else '无夹爪'}   超时 {args.timeout}s，"
          f"最坏 {total * args.timeout / 60:.0f} 分钟\n")

    meta_path = os.path.splitext(args.out)[0] + ".meta.json"
    guard_fingerprint(
        meta_path,
        fingerprint(grip, active, args.timeout,
                    [[qmap[i]["start"], qmap[i]["goal"]] for i in solvable]),
        args.out,
    )

    # 断点续跑：已完成的组合直接跳过（配置一致性由上面的指纹保证）
    done = set()
    if os.path.exists(args.out):
        with open(args.out) as f:
            for row in csv.DictReader(f):
                done.add((row["gap"], row["query_id"], row["planner"], row["rep"]))
        print(f"发现已有结果 {len(done)} 条，跳过\n")

    new = not os.path.exists(args.out)
    f = open(args.out, "a", newline="")
    w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
    if new:
        w.writeheader()

    n, t0 = 0, time.time()
    try:
        # 循环顺序（§9.1① 交错要求）：gap → query → rep → planner。
        # 同一 rep 编号内轮换算法，中和机器状态漂移；done 集合按
        # (gap, query_id, planner, rep) 去重，与顺序无关，续跑安全。
        for gap in gaps:
            node.publish_wall(gap, data["meta"]["frame"])
            gk = f"{gap:.3f}"
            for qid in solvable:
                q = qmap[qid]
                for rep in range(args.reps):
                    for planner in active:
                        n += 1
                        if (gk, str(qid), planner, str(rep)) in done:
                            continue
                        r = node.plan(q["start"], q["goal"], planner,
                                      data["meta"]["group"], args.timeout)
                        w.writerow({"gap": gk, "query_id": qid, "planner": planner,
                                    "rep": rep, **r})
                        f.flush()  # 每次落盘，崩了也不丢
                        if n % 50 == 0:
                            el = time.time() - t0
                            print(f"  {n}/{total}  已用 {el / 60:.1f} 分钟  "
                                  f"预计剩余 {el / n * (total - n) / 60:.1f} 分钟")
    finally:
        f.close()
        node.clear_wall(data["meta"]["frame"])
    print(f"\n完成，结果在 {args.out}")


def main():
    ap = argparse.ArgumentParser(description="S3 benchmark runner")
    ap.add_argument("--prescreen", action="store_true", help="预筛选模式")
    ap.add_argument("--queries-file", default="s3_queries.json")
    ap.add_argument("--solvable-file", default="s3_solvable.json")
    ap.add_argument("--out", default="s3_results.csv")
    ap.add_argument("--max-queries", type=int, default=60, help="预筛选取多少个候选")
    ap.add_argument("--golden-timeout", type=float, default=60.0)
    ap.add_argument("--golden-attempts", type=int, default=3)
    ap.add_argument("--timeout", type=float, default=2.0, help="正式实验超时")
    ap.add_argument("--reps", type=int, default=5, help="每个组合重复次数")
    ap.add_argument("--planners", default=None,
                    help="逗号分隔的 planner label 子集（默认 mine 三项；"
                         "地基检查用 mine/RRTConnect,ompl/RRTConnect）")
    ap.add_argument("--gaps", default=None,
                    help="逗号分隔的 gap 子集（默认全部档位）")
    args = ap.parse_args()

    rclpy.init()
    node = Runner()
    try:
        node.wait_for(node.apply_cli, "/apply_planning_scene")
        node.wait_for(node.plan_cli, "/plan_kinematic_path")
        node.check_limits()  # 采样箱 vs 实际机器人，不一致直接中止
        if args.prescreen:
            prescreen(node, args)
        else:
            benchmark(node, args)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
