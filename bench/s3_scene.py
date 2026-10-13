#!/usr/bin/env python3
"""
S3 窄缝墙场景 —— AR4 运动规划 benchmark 的主力参数化场景。

机器人正前方立一堵墙，墙上开一个宽 w 的方洞，末端必须从洞里钻过去。
墙比臂展宽、比臂展高、落到地面，所以绕不过、翻不过、钻不过 —— 唯一通路是那个洞。
w 是唯一的自变量（难度旋钮），其余几何全部固定。

用法（先确保 move_group 在跑）:
    ros2 launch annin_ar4_moveit_config demo.launch.py

    # 发布一个 gap=0.20m 的场景，并在 RViz 里肉眼确认
    python3 s3_scene.py --gap 0.20

    # 发布 + 自动验证场景是否可用（墙两侧都要有可达的无碰撞构型）
    python3 s3_scene.py --gap 0.20 --verify

    # 粗扫，标定出难度区间的上下界（先跑这个！）
    python3 s3_scene.py --sweep 0.35 0.10 0.05 --verify

    # 清除场景
    python3 s3_scene.py --clear
"""

import argparse
import json
import math
import random
import sys

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetPositionFK, GetStateValidity
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

# ---------------------------------------------------------------------------
# 场景几何（单位: 米）。除 gap 外全部固定。
# ---------------------------------------------------------------------------
WALL_X = 0.28  # 墙面到底座的距离
WALL_THICK = 0.02  # 墙厚
WALL_Y_HALF = 0.60  # 墙左右半宽（> 臂展，绕不过去）
WALL_Z_TOP = 1.00  # 墙高（> 臂展，翻不过去）
HOLE_Z_CENTER = 0.40  # 洞中心高度
HOLE_H = 0.30  # 洞高，固定

OBJ_ID = "s3_wall"

# AR4 MK4 关节限位（单位: 度），取自 annin_ar4_description/config/mk4.yaml。
# 实验平台就是 mk4（bench.launch.py 的 ar_model 默认值）。
#
# 注意这里是硬编码，而 launch 里可以换型号 —— 两者会脱钩。各代限位:
#     mk1/mk2   j1 ±170   j4 ±165   j6 ±155
#     mk3/mk4   j1 ±170   j4 ±180   j6 ±180   ← 本组
#     mk5       j1 ±160   j4 ±180   j6 ±180   ← j1 窄 10°，换过去就不对了
# 所以 Runner 启动时会拿这组值和实际的 /robot_description 核对（见 check_limits）。
JOINT_LIMITS_DEG = [
    (-170.0, 170.0),  # joint_1
    (-42.0, 90.0),  # joint_2
    (-89.0, 52.0),  # joint_3
    (-180.0, 180.0),  # joint_4
    (-105.0, 105.0),  # joint_5
    (-180.0, 180.0),  # joint_6
]
JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]


def make_wall(gap_w, frame_id):
    """把窄缝墙拼成一个 CollisionObject（4 个 box 子图元）。

    不能在 box 上"挖洞"，只能用 4 块板围出中间的空洞：
    左右板跨满整个高度，上下板只跨洞宽，四块互不重叠。
    """
    if gap_w >= 2 * WALL_Y_HALF:
        raise ValueError("gap 比墙还宽，等于没有墙")

    half = gap_w / 2.0
    hole_bottom = HOLE_Z_CENTER - HOLE_H / 2.0
    hole_top = HOLE_Z_CENTER + HOLE_H / 2.0

    # (尺寸 xyz, 中心 xyz)
    boxes = [
        # 左板: y ∈ [-WALL_Y_HALF, -half], z ∈ [0, WALL_Z_TOP]
        (
            (WALL_THICK, WALL_Y_HALF - half, WALL_Z_TOP),
            (WALL_X, -(WALL_Y_HALF + half) / 2.0, WALL_Z_TOP / 2.0),
        ),
        # 右板: 镜像
        (
            (WALL_THICK, WALL_Y_HALF - half, WALL_Z_TOP),
            (WALL_X, (WALL_Y_HALF + half) / 2.0, WALL_Z_TOP / 2.0),
        ),
        # 下板: y ∈ [-half, half], z ∈ [0, hole_bottom]
        (
            (WALL_THICK, gap_w, hole_bottom),
            (WALL_X, 0.0, hole_bottom / 2.0),
        ),
        # 上板: y ∈ [-half, half], z ∈ [hole_top, WALL_Z_TOP]
        (
            (WALL_THICK, gap_w, WALL_Z_TOP - hole_top),
            (WALL_X, 0.0, (hole_top + WALL_Z_TOP) / 2.0),
        ),
    ]

    obj = CollisionObject()
    obj.header.frame_id = frame_id
    obj.id = OBJ_ID
    obj.operation = CollisionObject.ADD

    for size, center in boxes:
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = list(size)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = center
        pose.orientation.w = 1.0
        obj.primitives.append(prim)
        obj.primitive_poses.append(pose)

    return obj


class S3Client(Node):
    def __init__(self):
        super().__init__("s3_scene_builder")
        self.apply_cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.valid_cli = self.create_client(GetStateValidity, "/check_state_validity")
        self.fk_cli = self.create_client(GetPositionFK, "/compute_fk")

    def wait_for(self, cli, name, timeout=10.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f"服务 {name} 不可用 —— move_group 起来了吗？")
            sys.exit(1)

    def call(self, cli, req):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        return fut.result()

    # -- 场景 ---------------------------------------------------------------
    def publish_wall(self, gap_w, frame_id):
        self.wait_for(self.apply_cli, "/apply_planning_scene")

        # 先删后加：同 id 直接 ADD 在部分版本里是追加而不是替换
        self._apply([self._remove_obj(frame_id)])
        self._apply([make_wall(gap_w, frame_id)])

    def clear_wall(self, frame_id):
        self.wait_for(self.apply_cli, "/apply_planning_scene")
        self._apply([self._remove_obj(frame_id)])

    def _remove_obj(self, frame_id):
        obj = CollisionObject()
        obj.header.frame_id = frame_id
        obj.id = OBJ_ID
        obj.operation = CollisionObject.REMOVE
        return obj

    def _apply(self, objects):
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = objects
        req = ApplyPlanningScene.Request()
        req.scene = scene
        self.call(self.apply_cli, req)

    # -- 验证 ---------------------------------------------------------------
    def _robot_state(self, q):
        js = JointState()
        js.name = JOINT_NAMES
        js.position = list(q)
        rs = RobotState()
        rs.joint_state = js
        rs.is_diff = True
        return rs

    def is_valid(self, q, group):
        req = GetStateValidity.Request()
        req.robot_state = self._robot_state(q)
        req.group_name = group
        res = self.call(self.valid_cli, req)
        return bool(res and res.valid)

    def fk_x(self, q, tip, frame_id):
        """返回末端在 frame_id 下的 x 坐标（用来判断在墙的哪一侧）。"""
        req = GetPositionFK.Request()
        req.header.frame_id = frame_id
        req.fk_link_names = [tip]
        req.robot_state = self._robot_state(q)
        res = self.call(self.fk_cli, req)
        if not res or not res.pose_stamped:
            return None
        return res.pose_stamped[0].pose.position.x

    def verify(self, gap_w, group, tip, frame_id, n_samples, seed):
        """场景有效性检查：墙两侧是否都存在可达的无碰撞构型。

        这一步不能省。用代码生成场景极易把墙放在机器人根本够不到的地方，
        结果所有 planner 都 100% 成功，而你以为是算法好。
        """
        self.wait_for(self.valid_cli, "/check_state_validity")
        self.wait_for(self.fk_cli, "/compute_fk")

        rng = random.Random(seed)
        limits = [(math.radians(lo), math.radians(hi)) for lo, hi in JOINT_LIMITS_DEG]

        near, far, colliding = 0, 0, 0
        for _ in range(n_samples):
            q = [rng.uniform(lo, hi) for lo, hi in limits]
            if not self.is_valid(q, group):
                colliding += 1
                continue
            x = self.fk_x(q, tip, frame_id)
            if x is None:
                continue
            if x > WALL_X:
                far += 1
            else:
                near += 1

        free = near + far
        return {
            "gap": round(gap_w, 3),
            "samples": n_samples,
            "free": free,
            "colliding": colliding,
            "near_side": near,
            "far_side": far,
            "far_ratio": round(far / free, 4) if free else 0.0,
        }


def main():
    ap = argparse.ArgumentParser(description="S3 窄缝墙场景")
    ap.add_argument("--gap", type=float, help="缝宽 w (m)")
    ap.add_argument(
        "--sweep",
        nargs=3,
        type=float,
        metavar=("START", "STOP", "STEP"),
        help="从 START 递减到 STOP，步长 STEP，逐档验证",
    )
    ap.add_argument("--clear", action="store_true", help="清除场景")
    ap.add_argument("--verify", action="store_true", help="发布后做有效性检查")
    ap.add_argument("--samples", type=int, default=300, help="验证时的采样数")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--group", default="ar_manipulator")
    ap.add_argument("--tip", default="link_6", help="末端连杆名")
    ap.add_argument("--frame", default="base_link", help="场景参考坐标系")
    args = ap.parse_args()

    rclpy.init()
    node = S3Client()
    try:
        if args.clear:
            node.clear_wall(args.frame)
            print(f"已清除 {OBJ_ID}")
            return

        if args.sweep:
            start, stop, step = args.sweep
            gaps, w = [], start
            while w >= stop - 1e-9:
                gaps.append(round(w, 3))
                w -= step
        elif args.gap is not None:
            gaps = [args.gap]
        else:
            ap.error("需要 --gap / --sweep / --clear 之一")

        rows = []
        for w in gaps:
            node.publish_wall(w, args.frame)
            if not args.verify:
                print(f"已发布窄缝墙, gap = {w:.3f} m")
                continue
            r = node.verify(w, args.group, args.tip, args.frame, args.samples, args.seed)
            rows.append(r)
            print(
                f"gap={r['gap']:.3f}  无碰撞={r['free']:>4}/{r['samples']}  "
                f"近侧={r['near_side']:>4}  远侧={r['far_side']:>4}  "
                f"远侧占比={r['far_ratio']:.1%}"
            )

        if rows:
            with open("s3_verify.json", "w") as f:
                json.dump(rows, f, indent=2)
            print("\n结果已写入 s3_verify.json")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
