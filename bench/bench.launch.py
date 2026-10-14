"""
Benchmark 专用 launch —— 基于 annin_ar4_moveit_config/launch/demo.launch.py 修改。
上游包零修改，全部实验配置集中在这里。

与 demo.launch.py 的四处差异:

1. 关掉 OMPL 路径简化 (simplify_solutions: false)
   这是最重要的一条。默认开启时 PathSimplifier 会做 shortcutting，带来两个后果:

   (a) 污染因变量。裸 RRT 的路径通常比 RRTConnect 长 30~50%，简化器把两者
       都捋平了，实测 gap=0.45 时 6.07 vs 6.29 —— 测到的是简化器的能力，
       不是规划器的能力。
   (b) 制造假失败。shortcutting 把路径往障碍边界压（日志里的
       "Solution path may slightly touch on an invalid region"），
       简化时按 longest_valid_segment_fraction 判为合法，
       ValidateSolution 换个分辨率复检就判非法。
       表现为: 失败耗时 ≈ 成功耗时，且从不超时 —— 路径找到了，被下游毙了。

   本实验测的是【搜索本身】，简化必须关掉。

2. response_adapters 去掉 AddTimeOptimalParameterization
   TOTG 算不出时间参数时会写 error_code = FAILURE (99999)，混进"规划失败"
   的统计里。而 benchmark 只关心几何路径，planning_time 和 path_length
   都不依赖时间参数化。
   保留 ValidateSolution —— 它是正确性关卡，不能省。

3. 不启 RViz
   抢 CPU，污染 planning_time。建场景时用 use_rviz:=true 打开看一眼即可。

4. include_gripper 提成 launch 参数（默认 true）
   demo.launch.py 没把这个 xacro 参数传下去，只能用默认值。提出来是为了让
   "有夹爪 / 无夹爪" 成为可切换的实验变量。

   注意: 夹爪【一直】是被碰撞检查覆盖的。规划 group 是 ar_manipulator
   （6 个转动关节），但 JointModelGroup::getUpdatedLinkModels() 包含下游
   连杆，夹爪三个 link 随 link_6 刚性运动，一样参与 FCL 检查。
   摘掉夹爪只让失败数从 1064 降到 928 —— 剩下的和夹爪无关，是上面第 1/2 条。

   夹爪几何（换算到 link_6 系，详见 README）:
       沿工具轴伸出 64.2 mm，最大横截面 72.0 × 45.8 mm

用法:
    ros2 launch ./bench.launch.py                        # 跑分用（含夹爪）
    ros2 launch ./bench.launch.py include_gripper:=false # 无夹爪对照组
    ros2 launch ./bench.launch.py use_rviz:=true         # 建场景/肉眼确认
    ros2 launch ./bench.launch.py use_rviz:=true rviz_config:=$PWD/demo.rviz  # 录演示动画（demo_viz.py）
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, file_name):
    path = os.path.join(get_package_share_directory(package_name), file_name)
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    ar_model = LaunchConfiguration("ar_model")
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_rviz = LaunchConfiguration("use_rviz")
    include_gripper = LaunchConfiguration("include_gripper")
    rviz_config = LaunchConfiguration("rviz_config")

    args = [
        DeclareLaunchArgument("ar_model", default_value="mk4",
                              choices=["mk1", "mk2", "mk3", "mk4", "mk5"]),
        DeclareLaunchArgument("tf_prefix", default_value=""),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("rviz_config", default_value=os.path.join(
            get_package_share_directory("annin_ar4_moveit_config"), "rviz", "moveit.rviz")),
        DeclareLaunchArgument("include_gripper", default_value="true",
                              choices=["true", "false"],
                              description="夹爪是实验变量，默认带上（更贴近真实作业）"),
    ]

    # URDF 和 SRDF 都要传同一个 include_gripper —— 上游包两边都有这个 xacro 开关，
    # 所以切换夹爪【不需要改任何 URDF】。只传一边会导致 SRDF 引用不存在的连杆。
    robot_description = {
        "robot_description": ParameterValue(Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([
                FindPackageShare("annin_ar4_moveit_config"),
                "urdf", "fake_ar.urdf.xacro",
            ]), " ",
            "ar_model:=", ar_model, " ",
            "tf_prefix:=", tf_prefix, " ",
            "include_gripper:=", include_gripper,
        ]), value_type=str)
    }

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([
                FindPackageShare("annin_ar4_moveit_config"),
                "srdf", "ar.srdf.xacro",
            ]), " ",
            "name:=", ar_model, " ",
            "tf_prefix:=", tf_prefix, " ",
            "include_gripper:=", include_gripper,
        ]), value_type=str)
    }

    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            "annin_ar4_moveit_config", os.path.join("config", "kinematics.yaml")
        )
    }

    joint_limits = ParameterFile(
        PathJoinSubstitution([
            FindPackageShare("annin_ar4_moveit_config"),
            "config/joint_limits.yaml",
        ]),
        allow_substs=True,
    )

    # ---- OMPL 配置：读上游 yaml，然后在内存里覆盖三个实验参数 -------------
    # 这样做的好处：上游包保持不动，改参数不用 colcon build，
    # 而且"这次实验到底用了什么设置"在一个地方就能看全。
    ompl_config = load_yaml("annin_ar4_moveit_config", "config/ompl_planning.yaml")

    # (1) 只保留正确性关卡，去掉时间参数化 —— 它是 error_code 99999 的来源
    ompl_config["response_adapters"] = [
        "default_planning_response_adapters/ValidateSolution",
    ]

    # (2) 关掉 shortcutting，让 path_length 反映规划器而非简化器
    # (3) 细化离散运动校验（yaml 里已是 0.005，这里重申，使本文件自解释）
    ompl_config["ar_manipulator"]["simplify_solutions"] = False
    ompl_config["ar_manipulator"]["longest_valid_segment_fraction"] = 0.005

    planning_pipeline_config = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["ompl", "pilz", "mine", "mine_viz"],
        "ompl": ompl_config,
        "pilz": load_yaml("annin_ar4_moveit_config", "config/pilz_planning.yaml"),
        "mine": {
            "planning_plugins": ["ar4_my_planners/MyPlanner"],
            "request_adapters": [
                "default_planning_request_adapters/CheckStartStateBounds",
                "default_planning_request_adapters/CheckStartStateCollision",
                "default_planning_request_adapters/ResolveConstraintFrames",
                "default_planning_request_adapters/ValidateWorkspaceBounds",
            ],
            "response_adapters": [
                "default_planning_response_adapters/ValidateSolution",
            ],
        },
    }

    # mine_viz = mine + 时间参数化 + 结果显示：仅供 RViz 可视化（Plan & Execute）。
    # 跑分一律用 mine —— TOTG 会合并/重采样路点，会污染 path_length/num_waypoints。
    # adapter 顺序照抄官方默认链（moveit_configs_utils/default_configs/ompl_planning.yaml）。
    mine_viz = dict(planning_pipeline_config["mine"])
    mine_viz["response_adapters"] = [
        "default_planning_response_adapters/AddTimeOptimalParameterization",
        "default_planning_response_adapters/ValidateSolution",
        "default_planning_response_adapters/DisplayMotionPath",
    ]
    planning_pipeline_config["mine_viz"] = mine_viz

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            planning_pipeline_config,
            {
                "moveit_manage_controllers": False,
                "trajectory_execution.allowed_execution_duration_scaling": 1.2,
                "trajectory_execution.allowed_goal_duration_margin": 0.5,
                "trajectory_execution.allowed_start_tolerance": 0.01,
            },
            {"moveit_controller_manager":
             "moveit_simple_controller_manager/MoveItSimpleControllerManager"},
            ParameterFile(
                PathJoinSubstitution([
                    FindPackageShare("annin_ar4_moveit_config"),
                    "config/controllers.yaml",
                ]),
                allow_substs=True,
            ),
            {
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
            },
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # mock hardware —— move_group 的 PlanningSceneMonitor 需要 /joint_states
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ParameterFile(
                PathJoinSubstitution([
                    FindPackageShare("annin_ar4_driver"), "config", "controllers.yaml"
                ]),
                allow_substs=True,
            ),
            {"tf_prefix": tf_prefix},
        ],
        output="both",
    )

    spawners = [
        Node(package="controller_manager", executable="spawner",
             arguments=[name, "-c", "/controller_manager"])
        for name in ("joint_state_broadcaster", "joint_trajectory_controller")
    ]

    # 夹爪有独立的 ros2_control 系统，没夹爪时这个 spawner 会失败刷屏
    spawners.append(
        Node(package="controller_manager", executable="spawner",
             condition=IfCondition(include_gripper),
             arguments=["gripper_controller", "-c", "/controller_manager"])
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        condition=IfCondition(use_rviz),
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipeline_config,
        ],
    )

    return LaunchDescription(
        args + [move_group, robot_state_publisher, ros2_control_node, rviz] + spawners
    )
