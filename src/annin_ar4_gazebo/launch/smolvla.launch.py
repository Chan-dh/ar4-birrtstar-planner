"""SmolVLA launch file — starts the full AR4 simulation + inference pipeline.

What it launches
----------------
1.  Gazebo Harmonic tabletop simulation  (via gazebo.launch.py)
2.  MoveIt 2 + MoveIt Servo              (via moveit.launch.py, servo enabled)
3.  SmolVLA inference script             (ar4_policy_server/ros_bridge/inference_node.py)

Usage
-----
    ros2 launch annin_ar4_gazebo smolvla.launch.py \\
        task:="pick the teal cube and place it on the bottom shelf" \\
        checkpoint:=lerobot/smolvla_base \\
        device:=cuda \\
        ar_model:=mk5

Arguments
---------
ar_model    : Robot model (mk1–mk5).       Default: mk5
task        : Natural language goal string. Default: pick the teal cube and
              place it on the bottom shelf
checkpoint  : SmolVLA checkpoint: HuggingFace repo id or local path.
              Default: lerobot/smolvla_base
device      : Inference device (cuda/cpu/mps). Default: cuda

Notes
-----
* MoveIt Servo is enabled (moveit_servo:=True) because AnninAR4 uses
  CARTESIAN_VELOCITY control which requires Servo.
* use_sim_time:=True is passed to MoveIt so it follows the Gazebo clock.
* The SmolVLA process starts after a 15-second delay so that MoveIt Servo
  has time to fully initialise before the first action is sent.
* Gravity is disabled in tabletop.world so objects hold position until
  the gripper acts on them.
* smolvla_base will run but produce untrained actions. Fine-tune on AR4
  pick-and-place demos recorded with lerobot-ros first.
"""

import sys
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ar_model_arg = DeclareLaunchArgument(
        "ar_model",
        default_value="mk5",
        choices=["mk1", "mk2", "mk3", "mk4", "mk5"],
        description="AR4 robot model version",
    )
    task_arg = DeclareLaunchArgument(
        "task",
        default_value="pick the teal cube and place it on the bottom shelf",
        description="Natural language task instruction for SmolVLA",
    )
    checkpoint_arg = DeclareLaunchArgument(
        "checkpoint",
        default_value="lerobot/smolvla_base",
        description="SmolVLA checkpoint: HuggingFace repo id or local path",
    )
    device_arg = DeclareLaunchArgument(
        "device",
        default_value="cpu",
        description="Inference device: cuda, cpu, or mps",
    )

    ar_model = LaunchConfiguration("ar_model")
    task = LaunchConfiguration("task")
    checkpoint = LaunchConfiguration("checkpoint")
    device = LaunchConfiguration("device")

    # 1. Gazebo simulation (wrist camera bridge included)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("annin_ar4_gazebo"), "launch", "gazebo.launch.py"]
            )
        ),
        launch_arguments={"ar_model": ar_model}.items(),
    )

    # 2. MoveIt 2 with Servo enabled and sim time on
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("annin_ar4_moveit_config"),
                    "launch",
                    "moveit.launch.py",
                ]
            )
        ),
        launch_arguments={
            "ar_model": ar_model,
            "use_sim_time": "True",
            "moveit_servo": "True",
        }.items(),
    )

    # 3. SmolVLA inference — run as a plain Python module using the workspace
    #    .venv (which has rclpy, lerobot 0.5.1+smolvla, cv_bridge).
    #    PYTHONPATH is extended so ar4_policy_server and lerobot_robot_ros
    #    (AnninAR4) are importable without being installed as ROS2 packages.
    workspace_root = os.path.normpath(
        os.path.join(
            get_package_share_directory("annin_ar4_gazebo"),
            "..", "..", "..", "..",  # share/annin_ar4_gazebo → workspace root
        )
    )
    venv_python = os.path.join(workspace_root, ".venv", "bin", "python")
    ar4_src = os.path.join(workspace_root, "ar4-physical-ai")
    lerobot_ros_src = os.path.join(workspace_root, "lerobot-ros", "lerobot_robot_ros")

    extra_pythonpath = ":".join([
        ar4_src,
        lerobot_ros_src,
        os.environ.get("PYTHONPATH", ""),
    ])

    smolvla_process = ExecuteProcess(
        cmd=[
            venv_python, "-m", "ar4_policy_server.ros_bridge.inference_node",
            "--task", task,
            "--checkpoint", checkpoint,
            "--device", device,
        ],
        additional_env={"PYTHONPATH": extra_pythonpath},
        output="screen",
    )

    # Move to a non-singular home position before inference starts.
    # joint_5 = 0 is a wrist singularity for the AR4; Servo emergency-stops on
    # every command when the robot is near it.  We use joint trajectory control
    # (bypassing Servo) to reach a safe configuration first.
    home_joints = (
        '{joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], '
        'points: [{positions: [0.0, -0.3, 0.5, 0.0, 0.5, 0.0], '
        'time_from_start: {sec: 4, nanosec: 0}}]}'
    )
    home_process = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '/joint_trajectory_controller/joint_trajectory',
            'trajectory_msgs/msg/JointTrajectory',
            home_joints,
        ],
        output='screen',
    )
    # t=15s: send home trajectory (15s gives joint_trajectory_controller time to
    #        fully initialise in Gazebo before the topic message arrives).
    # After home_process exits (~15s + epsilon): wait 8 more seconds for the
    #        4-second motion to complete and the robot to settle, then start
    #        SmolVLA.  This is event-driven so it is robust to slow startups.
    home_delayed = TimerAction(period=15.0, actions=[home_process])
    smolvla_after_home = RegisterEventHandler(
        OnProcessExit(
            target_action=home_process,
            on_exit=[TimerAction(period=8.0, actions=[smolvla_process])],
        )
    )

    return LaunchDescription(
        [
            ar_model_arg,
            task_arg,
            checkpoint_arg,
            device_arg,
            gazebo,
            moveit,
            home_delayed,
            smolvla_after_home,
        ]
    )
