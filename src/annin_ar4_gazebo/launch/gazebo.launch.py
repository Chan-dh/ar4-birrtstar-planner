import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitution import Substitution
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


class ControllerConfigSubstitution(Substitution):
    """Substitution that fills out tf_prefix in controllers.yaml."""

    def __init__(self, file_path: Substitution, tf_prefix: Substitution):
        super().__init__()
        self._file_path = file_path
        self._tf_prefix = tf_prefix

    def perform(self, context):
        # Evaluate the file path and namespace substitutions
        file_path_val = self._file_path.perform(context)
        tf_prefix_val = self._tf_prefix.perform(context)

        with open(file_path_val, "r") as f:
            content = f.read()

        content = content.replace('$(var tf_prefix)', tf_prefix_val)

        temp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".yaml")
        temp_file.write(content.encode("utf-8"))
        temp_file.close()
        return temp_file.name


class ResolvePackageURIs(Substitution):
    """Converts package://pkg/... to file:///absolute/path/... so Gazebo can find meshes.

    Gazebo converts package:// to model:// internally but cannot resolve model:// URIs
    without explicit resource path configuration. Converting to file:// at xacro time
    (the same approach the gripper meshes use) avoids this entirely.
    """

    _PACKAGES = ["annin_ar4_description", "annin_ar4_driver"]

    def __init__(self, content: Substitution):
        super().__init__()
        self._content = content

    def perform(self, context):
        urdf = self._content.perform(context)
        for pkg in self._PACKAGES:
            try:
                share = get_package_share_directory(pkg)
                urdf = urdf.replace(f"package://{pkg}", f"file://{share}")
            except Exception:
                pass
        return urdf


def generate_launch_description():
    ar_model_arg = DeclareLaunchArgument("ar_model",
                                         default_value="mk5",
                                         choices=["mk1", "mk2", "mk3", "mk4", "mk5"],
                                         description="Model of AR4")
    ar_model_config = LaunchConfiguration("ar_model")
    tf_prefix_arg = DeclareLaunchArgument("tf_prefix",
                                          default_value="",
                                          description="Prefix for AR4 tf_tree")
    tf_prefix = LaunchConfiguration("tf_prefix")
    world_arg = DeclareLaunchArgument("world",
                                      default_value="tabletop.world",
                                      description="Gazebo world file (relative to annin_ar4_gazebo/worlds/)")
    world_config = LaunchConfiguration("world")

    initial_joint_controllers = ControllerConfigSubstitution(
        PathJoinSubstitution([
            FindPackageShare("annin_ar4_gazebo"), "config", "controllers_gazebo.yaml"
        ]),
        tf_prefix=tf_prefix)

    robot_description_content = ResolvePackageURIs(Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("annin_ar4_description"),
            "urdf",
            "ar_gazebo.urdf.xacro",
        ]),
        " ",
        "ar_model:=",
        ar_model_config,
        " ",
        "tf_prefix:=",
        tf_prefix,
        " ",
        "simulation_controllers:=",
        initial_joint_controllers,
    ]))
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster", "-c", "/controller_manager",
            "--controller-manager-timeout", "60"
        ],
    )

    # There may be other controllers of the joints, but this is the initially-started one
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller", "-c", "/controller_manager",
            "--controller-manager-timeout", "60"
        ],
    )

    gripper_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller", "-c", "/controller_manager",
            "--controller-manager-timeout", "60"
        ],
    )

    # Bridge — maps Gazebo topics to ROS 2.
    # Uses a config file so we can set RELIABLE QoS on camera topics; the
    # default inline-arg mode uses BEST_EFFORT which causes a grey screen in
    # rqt_image_view (subscriber/publisher QoS mismatch).
    bridge_config = PathJoinSubstitution([
        FindPackageShare("annin_ar4_gazebo"), "config", "bridge.yaml"
    ])
    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen')

    def launch_gazebo(context, *args, **kwargs):
        world_name = world_config.perform(context)
        world_path = os.path.join(
            get_package_share_directory('annin_ar4_gazebo'), 'worlds', world_name)
        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("ros_gz_sim"), "/launch", "/gz_sim.launch.py"]),
            launch_arguments={
                'gz_args':
                f'-r -s -v 4 --headless-rendering --physics-engine gz-physics-bullet-featherstone-plugin --render-engine ogre2 {world_path}',
                'on_exit_shutdown': 'True'
            }.items())
        return [gazebo]

    def spawn_robot(context, *args, **kwargs):
        """Write the fixed URDF to a temp file and spawn from file.

        Avoids a race condition where gz create subscribes to /robot_description
        before robot_state_publisher has published, which caused intermittent
        model:// URI resolution failures for collision meshes.
        """
        urdf = robot_description_content.perform(context)
        tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False)
        tmp.write(urdf)
        tmp.close()
        return [
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-name", ar_model_config.perform(context),
                    "-file", tmp.name,
                    "-Y", "1.5708",  # rotate -90° so gripper faces +X toward the objects
                ],
                output="screen",
            )
        ]

    return LaunchDescription([
        ar_model_arg,
        tf_prefix_arg,
        world_arg,
        gazebo_bridge,
        OpaqueFunction(function=launch_gazebo),
        OpaqueFunction(function=spawn_robot),
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        initial_joint_controller_spawner_started,
        gripper_joint_controller_spawner_started,
    ])
