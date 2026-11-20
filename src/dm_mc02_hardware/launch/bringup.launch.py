"""Real hardware, RViz and optional MoveIt share one calibrated robot description."""
from pathlib import Path
import runpy

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def setup(context):
    share = Path(get_package_share_directory("dm_mc02_hardware"))
    moveit_share = Path(get_package_share_directory("test_robot_moveit_config"))
    arg = lambda name: LaunchConfiguration(name).perform(context)
    enabled = lambda name: arg(name).lower() in ("true", "1")
    model = runpy.run_path(str(share / "launch/real_model.py"))
    xml = xacro.process_file(str(share / "urdf/test_robot_real.urdf.xacro"), mappings={
        "allow_enable": arg("allow_enable"), "device": arg("device")}).toxml()
    xml, limits, params = model["adapt_description"](xml)
    description = {"robot_description": ParameterValue(xml, value_type=str)}
    nodes = [
        Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[description]),
        Node(package="controller_manager", executable="ros2_control_node", output="screen",
             parameters=[description, str(share / "config/ros2_controllers.yaml")]),
        Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"], output="screen"),
    ]
    visual_params = [description]
    if enabled("use_moveit"):
        if float(params["mapping_calibrated"]) <= 0.5 or float(params["grip_calibrated"]) <= 0.5:
            raise ValueError("MoveIt requires calibrated arm and gripper feedback; use telemetry for calibration first")
        from moveit_configs_utils import MoveItConfigsBuilder
        config = MoveItConfigsBuilder("test_robot", package_name="test_robot_moveit_config").to_moveit_configs()
        config.robot_description = description
        config.joint_limits = {"robot_description_planning": {"joint_limits": limits,
            "default_velocity_scaling_factor": 0.1, "default_acceleration_scaling_factor": 0.1}}
        srdf = (moveit_share / "config/test_robot.srdf").read_text(encoding="utf-8")
        config.robot_description_semantic = {"robot_description_semantic":
            ParameterValue(model["filter_named_states"](srdf, xml), value_type=str)}
        if float(params["grip_command_enable"]) <= 0.5:
            controllers = config.trajectory_execution["moveit_simple_controller_manager"]
            controllers["controller_names"] = ["arm_controller"]
            controllers.pop("gripper_controller", None)
        nodes.append(Node(package="moveit_ros_move_group", executable="move_group", output="screen",
                          parameters=[config.to_dict()]))
        visual_params = [config.to_dict()]
        nodes.append(Node(package="tf2_ros", executable="static_transform_publisher",
                          arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"]))
    if enabled("use_rviz"):
        nodes.append(Node(package="rviz2", executable="rviz2", output="screen",
                          arguments=["-d", str(moveit_share / "config/moveit.rviz")], parameters=visual_params))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("allow_enable", default_value="0"),
        DeclareLaunchArgument("device", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_moveit", default_value="false"),
        OpaqueFunction(function=setup),
    ])
