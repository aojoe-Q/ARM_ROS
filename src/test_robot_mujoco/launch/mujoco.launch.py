"""Run test_robot with MuJoCo, ros2_control and optional MoveIt/UI."""
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def setup(context):
    share = Path(get_package_share_directory('test_robot_mujoco'))
    moveit_share = Path(get_package_share_directory('test_robot_moveit_config'))
    config = (MoveItConfigsBuilder('test_robot', package_name='test_robot_moveit_config')
              .robot_description(file_path='config/test_robot.urdf.xacro',
                                 mappings={'initial_positions_file': str(moveit_share / 'config/initial_positions.yaml')})
              .robot_description_semantic(file_path='config/test_robot.srdf')
              .robot_description_kinematics(file_path='config/kinematics.yaml')
              .joint_limits(file_path='config/joint_limits.yaml')
              .trajectory_execution(file_path='config/moveit_controllers.yaml')
              .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
              .to_moveit_configs())
    model_path = str(share / 'model/test_robot.xml')
    root = ET.fromstring(config.robot_description['robot_description'])
    hardware = root.find('ros2_control/hardware')
    hardware.find('plugin').text = 'test_robot_mujoco/MujocoSystem'
    for joint in root.findall('ros2_control/joint'):
        ET.SubElement(joint, 'state_interface', name='effort')
    ET.SubElement(hardware, 'param', name='model_path').text = model_path
    ET.SubElement(hardware, 'param', name='gravity_compensation').text = (
        LaunchConfiguration('gravity_compensation').perform(context).lower())
    config.robot_description['robot_description'] = ET.tostring(root, encoding='unicode')
    for limits in config.joint_limits['robot_description_planning']['joint_limits'].values():
        for key in ('min_position', 'max_position', 'max_velocity', 'max_acceleration'):
            if key in limits:
                limits[key] = float(limits[key])
    # controller_manager drives physics with measured wall-time periods.
    clock = {'use_sim_time': False}
    params = config.to_dict()
    manager = Node(package='controller_manager', executable='ros2_control_node',
                   parameters=[config.robot_description,
                               str(moveit_share / 'config/ros2_controllers.yaml'), clock],
                   output='screen')
    return [
        RegisterEventHandler(OnProcessExit(target_action=manager, on_exit=[
            EmitEvent(event=Shutdown(reason='MuJoCo controller manager exited'))])),
        manager,
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', 'arm_controller', 'gripper_controller',
                        '--controller-manager', '/controller_manager',
                        '--controller-manager-timeout', '60'], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[config.robot_description, clock], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--frame-id', 'world', '--child-frame-id', 'base_link'],
             parameters=[clock], output='screen'),
        Node(package='moveit_ros_move_group', executable='move_group',
             parameters=[params, clock, {'publish_robot_description_semantic': True}],
             condition=IfCondition(LaunchConfiguration('use_moveit')), output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(moveit_share / 'config/moveit.rviz')],
             parameters=[params, clock],
             condition=IfCondition(LaunchConfiguration('use_rviz')), output='screen'),
        Node(package='test_robot_mujoco', executable='viewer.py',
             parameters=[{'model_path': model_path}, clock],
             condition=IfCondition(LaunchConfiguration('use_viewer')), output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_moveit', default_value='true', description='Start MoveIt'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument('use_viewer', default_value='true', description='Start MuJoCo state viewer'),
        DeclareLaunchArgument('gravity_compensation', default_value='true',
                              choices=['true', 'false'],
                              description='Ideal gravity/Coriolis feed-forward compensation'),
        OpaqueFunction(function=setup),
    ])
