"""Launch slave arm: MoveIt Servo + force sensor + impedance tracker."""
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    slave_ip = LaunchConfiguration('slave_ip').perform(context)
    use_fake = LaunchConfiguration('use_fake').perform(context)

    teleop_pkg = get_package_share_directory('xarm_bilateral_teleop')
    params_file = os.path.join(teleop_pkg, 'config', 'teleop_params.yaml')

    # Namespace-aware servo bringup (replaces xarm_moveit_servo include)
    servo_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(teleop_pkg, 'launch', 'servo_bringup.launch.py')),
        launch_arguments={
            'arm_ns': 'slave',
            'robot_ip': slave_ip,
            'use_fake': use_fake,
            'launch_rviz': 'true',
        }.items(),
    )

    # Force sensor node
    force_sensor = Node(
        package='xarm_bilateral_teleop',
        executable='force_sensor',
        namespace='slave',
        name='force_sensor',
        output='screen',
        parameters=[params_file],
    )

    # Slave impedance controller
    slave_ctrl = Node(
        package='xarm_bilateral_teleop',
        executable='slave_impedance',
        name='slave_impedance',
        output='screen',
        parameters=[params_file],
    )

    return [servo_bringup, force_sensor, slave_ctrl]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'slave_ip', default_value='192.168.1.YYY',
            description='IP address of the slave xArm Lite6'),
        DeclareLaunchArgument(
            'use_fake', default_value='false',
            description='Use fake servo (simulation) instead of real robot'),
        OpaqueFunction(function=launch_setup),
    ])
