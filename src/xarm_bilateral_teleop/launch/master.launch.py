"""Launch master arm: MoveIt Servo + force sensor + admittance controller."""
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
    master_ip = LaunchConfiguration('master_ip').perform(context)
    use_fake = LaunchConfiguration('use_fake').perform(context)

    teleop_pkg = get_package_share_directory('xarm_bilateral_teleop')
    params_file = os.path.join(teleop_pkg, 'config', 'teleop_params.yaml')

    # Namespace-aware servo bringup (replaces xarm_moveit_servo include)
    servo_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(teleop_pkg, 'launch', 'servo_bringup.launch.py')),
        launch_arguments={
            'arm_ns': 'master',
            'robot_ip': master_ip,
            'use_fake': use_fake,
            'launch_rviz': 'true',
        }.items(),
    )

    # Force sensor node
    force_sensor = Node(
        package='xarm_bilateral_teleop',
        executable='force_sensor',
        namespace='master',
        name='force_sensor',
        output='screen',
        parameters=[params_file],
    )

    # Master admittance controller
    master_ctrl = Node(
        package='xarm_bilateral_teleop',
        executable='master_admittance',
        name='master_admittance',
        output='screen',
        parameters=[params_file],
    )

    return [servo_bringup, force_sensor, master_ctrl]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'master_ip', default_value='192.168.1.XXX',
            description='IP address of the master xArm Lite6'),
        DeclareLaunchArgument(
            'use_fake', default_value='false',
            description='Use fake servo (simulation) instead of real robot'),
        OpaqueFunction(function=launch_setup),
    ])
