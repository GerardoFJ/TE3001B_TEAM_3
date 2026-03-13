"""
Launch full bilateral teleoperation system (both arms from one computer).

Usage:
  # Simulation (no real robots):
  ros2 launch xarm_bilateral_teleop bilateral_teleop.launch.py use_fake:=true

  # Real robots:
  ros2 launch xarm_bilateral_teleop bilateral_teleop.launch.py \
      master_ip:=192.168.1.175 slave_ip:=192.168.1.226
"""
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    teleop_pkg = get_package_share_directory('xarm_bilateral_teleop')

    return LaunchDescription([
        DeclareLaunchArgument(
            'master_ip', default_value='192.168.1.XXX',
            description='IP address of the master xArm Lite6'),
        DeclareLaunchArgument(
            'slave_ip', default_value='192.168.1.YYY',
            description='IP address of the slave xArm Lite6'),
        DeclareLaunchArgument(
            'use_fake', default_value='false',
            description='Use fake servo (simulation) instead of real robots'),

        # Master arm
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(teleop_pkg, 'launch', 'master.launch.py')),
            launch_arguments={
                'master_ip': LaunchConfiguration('master_ip'),
                'use_fake': LaunchConfiguration('use_fake'),
            }.items(),
        ),

        # Slave arm
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(teleop_pkg, 'launch', 'slave.launch.py')),
            launch_arguments={
                'slave_ip': LaunchConfiguration('slave_ip'),
                'use_fake': LaunchConfiguration('use_fake'),
            }.items(),
        ),
    ])
