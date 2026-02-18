import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    s0s1_gazebo_dir = get_package_share_directory('s0s1_gazebo')
    
    urdf_file = os.path.join(s0s1_gazebo_dir, 'urdf', 'SO101', 's0s1.urdf')
    assets_dir = os.path.join(s0s1_gazebo_dir, 'urdf', 'SO101', 'assets')
    
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    
    robot_desc = robot_desc.replace(
        'filename="assets/',
        f'filename="{assets_dir}/'
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )
    
    launch_gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', 'empty.sdf'],
        output='screen'
    )
    
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 's0s1'],
        output='screen'
    )
    
    return LaunchDescription([
        launch_gazebo,
        robot_state_publisher,
        spawn_robot,
    ])