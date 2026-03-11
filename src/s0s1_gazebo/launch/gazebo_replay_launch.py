"""
gazebo_replay_launch.py — Launch file for the Gazebo trajectory replay node.

Usage:
    ros2 launch s0s1_gazebo gazebo_replay_launch.py \
        csv_path:=/abs/path/results/PID5/cmd.csv \
        speed_factor:=1.0 \
        loop:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    csv_path_arg = DeclareLaunchArgument(
        "csv_path",
        default_value="",
        description="Absolute path to cmd.csv produced by run_experiments.py",
    )
    speed_arg = DeclareLaunchArgument(
        "speed_factor",
        default_value="1.0",
        description="Playback speed multiplier (1.0 = real-time)",
    )
    loop_arg = DeclareLaunchArgument(
        "loop",
        default_value="false",
        description="Loop the replay continuously",
    )
    topic_arg = DeclareLaunchArgument(
        "topic",
        default_value="/so101_joint_trajectory_controller/joint_trajectory",
        description="JointTrajectory topic to publish on",
    )

    replay_node = Node(
        package="s0s1_gazebo",
        executable="gazebo_replay_node.py",
        name="gazebo_replay_node",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "csv_path":     LaunchConfiguration("csv_path"),
            "speed_factor": LaunchConfiguration("speed_factor"),
            "loop":         LaunchConfiguration("loop"),
            "topic":        LaunchConfiguration("topic"),
        }],
    )

    return LaunchDescription([
        csv_path_arg,
        speed_arg,
        loop_arg,
        topic_arg,
        replay_node,
    ])
