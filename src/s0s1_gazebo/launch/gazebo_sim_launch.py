"""
gazebo_sim_launch.py — Launch Gazebo Classic with the SO101 robot.

Fix for gazebo_ros2_control --param robot_description parsing error:
  gazebo_ros2_control internally passes robot_description to the controller_manager
  via --param robot_description:=<xml>.  The <?xml version='1.0' ?> processing
  instruction at the start of the string causes rcl/arguments.c to reject the
  argument ("not a valid rosparam argument").

  Fix: strip the XML declaration before using the string anywhere.  URDF parsers
  (urdfdom, robot_state_publisher, gazebo) do not require the declaration.
"""
import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg = get_package_share_directory("s0s1_gazebo")

    # ── Process xacro → URDF string ──────────────────────────────────────
    xacro_path = os.path.join(pkg, "urdf", "so101_gazebo.urdf.xacro")
    robot_desc  = xacro.process_file(xacro_path).toxml()

    # gazebo_ros2_control passes robot_description via:
    #   --param robot_description:=<xml>
    # rcl parses that value as a YAML plain scalar.  Three things break it:
    #   1. <?xml ...?>  — the ? is a YAML mapping-key indicator
    #   2. <!-- ... --> — the ! in <!-- is a YAML tag indicator
    #   3. embedded newlines — YAML plain scalars cannot be multi-line
    # Strip all three before the string is ever used as a param value.
    robot_desc = re.sub(r"<\?xml[^?]*\?>", "", robot_desc)          # XML decl
    robot_desc = re.sub(r"<!--.*?-->", "", robot_desc, flags=re.DOTALL)  # comments
    robot_desc = re.sub(r"\s+", " ", robot_desc).strip()             # newlines

    # ── Write URDF to a plain temp file (for spawn_entity -file) ─────────
    urdf_tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".urdf", delete=False, prefix="so101_"
    )
    urdf_tmp.write(robot_desc)
    urdf_tmp.flush()
    urdf_tmp_path = urdf_tmp.name

    # urdf_tmp already written above; no further patching needed.
    # xacro expands $(find s0s1_gazebo)/config/so101_controllers.yaml to the
    # installed absolute path, so the plugin will find the correct YAML.
    patched_desc = robot_desc

    # ── Gazebo (empty world) ──────────────────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={"world": "", "verbose": "false"}.items(),
    )

    # ── robot_state_publisher ─────────────────────────────────────────────
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": patched_desc}],
    )

    # ── Spawn robot from temp URDF file ───────────────────────────────────
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file",   urdf_tmp_path,
            "-entity", "so101",
            "-x", "0", "-y", "0", "-z", "0.01",
        ],
        output="screen",
    )

    # ── Load controllers via spawner ──────────────────────────────────────
    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
        output="screen",
    )
    spawn_jtc = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["so101_joint_trajectory_controller",
                   "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Chain: spawn_robot → joint_state_broadcaster → trajectory_controller
    after_spawn_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[spawn_jsb],
        )
    )
    after_jsb_jtc = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_jsb,
            on_exit=[spawn_jtc],
        )
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_pub,
        spawn_robot,
        after_spawn_jsb,
        after_jsb_jtc,
    ])
