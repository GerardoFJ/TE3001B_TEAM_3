"""
Launch bilateral teleoperation with Isaac Sim as the physics backend.

Isaac Sim replaces ros2_control and the hardware interface. It publishes
joint states and subscribes to joint commands directly. MoveIt Servo still
runs on the ROS2 side to convert Cartesian twist commands into joint
trajectories.

Prerequisites:
  1. Start Isaac Sim first:
     cd /home/danielh/isaacsim/_build/linux-x86_64/release
     ./python.sh <path_to>/dual_lite6_sim.py

  2. Then run this launch file:
     ros2 launch xarm_bilateral_teleop bilateral_teleop_isaac.launch.py

Architecture:
  Isaac Sim (physics + rendering)
    ├── publishes /master/joint_states, /slave/joint_states
    ├── subscribes /master/joint_command, /slave/joint_command
    └── subscribes /master/lite6_traj_controller/joint_trajectory
                   /slave/lite6_traj_controller/joint_trajectory

  This launch (ROS2 side)
    ├── MoveIt Servo (per arm) → converts twist to joint trajectory
    ├── Master admittance controller
    └── Slave impedance tracker

  Note: Force sensing is handled inside Isaac Sim (dual_lite6_sim.py).
  Isaac Sim publishes WrenchStamped to /{arm_ns}/force_sensor/force_estimate
  and Bool to /{arm_ns}/force_sensor/contact_detected directly.
"""
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import load_yaml


def launch_setup(context, *args, **kwargs):
    teleop_pkg = get_package_share_directory('xarm_bilateral_teleop')
    params_file = os.path.join(teleop_pkg, 'config', 'teleop_params.yaml')

    nodes = []

    for arm_ns in ['master', 'slave']:
        # ----------------------------------------------------------------
        # MoveIt config (needed for Servo kinematics)
        # ----------------------------------------------------------------
        moveit_config = MoveItConfigsBuilder(
            context=context,
            controllers_name='fake_controllers',
            robot_ip=LaunchConfiguration('robot_ip', default=''),
            report_type=LaunchConfiguration('report_type', default='normal'),
            baud_checkset=LaunchConfiguration('baud_checkset', default=True),
            default_gripper_baud=LaunchConfiguration(
                'default_gripper_baud', default=2000000),
            dof=LaunchConfiguration('dof', default=6),
            robot_type=LaunchConfiguration('robot_type', default='lite'),
            prefix=LaunchConfiguration('prefix', default=''),
            hw_ns=LaunchConfiguration('hw_ns', default='xarm'),
            limited=LaunchConfiguration('limited', default=True),
            effort_control=LaunchConfiguration('effort_control', default=False),
            velocity_control=LaunchConfiguration(
                'velocity_control', default=False),
            model1300=LaunchConfiguration('model1300', default=False),
            robot_sn=LaunchConfiguration('robot_sn', default=''),
            attach_to=LaunchConfiguration('attach_to', default='world'),
            attach_xyz=LaunchConfiguration('attach_xyz', default='"0 0 0"'),
            attach_rpy=LaunchConfiguration('attach_rpy', default='"0 0 0"'),
            mesh_suffix=LaunchConfiguration('mesh_suffix', default='stl'),
            kinematics_suffix=LaunchConfiguration(
                'kinematics_suffix', default=''),
            ros2_control_plugin=LaunchConfiguration(
                'ros2_control_plugin',
                default='uf_robot_hardware/UFRobotFakeSystemHardware'),
            ros2_control_params='',
            add_gripper=LaunchConfiguration('add_gripper', default=False),
            add_vacuum_gripper=LaunchConfiguration(
                'add_vacuum_gripper', default=False),
            add_bio_gripper=LaunchConfiguration(
                'add_bio_gripper', default=False),
            add_realsense_d435i=LaunchConfiguration(
                'add_realsense_d435i', default=False),
            add_d435i_links=LaunchConfiguration(
                'add_d435i_links', default=True),
            add_other_geometry=LaunchConfiguration(
                'add_other_geometry', default=False),
            geometry_type=LaunchConfiguration('geometry_type', default='box'),
            geometry_mass=LaunchConfiguration('geometry_mass', default=0.1),
            geometry_height=LaunchConfiguration(
                'geometry_height', default=0.1),
            geometry_radius=LaunchConfiguration(
                'geometry_radius', default=0.1),
            geometry_length=LaunchConfiguration(
                'geometry_length', default=0.1),
            geometry_width=LaunchConfiguration('geometry_width', default=0.1),
            geometry_mesh_filename=LaunchConfiguration(
                'geometry_mesh_filename', default=''),
            geometry_mesh_origin_xyz=LaunchConfiguration(
                'geometry_mesh_origin_xyz', default='"0 0 0"'),
            geometry_mesh_origin_rpy=LaunchConfiguration(
                'geometry_mesh_origin_rpy', default='"0 0 0"'),
            geometry_mesh_tcp_xyz=LaunchConfiguration(
                'geometry_mesh_tcp_xyz', default='"0 0 0"'),
            geometry_mesh_tcp_rpy=LaunchConfiguration(
                'geometry_mesh_tcp_rpy', default='"0 0 0"'),
        ).to_moveit_configs()

        robot_description_parameters = {}
        robot_description_parameters.update(moveit_config.robot_description)
        robot_description_parameters.update(
            moveit_config.robot_description_semantic)
        robot_description_parameters.update(
            moveit_config.robot_description_kinematics)
        robot_description_parameters.update(moveit_config.joint_limits)
        robot_description_parameters.update(moveit_config.planning_pipelines)

        # -- Servo config --
        servo_yaml = load_yaml(
            'xarm_moveit_servo', 'config/xarm_moveit_servo_config.yaml')
        servo_yaml['move_group_name'] = 'lite6'
        servo_yaml['joint_topic'] = f'/{arm_ns}/joint_states'
        servo_yaml['command_out_topic'] = \
            f'/{arm_ns}/lite6_traj_controller/joint_trajectory'
        servo_yaml['lower_singularity_threshold'] = 100.0
        servo_yaml['hard_stop_singularity_threshold'] = 200.0
        servo_yaml['check_collisions'] = False
        servo_params = {'moveit_servo': servo_yaml}

        # TF remappings per arm
        tf_remappings = [
            ('/tf', f'/{arm_ns}/tf'),
            ('/tf_static', f'/{arm_ns}/tf_static'),
        ]

        # ----------------------------------------------------------------
        # Servo container (robot_state_publisher + MoveIt Servo)
        # No ros2_control needed — Isaac Sim is the hardware backend
        # ----------------------------------------------------------------
        container = ComposableNodeContainer(
            name=f'{arm_ns}_servo_container',
            namespace='/',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='robot_state_publisher',
                    plugin='robot_state_publisher::RobotStatePublisher',
                    name='robot_state_publisher',
                    namespace=arm_ns,
                    parameters=[robot_description_parameters],
                    remappings=tf_remappings,
                ),
                ComposableNode(
                    package='tf2_ros',
                    plugin='tf2_ros::StaticTransformBroadcasterNode',
                    name='static_tf2_broadcaster',
                    namespace=arm_ns,
                    parameters=[{
                        'child_frame_id': 'link_base',
                        'frame_id': 'world',
                    }],
                    remappings=tf_remappings,
                ),
                ComposableNode(
                    package='moveit_servo',
                    plugin='moveit_servo::ServoNode',
                    name='servo_server',
                    namespace=arm_ns,
                    parameters=[
                        servo_params,
                        robot_description_parameters,
                    ],
                    remappings=tf_remappings,
                ),
            ],
            output='screen',
        )

        # Start servo service after container is up
        start_servo = ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call',
                f'/{arm_ns}/servo_server/start_servo',
                'std_srvs/srv/Trigger', '{}',
            ],
            output='screen',
        )

        # Delay servo start to let container initialize
        nodes.append(container)
        nodes.append(RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=container,
                on_start=[TimerAction(period=5.0, actions=[start_servo])],
            )
        ))

    # ----------------------------------------------------------------
    # JointTrajectory → JointState bridges
    # (MoveIt Servo publishes JointTrajectory, Isaac Sim expects JointState)
    # ----------------------------------------------------------------
    for ns in ['master', 'slave']:
        nodes.append(Node(
            package='xarm_bilateral_teleop',
            executable='traj_to_js',
            name=f'{ns}_traj_bridge',
            output='screen',
            parameters=[{'arm_ns': ns}],
        ))

    # ----------------------------------------------------------------
    # Teleop controller nodes
    # (Force sensors are now inside Isaac Sim — no ROS nodes needed)
    # ----------------------------------------------------------------

    # Master admittance controller
    nodes.append(Node(
        package='xarm_bilateral_teleop',
        executable='master_admittance',
        name='master_admittance',
        output='screen',
        parameters=[params_file],
    ))

    # Slave impedance tracker
    nodes.append(Node(
        package='xarm_bilateral_teleop',
        executable='slave_impedance',
        name='slave_impedance',
        output='screen',
        parameters=[params_file],
    ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
