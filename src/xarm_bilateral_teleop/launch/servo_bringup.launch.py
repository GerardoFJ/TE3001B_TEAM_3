"""Namespace-aware MoveIt Servo bringup for bilateral teleoperation.

Replaces direct inclusion of xarm_moveit_servo launch files, which hardcode
the ComposableNodeContainer namespace to '/' and joint_topic to '/joint_states'.
This version creates properly namespaced containers so two arms can run
simultaneously without topic collisions.

Usage (from another launch file):
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(servo_bringup.launch.py),
        launch_arguments={'arm_ns': 'master', 'use_fake': 'true'}.items(),
    )
"""
import os
import yaml

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import (
    generate_ros2_control_params_temp_file,
    load_yaml,
)


def launch_setup(context, *args, **kwargs):
    arm_ns = LaunchConfiguration('arm_ns').perform(context)
    use_fake = LaunchConfiguration('use_fake').perform(context)
    robot_ip = LaunchConfiguration('robot_ip').perform(context)
    launch_rviz = LaunchConfiguration('launch_rviz').perform(context)

    # -- Fixed parameters for Lite6 --
    dof = LaunchConfiguration('dof', default=6)
    robot_type = LaunchConfiguration('robot_type', default='lite')
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=True)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    model1300 = LaunchConfiguration('model1300', default=False)
    robot_sn = LaunchConfiguration('robot_sn', default='')
    attach_to = LaunchConfiguration('attach_to', default='world')
    attach_xyz = LaunchConfiguration('attach_xyz', default='"0 0 0"')
    attach_rpy = LaunchConfiguration('attach_rpy', default='"0 0 0"')
    mesh_suffix = LaunchConfiguration('mesh_suffix', default='stl')
    kinematics_suffix = LaunchConfiguration('kinematics_suffix', default='')
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)
    add_d435i_links = LaunchConfiguration('add_d435i_links', default=True)
    add_other_geometry = LaunchConfiguration('add_other_geometry', default=False)
    geometry_type = LaunchConfiguration('geometry_type', default='box')
    geometry_mass = LaunchConfiguration('geometry_mass', default=0.1)
    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')

    is_fake = use_fake.lower() == 'true'

    if is_fake:
        ros2_control_plugin = LaunchConfiguration(
            'ros2_control_plugin',
            default='uf_robot_hardware/UFRobotFakeSystemHardware')
        controllers_name = 'fake_controllers'
    else:
        ros2_control_plugin = LaunchConfiguration(
            'ros2_control_plugin',
            default='uf_robot_hardware/UFRobotSystemHardware')
        controllers_name = 'controllers'

    xarm_type = 'lite6'

    # -- ros2_control params (temp file) --
    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(
            get_package_share_directory('xarm_controller'),
            'config', 'lite6_controllers.yaml'),
        prefix='',
        add_gripper=False,
        add_bio_gripper=False,
        ros_namespace=arm_ns,
        robot_type='lite',
    )

    # -- MoveIt config --
    moveit_config = MoveItConfigsBuilder(
        context=context,
        controllers_name=controllers_name,
        robot_ip=LaunchConfiguration('robot_ip', default=''),
        report_type=LaunchConfiguration('report_type', default='normal'),
        baud_checkset=LaunchConfiguration('baud_checkset', default=True),
        default_gripper_baud=LaunchConfiguration('default_gripper_baud', default=2000000),
        dof=dof,
        robot_type=robot_type,
        prefix=prefix,
        hw_ns=hw_ns,
        limited=limited,
        effort_control=effort_control,
        velocity_control=velocity_control,
        model1300=model1300,
        robot_sn=robot_sn,
        attach_to=attach_to,
        attach_xyz=attach_xyz,
        attach_rpy=attach_rpy,
        mesh_suffix=mesh_suffix,
        kinematics_suffix=kinematics_suffix,
        ros2_control_plugin=ros2_control_plugin,
        ros2_control_params=ros2_control_params,
        add_gripper=add_gripper,
        add_vacuum_gripper=add_vacuum_gripper,
        add_bio_gripper=add_bio_gripper,
        add_realsense_d435i=add_realsense_d435i,
        add_d435i_links=add_d435i_links,
        add_other_geometry=add_other_geometry,
        geometry_type=geometry_type,
        geometry_mass=geometry_mass,
        geometry_height=geometry_height,
        geometry_radius=geometry_radius,
        geometry_length=geometry_length,
        geometry_width=geometry_width,
        geometry_mesh_filename=geometry_mesh_filename,
        geometry_mesh_origin_xyz=geometry_mesh_origin_xyz,
        geometry_mesh_origin_rpy=geometry_mesh_origin_rpy,
        geometry_mesh_tcp_xyz=geometry_mesh_tcp_xyz,
        geometry_mesh_tcp_rpy=geometry_mesh_tcp_rpy,
    ).to_moveit_configs()

    # Merged parameters for servo, RSP, and RViz
    robot_description_parameters = {}
    robot_description_parameters.update(moveit_config.robot_description)
    robot_description_parameters.update(moveit_config.robot_description_semantic)
    robot_description_parameters.update(moveit_config.robot_description_kinematics)
    robot_description_parameters.update(moveit_config.joint_limits)
    robot_description_parameters.update(moveit_config.planning_pipelines)

    # -- Servo config with CORRECTED topics for this namespace --
    servo_yaml = load_yaml(
        'xarm_moveit_servo', 'config/xarm_moveit_servo_config.yaml')
    servo_yaml['move_group_name'] = xarm_type
    servo_yaml['joint_topic'] = '/{}/joint_states'.format(arm_ns)
    servo_yaml['command_out_topic'] = '/{}/lite6_traj_controller/joint_trajectory'.format(arm_ns)
    # Relax singularity thresholds (Lite6 home position is near-singular)
    servo_yaml['lower_singularity_threshold'] = 100.0
    servo_yaml['hard_stop_singularity_threshold'] = 200.0
    # Disable collision checking (no planning scene in teleop mode)
    servo_yaml['check_collisions'] = False
    servo_params = {'moveit_servo': servo_yaml}

    # ----------------------------------------------------------------
    # 1. ros2_control node (namespaced via GroupAction + PushRosNamespace)
    # ----------------------------------------------------------------
    ros2_control_group = GroupAction([
        PushRosNamespace(arm_ns),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('xarm_controller'),
                'launch', '_ros2_control.launch.py',
            ])),
            launch_arguments={
                'robot_description': yaml.dump(moveit_config.robot_description),
                'ros2_control_params': ros2_control_params,
            }.items(),
        ),
    ])

    # ----------------------------------------------------------------
    # 2. Controller spawners
    #    Use explicit namespace on the Node so PushRosNamespace resolves
    #    the controller_manager correctly, AND the traj_spawner can be
    #    referenced by RegisterEventHandler (needs to be a top-level Node).
    # ----------------------------------------------------------------
    xarm_traj_controller = 'lite6_traj_controller'

    cm_name = '/{}/controller_manager'.format(arm_ns)

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', cm_name,
        ],
    )

    traj_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            xarm_traj_controller,
            '--controller-manager', cm_name,
        ],
    )

    # ----------------------------------------------------------------
    # 3. Composable node container (PROPERLY NAMESPACED)
    #    - Unique container name per arm avoids name collisions
    #    - Each composable node gets explicit namespace
    #    - servo joint_topic already points to /{arm_ns}/joint_states
    # ----------------------------------------------------------------
    # TF remappings: isolate each arm's TF to /{arm_ns}/tf so
    # the two arms don't collide on frame names.
    tf_remappings = [
        ('/tf', '/{}/tf'.format(arm_ns)),
        ('/tf_static', '/{}/tf_static'.format(arm_ns)),
    ]

    container = ComposableNodeContainer(
        name='{}_servo_container'.format(arm_ns),
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

    # ----------------------------------------------------------------
    # Startup sequence (after traj_spawner exits):
    #   1. Wait 1s, then send action goal to move to a non-singular "ready" pose
    #      (ros2 action send_goal is BLOCKING -- waits until trajectory completes)
    #      (Lite6 at q=0 has a rank-deficient 6x6 Jacobian → servo emergency stop)
    #   2. When action goal exits (trajectory done), start the container
    #   3. After container loads (5s), call start_servo service
    # ----------------------------------------------------------------
    initial_trajectory = ExecuteProcess(
        cmd=[
            'ros2', 'action', 'send_goal',
            '/{}/lite6_traj_controller/follow_joint_trajectory'.format(arm_ns),
            'control_msgs/action/FollowJointTrajectory',
            '{"trajectory": {"joint_names": '
            '["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"], '
            '"points": [{"positions": [0.0, 0.0, 0.5, 0.0, 0.5, 0.0], '
            '"time_from_start": {"sec": 4, "nanosec": 0}}]}}'
        ],
        output='screen',
    )

    # After traj_spawner exits: wait 1s for controller to settle, then send
    # the action goal (blocking -- waits for completion)
    traj_and_container = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=traj_spawner,
            on_exit=[
                TimerAction(period=1.0, actions=[initial_trajectory]),
            ],
        )
    )

    # Start container only AFTER the trajectory action completes
    container_after_traj = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=initial_trajectory,
            on_exit=[container],
        )
    )

    start_servo = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/{}/servo_server/start_servo'.format(arm_ns),
            'std_srvs/srv/Trigger', '{}',
        ],
        output='screen',
    )

    # Give the container time to fully load all composable nodes
    start_servo_delayed = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=container,
            on_start=[TimerAction(period=5.0, actions=[start_servo])],
        )
    )

    # ----------------------------------------------------------------
    # 4. Results
    # ----------------------------------------------------------------
    nodes = [
        ros2_control_group,
        jsb_spawner,
        traj_spawner,
        traj_and_container,
        container_after_traj,
        start_servo_delayed,
    ]

    # Optional RViz
    if launch_rviz.lower() == 'true':
        rviz_config_file = PathJoinSubstitution([
            FindPackageShare('xarm_moveit_servo'), 'rviz', 'servo.rviz'])
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=arm_ns,
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[robot_description_parameters],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
        )
        nodes.append(rviz_node)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'arm_ns',
            description='Namespace for this arm (e.g. master or slave)'),
        DeclareLaunchArgument(
            'robot_ip', default_value='',
            description='Robot IP (empty for fake mode)'),
        DeclareLaunchArgument(
            'use_fake', default_value='false',
            description='Use fake hardware instead of real robot'),
        DeclareLaunchArgument(
            'launch_rviz', default_value='true',
            description='Launch RViz for this arm'),
        OpaqueFunction(function=launch_setup),
    ])
