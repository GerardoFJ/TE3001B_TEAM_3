from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument,OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import PythonExpression
from launch import LaunchDescription
import os



def launch_setup(context, *args, **kwargs):
    # publish_urdf = LaunchConfiguration('publish_tf', default='false')
    
    inverseKinematics = Node(
            package='xarm_perturbations',
            executable='ik_reference_generator',
            name='ik_reference_generator',
            output='screen',
            parameters=[{
                'wz': 2.5,
                'lam': 0.015,
                'k_task': 14.0,
                'k_null': 1.5,
                'dwell_sec': 1.5,
                'segment_sec': 2.0,
                'control_rate_hz': 200.0,
                'wp_tolerance': 0.03,
            }]
            )
    controller = Node(
            package='xarm_perturbations',
            executable='joint_space_controller',
            name='joint_space_controller',
            output='screen',
            parameters=[{
                'controller_type': 'ctc',
                'output_topic': '/controller_output',
                'trial_name': 'trial_ctc_pert',
                'estop_threshold': 0.15
            }]
        )
    
    perturbations = Node(
            package='xarm_perturbations',
            executable='perturbation_injector',
            name='perturbation_injector',
            output='screen',
            parameters=[{
                'input_topic': '/controller_output',
                'output_topic': '/servo_server/delta_twist_cmds',
                'pub_reliability': 'reliable',
                'enabled': True,
                'mode': 'gaussian',
                'gauss_std_linear': 0.01,
                'gauss_axis': 'x',
                'debug': True
            }]
        )
    xarm_servo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('xarm_moveit_servo'), 
                             'launch', 'lite6_moveit_servo_realmove.launch.py')
            ]),
            launch_arguments={'robot_ip': '192.168.1.175'}.items(),
        )

    return  [
    inverseKinematics,
    controller,
    perturbations,
    xarm_servo_launch
    ]

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
