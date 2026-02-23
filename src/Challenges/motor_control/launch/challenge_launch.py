from launch import LaunchDescription
from launch_ros.actions import Node


def make_group(namespace, kp, ki, kd, amplitude, omega, signal_type,
               sys_gain_K, sys_tau_T):
    """Helper: returns the three nodes for one motor control group."""

    motor_node = Node(
        name='motor_sys',
        namespace=namespace,
        package='motor_control',
        executable='dc_motor',
        emulate_tty=True,
        output='screen',
        parameters=[{
            'sample_time': 0.01,
            'sys_gain_K': sys_gain_K,
            'sys_tau_T': sys_tau_T,
            'initial_conditions': 0.0,
        }]
    )

    ctrl_node = Node(
        name='ctrl',
        namespace=namespace,
        package='motor_control',
        executable='controller',
        emulate_tty=True,
        output='screen',
        parameters=[{
            'gain_Kp': kp,
            'gain_Ki': ki,
            'gain_Kd': kd,
            'sample_time': 0.01,
        }]
    )

    sp_node = Node(
        name='sp_gen',
        namespace=namespace,
        package='motor_control',
        executable='set_point',
        emulate_tty=True,
        output='screen',
        parameters=[{
            'amplitude': amplitude,
            'omega': omega,
            'timer_period': 0.1,
            'signal_type': signal_type,
        }]
    )

    return [motor_node, ctrl_node, sp_node]


def generate_launch_description():

    group1_nodes = make_group(
        namespace='group1',
        kp=0.2, ki=2.0, kd=0.0,
        amplitude=2.0, omega=1.0, signal_type='sine',
        sys_gain_K=2.16, sys_tau_T=0.05,
    )

    group2_nodes = make_group(
        namespace='group2',
        kp=0.3, ki=1.5, kd=0.0,
        amplitude=2.0, omega=0.5, signal_type='square',
        sys_gain_K=2.16, sys_tau_T=0.05,
    )

    group3_nodes = make_group(
        namespace='group3',
        kp=0.5, ki=3.0, kd=0.01,
        amplitude=1.5, omega=2.0, signal_type='sine',
        sys_gain_K=2.16, sys_tau_T=0.05,
    )

    return LaunchDescription(group1_nodes + group2_nodes + group3_nodes)
