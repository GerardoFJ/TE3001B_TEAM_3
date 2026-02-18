#!/usr/bin/env python3
from __future__ import annotations
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

import os
import sys
import time

import mujoco
import mujoco.viewer

from so101_mujoco_utils2 import set_initial_pose, get_positions_dict
from so101_mujoco_pid_utils import move_to_pose_pid, hold_position_pid
from so101_mujoco_utils2 import RealtimeJointPlotter 
from so101_ros_bridge import CommandLogger 


class MujocoSim(Node):

    def __init__(self):
        super().__init__('mujocosim')
        scene_path = os.path.join(
            get_package_share_directory('s0s1_gazebo'),
            'urdf',
            'scene_urdf.xml'
        )
        self.model_path = self.declare_parameter('model', scene_path).value
        self.plot = self.declare_parameter('plot', False).value
        self.log_csv = self.declare_parameter('log_csv', False).value
        self.headless = self.declare_parameter('headless', False).value
        self.plot_host = self.declare_parameter('plot_host', '127.0.0.1').value
        self.plot_port = self.declare_parameter('plot_port', 8050).value
        self.time_stdby = self.declare_parameter('time_stdby', 5.0).value

        self.starting_position = {
            "shoulder_pan":  -4.4003158666,
            "shoulder_lift": -92.2462050161,
            "elbow_flex":     89.9543738355,
            "wrist_flex":     55.1185398916,
            "wrist_roll":      0.0,
            "gripper":         0.0,
        }

        self.desired_zero = {
            "shoulder_pan":  0.0,
            "shoulder_lift": 0.0,
            "elbow_flex":    0.0,
            "wrist_flex":    0.0,
            "wrist_roll":    0.0,
            "gripper":       0.0,
        }

        self.m = mujoco.MjModel.from_xml_path(self.model_path)
        self.d = mujoco.MjData(self.m)
        set_initial_pose(self.m, self.d, self.starting_position)
        self.plotter = None
        self.logger = None
        self.start_extras()
        self.simulation()
        # self.publisher_ = self.create_publisher(String, 'topic', 10)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0

    def start_extras(self):
        print("entre putos")
        if self.plot:
            try:
                self.plotter = RealtimeJointPlotter(max_points=4000)
                self.plotter.start(host=self.plot_host, port=self.plot_port, update_ms=100)
                self.get_logger().info(f"[plot] Realtime plot: http://{self.plot_host}:{self.plot_port}")
            except Exception as e:
                self.get_logger().error(f"[plot] Disabled (failed to start Dash): {e}")
                self.plotter = None
        
        if self.log_csv:
            self.logger = CommandLogger(self.log_csv)
            self.get_logger().info(f"[log] Writing commanded references to: {self.log_csv}")

    def run_sequence(self,viewer):
            self.get_logger().info(f"[sim] start q (deg/0..100): {get_positions_dict(self.m, self.d)}")

            # Move to zero, hold, return, hold  #Default movement, maybe custom positions later
            move_to_pose_pid(self.m, self.d, viewer, self.desired_zero, duration=2.0, realtime=True, plotter=self.plotter, logger=self.logger)
            hold_position_pid(self.m, self.d, viewer, self.desired_zero, duration=2.0, realtime=True, plotter=self.plotter, logger=self.logger)
            move_to_pose_pid(self.m, self.d, viewer, self.starting_position, duration=2.0, realtime=True, plotter=self.plotter, logger=self.logger)
            hold_position_pid(self.m, self.d, viewer, self.starting_position, duration=2.0, realtime=True, plotter=self.plotter, logger=self.logger)

            self.get_logger().info(f"[sim] end q (deg/0..100):{get_positions_dict(self.m, self.d)}")

    def simulation(self):
        try:
            if self.headless:
                # Headless loop: no OpenGL viewer; steps still run.
                # (Useful for servers or when GLXBadWindow appears.)
                self.run_sequence(viewer=None)

            else:
                # Viewer mode
                with mujoco.viewer.launch_passive(self.m, self.d) as viewer:
                    # If the user closes the window early, stop cleanly.
                    if not viewer.is_running():
                        self.get_logger().error("[sim] Viewer not running; exiting.")
                        return

                    self.run_sequence(viewer)
                    # Optional: keep open a bit at the end so you can see final pose
                    t_end = time.time() + 1.0
                    while viewer.is_running() and time.time() < t_end:
                        viewer.sync()
                        time.sleep(self.m.opt.timestep)

        finally:
            if self.logger is not None:
                try:
                    self.logger.close()
                except Exception:
                    pass
    
    
        
def main(args=None):
    rclpy.init(args=args)
    node = MujocoSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()


