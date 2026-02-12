import rclpy
from rclpy.node import Node
from __future__ import annotations
import os
import sys
import time
import mujoco
import mujoco.viewer
from so101_mujoco_utils2 import set_initial_pose, get_positions_dict
from so101_mujoco_pid_utils import move_to_pose_pid, hold_position_pid
from so101_mujoco_utils2 import RealtimeJointPlotter #Dash plotter
from so101_ros_bridge import CommandLogger #LOG SAVER




class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('mujocosim')
        self.model_path = self.declare_parameter('model', '/dev/ttyUSB0').value
        self.plot = self.declare_parameter('plot', True).value
        self.log_csv = self.declare_parameter('log_csv', True).value
        self.headless = self.declare_parameter('headless', False).value
        self.plot_host = self.delcare_parameter('plot_host', '127.0.0.1').value
        self.plot_port = self.declare_parameter('plot_port', 8050).value
        self.time_stdby = self.declare_parameter('time_stdby', 5.0).value

        # self.publisher_ = self.create_publisher(String, 'topic', 10)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0

    def simulation(self):
        m = mujoco.MjModel.from_xml_path(self.model_path)
        d = mujoco.MjData(m)

        # Poses are in DEGREES for revolute joints + gripper 0..100 (your convention) Update to an interface maybe later
        starting_position = {
            "shoulder_pan":  -4.4003158666,
            "shoulder_lift": -92.2462050161,
            "elbow_flex":     89.9543738355,
            "wrist_flex":     55.1185398916,
            "wrist_roll":      0.0,
            "gripper":         0.0,
        }

        desired_zero = {
            "shoulder_pan":  0.0,
            "shoulder_lift": 0.0,
            "elbow_flex":    0.0,
            "wrist_flex":    0.0,
            "wrist_roll":    0.0,
            "gripper":       0.0,
        }

        # Put robot at starting pose (this uses your util conversion and mj_forward)
        set_initial_pose(m, d, starting_position)

        # Optional plotter
        plotter = None
        if self.plot:
            try:
                plotter = RealtimeJointPlotter(max_points=4000)
                plotter.start(host=self.plot_host, port=self.plot_port, update_ms=100)
                self.get_logger().info(f"[plot] Realtime plot: http://{self.plot_host}:{self.plot_port}")
            except Exception as e:
                self.get_logger().error(f"[plot] Disabled (failed to start Dash): {e}")
                plotter = None

        # Optional CSV logger
        logger = None
        if self.log_csv:
            logger = CommandLogger(self.log_csv)
            self.get_logger().info(f"[log] Writing commanded references to: {self.log_csv}")

        def run_sequence(viewer):
            self.get_logger().info("[sim] start q (deg/0..100):", get_positions_dict(m, d))

            # Move to zero, hold, return, hold  #Default movement, maybe custom positions later
            move_to_pose_pid(m, d, viewer, desired_zero, duration=2.0, realtime=True, plotter=plotter, logger=logger)
            hold_position_pid(m, d, viewer, desired_zero, duration=2.0, realtime=True, plotter=plotter, logger=logger)
            move_to_pose_pid(m, d, viewer, starting_position, duration=2.0, realtime=True, plotter=plotter, logger=logger)
            hold_position_pid(m, d, viewer, starting_position, duration=2.0, realtime=True, plotter=plotter, logger=logger)

            self.get_logger().info("[sim] end q (deg/0..100):", get_positions_dict(m, d))

        try:
            if self.headless:
                # Headless loop: no OpenGL viewer; steps still run.
                # (Useful for servers or when GLXBadWindow appears.)
                run_sequence(viewer=None)

            else:
                # Viewer mode
                with mujoco.viewer.launch_passive(m, d) as viewer:
                    # If the user closes the window early, stop cleanly.
                    if not viewer.is_running():
                        self.get_logger().error("[sim] Viewer not running; exiting.")
                        return

                    run_sequence(viewer)

                    # Optional: keep open a bit at the end so you can see final pose
                    t_end = time.time() + 1.0
                    while viewer.is_running() and time.time() < t_end:
                        viewer.sync()
                        time.sleep(m.opt.timestep)

        finally:
            if logger is not None:
                try:
                    logger.close()
                except Exception:
                    pass
    
    
        
def main(args=None):
    rclpy.init(args=args)

    rclpy.shutdown()



if __name__ == '__main__':
    main()


