#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from geometry_msgs.msg import Pose

class MarkerWriter(Node):
    def __init__(self):
        super().__init__('marker_writer')
        
        # Connect to the Cartesian Path Service (IK Calculator)
        self.cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        
        # Connect to the Execution Action (Motor Driver)
        self.execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        
        self.timer = self.create_timer(1.0, self.draw_line)
        self.started = False

    def slow_down_trajectory(self, trajectory, speed_scale=0.3):
        """
        Mathematically slows down a calculated trajectory.
        speed_scale=0.1 means it will move at 10% of the originally planned speed.
        """
        for point in trajectory.joint_trajectory.points:
            time_in_seconds = point.time_from_start.sec + (point.time_from_start.nanosec * 1e-9)
            new_time = time_in_seconds / speed_scale
            
            point.time_from_start.sec = int(new_time)
            point.time_from_start.nanosec = int((new_time - int(new_time)) * 1e9)
            
            if point.velocities:
                point.velocities = [v * speed_scale for v in point.velocities]
                
            if point.accelerations:
                point.accelerations = [a * (speed_scale ** 2) for a in point.accelerations]
                
        return trajectory

    def draw_line(self):
        if self.started:
            return
        self.started = True

        self.get_logger().info("Connecting to MoveIt Cartesian Service...")
        self.cartesian_client.wait_for_service()

        req = GetCartesianPath.Request()
        req.header.frame_id = "link_base"
        req.group_name = "lite6"
        req.max_step = 0.01       # Calculate a point every 1 cm
        req.jump_threshold = 0.0  
        
        req.start_state.is_diff = True 
        req.avoid_collisions = False

        # --- YOUR 3D COORDINATE ARRAYS ---
        # Note: All three arrays MUST have the exact same number of items.
        self.x_array = [0.0, 0.05, 0.05, 0.15, 0.15, 0.05]
        self.y_array = [-0.16006, -0.17,   -0.18,   -0.19,   -0.20,   -0.25]
        self.z_array = [0.33887,  0.2, 0.15, 0.2, 0.15, 0.2]
        
        # Safety check to prevent array index crashes
        if not (len(self.x_array) == len(self.y_array) == len(self.z_array)):
            self.get_logger().error("Your X, Y, and Z arrays must be the exact same length!")
            rclpy.shutdown()
            return

        # The horizontal orientation
        qx, qy, qz, qw = 0.0, 1.0, 0.0, 0.0

        # Zip pairs up the 1st items, then 2nd items, then 3rd items...
        for x, y, z in zip(self.x_array, self.y_array, self.z_array):
            waypoint = Pose()
            waypoint.position.x = float(x)
            waypoint.position.y = float(y)
            waypoint.position.z = float(z)
            waypoint.orientation.x = qx
            waypoint.orientation.y = qy
            waypoint.orientation.z = qz
            waypoint.orientation.w = qw
            req.waypoints.append(waypoint)

        self.get_logger().info(f"Calculating straight-line IK for {len(self.y_array)} waypoints...")
        future = self.cartesian_client.call_async(req)
        future.add_done_callback(self.on_path_calculated)

    def on_path_calculated(self, future):
        response = future.result()
        fraction_pct = response.fraction * 100.0
        
        if response.fraction < 0.95:
            self.get_logger().error(f"IK Failed! Only calculated {fraction_pct:.1f}% of the line.")
            rclpy.shutdown()
            return
            
        self.get_logger().info(f"IK Success ({fraction_pct:.1f}%). Slowing down and executing...")
        
        # Slowing down to 10% speed
        slow_trajectory = self.slow_down_trajectory(response.solution, speed_scale=0.1)
        
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = slow_trajectory
        
        self.execute_client.wait_for_server()
        exec_future = self.execute_client.send_goal_async(goal_msg)
        exec_future.add_done_callback(self.on_execution_finished)

    def on_execution_finished(self, future):
        self.get_logger().info("Finished drawing the path!")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MarkerWriter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()