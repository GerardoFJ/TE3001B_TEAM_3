#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory

from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from geometry_msgs.msg import Pose

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import csv
import os
class MarkerWriter(Node):
    def __init__(self):
        super().__init__('marker_writer')
        draw_path = os.path.join(
            get_package_share_directory('lite6_move'),
            'data',
            'team.csv'
        )
        self.draw_coords = self.declare_parameter('draw_coords', draw_path).value
        
        # Connect to the Cartesian Path Service (IK Calculator)
        self.cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        
        # Connect to the Execution Action (Motor Driver)
        self.execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
         
        self.timer = self.create_timer(1.0, self.draw_line)
        self.timer = self.create_timer(1.0, self.get_tf)
        #Initialize the variables for position and trajectory
        self.started = False
        self.qx, self.qy, self.qz, self.qw = 0.0, 0.0, 0.0, 0.0
        self.y_start = 0.0
        self.z_start = 0.0
        self.x_start = 0.0
        self.tf_buffer = Buffer()
        self.x_array = []
        self.y_array = []
        self.z_array = []
        #Start the TF listener
        self.tf_listener = TransformListener(self.tf_buffer, self)
            
    def update_coords(self):
        #Import coordinates on csv
        with open(self.draw_coords, newline='') as csvfile:
            rows = csv.reader(csvfile)
            next(rows, None) 
            for row in rows:
                #Save coordinates 
                self.x_array.append(float(row[0])+self.x_start)
                self.y_array.append(float(row[1])+self.z_start)
                self.z_array.append(float(row[2]))
                
        
    def get_tf(self):
        if self.y_start != 0.0:
            return
        try:
            # Look up the transform from the base to the end-effector
            t = self.tf_buffer.lookup_transform(
                'link_base',   # Target frame (Parent)
                'link_eef',    # Source frame (Child)
                rclpy.time.Time()
            )
            
            # Extract position and orientation
            pos = t.transform.translation
            rot = t.transform.rotation
            self.x_start = pos.x
            self.y_start = pos.y
            self.z_start = pos.z
            self.qx = rot.x
            self.qy = rot.y
            self.qz = rot.z
            self.qw = rot.w
            # Print cleanly to the terminal
            self.get_logger().info(
                f"\n--- Pose of 'link_eef' relative to 'link_base' ---\n"
                f"Position (X, Y, Z):      [{pos.x:.5f}, {pos.y:.5f}, {pos.z:.5f}]\n"
                f"Quaternion (X, Y, Z, W): [{rot.x:.5f}, {rot.y:.5f}, {rot.z:.5f}, {rot.w:.5f}]\n"
            )
            self.update_coords()

        except TransformException as ex:
            self.get_logger().warning(f'Could not get transform: {ex}')

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
        if self.started or self.y_start == 0.0:
            return
        self.started = True

        print(self.y_array)

        self.get_logger().info("Connecting to MoveIt Cartesian Service...")
        self.cartesian_client.wait_for_service()

        req = GetCartesianPath.Request()
        req.header.frame_id = "link_base"
        req.group_name = "lite6"
        req.max_step = 0.01       # Calculate a point every 1 cm
        req.jump_threshold = 0.0  
        
        req.start_state.is_diff = True 
        req.avoid_collisions = False

        
        for x,z,y in zip(self.x_array, self.y_array,self.z_array):
            waypoint = Pose()
            waypoint.position.x = float(x)
            if y == 0:
                waypoint.position.y = self.y_start - 0.05
            else:
                waypoint.position.y = self.y_start
            waypoint.position.z = float(z)
            waypoint.orientation.x = self.qx
            waypoint.orientation.y = self.qy
            waypoint.orientation.z = self.qz
            waypoint.orientation.w = self.qw
            self.get_logger().info(f"{x} {z} {self.x_start} {self.z_start} ")
            req.waypoints.append(waypoint)

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