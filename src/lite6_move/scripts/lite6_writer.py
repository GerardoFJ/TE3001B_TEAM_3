#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, 
    PlanningOptions, 
    Constraints, 
    PositionConstraint, 
    BoundingVolume
)
from geometry_msgs.msg import PoseStamped, Point
from shape_msgs.msg import SolidPrimitive

class LiteWriter(Node):

    def __init__(self):
        super().__init__('lite6Writer')
        
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.goal_sent = False # Prevent spamming the action server

        # Trigger the movement sequence once after 1 second
        self.timer = self.create_timer(1.0, self.send_movement_goal)

    def create_target_constraint(self, frame_id, link_name, x, y, z, tolerance=0.01):
        """Helper function to hide the verbose bounding volume logic."""
        pos_constraint = PositionConstraint()
        pos_constraint.link_name = link_name
        pos_constraint.header.frame_id = frame_id
        pos_constraint.weight = 1.0

        # Define the tolerance sphere
        tolerance_volume = BoundingVolume()
        primitive = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[tolerance])
        tolerance_volume.primitives.append(primitive)
        
        # Place the sphere at your target X, Y, Z
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position = Point(x=x, y=y, z=z)
        tolerance_volume.primitive_poses.append(pose.pose)
        
        pos_constraint.constraint_region = tolerance_volume
        
        # Package into standard Constraints message
        constraint = Constraints()
        constraint.position_constraints.append(pos_constraint)
        return constraint

    def send_movement_goal(self):
        # Stop the timer from firing again
        if self.goal_sent:
            return
        self.goal_sent = True

        goal_msg = MoveGroup.Goal()
        
        # 1. Setup Motion Plan Request
        req = MotionPlanRequest()
        req.group_name = "lite6"
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.5
        req.max_acceleration_scaling_factor = 0.5
        
        # 2. Add our simplified constraint
        target_constraint = self.create_target_constraint(
            frame_id="link_base", 
            link_name="link_eef", 
            x=-0.20485, 
            y=-0.13699, 
            z=0.35956, 
            tolerance=0.01 # 1cm tolerance
        )
        req.goal_constraints.append(target_constraint)
        goal_msg.request = req

        # 3. Setup Planning Options
        goal_msg.planning_options = PlanningOptions(plan_only=False)

        # 4. Send Goal
        self.get_logger().info('Waiting for /move_action server...')
        self._action_client.wait_for_server()

        self.get_logger().info('Sending goal to move...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by MoveIt.')
            return
        self.get_logger().info('Goal accepted! Planning and moving...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Finished moving. MoveIt Error Code: {result.error_code.val}')
        rclpy.shutdown()
        
def main(args=None):
    rclpy.init(args=args)
    node = LiteWriter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()``