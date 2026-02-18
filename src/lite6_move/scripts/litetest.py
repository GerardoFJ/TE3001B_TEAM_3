#!/usr/bin/env python3
import rclpy
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped

def main(args=None):
    rclpy.init(args=args)

    # Create MoveItPy instance
    moveit = MoveItPy(node_name="moveit_py_node")
    manipulator = moveit.get_planning_component("manipulator")

    # Define a sequence of (x, y, z) coordinates for writing letters
    # Example: coordinates for writing 'L' (customize as needed)
    coordinates = [
        (-0.20, -0.14, 0.36),  # Start
        (-0.20, -0.18, 0.36),  # Down
        (-0.16, -0.18, 0.36),  # Right
        (-0.16, -0.14, 0.36),  # Up
    ]

    for idx, (x, y, z) in enumerate(coordinates):
        target_pose = PoseStamped()
        target_pose.header.frame_id = "link_base"
        target_pose.pose.orientation.w = 1.0
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z

        manipulator.set_goal_state(pose_stamped_msg=target_pose, pose_link="tool0")
        plan_result = manipulator.plan()

        if plan_result:
            moveit.get_logger().info(f"Step {idx+1}: Planning successful! Executing...")
            moveit.execute(plan_result.trajectory, controllers=[])
        else:
            moveit.get_logger().error(f"Step {idx+1}: Planning failed!")

    rclpy.shutdown()

if __name__ == '__main__':
    main()