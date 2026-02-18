#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class TfViewer(Node):
    def __init__(self):
        super().__init__('simple_tf_viewer')
        
        # Set up the TF Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Check the transform every 1 second
        self.timer = self.create_timer(1.0, self.print_transform)
        self.get_logger().info("TF Viewer started. Waiting for transforms...")

    def print_transform(self):
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
            
            # Print cleanly to the terminal
            self.get_logger().info(
                f"\n--- Pose of 'link_eef' relative to 'link_base' ---\n"
                f"Position (X, Y, Z):      [{pos.x:.5f}, {pos.y:.5f}, {pos.z:.5f}]\n"
                f"Quaternion (X, Y, Z, W): [{rot.x:.5f}, {rot.y:.5f}, {rot.z:.5f}, {rot.w:.5f}]\n"
            )

        except TransformException as ex:
            self.get_logger().warning(f'Could not get transform: {ex}')

def main(args=None):
    rclpy.init(args=args)
    node = TfViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()