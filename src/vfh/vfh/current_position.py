#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener, Buffer
import math

class RobotPositionGetter(Node):
    def __init__(self):
        super().__init__('position_node')
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)  # Use current node

    def quaternion_to_euler(self, orientation):
        """Converts a quaternion into yaw."""
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def get_robot_position_and_yaw(self):
        try:
            transform = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
            position = (transform.transform.translation.x, transform.transform.translation.y)
            orientation = transform.transform.rotation

            # Convert quaternion to yaw
            yaw = self.quaternion_to_euler(orientation)
            self.get_logger().info(f'x, y = ({position[0]}, {position[1]})   yaw = {yaw}')
        except Exception as e:
            self.get_logger().error(f"Error retrieving transform: {e}")

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self)  # Use spin_once to process callbacks
            self.get_robot_position_and_yaw()


def main(args=None):
    rclpy.init(args=args)
    node = RobotPositionGetter()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
