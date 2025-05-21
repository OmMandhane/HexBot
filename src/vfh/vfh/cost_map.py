import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2

class MapVisualizer(Node):
    def __init__(self):
        super().__init__('map_visualizer')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.get_logger().info("Subscribed to /map topic")

    def map_callback(self, msg):
        # Extract map metadata
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution

        # Convert occupancy grid data to a numpy array
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # Normalize data for visualization
        normalized_data = np.where(data == -1, 127,  # Unknown space
                          np.where(data == 0, 255,   # Free space
                                   0))              # Obstacles

        # Display using OpenCV
        cv2.imshow('Map Visualization', normalized_data.astype(np.uint8))
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = MapVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Map Visualizer")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
