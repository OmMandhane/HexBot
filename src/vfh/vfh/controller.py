#!/usr/bin/env python3
from vfh_msgs.srv import ObstacleAvoidanceService
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros import TransformListener, Buffer
from rclpy.time import Time
import csv

class RobotPositionGetter:
    def __init__(self,node):
        self.node = node
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self.node)  # Use current node name

    def quaternion_to_euler(self, orientation):
        """Converts a quaternion into yaw."""
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def get_robot_position_and_yaw(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time(),timeout=rclpy.duration.Duration(seconds=1.0))
            position = (transform.transform.translation.x, transform.transform.translation.y)
            orientation = transform.transform.rotation

            # Use your custom function to convert quaternion to yaw
            yaw = self.quaternion_to_euler(orientation)
            print(f'x,y: ({position[0]}, {position[1]})')
            return position, yaw
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(f"Error retrieving transform: {e}")
            return None, None

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.robot_position_getter = RobotPositionGetter(self)

        self.move = Twist()
        self.move.linear.x = 0.1

        self.freeze = Twist()
        self.freeze.linear.x = 0.0
        self.freeze.angular.z = 0.0

        self.current_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0

        self.goals = [(4.3,4.2),(6.2,-0.1),(-4.5,4.0),(-6.9,4.2),(0.0,0.0)] #0,2 not working, moving in circle
        self.get_next_goal()
        self.angular_vel_coef = 0.35

        self.client = self.create_client(ObstacleAvoidanceService, 'vfh_obstacle_avoidance')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the obstacle avoidance service...')

        self.cmd_vel_publisher = self.create_publisher(Twist, '/new_robot_controller/cmd_vel_unstamped', 10)
        # self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        

    # def odom_callback(self, msg):
    #     self.current_x = msg.pose.pose.position.x
    #     self.current_y = msg.pose.pose.position.y

    #     orientation = msg.pose.pose.orientation
    #     self.current_yaw = self.quaternion_to_euler(orientation)

        


    def quaternion_to_euler(self, orientation):
        """Converts a quaternion into yaw."""
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            try:
                current_position, current_yaw = self.robot_position_getter.get_robot_position_and_yaw()
                if current_position:
                    self.current_x, self.current_y = float(current_position[0]), float(current_position[1])
                    self.get_logger().info(f'x,y = ({self.current_x}, {self.current_y})')
                    self.current_yaw = float(current_yaw) if current_yaw is not None else 0.0
                response = self.call_obstacle_avoidance()
                distance_error = math.sqrt((self.current_x - self.current_goal_x) ** 2 +
                                           (self.current_y - self.current_goal_y) ** 2)

                if distance_error < 0.3:
                    with open("reached_points.csv", mode="a", newline="") as file:
                        writer = csv.writer(file)
                        writer.writerow([self.current_x, self.current_y])

                    self.get_next_goal()
                    self.get_logger().info(f'GOAL REACHED Next goal: ({self.current_goal_x}, {self.current_goal_y})')
                    continue

                self.adjust_robot_movement(response.steering_direction, response.velocity)

            except Exception as e:
                self.get_logger().error(f'Error in main loop: {e}')

    def call_obstacle_avoidance(self):
        request = ObstacleAvoidanceService.Request()
        request.current_x = self.current_x
        request.current_y = self.current_y
        request.current_yaw = self.current_yaw
        request.current_goal_x = self.current_goal_x
        request.current_goal_y = self.current_goal_y

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error('Failed to call obstacle avoidance service.')
            return None

    def adjust_robot_movement(self, steering_direction, velocity):
        error = self.calculate_error(steering_direction)
        self.move.angular.z = self.angular_vel_coef * error
        self.move.linear.x = velocity
        self.cmd_vel_publisher.publish(self.move)

    def calculate_error(self, steering_direction):
        #error = streering_direction - self.current_yaw
        error = steering_direction
        if abs(error) > math.pi:
            if error < 0.2: 
                error += 2 * math.pi
            else: 
                error -= 2 * math.pi 
        # rospy.loginfo(f'streering direction: {streering_direction}, current yaw: {self.current_yaw}, error: {error}')
        # if abs(error) >= 2.87 and abs(error) <= 3.4:
        #     error = 0.001
        return error

    def get_next_goal(self):
        if self.goals:
            self.current_goal_x, self.current_goal_y = self.goals.pop(0)
        else:
            self.cmd_vel_publisher.publish(self.freeze)
            self.get_logger().info("No more goals.")

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    try:
        controller.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
