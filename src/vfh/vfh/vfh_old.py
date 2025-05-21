#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from turtlebot3_msgs.srv import ObstacleAvoidanceService
import numpy as np
import math


class VFH(Node):
    def __init__(self, vfh_config):
        super().__init__('vfh_obstacle_avoidance')

        self.destination = vfh_config['destination']
        self.a = vfh_config['a']
        self.b = vfh_config['b']
        self.smoothing_factor = vfh_config['smoothing_factor']
        self.sector_size = vfh_config['sector_size']
        self.threshold = vfh_config['threshold']
        self.vmax = 0.2  # Maximum velocity

        self.histogram_field_vector = None

        self.service = self.create_service(
            ObstacleAvoidanceService,
            'vfh_obstacle_avoidance',
            self.obstacle_avoidance_callback
        )

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

    def obstacle_avoidance_callback(self, request, response):
        self.current_x = request.current_x
        self.current_y = request.current_y
        self.current_yaw = request.current_yaw
        self.destination = [request.current_goal_x, request.current_goal_y]

        steering, velocity = self.find_steering_direction()
        response.steering_direction = steering
        response.velocity = velocity
        return response

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.arange(msg.angle_min, msg.angle_max + msg.angle_increment, msg.angle_increment)
        histogram = self.calculate_histogram_field_vector(ranges, angles)
        self.histogram_field_vector = self.smooth_histogram(histogram)

    def calculate_histogram_field_vector(self, ranges, angles):
        num_sectors = int(360 / self.sector_size)
        sector_histogram = np.zeros(num_sectors)

        for i, distance in enumerate(ranges):
            if np.isnan(distance) or np.isinf(distance):
                continue
            angle_normalized = (angles[i] + 2 * np.pi) % (2 * np.pi)
            sector_index = int(np.degrees(angle_normalized) / self.sector_size) % num_sectors
            certainty = 1
            obstacle_presence = certainty ** 2 * (self.a - self.b * distance)
            sector_histogram[sector_index] += obstacle_presence

        return sector_histogram

    def smooth_histogram(self, histogram):
        smoothed_histogram = np.zeros_like(histogram)
        weights = np.concatenate(
            (np.arange(1, self.smoothing_factor + 1),
             np.arange(self.smoothing_factor + 1, 0, -1))
        ) / (2 * self.smoothing_factor + 1)

        n = len(histogram)
        for i in range(n):
            indices = [(i - self.smoothing_factor + j) % n for j in range(len(weights))]
            smoothed_histogram[i] = np.dot(histogram[indices], weights)

        return smoothed_histogram

    def get_goal_sector(self):
        current_position = np.array([self.current_x, self.current_y])
        goal_angle = np.arctan2(
            self.destination[1] - current_position[1],
            self.destination[0] - current_position[0]
        )

        if goal_angle < 0:
            goal_angle += 2 * math.pi

        goal_sector = int(np.degrees(goal_angle) / self.sector_size)
        return goal_sector

    def get_candidate_valleys(self):
        threshold = self.threshold
        valleys = []

        while not valleys:
            valleys_idx = [i for i, h in enumerate(self.histogram_field_vector) if h < threshold]
            candidate_valley = []

            for idx in valleys_idx:
                if candidate_valley and idx == (candidate_valley[-1] + 1) % len(self.histogram_field_vector):
                    candidate_valley.append(idx)
                else:
                    if candidate_valley:
                        valleys.append(candidate_valley)
                    candidate_valley = [idx]

            if candidate_valley:
                valleys.append(candidate_valley)

            if not valleys:
                threshold += 0.5

        return valleys

    def find_velocity(self, steering_sector):
        h_current = self.histogram_field_vector[steering_sector]
        h_max = max(self.histogram_field_vector)
        velocity = self.vmax * (1 - (h_current / h_max))
        k1 = 4
        k2 = 3
        adaptive_threshold = (k1 + k2 * h_current) * (velocity / self.vmax)
        self.threshold = adaptive_threshold
        self.get_logger().info(f"VELOCITY: {velocity} /n THRESH: {adaptive_threshold}")
        return velocity

    def find_steering_direction(self):
        if self.histogram_field_vector is None:
            return 0.0, 0.0

        goal_sector = self.get_goal_sector()
        valleys = self.get_candidate_valleys()

        nearest_valley = min(
            valleys,
            key=lambda valley: min(abs(goal_sector - sector) for sector in valley)
        )

        steering_sector = nearest_valley[len(nearest_valley) // 2]
        velocity = self.find_velocity(steering_sector)

        steering_angle = math.radians(steering_sector * self.sector_size)
        return steering_angle, velocity


def main(args=None):
    rclpy.init(args=args)
    config = {
        'destination': [13.0, 7.0],
        'a': 1,
        'b': 0.25,
        'smoothing_factor': 2,
        'sector_size': 5,
        'threshold': 6
    }
    vfh_node = VFH(vfh_config=config)
    try:
        rclpy.spin(vfh_node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
