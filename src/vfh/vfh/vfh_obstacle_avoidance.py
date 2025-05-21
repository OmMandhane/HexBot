#!/usr/bin/env python3
from copy import deepcopy
import rclpy
from rclpy.node import Node
from scipy.signal import argrelmin
from sensor_msgs.msg import LaserScan
from vfh_msgs.srv import ObstacleAvoidanceService
import numpy as np
import math
import matplotlib
# matplotlib.use('SVG')
import matplotlib.pyplot as plt


class VFH(Node):
    def __init__(self, vfh_config):
        super().__init__('vfh_obstacle_avoidance')

        # Initial parameters
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.bars = None
        self.ax.set_theta_offset(np.pi / 2)
        self.destination = vfh_config['destination']
        self.a = vfh_config['a']
        self.b = vfh_config['b']
        self.smoothing_factor = vfh_config['smoothing_factor']
        self.sector_size = vfh_config['sector_size']
        self.threshold = vfh_config['threshold']
        self.vmax = 0.35  # Maximum velocity 0.3 prev
        # edited for vfh+ 
        self.robot_radius = vfh_config['robot_radius']
        self.safety_distance = vfh_config['safety_distance']


        self.histogram_field_vector = None
        self.last_scan_time = None

        self.smax = 15  #old val  15 intern edit
        #edit by intern
        # ROS 2 service and subscriber
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
        # Update robot's current position and goal
        self.current_x = request.current_x
        self.current_y = request.current_y
        self.current_yaw = request.current_yaw
        self.destination = [request.current_goal_x, request.current_goal_y]

        # Find steering and velocity
        steering, velocity = self.find_steering_direction()
        response.steering_direction = steering
        response.velocity = velocity
        return response

    def scan_callback(self, msg):
        # Read and process the laser scan data
        ranges = np.array(msg.ranges)
        angles = np.arange(msg.angle_min, msg.angle_max + msg.angle_increment, msg.angle_increment)

        # Update histogram field vector
        histogram = self.calculate_histogram_field_vector(ranges, angles)
        self.histogram_field_vector = self.smooth_histogram(histogram)

        # Track the timestamp of the scan
        # self.last_scan_time = self.get_clock().now()

    def calculate_histogram_field_vector(self, ranges, angles):
        num_sectors = int(360 / self.sector_size)
        sector_histogram = np.zeros(num_sectors) #commented by Namita on 21st Nov,2024
        #added by Namita on 21st Nov,2024
        ########################################################
        #sector_histogram = np.full(num_sectors,100)# filling with large value by default for all the sectors
        ########################################################
        angles_mapped = np.zeros_like(angles)
        for i, distance in enumerate(ranges):
            if angles[i]<0:
                angles_mapped[i] = np.degrees(angles[i]) + 360.0
            else:
                angles_mapped[i] = np.degrees(angles[i])
            sector_index = int((angles_mapped[i] / self.sector_size)) % num_sectors
            if np.isnan(distance) or np.isinf(distance):
                #certainty = 1/(1+np.exp(-(self.a - self.b * 1)))
                #obstacle_presence = certainty ** 2 * (self.a - self.b * 1)
                #sector_histogram[sector_index] += obstacle_presence
                sector_histogram[sector_index] += 0.5
                continue
            
            #sector_index = int((np.degrees(angles[i]) / self.sector_size)) % num_sectors
            #sector_index = int((angles_mapped[i] / self.sector_size)) % num_sectors
            # adjusted distance is for the windowing 
            adjusted_distance = max(distance - self.robot_radius - self.safety_distance, 0.01)
            certainty = 100/(100+np.exp(-(self.a - self.b * adjusted_distance))) 
            #certainty = 100/(100+np.exp(-(self.a - self.b * distance)))  this was our older one
            # TODO: maybe we need to calculate as VFF does
            #obstacle_presence = certainty ** (self.a - self.b * distance) * (self.a - self.b * distance)
            #obstacle_presence = certainty ** 2 * (self.a - self.b * distance)  # TODO: tune a and b as the obstacle presence would not be negative anymore
            # obstacle presence should be sensitive to the magnitude of distance of the obstacle present
            obstacle_presence = certainty ** 2 / distance
            #added by Namita on 21st Nov,2024
            ########################################################
            #if(sector_histogram[sector_index] >= 99):
                #sector_histogram[sector_index] = 0
            ########################################################
            sector_histogram[sector_index] += obstacle_presence
            
            #print(sector_index,sector_histogram[sector_index])
        return sector_histogram


    def smooth_histogram(self, histogram):
        smoothed_histogram = np.zeros_like(histogram)
        
        smoothing_factor_int = int(self.smoothing_factor)  # Cast to integer
        smoothing_window = 2 * smoothing_factor_int + 1
        weights = np.concatenate((np.arange(1, smoothing_factor_int + 1), np.arange(smoothing_factor_int + 1, 0, -1))) / float(smoothing_window)
        n = len(histogram)

        for i in range(n):
            indices = [(i - smoothing_factor_int + j) % n for j in range(smoothing_window)]
            smoothed_histogram[i] = np.dot(histogram[indices], weights)

        return smoothed_histogram
    # def compute_adaptive_threshold(self, h_current):
    #     # Compute an adaptive threshold based on current obstacle density
    #     k1 = 4
    #     k2 = 3
    #     return (k1 + k2 * h_current) * (self.vmax / self.vmax)

    def get_goal_sector(self):
        current_position = np.array([self.current_x, self.current_y])
        goal_angle = np.arctan2(self.destination[1] - current_position[1], self.destination[0] - current_position[0])

        if goal_angle < 0:
            goal_angle += 2 * math.pi

        dif = goal_angle - self.current_yaw
        if dif < 0:
            dif += 2 * math.pi
        
        goal_angle_deg = np.degrees(dif) % 360
        goal_sector = int(goal_angle_deg / self.sector_size)
        
        return goal_sector

    def get_candidate_valleys(self):
        threshold = self.threshold
        while True:
            valleys_idx = []
            for i in range(len(self.histogram_field_vector)):
                if self.histogram_field_vector[i] < threshold:
                    valleys_idx.append(i)
                    
            valleys = []
            candidate_valley = []

            i = 0
            while i < len(valleys_idx):
                while i < len(valleys_idx) and valleys_idx[i] == (valleys_idx[i - 1]) % len(self.histogram_field_vector) + 1:
                    candidate_valley.append(valleys_idx[i])
                    i += 1
                    #changed by Namita from 0 to 3 to avoid isolated peaks
                if len(candidate_valley) > 0:
                    valleys.append(deepcopy(candidate_valley))
                    candidate_valley = []
                    if i >= len(valleys_idx):
                        break
                candidate_valley.append(valleys_idx[i])
                i += 1

            if len(valleys) >= 2:
                val_begin = valleys[0]
                val_end = valleys[-1]
                if (val_end[-1] + 1) % len(self.histogram_field_vector) == val_begin[0]:
                    val_begin = val_end + val_begin
                    valleys[0] = val_begin
                    valleys.pop(-1)

            filtered_valley = []
            values_to_remove = list(range(15, 52))# why remove these??13,58
            for v in valleys:
                #if len(v) <= 3:
                    #continue
                v = [h for h in v if h not in values_to_remove]
                if v:
                    filtered_valley.append(v)
            return filtered_valley
            
            #if len(filtered_valley) == 0:
                #threshold += 0.5
                # self.get_logger().info('threshold set to {}'.format(threshold))
                #continue
            #else:
                #threshold = self.threshold
                #return filtered_valley
    # def find_velocity(self, teta):
    #     steering_sector = int(teta / self.sector_size)
    #     h_current = self.histogram_field_vector[steering_sector]
    #     h_max = max(self.histogram_field_vector)   
    #      #v = self.vmax *0.25* (1.0 - (h_current/h_max))
    #     v = self.vmax (1.0-math.exp(-h_current/h_max ))*0.5
          
    #     k1 = 8
    #     k2 = 4
    #     #adaptive_threshold = (k1 + k2 * h_current) * (v / self.vmax)
    #     #self.threshold = min(max(adaptive_threshold, 6), 30)
   
    #     self.threshold = 6
    #     self.get_logger().info(f"VELOCITY: {v} /n THRESH: {self.threshold}")
    #     return v
    def find_velocity(self, teta):
        steering_sector = int(teta / self.sector_size)
        h_current = self.histogram_field_vector[steering_sector]
        h_max = max(self.histogram_field_vector)   
        v = self.vmax *0.4* (1.0 - (h_current/h_max))
          
        k1 = 8
        k2 = 4
        #adaptive_threshold = (k1 + k2 * h_current) * (v / self.vmax)
        #self.threshold = min(max(adaptive_threshold, 6), 30)
   
        self.threshold = 2
        self.get_logger().info(f"VELOCITY: {v} /n THRESH: {self.threshold}")
        return v
    def current_error(self):
        return math.sqrt((self.destination[0] - self.current_x)*2 + (self.destination[1] - self.current_y)*2)

    def find_steering_direction(self):
        if self.histogram_field_vector is None:
            return [0.0,0.0]
        self.plot_polar_density_histogram() #AJREdit
        # self.get_logger().info('vector-fields: {}'.format(self.histogram_field_vector))
        valleys = self.get_candidate_valleys()
        # self.get_logger().info('valleys: {}'.format(valleys))

        goal_sector = self.get_goal_sector()

        min_distance = np.inf
        nearest_valley = None
        if len(valleys) == 0:
             return [0.0,0.0]
        for candidate_valley in valleys:
            if goal_sector in candidate_valley:
                # choose the middle angle spanned by the candidate valley, added by Namita
                #data = np.array(candidate_valley)
                desired_index = len(candidate_valley)
                if desired_index > self.smax:
                    # It is a wide valley
                    self.get_logger().info('Its a WIDE valley with goal sector in it')
                    #Edit by om: directly setting direction to goal sector
                    # if self.current_error() < 0.4:
                    #     self.get_logger().info('BUFFER ZONE')
                    #     theta = goal_sector * self.sector_size % 360
                    # else:    
                    theta1 = candidate_valley[(len(candidate_valley) // 2)]
                    theta1 = theta1 * self.sector_size % 360
                    theta2 = goal_sector
                    theta2 = theta2 * self.sector_size % 360
                    theta = (theta2 + theta1)/2
                
                    if theta >120 and theta < 220:
                    	theta = theta +180
                    	theta = theta % 360    
                else:
                    # It is a narrow valley
                    self.get_logger().info('Its a NARROW valley with goal sector in it')
                    # theta = goal_sector * self.sector_size % 360
                    theta = candidate_valley[(len(candidate_valley) // 2)]
                    theta = theta * self.sector_size % 360
                #theta = goal_sector
                #theta = candidate_valley[(len(candidate_valley) // 2)]
                #theta = goal_sector
                # self.get_logger().info('candidate valley:{}, GOAL-SECTOR: {}, theta: {}'.format(candidate_valley, goal_sector, theta))
                v = self.find_velocity(theta) # edited  intern
                return [math.radians(theta),v]  # Goal sector is in a candidate valley

            distances = np.abs(np.array(candidate_valley) + int(360 / self.sector_size) - goal_sector)
            min_idx = np.argmin(distances)
            if distances[min_idx] < min_distance:
                min_distance = distances[min_idx]
                nearest_valley = candidate_valley

            distances = np.abs(np.array(candidate_valley) - goal_sector)
            min_idx = np.argmin(distances)
            if distances[min_idx] < min_distance:#M+M
                min_distance = distances[min_idx]
                nearest_valley = candidate_valley
        
        desired_index = len(nearest_valley)
        if desired_index > self.smax:
            # It is a wide valley
            #find the nearest sector to goal sector
            self.get_logger().info('Its a WIDE valley with goal sector not in it')
            mindif1 =np.inf
            mindif2 =np.inf
            dif1 = np.abs(goal_sector - nearest_valley[0])
            dif2 = np.abs(goal_sector - nearest_valley[desired_index-1])
            mindif1 = dif1
            mindif2 = dif2
            dif1 = np.abs(goal_sector + int(360 / self.sector_size) - nearest_valley[0])
            dif2 = np.abs(goal_sector +int(360 / self.sector_size) - nearest_valley[desired_index-1])
            if dif1 < mindif1:
                mindif1 = dif1
            if dif2 < mindif2:
                mindif2 = dif2
            dif1 = np.abs(goal_sector - (int(360 / self.sector_size) + nearest_valley[0]))
            dif2 = np.abs(goal_sector - (int(360 / self.sector_size) + nearest_valley[desired_index-1]))
            if dif1 < mindif1:
                mindif1 = dif1
            if dif2 < mindif2:
                mindif2 = dif2
             
            if mindif1 < mindif2:
                #
                theta1 = nearest_valley[0]
                theta2 = nearest_valley[self.smax-1]
            else:
                #
                theta1 = nearest_valley[desired_index-1]
                theta2 = nearest_valley[desired_index - self.smax]
                
            theta1 = theta1 * self.sector_size
            theta1 = theta1 % 360
            #theta1 = (theta1 + goal_sector)/2
            theta2 = theta2 * self.sector_size
            theta2 = theta2 % 360
            theta = (theta2 + theta1)/2

                   
            if theta >120 and theta < 220:
                theta = theta +180
                theta = theta % 360
         
        else:
            # It is a narrow valley
            self.get_logger().info('Its a NARROW valley with goal sector not in it')
            theta = nearest_valley[(len(nearest_valley) // 2)]
            theta = theta * self.sector_size
            theta = theta % 360
        
        #theta = nearest_valley[len(nearest_valley) // 2]
        #theta = theta * self.sector_size
        #theta = theta % 360
        # self.get_logger().info('Goal Sector not in candidate valley, candidate valley:{}, goal-sector: {}, theta: {}'.format(nearest_valley, goal_sector, theta))

        v = self.find_velocity(theta)# edited  intern
        return [math.radians(theta),v]

    def plot_polar_density_histogram(self):
        angles = np.arange(0, 360, self.sector_size)
        density_values = self.histogram_field_vector

        # Convert angles to radians
        theta = np.radians(angles)

        plt.ion()  # Turn on interactive mode

        if self.bars is None:
            # Initial plot setup
            self.bars = self.ax.bar(theta, density_values, width=np.radians(self.sector_size), align='edge', alpha=0.7, color='blue')
            self.ax.set_title("Polar Bar Chart")
            self.ax.set_ylim([0, np.max(density_values)])
        else:
            # Update the bar heights
            for bar, height in zip(self.bars, density_values):
                bar.set_height(height)
            self.ax.set_ylim([0, np.max(density_values)])

        plt.draw()  # Redraw the updated plot
        plt.pause(0.1)


def main(args=None):
    rclpy.init(args=args)

    # Configuration parameters
    config = {
        'destination': [13.0, 7.0],
        'a': 5, #4
        'b': 1.5, #2
        'smoothing_factor': 0.4,
        'sector_size': 5,
        'threshold': 11,
        'robot_radius': 0.4,
        'safety_distance': 0.1,
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