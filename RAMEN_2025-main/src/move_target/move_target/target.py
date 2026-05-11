################################################################
#注意！！！！！！
#このコードは，100回試験用に提供位置をがっつり短く調整したものです．
#133行目くらいのfront_target_dist = 1.7 →　0.2 に変更してます．
#100回試験で使える場所が限られているため，距離を短くしてます．

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import random
from status_manager import (NAME)

class MoveTarget(Node):
    def __init__(self):
        super().__init__('target_stop_node')
        self.name = NAME
        self.target = "none"
        self.target_dict = {
            "none": 0,
            "serve": 1,
            "return": 2,
        }
        self.subscription = self.create_subscription(
            String,
            'move_target',
            self.move_target_callback,
            10
        )
        
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        self.subscription = self.create_subscription(
            String,
            'turning',
            self.turning_callback,
            10
        )

        self.publisher_stop = self.create_publisher(String, '/stop', 10)
        self.publisher_arrive = self.create_publisher(String, 'serve_arrive', 10)
        self.get_logger().info(f'MoveTarget initialized with name: {self.name}')
        
    def ransac(self,scan):
        ranges = np.array(scan.ranges)
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        start_deg = 60
        end_deg = 120
        start_idx = int((math.radians(start_deg) - angle_min) / angle_inc)
        end_idx = int((math.radians(end_deg) - angle_min) / angle_inc)
        points = []
        for i in range(start_idx, end_idx):
            if i < 0 or i >= len(ranges):
                continue
            r = ranges[i]
            if not math.isfinite(r):
                continue
            angle = angle_min + i * angle_inc
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            points.append([x, y])

        if len(points) < 2:
            self.get_logger().info("Not enough points for RANSAC")
            return
        best_inliers = []
        best_model = (0, 0, 0)

        for _ in range(150):
            p1, p2 = random.sample(points, 2)
            a = p2[1] - p1[1]
            b = p1[0] - p2[0]
            c = -a * p1[0] - b * p1[1]
            norm = math.hypot(a, b)
            if norm == 0:
                continue
            inliers = []
            for x, y in points:
                dist = abs(a * x + b * y + c) / norm
                if dist < 0.08:
                    inliers.append((x, y))
            if len(inliers) > len(best_inliers):
                best_inliers = inliers
                best_model = (a, b, c)

        if not best_inliers:
            self.get_logger().info("No line found")
            return
        a, b, c = best_model
        distance = abs(c) / math.hypot(a, b)
        inliers = np.array(best_inliers)
        x = inliers[:, 0]
        y = inliers[:, 1]
        A = np.vstack([x, np.ones(len(x))]).T
        m, _ = np.linalg.lstsq(A, y, rcond=None)[0]
        wall_angle = math.atan(m)

        inliers = np.array(best_inliers)
        x = inliers[:, 0]
        y = inliers[:, 1]
        A = np.vstack([x, np.ones(len(x))]).T
        m, _ = np.linalg.lstsq(A, y, rcond=None)[0]
        wall_angle = math.atan(m)
        
        wall_dir_index = int(wall_angle / angle_inc)
        if 0 <= wall_dir_index :
            distance_along_wall = ranges[wall_dir_index]
            
        elif wall_dir_index < 0 :
            distance_along_wall = ranges[1800 + wall_dir_index]

        else:
            distance_along_wall = float('inf')
            
        return distance_along_wall
        
    def move_target_callback(self, msg):
        if msg.data in self.target_dict:
            self.target = msg.data
            self.get_logger().info(f"Target set to: {self.target}")
        else:
            self.get_logger().info(f"Target {msg.data} doesn't exist")
        
    def scan_callback(self, scan):
        if self.target == "serve":
            front_target_dist = 0.2 #要調整 単位はおそらくメートル(m)
            error_threshold = 0.0 
            front_distance = self.ransac(scan)
            #self.get_logger().info(f'front distance: {front_distance}')
            error = front_distance - front_target_dist
            if error < error_threshold:
                self.publisher_stop.publish(String(data=f'{self.name}'))
                self.publisher_arrive.publish(String(data=f'{self.name}'))
                self.get_logger().info(f'Wall detacted. Stop for serve.')
                self.target = "none"
        return
    
    def turning_callback(self, msg):
        if self.target == "return":
            self.publisher_stop.publish(String(data=f'{self.name}'))
            self.get_logger().info('Returning completed.')
            self.target = "none"
            
                
                

    def destroy_node(self):
        self.get_logger().info('MoveTarget node is being destroyed')
        super().destroy_node()
            
def main(args=None):
    rclpy.init(args=args)
    move_target = MoveTarget()
    rclpy.spin(move_target)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

            