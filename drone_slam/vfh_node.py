#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np

class VFHNode(Node):
    def __init__(self):
        super().__init__('vfh_node')
        
        # 파라미터
        self.vfh_threshold = 1.5  # 회피 거리 (m)
        self.sector_count = 36
        self.alpha = 2.0
        
        # Subscription
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.rl_goal_sub = self.create_subscription(PoseStamped, '/goal', self.rl_goal_callback, 10)
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/vfh_cmd_vel', 10)
        
        # State
        self.latest_scan = None
        self.rl_goal = None
        self.vfh_active = False
        
        self.get_logger().info("VFH Node - Compatible with offboard_mission (mapless)")

    def scan_callback(self, msg):
        self.latest_scan = msg
        if self.rl_goal is not None:
            self.compute_vfh()

    def rl_goal_callback(self, msg):
        # RL에서 odom 기준 목표 수신
        self.rl_goal = msg
        self.get_logger().info(f"RL Goal: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")

    def compute_vfh(self):
        if self.latest_scan is None:
            return

        ranges = np.array(self.latest_scan.ranges)
        ranges = np.nan_to_num(ranges, nan=self.latest_scan.range_max, posinf=self.latest_scan.range_max)
        
        min_dist = np.min(ranges)
        
        # 장애물 임계값 체크
        if min_dist < self.vfh_threshold:
            self.vfh_active = True
            self.get_logger().warn(f"VFH ACTIVE: min_dist={min_dist:.2f}")
            
            # VFH histogram 계산
            histogram = np.zeros(self.sector_count)
            angle_min = self.latest_scan.angle_min
            angle_increment = self.latest_scan.angle_increment
            
            for i, r in enumerate(ranges):
                if r < self.vfh_threshold:
                    angle = angle_min + i * angle_increment
                    sector = int((angle + np.pi) / (2 * np.pi) * self.sector_count) % self.sector_count
                    histogram[sector] += (self.vfh_threshold - r) ** self.alpha
            
            # RL 목표 방향 (odom 기준)
            target_angle = np.arctan2(self.rl_goal.pose.position.y, self.rl_goal.pose.position.x)
            target_sector = int((target_angle + np.pi) / (2 * np.pi) * self.sector_count) % self.sector_count
            
            # Valley 선택 (최소 cost + 목표 근접)
            best_sector = target_sector
            min_cost = float('inf')
            for s in range(self.sector_count):
                cost = histogram[s] + 0.5 * abs(s - target_sector)
                if cost < min_cost:
                    min_cost = cost
                    best_sector = s
            
            # 명령 생성
            cmd = Twist()
            chosen_angle = -np.pi + best_sector * (2 * np.pi / self.sector_count)
            cmd.linear.x = 0.3  # 감속
            cmd.angular.z = 1.0 * np.sign(chosen_angle)  # 회피 회전
            self.cmd_vel_pub.publish(cmd)
        else:
            self.vfh_active = False
            # RL 명령 그대로 통과 (publish 없음)

def main():
    rclpy.init()
    node = VFHNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
