# ==============================
# Author: sqplab
# Date: 2025-09-23
# Description: RL 기반 드론 제어 노드
# ==============================

# ROS2 라이브러리
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import time

# RL 라이브러리
#import gymnasium as gym
import stable_baselines3 as sb3

class RLDroneFollowerNode(Node):
    def __init__(self):
        super().__init__('rl_node')

        self.takeoff_done = False #이륙 했을때만 동작 
        #fail safe alret!!!!!!!!
        self.takeoff_time = None

        # 고정 목적지 (예: 미리 지정된 고정 위치)
        self.target_position = np.array([10.0, 15.0, 3.0])  # ENU 좌표계

        self.lidar_ranges = np.full(36, 20.0, dtype=np.float32)
        
        # subscriber: follower drone odom (ENU 좌표)
        self.subscription = self.create_subscription(
            Odometry,
            '/mavros/odometry/in',
            self.odom_callback,
            10)

        self.subscription_lidar = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)

        # publisher: 목표 좌표 (RL에서 생성된 좌표)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # RL 모델 로드 (PyTorch SAC 모델 가정)
        self.rl_model = self.load_rl_model()

        # follower 현재 상태 저장 변수
        self.current_state = None

    def load_rl_model(self):
        model = sb3.SAC.load(
        	'/home/sqplab/ws_ros2/src/SQPLab-UAV/drone_slam/model.zip'
        	)
        self.get_logger().info("RL 모델 연결")
        self.max_steps = 500000
        self.step_count = 0
        return model


    def odom_callback(self, msg):
        # 현재 follower 위치, 상태 수신
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear

        # dx, dy, dz: 목적지까지 상대 위치
        dx = self.target_position[0] - pos.x
        dy = self.target_position[1] - pos.y
        dz = self.target_position[2] - pos.z

        clipped_dist = np.clip(np.linalg.norm([dx, dy]), 0.0, 30.0)
        state = np.concatenate([self.lidar_ranges, [clipped_dist], [dx, dy]]).astype(np.float32)

        self.current_state = state

        self.current_pos = np.array([pos.x, pos.y, pos.z], dtype=np.float32)

        # 이륙 완료 시점 판단
        if self.takeoff_time is None:
            self.takeoff_time = self.get_clock().now().nanoseconds
            self.takeoff_done = False # fail safe alret!!!!!!!
            self.get_logger().info("이륙 대기 시작")
        elapsed_time = (self.get_clock().now().nanoseconds - self.takeoff_time) / 1e9
        if self.check_takeoff_stable(msg):
            self.takeoff_done = True
            self.get_logger().info("이륙 완료, RL 컨트롤 시작")

        if self.takeoff_done:
            action, _ = self.rl_model.predict(self.current_state, deterministic=True)
            self.action = action

            self.step_count += 1

            self.infer_and_publish_goal()

    def lidar_callback(self, msg: LaserScan):
        # LaserScan 메시지 angle_min, angle_max, ranges 샘플 예시
        # 36개 방향으로 각 구간 최소거리 계산
        num_sectors = 36
        sector_size = (msg.angle_max - msg.angle_min) / num_sectors

        sector_min_ranges = np.full(num_sectors, 20.0, dtype=np.float32)

        for i, r in enumerate(msg.ranges):
            if np.isfinite(r):
                # 해당 각도를 구간(섹터) 인덱스로 변환
                angle = msg.angle_min + i * msg.angle_increment
                sector = int((angle - msg.angle_min) / sector_size)
                if sector >= num_sectors:
                    sector = num_sectors - 1
                sector_min_ranges[sector] = min(sector_min_ranges[sector], r)

        self.lidar_ranges = sector_min_ranges

    def infer_and_publish_goal(self):
        if self.current_state is None or self.action is None:
            return

        # action이 (dx, dy) 형태라면 z 좌표는 상태 그대로 유지
        goal_x = self.current_pos[0] + self.action[0]
        goal_y = self.current_pos[1] + self.action[1]
        goal_z = self.current_pos[2]  # 필요 시 action으로 조정 가능


        goal_pos = np.array([goal_x, goal_y, goal_z])

        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'odom'

        goal_msg.pose.position.x = float(goal_pos[0])
        goal_msg.pose.position.y = float(goal_pos[1])
        goal_msg.pose.position.z = float(goal_pos[2])

        goal_msg.pose.orientation.w = 1.0
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = 0.0

        self.goal_pub.publish(goal_msg)

    def check_takeoff_stable(self, odom_msg):
        # 목표 이륙 고도
        TARGET_ALTITUDE = 3.0   # meters

        # 고도 허용 오차
        ALTITUDE_TOLERANCE = 0.3

        # 최대 허용 속도 (m/s), 이륙 후 안정 상태 판단용
        MAX_SPEED = 0.2

        # 최대 허용 기울기(자세) 각도 (rad)
        MAX_ANGLE_RADIANS = 0.1  # 약 5.7도

        # 최소 이륙 보장 시간 (초)
        MIN_TAKEOFF_TIME = 5.0

        # 고도
        z = odom_msg.pose.pose.position.z

        # 속도 계산
        vx = odom_msg.twist.twist.linear.x
        vy = odom_msg.twist.twist.linear.y
        vz = odom_msg.twist.twist.linear.z
        speed = np.linalg.norm([vx, vy, vz])

        # 자세(orientation) quaternion -> euler (roll, pitch, yaw)
        orientation = odom_msg.pose.pose.orientation
        roll, pitch, _ = self.quaternion_to_euler(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )

        # 시간 체크
        elapsed_time = 0.0
        if self.takeoff_time is not None:
            elapsed_time = (self.get_clock().now().nanoseconds - self.takeoff_time) / 1e9

        altitude_ok = abs(z - TARGET_ALTITUDE) <= ALTITUDE_TOLERANCE
        speed_ok = speed <= MAX_SPEED
        attitude_ok = (abs(roll) <= MAX_ANGLE_RADIANS) and (abs(pitch) <= MAX_ANGLE_RADIANS)
        time_ok = elapsed_time >= MIN_TAKEOFF_TIME

        return altitude_ok and speed_ok and attitude_ok and time_ok


    def quaternion_to_euler(self, x, y, z, w):
        # Quaternion to Euler angles (roll, pitch, yaw)
        import math

        # roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw




def main(args=None):
    rclpy.init(args=args)
    node = RLDroneFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
