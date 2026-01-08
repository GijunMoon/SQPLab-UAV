# ==============================
# Author: sqplab
# Date: 2025-09-23
# Description: RL 기반 드론 제어 노드
# ==============================

# ROS2 라이브러리
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import TrajectorySetpoint
import asyncio
import threading

from mavros_msgs.srv import WaypointPull
from mavros_msgs.msg import WaypointList
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

import numpy as np
import time

# RL 라이브러리
#import gymnasium as gym
import stable_baselines3 as sb3

class RLDroneFollowerNode(Node):
    def __init__(self):
        super().__init__('rl_node')
        self.utility = Utility()

        self.takeoff_done = True #이륙 했을때만 동작 
        self.takeoff_time = None

        self.initial_position = None
        self.target_position = None  # Waypoint의 절대 좌표

        self.get_logger().info("Waypoint 가져오기...")
        self.waypoints = []

        # 사람 감지 상태 구독
        self.rescue_active = False  # 구조 동작 활성화 여부
        self.create_subscription(Bool, '/rescue_mode', self.rescue_mode_callback, 10)

        self.subscription_human = self.create_subscription(
            Bool, '/human_detected', self.human_detection_callback, 10)
        self.human_detected = False

        self.alert_pub = self.create_publisher(String, '/alert', 10)

        self._get_waypoints_from_mavros()
        if not self.waypoints or len(self.waypoints) < 2:
            self.get_logger().error("Waypoint 부족! 기본값 사용")
            self.target_position = np.array([10.0, 10.0, 3.0])
        else:
            self._initialize_waypoints()

        self.lidar_ranges = np.full(36, 20.0, dtype=np.float32)
        
        # subscriber: follower drone odom (ENU 좌표)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        self.subscription_lidar = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)

        # publisher: 목표 좌표 (RL에서 생성된 좌표)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal', 10)

        # RL 모델 로드
        self.rl_model = self.load_rl_model()

        # follower 현재 상태 저장 변수
        self.current_state = None
        self.action = None

    def _get_waypoints_from_mavros(self):
        """MAVROS를 통한 Waypoint 가져오기"""
        try:
            # Waypoint Pull 서비스 클라이언트
            cli = self.create_client(WaypointPull, '/mavros/mission/pull')
        
            self.get_logger().info("MAVROS mission service 대기 중...")
        
            if not cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("❌ MAVROS mission service 사용 불가")
                return
        
            self.get_logger().info("✅ MAVROS mission service 연결됨")
        
            # Waypoint 다운로드 요청
            req = WaypointPull.Request()
            future = cli.call_async(req)
        
            # 응답 대기
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
            if future.result() is not None and future.result().success:
                wp_count = future.result().wp_received
                self.get_logger().info(f"✅ MAVROS waypoints pulled: {wp_count}개")
            
                # ✅ 충분한 대기 시간
                time.sleep(1.0)
            
                # ✅ Waypoint 리스트 직접 가져오기 (동기 방식)
                self._fetch_waypoint_list_sync()
            else:
                self.get_logger().error("❌ Waypoint pull 실패")
    
        except Exception as e:
            self.get_logger().error(f"❌ MAVROS waypoint 오류: {e}")

    def _fetch_waypoint_list_sync(self):
        """MAVROS Waypoint 리스트 동기 방식으로 가져오기"""
        try:
            # 일회성 메시지 수신
            from rclpy.qos import QoSProfile, QoSReliabilityPolicy
        
            qos = QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
            )

            received = []
            msg = None
        
            def callback(data):
                nonlocal msg
                msg = data
        
            # 임시 구독
            sub = self.create_subscription(
                WaypointList,
                '/mavros/mission/waypoints',
                callback,
                qos
            )
        
            # 메시지 수신 대기 (최대 3초)
            timeout = 3.0
            start_time = time.time()
        
            while msg is None and (time.time() - start_time) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
        
            # 구독 해제
            self.destroy_subscription(sub)
        
            if msg is None:
                self.get_logger().error("Waypoint 리스트 수신 타임아웃")
                return
        
            # Waypoint 처리
            self.waypoints = []
        
            if not msg.waypoints:
                self.get_logger().warn("Waypoint 리스트가 비어있습니다")
                return
        
            for idx, wp in enumerate(msg.waypoints):
                lat = wp.x_lat
                lon = wp.y_long
                alt = wp.z_alt
            
                self.waypoints.append((lat, lon, alt))
                self.get_logger().info(
                    f"Waypoint {idx}: lat={lat:.6f}, lon={lon:.6f}, alt={alt:.2f}"
                )
        
            self.get_logger().info(f"✅ 총 {len(self.waypoints)}개 waypoint 로드됨")
    
        except Exception as e:
            self.get_logger().error(f"Waypoint 리스트 가져오기 실패: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

    
    def _subscribe_waypoint_list(self):
        """MAVROS Waypoint 리스트 구독"""
        def waypoint_callback(msg):
            self.waypoints = []
            for wp in msg.waypoints:
                # WGS84 좌표
                lat = wp.x_lat
                lon = wp.y_long
                alt = wp.z_alt
                self.waypoints.append((lat, lon, alt))
                self.get_logger().info(f"WP: {lat:.6f}, {lon:.6f}, {alt:.2f}")
            
            # Waypoint 초기화
            if len(self.waypoints) >= 2:
                self._initialize_waypoints()
        
        self.create_subscription(
            WaypointList, 
            '/mavros/mission/waypoints', 
            waypoint_callback, 
            10
        )

    def _initialize_waypoints(self):
        """Waypoint 기반 초기화"""
        self.get_logger().info(f"_initialize_waypoints 호출: {len(self.waypoints)}개")

        self.current_wp_index = 1
    
        if not self.waypoints or len(self.waypoints) < 2:
            self.get_logger().error(
                f"Waypoint 부족! (현재: {len(self.waypoints)}, 필요: 2)"
            )
            self.target_position = np.array([10.0, 10.0, 3.0])
            return
    
        try:
            # Waypoint 1: 기준점
            self.ref_lat, self.ref_lon, self.ref_alt = self.waypoints[0]
            self.get_logger().info(
                f"Reference (WP1): {self.ref_lat:.6f}, {self.ref_lon:.6f}, {self.ref_alt:.2f}"
            )
        
            # Waypoint 2: 목표점
            target_lat, target_lon, target_alt = self.waypoints[1]
            self.get_logger().info(
                f"Target (WP2): {target_lat:.6f}, {target_lon:.6f}, {target_alt:.2f}"
            )
        
            self.target_position = self.utility.ecef_to_enu(
                self.ref_lat, self.ref_lon, self.ref_alt,
                target_lat, target_lon, target_alt
            )
        
            if self.target_position is None:
                raise ValueError("ecef_to_enu returned None")
        
            self.get_logger().info(
                f"Target ENU: x={self.target_position[0]:.2f}, "
                f"y={self.target_position[1]:.2f}, "
                f"z={self.target_position[2]:.2f}"
            )

            self.update_target_position()
    
        except Exception as e:
            self.get_logger().error(f"Waypoint 변환 실패: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            self.target_position = np.array([10.0, 10.0, 3.0])

    def update_target_position(self):
        if self.current_wp_index < len(self.waypoints):
            lat, lon, alt = self.waypoints[self.current_wp_index]
            self.target_position = self.utility.ecef_to_enu(
                self.ref_lat, self.ref_lon, self.ref_alt,
                lat, lon, alt
            )
            self.get_logger().info(f"다음 waypoint 설정: #{self.current_wp_index}")
        else:
            self.get_logger().info("모든 waypoint 완료")
            self.target_position = None



    def load_rl_model(self):
        model = sb3.SAC.load(
        	'/home/sqplab/ws_ros2/src/drone_slam/drone_slam/model'
        	)
        self.get_logger().info("RL 모델 연결")
        self.max_steps = 500000
        self.step_count = 0
        return model

    def rescue_mode_callback(self, msg):
        self.rescueactive = msg.data
        if self.rescueactive:
            self.humandetected = False  # 강제 리셋


    def odom_callback(self, msg):
        # 현재 follower 위치, 상태 수신
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear

        if self.initial_position is None:
            self.initial_position = np.array([pos.x, pos.y, pos.z])
            self.get_logger().info(f"Initial odom position: {self.initial_position}")
        
        current_pos = np.array([pos.x, pos.y, pos.z])
        relative_pos = current_pos - self.initial_position

        # dx, dy, dz: 목적지까지 상대 위치
        dx = self.target_position[0] - relative_pos[0]
        dy = self.target_position[1] - relative_pos[1]
        dz = self.target_position[2] - relative_pos[2]

        clipped_dist = np.clip(np.linalg.norm([dx, dy]), 0.0, 30.0)
        state = np.concatenate([self.lidar_ranges, [clipped_dist], [dx, dy]]).astype(np.float32)

        self.current_state = state
        self.current_pos = np.array([pos.x, pos.y, pos.z], dtype=np.float32)

        # 이륙 완료 시점 판단
        if self.takeoff_time is None:
            self.takeoff_time = self.get_clock().now().nanoseconds
            self.takeoff_done = False
            self.get_logger().info("이륙 대기 시작")
            
        if not self.takeoff_done and self.check_takeoff_stable(msg):
            self.takeoff_done = True
            self.get_logger().info("이륙 완료, RL 컨트롤 시작")


        dist_xy = np.linalg.norm([dx, dy])
    
        if dist_xy < 1.0:
            self.get_logger().info(f"Waypoint #{self.current_wp_index} 도달")
            self.current_wp_index += 1
            if self.current_wp_index < len(self.waypoints):
                self.update_target_position()
            else:
                self.get_logger().info("모든 waypoint 방문 완료. 호버링 모드 돌입")
                self.action = np.array([0.0, 0.0])
                return

        if self.takeoff_done:
            if self.human_detected:
                # RL 액션 대신 현재 위치 유지
                self.get_logger().debug("호버링 중...")
            else:
                # 정상 RL 제어
                action, _ = self.rl_model.predict(
                    self.current_state, 
                    deterministic=True
                )
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

    def human_detection_callback(self, msg):
        """사람 감지 콜백"""
        if msg.data and not self.human_detected:
            # 최초 감지 시
            self.human_detected = True
            self.get_logger().warn("⚠️ 사람 감지! 호버링 모드 진입")
            
        
        elif not msg.data and self.human_detected:
            # 감지 해제 시
            self.get_logger().info("✅ 사람 감지 해제, RL 제어 재개")
            self.human_detected = False
            self.hover_position = None

    def infer_and_publish_goal(self):
        if self.current_state is None or self.action is None:
            return

        if np.linalg.norm(self.action) < 1e-3:
            return

        if self.rescue_active:
            # 구조 모드에서는 RL 제어 스킵
            return
    
        # Action 범위 제한
        MAX_DISPLACEMENT = 0.3
        dx = np.clip(self.action[0], -MAX_DISPLACEMENT, MAX_DISPLACEMENT)
        dy = np.clip(self.action[1], -MAX_DISPLACEMENT, MAX_DISPLACEMENT)
    
        # ENU 좌표계에서 목표 위치 계산 (변환 불필요)
        goal_enu_x = float(self.current_pos[0] + dx)
        goal_enu_y = float(self.current_pos[1] + dy)
        goal_enu_z = float(self.current_pos[2])  # 고도 유지
    
        # PoseStamped 메시지 생성
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "odom"
    
        goal_msg.pose.position.x = goal_enu_x
        goal_msg.pose.position.y = goal_enu_y
        goal_msg.pose.position.z = goal_enu_z
    
        # 방향은 기본값 (identity quaternion)
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
    

    def __del__(self):
        """노드 소멸 시 리소스 정리"""
        if hasattr(self, 'waypoint_thread'):
            self.waypoint_thread.join(timeout=5.0)


class Utility:
    # WGS84 상수
    a = 6378137.0          # 지구 장반경 [m]
    f = 1 / 298.257223563  # 편평률
    e_sq = f * (2 - f)     # 이심률 제곱

    def __init__(self):
        pass

    @classmethod
    def geodetic_to_ecef(cls, lat, lon, alt):
        """
        WGS84 geodetic 좌표를 ECEF 좌표로 변환
        
        Args:
            lat: 위도 (degrees)
            lon: 경도 (degrees)
            alt: 고도 (meters)
        
        Returns:
            ECEF 좌표 [x, y, z] (meters)
        """
        lat_rad = np.radians(lat)
        lon_rad = np.radians(lon)

        N = cls.a / np.sqrt(1 - cls.e_sq * np.sin(lat_rad)**2)
        
        x = (N + alt) * np.cos(lat_rad) * np.cos(lon_rad)
        y = (N + alt) * np.cos(lat_rad) * np.sin(lon_rad)
        z = ((1 - cls.e_sq) * N + alt) * np.sin(lat_rad)
        
        return np.array([x, y, z])

    @classmethod
    def ecef_to_enu(cls, lat_ref, lon_ref, alt_ref, lat, lon, alt):
        """
        ECEF 좌표를 ENU 좌표로 변환
        
        Args:
            lat_ref, lon_ref, alt_ref: 기준점의 geodetic 좌표 (degrees, meters)
            lat, lon, alt: 변환할 점의 geodetic 좌표 (degrees, meters)
        
        Returns:
            ENU 좌표 [east, north, up] (meters)
        """
        # Geodetic -> ECEF 변환
        ref_ecef = cls.geodetic_to_ecef(lat_ref, lon_ref, alt_ref)
        target_ecef = cls.geodetic_to_ecef(lat, lon, alt)
        
        # ECEF 차이 벡터
        diff = target_ecef - ref_ecef

        # ENU 회전 행렬
        lat_rad = np.radians(lat_ref)
        lon_rad = np.radians(lon_ref)
        
        R = np.array([
            [-np.sin(lon_rad), np.cos(lon_rad), 0],
            [-np.sin(lat_rad)*np.cos(lon_rad), -np.sin(lat_rad)*np.sin(lon_rad), np.cos(lat_rad)],
            [np.cos(lat_rad)*np.cos(lon_rad), np.cos(lat_rad)*np.sin(lon_rad), np.sin(lat_rad)]
        ])

        enu = R @ diff
        return enu  # [east, north, up]


def main(args=None):
    rclpy.init(args=args)
    node = RLDroneFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
