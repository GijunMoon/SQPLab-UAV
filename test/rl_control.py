import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.parameter import Parameter

import numpy as np
import math
import time
from collections import deque

# ROS2 및 PX4 메시지 타입
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, VehicleAttitude, VehicleStatus

# Stable Baselines3 AI 라이브러리
# 설치: pip install stable-baselines3[extra] torch
from stable_baselines3 import SAC

# ==============================================================================
# 1. 강화학습 환경 (SafeDroneEnvironment) 클래스
#    - 역할: 시뮬레이션 속 드론과의 통신, 센서 데이터 수신, 보상 계산 등 '게임 환경' 역할
# ==============================================================================
class SafeDroneEnvironment(Node):
    def __init__(self):
        super().__init__('safe_drone_environment')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        
        # --- 환경 파라미터 ---
        self.max_episode_steps = 1000
        self.target_altitude = 2.5
        self.goal_position = np.array([10.0, 10.0])
        
        # --- 상태 변수 ---
        self.current_pose = None
        self.laser_data = None
        self.current_attitude = None
        self.previous_distance = None
        self.current_nav_state = None
        self.offboard_mode_active = False

        # --- 안전 및 제어 파라미터 ---
        self.max_position_step = 0.5
        self.max_tilt_angle_deg = 25.0
        self.collision_threshold = 0.8
        self.pre_flight_checks_passed = False
        
        # --- 보상 함수 파라미터 ---
        self.goal_reward = 100.0
        self.collision_penalty = -50.0
        self.distance_reward_scale = 2.0
        self.attitude_penalty_scale = 5.0
        
        # --- 내부 상태 플래그 ---
        self.current_step = 0
        self.data_ready = False
        self.target_position = np.array([0.0, 0.0, self.target_altitude])

        # QoS 프로파일 설정
        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=10)
        
        # 퍼블리셔 및 서브스크라이버
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_reliable)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_reliable)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, qos_profile_sensor_data)
        self.attitude_sub = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, qos_profile_sensor_data)
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile_sensor_data)
        
        self.get_logger().info("Safe Drone Environment initialized.")

    # --- 콜백 함수들 (데이터 수신) ---
    def odom_callback(self, msg):
        self.current_pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        if not self.data_ready: self._check_data_ready()

    def laser_callback(self, msg):
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = msg.range_max
        self.laser_data = ranges
        if not self.data_ready: self._check_data_ready()

    def attitude_callback(self, msg):
        q = msg.q
        sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3]); cosr_cosp = 1 - 2 * (q[1]**2 + q[2]**2)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (q[0] * q[2] - q[3] * q[1])
        pitch = math.asin(sinp) if abs(sinp) < 1.0 else math.copysign(math.pi / 2, sinp)
        self.current_attitude = np.array([roll, pitch, 0.0]) # Yaw is not used here
        if not self.data_ready: self._check_data_ready()
        
    def vehicle_status_callback(self, msg):
        self.current_nav_state = msg.nav_state
        self.offboard_mode_active = (self.current_nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.pre_flight_checks_passed = msg.pre_flight_checks_pass

    def _check_data_ready(self):
        if (self.current_pose is not None and self.laser_data is not None and self.current_attitude is not None):
            self.data_ready = True
            self.get_logger().info("All sensor data received. Environment is ready.")

    # --- Gym API와 유사한 함수들 ---
    def reset(self):
        self.get_logger().info("Resetting episode...")
        self.current_step = 0
        self.target_position = np.array([0.0, 0.0, self.target_altitude])

        # 데이터가 준비될 때까지 대기
        while not self.data_ready:
            self.get_logger().warn("Waiting for sensor data...")
            rclpy.spin_once(self, timeout_sec=0.5)
        
        self._set_offboard_mode()

        # 이륙 및 고도 안정화 대기
        start_time = time.time()
        while time.time() - start_time < 15.0:
            self.publish_position_command()
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_pose is not None and abs(self.current_pose[2] - self.target_altitude) < 0.2:
                self.get_logger().info("Takeoff and altitude stabilization complete.")
                self.previous_distance = np.linalg.norm(self.current_pose[:2] - self.goal_position)
                return self.get_state()
        
        self.get_logger().error("Takeoff failed!")
        return None

    def step(self, action):
        if not self.data_ready or not self.offboard_mode_active:
            return self.get_state(), 0.0, False, {}

        self.current_step += 1
        action = np.clip(action, -1.0, 1.0) * self.max_position_step
        self.target_position[:2] += action
        
        self.publish_position_command()
        rclpy.spin_once(self, timeout_sec=0.05)

        reward = self._calculate_reward()
        done = self._is_episode_done()
        info = {'distance_to_goal': np.linalg.norm(self.current_pose[:2] - self.goal_position)}
        
        return self.get_state(), reward, done, info

    def get_state(self):
        if not self.data_ready:
            return np.zeros(38, dtype=np.float32)

        rel_pos = self.goal_position - self.current_pose[:2]
        num_sectors = 36
        sector_size = len(self.laser_data) // num_sectors
        laser_features = [np.min(self.laser_data[i*sector_size:(i+1)*sector_size]) for i in range(num_sectors)]
        
        return np.concatenate([rel_pos, laser_features]).astype(np.float32)

    def _calculate_reward(self):
        reward = 0.0
        current_distance = np.linalg.norm(self.current_pose[:2] - self.goal_position)
        if self.previous_distance is not None:
            reward += (self.previous_distance - current_distance) * self.distance_reward_scale
        self.previous_distance = current_distance
        
        if current_distance < 1.0: reward += self.goal_reward
        if np.min(self.laser_data) < self.collision_threshold: reward += self.collision_penalty
        
        roll_deg = abs(math.degrees(self.current_attitude[0])); pitch_deg = abs(math.degrees(self.current_attitude[1]))
        if max(roll_deg, pitch_deg) > self.max_tilt_angle_deg: reward -= self.attitude_penalty_scale
            
        return reward

    def _is_episode_done(self):
        if self.current_step >= self.max_episode_steps: return True
        if np.linalg.norm(self.current_pose[:2] - self.goal_position) < 1.0: return True
        if np.min(self.laser_data) < self.collision_threshold: return True
        roll_deg = abs(math.degrees(self.current_attitude[0])); pitch_deg = abs(math.degrees(self.current_attitude[1]))
        if max(roll_deg, pitch_deg) > self.max_tilt_angle_deg + 10: return True
        return False
        
    # --- PX4 제어 함수들 ---
    def publish_position_command(self):
        msg = TrajectorySetpoint()
        msg.position[0] = float(self.target_position[0]); msg.position[1] = float(self.target_position[1])
        msg.position[2] = -float(self.target_altitude)
        msg.yaw = math.nan
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command; msg.param1 = params.get("param1", 0.0); msg.param2 = params.get("param2", 0.0)
        msg.target_system = 1; msg.target_component = 1; msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def _set_offboard_mode(self):
        self.get_logger().info("Setting Offboard mode...")
        for _ in range(20):
            self.publish_position_command()
            rclpy.spin_once(self, timeout_sec=0.1)

        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        time.sleep(1)

        self.get_logger().info("Waiting for pre-flight checks to pass...")
        wait_start_time = time.time()
        while time.time() - wait_start_time < 10:
            rclpy.spin_once(self, timeout_sec=0.1) # 콜백 처리를 위해 spin_once 호출
            if self.pre_flight_checks_passed:
                self.get_logger().info("Pre-flight checks passed!")
                break
        
        if not self.pre_flight_checks_passed:
            self.get_logger().error("Pre-flight checks failed! Arming aborted.")
            # 실패 원인을 직접 확인하려면 PX4 콘솔에서 'commander check'를 입력하세요.
            return

        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        
        start_time = time.time()
        while not self.offboard_mode_active and time.time() - start_time < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.offboard_mode_active: self.get_logger().info("Offboard mode enabled.")
        else: self.get_logger().error("Failed to enable Offboard mode.")

# ==============================================================================
# 2. 메인 실행 함수 (main)
#    - 역할: AI 에이전트(SAC 모델)를 로드하고, 환경과 상호작용하며 제어 루프를 실행
# ==============================================================================
def main():
    rclpy.init()
    # 1. 환경(드론 노드) 생성
    env = SafeDroneEnvironment()
    
    # 2. 학습된 SAC 모델 로드
    try:
        model = SAC.load('model.zip')
        env.get_logger().info("성공적으로 'model.zip' 파일을 로드했습니다.")
    except Exception as e:
        env.get_logger().error(f"모델 로딩 실패: {e}")
        env.get_logger().error("스크립트와 같은 경로에 'model.zip' 파일이 있는지 확인하세요.")
        rclpy.shutdown()
        return

    # 3. 제어 루프 시작
    state = env.reset()
    if state is None:
        env.get_logger().error("환경 리셋에 실패하여 프로그램을 종료합니다.")
        env.destroy_node()
        rclpy.shutdown()
        return

    done = False
    try:
        while rclpy.ok():
            # 에피소드가 끝나면 리셋
            if done:
                state = env.reset()
                if state is None:
                    env.get_logger().error("환경 리셋 실패. 종료합니다."); break
            
            # 현재 상태를 기반으로 AI 모델이 다음 행동 예측
            action, _ = model.predict(state, deterministic=True)
            
            # 예측된 행동을 환경에 적용하고 다음 상태, 보상 등을 받음
            next_state, reward, done, info = env.step(action)
            state = next_state
            
            env.get_logger().info(f"Step: {env.current_step}, Reward: {reward:.2f}, Done: {done}, Dist: {info.get('distance_to_goal', -1):.2f}")

    except KeyboardInterrupt:
        env.get_logger().info("Ctrl-C 입력 감지, 프로그램을 종료합니다.")
    finally:
        env.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()