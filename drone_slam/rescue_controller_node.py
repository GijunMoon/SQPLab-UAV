# ==============================
# Author: sqplab (Revised)
# Description: 구조 동작 수행 노드
# ==============================

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
import numpy as np

class RescueControllerNode(Node):
    def __init__(self):
        super().__init__('rescue_controller_node')
        
        # QoS 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 상태 변수
        self.rescue_mode = False
        self.spraying = False
        self.current_position = None  # [x, y, z] (ENU)
        self.hold_position = None     # [x, y, z] (ENU)
        self.spray_start_time = None
        self.offboard_setpoint_counter = 0
        
        # Subscriber
        self.human_sub = self.create_subscription(
            Bool, '/human_detected', self.human_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Publisher
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.spray_pub = self.create_publisher(Bool, '/water_spray', 10)
        self.rescue_mode_pub = self.create_publisher(Bool, '/rescue_mode', 10)
        
        # 타이머 (0.1초 주기)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Rescue Controller Ready (Heartbeat Fix Applied)")

    def human_callback(self, msg):
        # 이미 구조 중이면 무시
        if msg.data and not self.rescue_mode:
            self.get_logger().warn("사람 발견! 구조 시퀀스 진입")
            self.rescue_mode = True
            self.offboard_setpoint_counter = 0
            
            # 현재 위치가 있으면 즉시 홀드 지점으로 설정 (없으면 나중에 설정됨)
            if self.current_position is not None:
                self.hold_position = self.current_position.copy()
            
            # 상태 알림
            rescue_msg = Bool()
            rescue_msg.data = True
            self.rescue_mode_pub.publish(rescue_msg)
            
            # Offboard 모드 변경 명령
            self.engage_offboard_mode()

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        # ROS 좌표계 (ENU) 그대로 저장
        self.current_position = np.array([pos.x, pos.y, pos.z])

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info("Offboard 모드 요청 전송")

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def control_loop(self):
        current_time = self.get_clock().now()
        
        # [중요] Heartbeat는 조건문 없이 항상 전송해야 Failsafe(원점복귀)가 안 걸림
        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.timestamp = int(current_time.nanoseconds / 1000)
        self.offboard_mode_pub.publish(offboard_msg)

        # 위치 데이터가 없거나 구조 모드가 아니면 여기서 종료 (Heartbeat는 보냈으므로 안전)
        if self.current_position is None or not self.rescue_mode:
            return

        # 1. 초기화 대기 (1초간 현재 위치 유지하며 setpoint 전송)
        if self.offboard_setpoint_counter < 10:
            self.offboard_setpoint_counter += 1
            # 아직 hold_position이 없으면 현재 위치 사용
            target = self.hold_position if self.hold_position is not None else self.current_position
            self.publish_trajectory(target)
            return

        # 2. Spray 로직 진입
        if not self.spraying:
            self.spraying = True
            self.spray_start_time = current_time
            # 스프레이 시작 시점의 위치를 고정 (표류 방지)
            self.hold_position = self.current_position.copy()
            self.get_logger().warn(f"Spraying... Hold Pos: {self.hold_position}")

        # 3. 시간 체크 및 종료 처리
        if self.spraying:
            elapsed = (current_time - self.spray_start_time).nanoseconds / 1e9
            
            if elapsed < 5.0:
                self.spray_pub.publish(Bool(data=True))
            else:
                self.spray_pub.publish(Bool(data=False))
                self.get_logger().info("구조 종료.")
                self.rescue_mode = False
                self.spraying = False
                # 종료 시 즉시 리턴하지 않고 마지막으로 한 번 더 위치를 잡아주는 것이 안전함
            
            # 4. 위치 유지 명령 전송
            # hold_position이 있으면 그것을, 없으면 현재 위치를 타겟으로
            target = self.hold_position if self.hold_position is not None else self.current_position
            self.publish_trajectory(target)

    def publish_trajectory(self, enu_target):
        # [핵심] ROS(ENU) -> PX4(NED) 좌표 변환
        # ENU: East(x), North(y), Up(z)
        # NED: North(x), East(y), Down(z)
        # 변환식: NED_X = ENU_Y, NED_Y = ENU_X, NED_Z = -ENU_Z
        
        traj = TrajectorySetpoint()
        
        # 좌표 변환 적용 (오도메트리가 ENU라고 가정 시)
        # 만약 오도메트리가 이미 NED라면 이 변환을 제거하고 그대로 넣으세요.
        ned_x = float(enu_target[1])       # ENU y -> NED x
        ned_y = float(enu_target[0])       # ENU x -> NED y
        ned_z = float(-enu_target[2])      # ENU z -> NED z (부호 반대)
        
        traj.position = [ned_x, ned_y, ned_z]
        traj.yaw = 0.0 # 필요 시 현재 yaw로 수정 가능 (NaN 처리 권장되나 0.0도 무방)
        traj.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        self.trajectory_pub.publish(traj)

def main(args=None):
    rclpy.init(args=args)
    node = RescueControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
