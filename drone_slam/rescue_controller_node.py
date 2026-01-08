# ==============================
# Author: sqplab
# Date: 2025-11-21
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
        
        # QoS 설정 (PX4용)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 상태
        self.rescue_mode = False
        self.descending = False
        self.hovering = False
        self.spraying = False
        
        self.current_position = None
        self.target_rescue_altitude = 2.0
        self.spray_duration = 5.0
        
        # Subscriber
        self.human_sub = self.create_subscription(
            Bool, '/human_detected', self.human_callback, 10)
        
        self.human_pos_sub = self.create_subscription(
            Point, '/human_position', self.human_pos_callback, 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Publisher (PX4 직접 제어)
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        self.spray_pub = self.create_publisher(Bool, '/water_spray', 10)

        self.rescue_mode_pub = self.create_publisher(Bool, '/rescue_mode', 10)
        
        # 타이머
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.rescue_start_time = None
        self.spray_start_time = None
        self.offboard_setpoint_counter = 0

        self.vfh_goal = None
        self.vfh_sub = self.create_subscription(PoseStamped, '/mavros/setpoint_position/local', self.vfh_callback, 10)
        
        self.get_logger().info("Rescue Controller 시작")
    
    def vfh_callback(self, msg):
        self.vfh_goal = msg.pose.position

    def human_callback(self, msg):
        if msg.data and not self.rescue_mode:
            self.get_logger().warn("사람 발견! 구조 작업 시작")
            self.rescue_mode = True
            self.descending = True
            self.rescue_start_time = self.get_clock().now()
            self.offboard_setpoint_counter = 0

            rescue_msg = Bool()
            rescue_msg.data = True
            self.rescue_mode_pub.publish(rescue_msg)
            
            # Offboard 모드 활성화
            self.engage_offboard_mode()
    
    def human_pos_callback(self, msg):
        self.human_position = [msg.x, msg.y, msg.z]
    
    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        self.current_position = np.array([pos.x, pos.y, pos.z])
    
    def engage_offboard_mode(self):
        """Offboard 모드 활성화"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info("Offboard 모드 활성화 명령 전송")
    
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """PX4 명령 전송"""
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
        if not self.rescue_mode or self.current_position is None:
            return
    
        current_time = self.get_clock().now()
    
        # Offboard 모드 유지
        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.timestamp = int(current_time.nanoseconds / 1000)
        self.offboard_mode_pub.publish(offboard_msg)
    
        if hasattr(self, 'human_position') and self.human_position is not None:
            enux, enuy, enuz = self.human_position  # ENU 유지
            targetx = enux  # 직접 ENU 사용
            targety = enuy
        else:
            targetx = float(self.current_position[0])
            targety = float(self.current_position[1])

        # offboard 상태 확인 추가
        if self.offboard_setpoint_counter < 10:  # 1초 대기
            self.offboard_setpoint_counter += 1
            return

        self.spraying = True
        self.spray_start_time = current_time
    
        """# 1단계: 하강
        if self.descending:
            desired_z_enu = self.target_rescue_altitude  # 2.0
            current_z_enu = self.current_position[2]

            hover_x = self.current_position[0]
            hover_y = self.current_position[1]

            if self.vfh_goal:
                vfh_z_safe = self.vfh_goal.z - 0.5
                target_z_enu = max(desired_z_enu, vfh_z_safe)

            if current_z_enu > target_z_enu + 0.4:
                target_z_enu = current_z_enu - 0.5  # 50cm/s 하강
    
            # 하강: 0.5m/s
            descent_rate = 0.5
            target_z_enu = max(desired_z_enu, current_z_enu - descent_rate * 0.1)  # 5cm/루프
    
            self.get_logger().warn(f"하강: cur={current_z_enu:.3f} → tgt={target_z_enu:.3f} rate={descent_rate}")
    
            # 전환: 0.3m 오차
            if current_z_enu <= desired_z_enu + 0.3:
                self.get_logger().error("하강 완료!")
                self.descending = False
                self.hovering = True
                self.spraying = True
                self.spray_start_time = current_time
                spray_msg = Bool(data=True)
                self.spray_pub.publish(spray_msg)
                return
    
            traj = TrajectorySetpoint()
            traj.position = [hover_x, hover_y, -target_z_enu]
            traj.yaw = 0.0
            traj.timestamp = int(current_time.nanoseconds / 1000)
            self.trajectory_pub.publish(traj)"""

    
        # 2단계: 물뿌리기
        if self.spraying:
            if self.spray_start_time is None:
                self.spray_start_time = current_time  # 시작 시간 기록
    
            spray_elapsed = (current_time - self.spray_start_time).nanoseconds / 1e9
    
            if spray_elapsed < 5.0:  # 5초 고정
                spray_msg = Bool()
                spray_msg.data = True
                self.spray_pub.publish(spray_msg)
        
                remaining = 5.0 - spray_elapsed
                self.get_logger().warn(f"🚿 물뿌리기 중... ({remaining:.1f}초 남음)")
            else:
                self.get_logger().warn("✅ 구조 완료! (총 5초)")
        
                # 스프레이 중지
                spray_msg = Bool()
                spray_msg.data = False
                self.spray_pub.publish(spray_msg)
        
                # 구조 모드 종료
                rescue_msg = Bool()
                rescue_msg.data = False
                self.rescue_mode_pub.publish(rescue_msg)
        
                # 상태 초기화
                self.spraying = False
                self.spray_start_time = None  # 재사용 위해 초기화
                self.hovering = False
                self.rescue_mode = False


def main(args=None):
    rclpy.init(args=args)
    node = RescueControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
