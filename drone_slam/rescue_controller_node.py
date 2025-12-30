# ==============================
# Author: sqplab
# Date: 2025-11-21
# Description: 구조 동작 수행 노드
# ==============================

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
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
        
        self.get_logger().info("Rescue Controller 시작")
    
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
    
        if hasattr(self, 'humanposition') and self.humanposition is not None:
            enux, enuy, enuz = self.humanposition  # ENU 유지
            targetx = enux  # 직접 ENU 사용
            targety = enuy
        else:
            targetx = float(self.currentposition[0])
            targety = float(self.currentposition[1])

        # offboard 상태 확인 추가
        if self.offboardsetpointcounter < 10:  # 1초 대기
            self.offboardsetpointcounter += 1
            return
    
        # 1단계: 하강
        if self.descending:
            target_z = -self.target_rescue_altitude  # NED 좌표계 (음수 = 위)
            current_z = -self.current_position[2]
        
            self.get_logger().warn(
                f"하강 중! 현재: {-current_z:.2f}m, 목표: {-target_z:.2f}m"
            )
        
            if abs(current_z - target_z) < 0.4:
                self.get_logger().warn("하강 완료! 물뿌리기 시작!")
                self.descending = False
                self.hovering = True
                self.spraying = True
                self.spray_start_time = current_time
            
                spray_msg = Bool()
                spray_msg.data = True
                self.spray_pub.publish(spray_msg)
        
            traj = TrajectorySetpoint()
            traj.position = [
                float(target_x),
                float(target_y),
                target_z
            ]
            traj.yaw = 0.0
            traj.timestamp = int(current_time.nanoseconds / 1000)
            self.trajectory_pub.publish(traj)
    
        # 2단계: 물뿌리기
        elif self.hovering and self.spraying:
            spray_elapsed = (current_time - self.spray_start_time).nanoseconds / 1e9
        
            traj = TrajectorySetpoint()
            traj.position = [
                float(target_x),
                float(target_y),
                -self.target_rescue_altitude
            ]
            traj.yaw = 0.0
            traj.timestamp = int(current_time.nanoseconds / 1000)
            self.trajectory_pub.publish(traj)
        
            if spray_elapsed < self.spray_duration:
                spray_msg = Bool()
                spray_msg.data = True
                self.spray_pub.publish(spray_msg)
            
                remaining = self.spray_duration - spray_elapsed
                self.get_logger().warn(f"물뿌리기 중... ({remaining:.1f}초)")
            else:
                self.get_logger().warn("구조 완료!")
            
                spray_msg = Bool()
                spray_msg.data = False
                self.spray_pub.publish(spray_msg)

                rescue_msg = Bool()
                rescue_msg.data = False
                self.rescue_mode_pub.publish(rescue_msg)
            
                self.spraying = False
                self.hovering = False
                self.rescue_mode = False

def main(args=None):
    rclpy.init(args=args)
    node = RescueControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
