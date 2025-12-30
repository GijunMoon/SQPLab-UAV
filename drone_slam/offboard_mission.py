# ==============================
# Author: sqplab
# Date: 2025-09-23
# Description: 드론 오프보드 제어 노드
# ==============================

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from mavros_msgs.srv import CommandBool, SetMode
import time

class OffboardTakeoffNode(Node):
    def __init__(self):
        super().__init__('offboard_takeoff_node')
        

        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10) # 로컬 위치 제어용
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming') # 아밍 서비스 클라이언트
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode') # 모드 변경 서비스 클라이언트

        self.rescue_active = False
        self.rescue_triggered = False
        self.create_subscription(Bool, '/rescue_mode', self.rescue_mode_callback, 10)

        # RL output 입력 받을 goal_pose
        self.subscription = self.create_subscription(
            PoseStamped, '/goal', self.goal_pose_callback, 10)

        self.target_pose = PoseStamped()
        self.target_pose.pose.position.x = 0.0
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.position.z = 3.0

        self.armed = False
        self.offboard_mode_set = False
        self.setpoint_sent = 0

        # 사람 감지 구독
        self.subscription_human = self.create_subscription(
            Bool, '/human_detected', self.human_callback, 10)
        
        self.is_holding = False

        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):

        self.get_logger().debug(f"Flags: rescue={self.rescue_active}, "
                       f"holding={self.is_holding}, "
                       f"triggered={getattr(self, 'rescue_triggered', False)}")
        
        if not self.armed:
            self.arm()
        elif not self.offboard_mode_set:
            self.set_offboard_mode()

        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.pose_pub.publish(self.target_pose)
        if self.setpoint_sent < 40:
            self.setpoint_sent += 1
            return

        if self.rescue_active or self.rescue_triggered or self.is_holding:
            # 구조 동작일 때는 구조동작에서 publish하는 setpoint만 따라감
            self.target_pose.pose.position.x = 0.0
            self.target_pose.pose.position.y = 0.0
            self.target_pose.pose.position.z = 3.0
            self.get_logger().debug("구조 모드: 호버링 유지")


    def goal_pose_callback(self, msg):
        # RL에서 새로운 목표 위치가 오면 위에 바로 반영
        self.target_pose.pose.position.x = msg.pose.position.x
        self.target_pose.pose.position.y = msg.pose.position.y
        self.target_pose.pose.position.z = msg.pose.position.z

    def rescue_mode_callback(self, msg):
        self.rescue_active = msg.data

    def arm(self):
        if self.arming_client.service_is_ready():
            req = CommandBool.Request()
            req.value = True
            future = self.arming_client.call_async(req)
            future.add_done_callback(self.arm_response_callback)

    def arm_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.armed = True
                self.get_logger().info("드론 아밍 성공")
            else:
                self.get_logger().warn("아밍 실패, 재시도 중...")
        except Exception as e:
            self.get_logger().error(f"아밍 서비스 예외: {e}")

    def set_offboard_mode(self):
        if self.mode_client.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = 'OFFBOARD'
            future = self.mode_client.call_async(req)
            future.add_done_callback(self.offboard_response_callback)

    def offboard_response_callback(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.offboard_mode_set = True
                self.get_logger().info("오프보드 모드 진입 성공")
            else:
                self.get_logger().warn("오프보드 모드 진입 실패, 재시도 중...")
        except Exception as e:
            self.get_logger().error(f"오프보드 서비스 예외: {e}")

    def human_callback(self, msg):
        pass # 구조 모드 진입은 rescue_controller_node.py에서 처리
    
    def set_hold_mode(self):
        """PX4 Hold 모드 설정"""
        if self.mode_client.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = 'LOITER'  # Hold 모드
            self.mode_client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = OffboardTakeoffNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
