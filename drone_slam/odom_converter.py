#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R
import numpy as np

class OdomConverter(Node):
    def get_odom_msg_type(self, odom_source):  # 클래스 내부에 정의
        if odom_source == "/fmu/out/vehicle_odometry":
            return VehicleOdometry
        elif odom_source == "mavros/local_position/pose":
            return PoseStamped
        return VehicleOdometry

    def __init__(self):
        super().__init__('odom_converter')

        # Parameter 설정
        self.is_sim = self.get_parameter('use_sim_time').get_parameter_value().bool_value

        if self.is_sim:
            self.odom_source = "fmu/out/vehicle_odometry"
            msg_type = VehicleOdometry
            callback = self.listener_callback  # PX4용
        else:
            self.odom_source = "mavros/local_position/pose"
            msg_type = PoseStamped
            callback = self.listener_callback_pose  # MAVROS용
        self.get_logger().info(f"Using odom source: {self.odom_source}")

        # Odometry 구독
        self.subscription = self.create_subscription(
            msg_type,
            self.odom_source,
            callback,
            qos_profile_sensor_data,
        )

        # ROS Odometry 퍼블리시
        self.publisher = self.create_publisher(Odometry, '/odom', 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Frame names
        self.odom_frame = 'odom'
        self.base_frame = 'base_link'

        # 좌표 변환 행렬 (NED→ENU, FRD→FLU)
        self.world_transform = np.array([
            [0, 1, 0],   # X_enu = Y_ned (East)
            [1, 0, 0],   # Y_enu = X_ned (North) 
            [0, 0, -1]   # Z_enu = -Z_ned (Up)
        ])

        self.body_transform = np.array([
            [1, 0, 0],    # X_flu = X_frd (Forward)
            [0, -1, 0],   # Y_flu = -Y_frd (Left)
            [0, 0, -1]    # Z_flu = -Z_frd (Up)
        ])

    def create_covariance_matrix(self, diagonal_values):
        """6x6 공분산 행렬 생성"""
        cov = np.zeros((6, 6))
        np.fill_diagonal(cov, diagonal_values)
        return cov.flatten().tolist()

    def transform_vector(self, px4_vector, transform_matrix):
        """3D 벡터 변환"""
        return transform_matrix @ px4_vector

    def transform_orientation(self, px4_quaternion):
        """방향 변환: PX4(NED/FRD) → ROS(ENU/FLU)"""
        # PX4 quat (w,x,y,z) → scipy (x,y,z,w)
        q_scipy = [px4_quaternion[1], px4_quaternion[2], px4_quaternion[3], px4_quaternion[0]]
        
        # 회전 행렬 변환
        r_ned_frd = R.from_quat(q_scipy).as_matrix()
        R_enu_flu = self.world_transform @ r_ned_frd @ self.body_transform
        
        # quat 복원 (w>0 보장)
        q_enu = R.from_matrix(R_enu_flu).as_quat()
        if q_enu[3] < 0:
            q_enu = [-x for x in q_enu]
        return q_enu

    def listener_callback(self, msg: VehicleOdometry):
        """PX4 Odometry → ROS Odometry 변환"""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        # 위치 변환 (NED → ENU)
        position_ned = np.array(msg.position)
        position_enu = self.transform_vector(position_ned, self.world_transform)
        odom.pose.pose.position.x = float(position_enu[0])
        odom.pose.pose.position.y = float(position_enu[1])
        odom.pose.pose.position.z = float(position_enu[2])

        # 방향 변환
        q_enu = self.transform_orientation(msg.q)
        odom.pose.pose.orientation.x = q_enu[0]
        odom.pose.pose.orientation.y = q_enu[1]
        odom.pose.pose.orientation.z = q_enu[2]
        odom.pose.pose.orientation.w = q_enu[3]

        # 속도 변환
        velocity_ned = np.array(msg.velocity)
        if msg.velocity_frame == 1:  # NED
            velocity_enu = self.transform_vector(velocity_ned, self.world_transform)
        else:  # Body FRD
            velocity_enu = self.transform_vector(velocity_ned, self.body_transform)
        odom.twist.twist.linear.x = float(velocity_enu[0])
        odom.twist.twist.linear.y = float(velocity_enu[1])
        odom.twist.twist.linear.z = float(velocity_enu[2])

        # 각속도 변환 (FRD → FLU)
        av_frd = np.array(msg.angular_velocity)
        av_flu = self.transform_vector(av_frd, self.body_transform)
        odom.twist.twist.angular.x = float(av_flu[0])
        odom.twist.twist.angular.y = float(av_flu[1])
        odom.twist.twist.angular.z = float(av_flu[2])

        # 공분산
        pose_cov = self.create_covariance_matrix([0.01]*6)
        twist_cov = self.create_covariance_matrix([0.01]*6)
        odom.pose.covariance = pose_cov
        odom.twist.covariance = twist_cov

        # 퍼블리시
        self.publisher.publish(odom)

        # TF (odom → base_link)
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position.z
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)
    
    def listener_callback_pose(self, msg: PoseStamped):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        # PoseStamped는 이미 ENU
        odom.pose.pose.position.x = msg.pose.position.x
        odom.pose.pose.position.y = msg.pose.position.y
        odom.pose.pose.position.z = msg.pose.position.z

        odom.pose.pose.orientation = msg.pose.orientation

        # 속도 정보는 없으니 일단 0
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.0

        # 공분산은 기존 함수 재사용
        odom.pose.covariance = self.create_covariance_matrix([0.01]*6)
        odom.twist.covariance = self.create_covariance_matrix([0.01]*6)

        self.publisher.publish(odom)

        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position.z
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomConverter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
