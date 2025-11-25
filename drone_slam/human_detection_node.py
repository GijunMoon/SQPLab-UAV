# ==============================
# Author: sqplab
# Date: 2025-11-21
# Description: YOLO 사람 감지 노드
# ==============================

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import numpy as np
import os

from ultralytics import YOLO
import time


class HumanDetectionNode(Node):
    def __init__(self):
        super().__init__('human_detection_node')
        
        # YOLO 모델 로드
        model_path = 'yolov8n.pt'
        """model_path = os.path.expanduser('~/ws_ros2/src/drone_slam/drone_slam/best.pt')
        if not os.path.exists(model_path):
            model_path = 'yolov8n.pt'"""
        
        self.model = YOLO(model_path)
        self.get_logger().info(f"YOLO 모델 로드: {model_path}")
        
        # 카메라 구독
        self.subscription = self.create_subscription(
            Image, '/camera_down/image', self.image_callback, 10)
        
        # 감지 결과 발행
        self.detection_pub = self.create_publisher(Bool, '/human_detected', 10)
        
        self.detection_confidence = 0.5
        self.person_class_id = 0

        # 마지막 사람 감지 시각
        self.last_human_detect_time = None
        # 재탐지 금지 시간
        self.suppress_duration = 60.0
        
    def image_callback(self, msg):
        """ROS Image를 직접 NumPy 배열로 변환"""
        try:
            if self.last_human_detect_time is not None:
                elapsed = (self.get_clock().now() - self.last_human_detect_time).nanoseconds / 1e9
                if elapsed < self.suppress_duration:
                    # 재탐지 금지 시간 내
                    detection_msg = Bool()
                    detection_msg.data = False
                    self.detection_pub.publish(detection_msg)
                    return
            # Image 메시지를 NumPy 배열로 직접 변환
            if msg.encoding == 'rgb8':
                dtype = np.uint8
                channels = 3
            elif msg.encoding == 'bgr8':
                dtype = np.uint8
                channels = 3
            else:
                self.get_logger().warn(f"지원하지 않는 인코딩: {msg.encoding}")
                return
            
            # NumPy 배열 생성
            img_array = np.frombuffer(msg.data, dtype=dtype)
            img_array = img_array.reshape((msg.height, msg.width, channels))
            
            # BGR → RGB 변환 (YOLO는 RGB 입력)
            if msg.encoding == 'bgr8':
                img_array = img_array[:, :, ::-1]
            
            # YOLO 추론
            results = self.model(img_array, verbose=False)
            
            # 사람 감지
            human_detected = False
            for result in results:
                for box in result.boxes:
                    if int(box.cls) == self.person_class_id and \
                       float(box.conf) >= self.detection_confidence:
                        human_detected = True
                        
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        conf = float(box.conf)
                        self.last_human_detect_time = self.get_clock().now()
                        
                        break
                if human_detected:
                    break
            
            # 결과 발행
            detection_msg = Bool()
            detection_msg.data = human_detected
            self.detection_pub.publish(detection_msg)
            
        except Exception as e:
            self.get_logger().error(f"이미지 처리 오류: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = HumanDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
