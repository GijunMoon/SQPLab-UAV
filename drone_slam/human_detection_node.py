# ==============================
# Author: sqplab (Red Box Patch)
# Date: 2026-01-09
# Description: YOLO + Red Color Detection
# ==============================

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import numpy as np
import cv2  # OpenCV 필수
from ultralytics import YOLO

class HumanDetectionNode(Node):
    def __init__(self):
        super().__init__('human_detection_node')
        
        # YOLO 모델
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info("YOLO 로드 완료")
        
        # 구독/발행
        self.subscription = self.create_subscription(
            Image, '/camera_down/image', self.image_callback, 10)
        self.detection_pub = self.create_publisher(Bool, '/human_detected', 10)
        
        # 파라미터
        self.yolo_conf = 0.75
        self.suppress_duration = 5.0
        self.last_detect_time = None
        
        # 빨간색 범위 (HSV)
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])

    def image_callback(self, msg):
        try:
            # 1. 재탐지 억제
            if self.last_detect_time is not None:
                elapsed = (self.get_clock().now() - self.last_detect_time).nanoseconds / 1e9
                if elapsed < self.suppress_duration:
                    self.publish_result(False)
                    return

            # 2. 이미지 변환
            if msg.encoding == 'rgb8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) # OpenCV용
            elif msg.encoding == 'bgr8':
                img_bgr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                img = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB) # YOLO용
            else:
                return

            detected = False
            detect_type = ""

            # 3. [방법 A] 빨간색 객체 탐지 (HSV)
            hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
            mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
            mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
            mask = mask1 + mask2
            
            # 노이즈 제거
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            # 컨투어 찾기
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 1000:  # 픽셀 크기 임계값 (박스 크기)
                    x, y, w, h = cv2.boundingRect(cnt)
                    aspect_ratio = float(w)/h
                    # 박스 형태 체크 (길쭉한 형태)
                    if 0.2 < aspect_ratio < 0.8: 
                        detected = True
                        detect_type = f"RED BOX (area={area:.0f})"
                        break

            # 5. 결과 처리
            if detected:
                self.get_logger().error(f"🚨 탐지 성공: {detect_type}")
                self.last_detect_time = self.get_clock().now()
                self.publish_result(True)
            else:
                self.publish_result(False)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def publish_result(self, status):
        msg = Bool()
        msg.data = status
        self.detection_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(HumanDetectionNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
