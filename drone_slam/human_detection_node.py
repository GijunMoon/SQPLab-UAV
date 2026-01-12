# ==============================
# Author: sqplab
# Date: 2026-01-09
# Description: 조난자 위치 탐지
# ==============================

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import numpy as np
import cv2
from ultralytics import YOLO

class HumanDetectionNode(Node):
    def __init__(self):
        super().__init__('human_detection_node')
        
        # YOLO 모델 (사용 안함, HSV만)
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info("YOLO 로드 완료")
        
        # 구독/발행
        self.subscription = self.create_subscription(
            Image, '/camera_down/image', self.image_callback, 10)
        self.detection_pub = self.create_publisher(Bool, '/human_detected', 10)
        
        # 파라미터
        self.suppress_duration = 5.0
        self.last_detect_time = None
        self.tolerance_x = 50.0
        self.tolerance_y = 200.0
        
        # 빨간색 범위 (HSV)
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])

    def image_callback(self, msg):
        try:
            # 1. 재탐지 억제 (도착 후만)
            if self.last_detect_time is not None:
                elapsed = (self.get_clock().now() - self.last_detect_time).nanoseconds / 1e9
                if elapsed < self.suppress_duration:
                    self.publish_result(False, 0, 0)
                    return

            # 2. 이미지 변환
            if msg.encoding == 'rgb8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'bgr8':
                img_bgr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                img = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
            else:
                return

            # 3. 빨간색 객체 탐지 (가장 큰 컨투어)
            hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
            mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
            mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
            mask = mask1 + mask2
            
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            best_cnt = None
            max_area = 0
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 1000 and area > max_area:
                    x, y, w, h = cv2.boundingRect(cnt)
                    aspect_ratio = float(w)/h
                    if 0.2 < aspect_ratio < 0.8:
                        best_cnt = cnt
                        max_area = area

            detected = False
            if best_cnt is not None:
                M = cv2.moments(best_cnt)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    # 중앙 오차 계산 (중앙 체크 제거!)
                    frame_center_x = img_bgr.shape[1] / 2
                    frame_center_y = img_bgr.shape[0] / 2
                    cx_error = cx - frame_center_x
                    cy_error = cy - frame_center_y
                    
                    
                    # 중앙 도착 여부로 detected 결정
                    detected = (abs(cx_error) < self.tolerance_x and abs(cy_error) < self.tolerance_y)
                    
                    if detected:
                        self.get_logger().error(f"탐지 성공 (error_x={cx_error:.1f}, error_y={cy_error:.1f})")
                        self.last_detect_time = self.get_clock().now()
                    else:
                        self.get_logger().info(f"조난자 위치: x_err={cx_error:.1f}, y_err={cy_error:.1f}")

            # 4. 결과 처리
            self.publish_result(detected, cx_error if 'cx_error' in locals() else 0, cy_error if 'cy_error' in locals() else 0)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def publish_result(self, status, cx_error=0, cy_error=0):
        msg = Bool()
        msg.data = status
        self.detection_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(HumanDetectionNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
