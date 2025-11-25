# ==============================
# Author: sqplab
# Date: 2025-11-21
# Description: Sim 환경에서 물뿌리기 모사 
# Note: 실제 구현에선 워터펌프 GPIO 제어 코드로 대체
# ==============================

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity
from nav_msgs.msg import Odometry
import numpy as np
import time


class WaterSprayGazeboNode(Node):
    def __init__(self):
        super().__init__('water_spray_gazebo')
        
        self.spraying = False
        self.drone_position = None
        self.spawned_drops = []
        
        # Subscriber
        self.spray_sub = self.create_subscription(
            Bool, '/water_spray', self.spray_callback, 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Service clients
        self.spawn_client = self.create_client(
            SpawnEntity, '/world/walls/create')
        self.delete_client = self.create_client(
            DeleteEntity, '/world/walls/remove')
        
        # Wait for services
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service 대기 중...')
        
        self.timer = self.create_timer(0.2, self.spawn_drops)
        self.cleanup_timer = self.create_timer(1.0, self.cleanup_old_drops)
        
        self.drop_id = 0
        
        self.get_logger().info("Water Spray Gazebo 시작")
    
    def spray_callback(self, msg):
        self.spraying = msg.data
    
    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        self.drone_position = [pos.x, pos.y, pos.z]
    
    def spawn_drops(self):
        """물방울 생성"""
        if not self.spraying or self.drone_position is None:
            return
        
        # 5개의 물방울 동시 생성
        for _ in range(5):
            self.spawn_single_drop()
    
    def spawn_single_drop(self):
      drop_name = f"water_drop_{self.drop_id}"
      self.drop_id += 1

      # 드론 아래 원뿔형 랜덤 위치
      angle = np.random.uniform(0, 2 * np.pi)
      radius = np.random.uniform(0, 0.2)
      x = self.drone_position[0] + radius * np.cos(angle)
      y = self.drone_position[1] + radius * np.sin(angle)
      z = self.drone_position[2] - 0.3
      # SDF 정의
      sdf = f'''<?xml version="1.0"?>
      <sdf version="1.9">
        <model name="{drop_name}">
          <pose>{x} {y} {z} 0 0 0</pose>
          <link name="link">
            <collision name="collision">
              <geometry>
                <sphere>
                  <radius>0.03</radius>
                </sphere>
              </geometry>
            </collision>
            <visual name="visual">
              <geometry>
                <sphere>
                  <radius>0.03</radius>
                </sphere>
              </geometry>
              <material>
                <ambient>0.2 0.6 1.0 0.8</ambient>
                <diffuse>0.3 0.7 1.0 0.8</diffuse>
                <specular>1 1 1 1</specular>
              </material>
            </visual>
            <inertial>
              <mass>0.001</mass>
              <inertia>
                <ixx>0.00001</ixx>
                <iyy>0.00001</iyy>
                <izz>0.00001</izz>
              </inertia>
            </inertial>
          </link>
        </model>
      </sdf>
      '''

      req = SpawnEntity.Request()
      req.entity_factory.name = drop_name
      req.entity_factory.sdf = sdf
      req.entity_factory.allow_renaming = False  # 동일 이름이면 overwrite 금지
      req.entity_factory.relative_to = "world"
      req.entity_factory.pose.position.x = x
      req.entity_factory.pose.position.y = y
      req.entity_factory.pose.position.z = z
      req.entity_factory.pose.orientation.x = 0.0
      req.entity_factory.pose.orientation.y = 0.0
      req.entity_factory.pose.orientation.z = 0.0
      req.entity_factory.pose.orientation.w = 1.0

      future = self.spawn_client.call_async(req)
      self.spawned_drops.append({
          'name': drop_name,
          'time': time.time()
      })

    
    def cleanup_old_drops(self):
      current_time = time.time()
      drops_to_remove = []
      for drop in self.spawned_drops:
          if current_time - drop['time'] > 2.0:
              req = DeleteEntity.Request()
              req.name = drop['name']        # 직접 할당
              req.type = DeleteEntity.Request.MODEL  # 모델 삭제
              self.delete_client.call_async(req)
              drops_to_remove.append(drop)
      for drop in drops_to_remove:
          self.spawned_drops.remove(drop)


def main(args=None):
    rclpy.init(args=args)
    node = WaterSprayGazeboNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
