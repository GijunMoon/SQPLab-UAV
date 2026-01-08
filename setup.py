import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'drone_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), 
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'worlds'), 
            glob(os.path.join('worlds', '*.sdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gijun Moon',
    maintainer_email='moongijun@gnu.ac.kr',
    description='A package to run Lidar SLAM on a PX4 drone in Gazebo.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Odom 변환 (Python 우선)
            'odom_converter_py = drone_slam.odom_converter:main',
            
            # 핵심 제어 노드들
            'human_detection_node = drone_slam.human_detection_node:main',
            'rl_node = drone_slam.rl_node:main',
            'rescue_controller_node = drone_slam.rescue_controller_node:main',
            'offboard_mission = drone_slam.offboard_mission:main',
            'water_spray_gazebo = drone_slam.water_spray_gazebo:main',
            'vfh_override = drone_slam.vfh_node:main',
            
            # 상태 관리자
            'state_manager = drone_slam.state_manager:main',
        ],
    },
)
