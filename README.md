# 강화학습 기반의 무인기 제어

PX4 + ROS2 + Gym 을 활용한 강화학습 기반 무인기 장애물 회피 패키지.

### Important:
Copy the content of the world folder inside: PX4-Autopilot/Tools/simulation/gz/worlds


패키지 구성:
- 시뮬레이션, 브리지, SLAM 노드, RViz2 시각화를 실행하는 런치 파일 (slam.launch.py)
- PX4 오도메트리를 ROS2에서 쓸 수 있도록 변환하는 스크립트 (odom_converter.py)
- 빌드를 위한 설정 파일
- SLAM Toolbox 설정 파일 (예: slam_params.yaml)
- 드론 제어용 SAC 강화학습 알고리즘

## Prerequisites

- **Operating System:** Ubuntu 24.04 LTS.
- **ROS2 Version:** Jazzy Jalisco.
- **PX4 Autopilot:** Latest stable version (v1.14 or higher recommended). Set up PX4 SITL (Software-In-The-Loop) with Gazebo support.
- **Gazebo Version:** Harmonic 8.9.0 (ensure it's installed via ROS2 integration, e.g., `ros-jazzy-ros-gz`).
- Python 3.12+ (comes with Ubuntu 24.04).

Additional tools:
- `colcon` for building ROS2 workspaces.
- `rosdep` for installing system dependencies.


