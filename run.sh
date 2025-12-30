#!/bin/bash

set -e

echo "=== SQPLab SITL 런처 ==="
echo "현재 시간: $(date)"

# 디렉토리 설정
PX4_DIR="$HOME/PX4-Autopilot"
ROS2_WS="$HOME/ws_ros2"
DRONE_SLAM_DIR="$HOME/ws_ros2/src/drone_slam/drone_slam"
ROS_ENV="$HOME/ros_env"

LOG_DIR="$HOME/slam_logs_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"

# 색상
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'

# 🔥 GUI 환경 필수 설정
export DISPLAY=:0
export QT_QPA_PLATFORM=xcb
xhost +local: 2>/dev/null || true
GZ_WEB=false

echo -e "${GREEN}✅ GUI 환경 설정 완료: DISPLAY=$DISPLAY${NC}"

# 1. 자기 사용자 프로세스만 kill (안전)
pkill -u $USER -f gzserver 2>/dev/null || true
pkill -u $USER -f gzclient 2>/dev/null || true
pkill -u $USER -f px4_sitl 2>/dev/null || true
pkill -u $USER -f MicroXRCEAgent 2>/dev/null || true
pkill -u $USER -f "rl_node" 2>/dev/null || true
pkill -u $USER -f "offboard_mission" 2>/dev/null || true

# 2. ROS2 노드 강제 종료
source ~/ws_ros2/install/setup.bash 2>/dev/null || true
ros2 node list | grep -E "(slam_toolbox|offboard|rl)" | xargs -r ros2 lifecycle set / shutdown 2>/dev/null || true

# 3. Gazebo 프로세스 강제 kill (SIGTERM 우선)
for pid in $(pgrep -f gzserver); do
    if [ "$(ps -o user= -p $pid)" = "$USER" ]; then
        kill $pid 2>/dev/null || true
    fi
done

sleep 2

# 1. PX4-Gazebo (GUI 확실)
echo -e "${YELLOW}=== 1/7 PX4-Gazebo (GUI) ===${NC}"
(
    cd "$PX4_DIR" || { echo -e "${RED}PX4 오류${NC}"; exit 1; }
    GZ_WEB=false PX4_GZ_WORLD=walls make px4_sitl gz_x500_lidar_2d > "$LOG_DIR/px4.log" 2>&1
) &
PX4_PID=$!
echo -e "${GREEN}PX4 PID: $PX4_PID${NC}"
sleep 15  # Takeoff! 대기

# GUI 수동 실행 (백그라운드)
gzclient &> "$LOG_DIR/gzclient.log" &
GZCLIENT_PID=$!

# 2. DDS
echo -e "${YELLOW}=== 2/7 MicroXRCEAgent ===${NC}"
MicroXRCEAgent udp4 -p 8888 > "$LOG_DIR/dds.log" 2>&1 &
DDS_PID=$!
echo -e "${GREEN}DDS PID: $DDS_PID${NC}"
sleep 3

# 3. SLAM
echo -e "${YELLOW}=== 3/7 ROS2 SLAM ===${NC}"
(
    source "$ROS2_WS/install/setup.bash"
    ros2 launch drone_slam slam.launch.py > "$LOG_DIR/slam.log" 2>&1
) &
SLAM_PID=$!
echo -e "${GREEN}SLAM PID: $SLAM_PID${NC}"
sleep 10

# SLAM 자동 활성화
source "$ROS2_WS/install/setup.bash"
if ros2 lifecycle get /slam_toolbox 2>/dev/null | grep -q "inactive"; then
    echo "SLAM 활성화..."
    ros2 lifecycle set /slam_toolbox activate
    sleep 3
fi

# 4. MAVROS
echo -e "${YELLOW}=== 4/7 MAVROS ===${NC}"
(
    source "$ROS2_WS/install/setup.bash"
    ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14580 > "$LOG_DIR/mavros.log" 2>&1
) &
MAVROS_PID=$!
echo -e "${GREEN}MAVROS PID: $MAVROS_PID${NC}"
sleep 5

# 5. Odometry 변환 (필수!)
echo -e "${YELLOW}=== 5/7 Odom Converter ===${NC}"
(
    source "$ROS2_WS/install/setup.bash"
    python3 "$ROS2_WS/src/drone_slam/odom_converter.py" > "$LOG_DIR/odom.log" 2>&1
) &
ODOM_PID=$!
echo -e "${GREEN}Odom PID: $ODOM_PID${NC}"
sleep 2

# 6. RL 노드 (핵심!)
echo -e "${YELLOW}=== 6/7 RL 제어 (SAC) ===${NC}"
(
    source "$ROS_ENV/bin/activate" 2>/dev/null || true
    source "$ROS2_WS/install/setup.bash"
    cd "$DRONE_SLAM_DIR"
    python3 rl_node.py > "$LOG_DIR/rl.log" 2>&1
) &
RL_PID=$!
echo -e "${GREEN}RL PID: $RL_PID${NC}"
sleep 3

# 7. Offboard Mission
echo -e "${YELLOW}=== 7/7 Offboard Mission ===${NC}"
(
    source "$ROS_ENV/bin/activate" 2>/dev/null || true
    source "$ROS2_WS/install/setup.bash"
    cd "$DRONE_SLAM_DIR"
    python3 offboard_mission.py > "$LOG_DIR/offboard.log" 2>&1
) &
CONTROL_PID=$!
echo -e "${GREEN}Control PID: $CONTROL_PID${NC}"

# 🔥 OFFBOARD 자동화 (핵심!)
sleep 5
source "$ROS2_WS/install/setup.bash"
echo -e "${YELLOW}🔧 OFFBOARD 자동 설정...${NC}"
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

echo -e "${GREEN}🚀 === 모든 프로세스 실행 완료! ===${NC}"
echo "📁 로그: $LOG_DIR"
echo "🔢 PID: PX4($PX4_PID) DDS($DDS_PID) SLAM($SLAM_PID) MAVROS($MAVROS_PID) ODOM($ODOM_PID) RL($RL_PID) CTRL($CONTROL_PID)"

# 종료 트랩
trap "echo -e '\n${RED}종료 중...${NC}'; kill $PX4_PID $DDS_PID $SLAM_PID $MAVROS_PID $ODOM_PID $RL_PID $CONTROL_PID $GZCLIENT_PID 2>/dev/null; exit" INT TERM

wait
