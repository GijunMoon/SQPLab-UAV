import os
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('drone_slam')

    return LaunchDescription([
        # 1) sim time (전역 설정)
        SetParameter(name='use_sim_time', value=True),

        # 2) Clock bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'
        ),

        # 3) Lidar bridge (VFH용)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='lidar_bridge',
            arguments=[
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '--ros-args', '--param', 'qos_overrides./scan/reliability:=reliable'
            ],
            output='screen'
        ),

        # 4) Camera bridge (YOLO용)
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            name='camera_bridge',
            arguments=['camera_down/image'],
            output='screen'
        ),

        # 5) Water spray service bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='service_bridge',
            arguments=[
                '/world/walls/create@ros_gz_interfaces/srv/SpawnEntity',
                '/world/walls/remove@ros_gz_interfaces/srv/DeleteEntity',
            ],
            output='screen'
        ),

        # 6) Odom converter (MAVROS → /odom 변환)
        ExecuteProcess(
            cmd=['python3', os.path.expanduser(
                '~/ws_ros2/src/drone_slam/drone_slam/odom_converter.py'
            )],
            output='screen'
        ),

        # 7) Static TF: base_link -> lidar_link (VFH용)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar_tf',
            arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'lidar_link'],
            output='screen'
        ),

        # 8) Static TF: base_link -> camera_link (YOLO용)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=['0', '0', '-0.10', '0', '1.5707963', '0', 'base_link', 'camera_link'],
            output='screen'
        ),

        # 9) RL node (3초 대기)
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['python3', os.path.expanduser(
                        '~/ws_ros2/src/drone_slam/drone_slam/rl_node.py'
                    )],
                    output='screen'
                )
            ]
        ),

        # 10) VFH override node (2.0초 대기)
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=['python3', os.path.expanduser(
                        '~/ws_ros2/src/drone_slam/drone_slam/vfh_node.py'
                    )],
                    output='screen'
                )
            ]
        ),

        # 11) Human detection (4초 대기)
        TimerAction(
            period=4.0,
            actions=[
                ExecuteProcess(
                    cmd=['python3', os.path.expanduser(
                        '~/ws_ros2/src/drone_slam/drone_slam/human_detection_node.py'
                    )],
                    output='screen'
                )
            ]
        ),

        # 12) Rescue controller (4.5초 대기)
        TimerAction(
            period=4.5,
            actions=[
                ExecuteProcess(
                    cmd=['python3', os.path.expanduser(
                        '~/ws_ros2/src/drone_slam/drone_slam/rescue_controller_node.py'
                    )],
                    output='screen'
                )
            ]
        ),

        # 13) Water spray node (5초 대기)
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['python3', os.path.expanduser(
                        '~/ws_ros2/src/drone_slam/drone_slam/water_spray_gazebo.py'
                    )],
                    output='screen'
                )
            ]
        ),

        # 14) RViz2 (odom 기반 시각화)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'drone_view.rviz')],
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])
