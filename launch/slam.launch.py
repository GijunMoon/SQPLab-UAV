import os
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('drone_slam')

    return LaunchDescription([
        # 1) sim time
        SetParameter(name='use_sim_time', value=True),

        # 2) Clock bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'
        ),

        # 3) Lidar bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='lidar_bridge',
            arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
            output='screen'
        ),

        # 4) Camera bridge
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

        # 6) Odom converter (Python 파일 직접 실행)
        ExecuteProcess(
            cmd=['python3', os.path.expanduser(
                '~/ws_ros2/src/drone_slam/drone_slam/odom_converter.py'
            )],
            output='screen'
        ),

        # 7) Static TF: base_link -> lidar_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar_tf',
            arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'lidar_link'],
            output='screen'
        ),

        # 8) Static TF: base_link -> camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=['0', '0', '-0.10', '0', '1.5707963', '0',
                       'base_link', 'camera_link'],
            output='screen'
        ),

        # 9) SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                os.path.expanduser('~/ws_ros2/src/drone_slam/config/slam_params.yaml')
            ],
            output='screen'
        ),

        # 10) SLAM lifecycle
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set',
                         '/slam_toolbox', 'configure'],
                    output='screen'
                )
            ]
        ),
        TimerAction(
            period=4.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set',
                         '/slam_toolbox', 'activate'],
                    output='screen'
                )
            ]
        ),

        # 11) Human detection
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['python3', os.path.expanduser(
                        '~/ws_ros2/src/drone_slam/drone_slam/human_detection_node.py'
                    )],
                    output='screen'
                )
            ]
        ),

        # 12) RL node
        TimerAction(
            period=6.0,
            actions=[
                ExecuteProcess(
                    cmd=['python3', os.path.expanduser(
                        '~/ws_ros2/src/drone_slam/drone_slam/rl_node.py'
                    )],
                    output='screen'
                )
            ]
        ),

        # 13) Rescue controller
        TimerAction(
            period=7.0,
            actions=[
                ExecuteProcess(
                    cmd=['python3', os.path.expanduser(
                        '~/ws_ros2/src/drone_slam/drone_slam/rescue_controller_node.py'
                    )],
                    output='screen'
                )
            ]
        ),

        # 14) Water spray node
        TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                    cmd=['python3', os.path.expanduser(
                        '~/ws_ros2/src/drone_slam/drone_slam/water_spray_gazebo.py'
                    )],
                    output='screen'
                )
            ]
        ),

        # 15) RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'drone_view.rviz')],
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])
