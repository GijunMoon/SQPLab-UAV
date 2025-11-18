import os
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=False),

        # 3) Lidar bridge
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type':'serial',
                         'serial_port': '/dev/ttyUSB0', 
                         'serial_baudrate': 256000, 
                         'frame_id': 'laser',
                         'scan_mode': 'Sensitivity'}],
            output='screen'),
            
        Node(
            package='mavros',
            executable='mavros_node',
            parameters=[{'fcu_url':'serial:///dev/ttyACM0:921600', }],
            output='screen'),
            
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.26', '0', '0', '0', 'odom', 'base_link'],
            output='screen'
        ),


        # 6) SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[os.path.expanduser('~/ws_ros2/src/SQPLab-UAV/config/slam_params.yaml')],
            output='screen'
        ),

        # 7) Configure SLAM Toolbox
        ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
            output='screen'
        ),

        # 8) Activate SLAM Toolbox
        ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
            output='screen'
        ),

        # 9) RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),

        ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
            output='screen'
        ),

        # 10) RL Node
        ExecuteProcess(
            cmd=['python3', os.path.expanduser('~/ws_ros2/src/SQPLab-UAV/drone_slam/rl_node.py')],
            output='screen'
        ),
        
        # 11) RC Node
        ExecuteProcess(
            cmd=['python3', os.path.expanduser('~/ws_ros2/src/SQPLab-UAV/drone_slam/rc_trigger.py')],
            output='screen'
        ),
    ])
