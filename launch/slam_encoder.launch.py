from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1. SLAM Toolbox async launch 포함
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        )
    )

    # 2. YDLIDAR 드라이버 런치 포함
    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ydlidar_ros2_driver'),
                'launch',
                'ydlidar_launch.py'
            )
        )
    )

    # 3. 내가 만든 노드 2개 실행 (패키지 이름: my_robot_nodes)
    odom_node = Node(
        package='echo_slam_encoder_tf',
        executable='slam_encoder_odom_node',
        name='slam_encoder_odom_node',
        output='screen'
    )

    reader_node = Node(
        package='echo_slam_encoder_tf',
        executable='slam_encoder_reader_node',
        name='slam_encoder_reader_node',
        output='screen'
    )

    return LaunchDescription([
        slam_toolbox_launch,
        ydlidar_launch,
        odom_node,
        reader_node
    ])
