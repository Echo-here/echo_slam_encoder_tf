from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    package_name='echo_slam_encoder_tf'

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

    slam_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"),
                "launch",
                "bringup_launch.py"
            ])
        ]),
        launch_arguments={
            "use_sim_time": "True",
            "slam": "True",
            "params_file": PathJoinSubstitution([
                FindPackageShare(package_name),
                "config",
                "slam_nav2.yaml"
            ])
        }.items()
    )

    uart_node = Node(
        package='echo_slam_encoder_tf',     # 실제 패키지 이름 (노드가 속한 곳)
        executable='cmd_val_to_uart_node',       # 설치된 실행 파일 이름
        name='cmd_val_to_uart_node',
        output='screen',
        parameters=[
            {"port": "/dev/ttyUSB0"},       # 기본값
            {"baudrate": 115200}
        ]
    )

    return LaunchDescription([
        slam_toolbox_launch,
        slam_nav2,
        ydlidar_launch,
        odom_node,
        reader_node,
        uart_node
    ])
