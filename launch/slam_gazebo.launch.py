from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('echo_slam_encoder_tf')

    robot_urdf = os.path.join(pkg_dir, 'urdf', 'robot.xacro')

    return LaunchDescription([

        # Gazebo 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            )
        ),

        # 로봇 모델 스폰
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[robot_urdf],
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'my_robot', '-file', robot_urdf],
            output='screen'
        ),

        # 컨트롤러 매니저 실행
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller', '--param-file', os.path.join(pkg_dir, 'config', 'diff_drive_controller.yaml')],
            output='screen'
        ),

        # encoder_counts 발행 노드
        Node(
            package='echo_slam_encoder_tf',
            executable='slam_encoder_gazebo_node',
            name='slam_encoder_gazebo_node',
            output='screen'
        ),

        # 기존 odom 노드
        Node(
            package='echo_slam_encoder_tf',
            executable='slam_encoder_odom_node',
            name='slam_encoder_odom_node',
            output='screen'
        ),

        # SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
            )
        ),
    ])
