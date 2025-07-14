from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('echo_slam_encoder_tf')

    # robot.xacro -> urdf로 파싱
    robot_description_content = Command(['xacro ', os.path.join(pkg_dir, 'urdf', 'robot.xacro')])
    robot_description = {'robot_description': robot_description_content}

    return LaunchDescription([
        # World 파일 인자 선언 (평면 world로 지정)
        DeclareLaunchArgument(
            name='world',
            default_value=os.path.join(pkg_dir, 'worlds', 'world.sdf'),  # 평면 world 파일명
            description='Gazebo world file'
        ),

        # Gazebo 설정 (world 포함)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': ["-r","-s","-v1",LaunchConfiguration('world')]}.items()
        ),

        # Gazebo GUI 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': ["-g"]}.items()
        ),

        # 로봇 상태 퍼블리셔
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': True}]
        ),
        
        # Gazebo에 로봇 스폰 (URDF 문자열 직접 전달)
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-name', 'my_robot',
                        '-string', robot_description_content,
                        '-z', '0.17'
                    ],
                    output='screen'
                )
            ]
        ),

        # 컨트롤러 매니저 실행
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'diff_drive_controller',
                '--param-file',
                os.path.join(pkg_dir, 'config', 'diff_drive_controller.yaml')
            ],
            output='screen'
        ),

        # encoder_counts 발행 노드
        Node(
            package='echo_slam_encoder_tf',
            executable='slam_encoder_gazebo_node',
            name='slam_encoder_gazebo_node',
            output='screen'
        ),

        # odom 노드
        Node(
            package='echo_slam_encoder_tf',
            executable='slam_encoder_odom_node',
            name='slam_encoder_odom_node',
            output='screen'
        ),

        # SLAM Toolbox 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
            )
        ),
    ])
