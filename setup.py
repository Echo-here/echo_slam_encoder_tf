from setuptools import find_packages, setup
from glob import glob

package_name = 'echo_slam_encoder_tf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bitbyte08',
    maintainer_email='bitbyte08@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slam_encoder_reader_node = echo_slam_encoder_tf.slam_encoder_reader_node:main',
            'slam_encoder_odom_node = echo_slam_encoder_tf.slam_encoder_odom_node:main',
            'slam_encoder_dummy_node = echo_slam_encoder_tf.slam_encoder_dummy_node:main',
            'slam_encoder_gazebo_node = echo_slam_encoder_tf.slam_encoder_gazebo_node:main',
            'slam_mqtt_nav2_goal_node = echo_slam_encoder_tf.slam_mqtt_nav2_goal_node:main' 
        ],
    },
)
