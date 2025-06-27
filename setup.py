from setuptools import find_packages, setup

package_name = 'echo_slam_encoder_tf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        ],
    },
)
