from setuptools import setup
import os
from glob import glob

package_name = 'ball_follower_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.com',
    description='Autonomous ball-following robot with ultrasonic mapping',
    license='MIT',
    entry_points={
        'console_scripts': [
            'object_tracker = ball_follower_robot.object_tracker:main',
            'motor_controller = ball_follower_robot.motor_controller:main',
            'ultrasonic_mapper = ball_follower_robot.ultrasonic_mapper:main',
            'path_tracker = ball_follower_robot.path_tracker:main',
            'serial_bridge = ball_follower_robot.serial_bridge:main',
        ],
    },
)
