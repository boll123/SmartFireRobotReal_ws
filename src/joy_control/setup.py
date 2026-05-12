from setuptools import setup
from glob import glob
import os

package_name = 'joy_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),  glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),  glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'),  glob('config/*.lua')),
        (os.path.join('share', package_name, 'urdf'),    glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'urdf'),    glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'meshes'),  glob('meshes/*.dae')),
        (os.path.join('share', package_name, 'meshes'),  glob('meshes/*.STL')),
        (os.path.join('share', package_name, 'rviz'),    glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'worlds'),  glob('worlds/*.world')),
        (os.path.join('share', package_name, 'maps'),    glob('maps/*.yaml')),
        (os.path.join('share', package_name, 'maps'),    glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Complete robot control with micro-ROS, URDF, and SLAM',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mqttsub           = joy_control.mqttsub:main',
            'joy_ps5_converter = joy_control.joy_ps5_converter:main',
            'esp32_simulator   = joy_control.esp32_simulator_node:main',
            'keyboard_teleop = joy_control.keyboard_teleop_node:main',
            'pose_saver_node = joy_control.pose_saver_node:main',
            'goal_nav_node     = joy_control.goal_nav_node:main',
            'xy_publisher_node = joy_control.xy_publisher_node:main',
        ],
    },
)