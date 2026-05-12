# launch/ros2controllReal.launch.py  —  v9-MapFix
#
# v9 changes (from v8-MapStable):
#
#   [FIX MAP TILT - KEY]
#       Cartographer start delay: t=35s → t=45s
#       เหตุผล: ให้ EKF stable (running 39s) ก่อน Cartographer เริ่ม
#               imu0_relative=false (แก้ใน ekf yaml) ต้องการ IMU warm up
#               อย่างน้อย ~30-40s เพื่อให้ absolute yaw stable
#               Cartographer ที่ start เร็วเกินไปจะ bake yaw ที่ยังไม่ stable
#               → แผนที่เอียง
#
#   [FIX MAP TILT]
#       reset_imu_yaw: t=5s → t=8s
#       เหตุผล: ให้ IMU node (imu_gyro_integrator) พร้อมสมบูรณ์ก่อน reset
#               t=4s start imu_gyro_integrator → t=8s reset (4s gap พอเพียง)
#
#   [KEEP v8]
#       published_frame="odom" ใน lua → Cartographer publish map→odom
#       EKF world_frame=odom → publish odom→base_footprint
#       ไม่มี tf conflict
#       ไม่มี odom_drift_corrector
#
# Timing (v9):
#   t=0s   robot_state_publisher, serial_bridge
#   t=1s   controller_manager
#   t=3s   spawners, joy, twist_mux
#   t=4s   lidar, imu_gyro_integrator
#   t=8s   reset imu yaw  ← เพิ่มจาก t=5s
#   t=9s   ekf_filter_node
#   t=45s  cartographer, occupancy_grid  ← เพิ่มจาก t=35s
#   t=46s  rviz
# ─────────────────────────────────────────────────────────────────

import os
import subprocess
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('joy_control')

    urdf_file               = os.path.join(pkg_share, 'urdf', 'robotNew.urdf.xacro')
    rviz_file               = os.path.join(pkg_share, 'rviz', 'slam_config.rviz')
    cartographer_config_dir = os.path.join(pkg_share, 'config')
    configuration_basename  = 'gbot_lidar_2d.lua'
    controllers_yaml        = os.path.join(pkg_share, 'config', 'ros2_controllersReal.yaml')
    twist_mux_yaml          = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    ekf_yaml                = os.path.join(pkg_share, 'config', 'ekf_odom_imu.yaml')

    robot_desc = subprocess.check_output(
        ['xacro', urdf_file, 'sim_mode:=false']
    ).decode('utf-8')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}]
    )

    serial_bridge = Node(
        package='serial_bridge',
        executable='serial_bridge_node',
        name='serial_bridge',
        parameters=[{
            'port': '/dev/ttyACM0',
            'baud': 115200,
        }],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_desc}, controllers_yaml],
        output='screen'
    )

    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster']
    )

    spawn_diff = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller']
    )
    keyboard_teleop = Node(
        package='joy_control',
        executable='keyboard_teleop',
        name='keyboard_teleop',
        parameters=[{
            'linear_vel':   0.5,
            'angular_vel':  1.2,
            'publish_rate': 20.0,
            'key_timeout':  0.08,
            'cmd_topic':    '/cmd_vel_keyboard',
        }],
        prefix='xterm -e',
        output='screen'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id':       0,
            'deadzone':        0.05,
            'autorepeat_rate': 20.0,
        }],
        output='screen'
    )

    ps5_converter = Node(
        package='joy_control',
        executable='joy_ps5_converter',
        name='joy_ps5_converter',
        output='screen'
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_yaml],
        remappings=[('cmd_vel_out', '/diff_drive_controller/cmd_vel_unstamped')]
    )

    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        parameters=[{
            'serial_port':      '/dev/ttyUSB0',
            'serial_baudrate':  115200,
            'frame_id':         'laser_frame',
            'angle_compensate': True,
            'scan_mode':        'Sensitivity',
        }],
        output='screen'
    )

    imu_gyro_integrator = Node(
        package='esp32_hardware',
        executable='imu_gyro_integrator',
        name='imu_gyro_integrator',
        output='screen'
    )

    ekf_filter = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[
            ekf_yaml,
            {'use_sim_time': False}
        ],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
            ('odom0', '/diff_drive_controller/odom'),
            ('imu0', '/imu/data'),
        ],
        output='screen'
    )

    # [FIX v9] เลื่อนจาก t=5s → t=8s
    # ให้ imu_gyro_integrator (start t=4s) พร้อมสมบูรณ์ก่อน reset
    # gap 4s เพียงพอสำหรับ node initialization + IMU data flow stable
    reset_imu_yaw = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/imu_integrator/reset',
            'std_srvs/srv/Empty', '{}'
        ],
        output='screen'
    )

    # [v8] Cartographer publish map→odom (published_frame="odom" ใน lua)
    # remapping odom → /odometry/filtered (EKF output)
    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename',  configuration_basename,
        ],
        parameters=[{'use_sim_time': False}],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odometry/filtered'),
            ('imu',  '/imu/data'),
        ],
        output='screen'
    )

    occupancy_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        arguments=['-resolution', '0.03']
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file]
    )

    return LaunchDescription([
        robot_state_publisher,
        serial_bridge,

        TimerAction(period=1.0,  actions=[controller_manager]),

        TimerAction(period=3.0,  actions=[
            spawn_jsb,
            spawn_diff,
            keyboard_teleop,
            joy_node,
            ps5_converter,
            twist_mux,
        ]),

        TimerAction(period=4.0,  actions=[lidar_node, imu_gyro_integrator]),

        # [FIX v9] reset imu yaw ที่ t=8s (จาก t=5s)
        # ให้ IMU node พร้อมก่อน reset → yaw reference ถูกต้อง
        TimerAction(period=8.0,  actions=[reset_imu_yaw]),

        # [FIX v9] EKF start ที่ t=9s (จาก t=6s) หลัง reset imu yaw 1s
        TimerAction(period=9.0,  actions=[ekf_filter]),

        # [FIX v9] Cartographer start ที่ t=45s (จาก t=35s)
        # ให้ EKF run 36s → IMU absolute yaw stable ก่อน Cartographer bake pose
        # แก้ปัญหาแผนที่เอียง
        TimerAction(period=45.0, actions=[cartographer, occupancy_grid]),
        TimerAction(period=46.0, actions=[rviz]),
    ])