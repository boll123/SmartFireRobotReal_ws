# launch/ros2controllSim2.launch.py  —  v3-GazeboSLAM
#
# Gazebo 11 Simulation สำหรับทำ SLAM Map
# อิงจาก ros2controllSim2.launch.py (v2-GazeboSLAM)
#
# v3 changes (from v2-GazeboSLAM):
#   - เพิ่ม clear_ekf_state (ExecuteProcess) → ล้าง /amcl initial_pose param เก่า
#     ก่อน Gazebo start → หุ่นกลับ origin ทุกครั้งที่รัน launch ใหม่
#
# สิ่งที่ต่างจาก real robot:
#   - ใช้ Gazebo 11 แทน real hardware
#   - sim_mode:=true → ใช้ GazeboSystem + gazebo_ros_imu_sensor
#   - ไม่มี serial_bridge (ไม่มี /dev/ttyACM0)
#   - ไม่มี imu_gyro_integrator (Gazebo publish /imu/data_raw แทน)
#   - เพิ่ม imu_filter_madgwick แปลง /imu/data_raw → /imu/data
#   - ไม่มี reset_imu_yaw service (ไม่จำเป็น ใน sim yaw เริ่มต้นที่ 0)
#   - world: worlds/robot_world.world
#
# Timing (Sim):
#   t=0s   clear_ekf_state (ล้าง AMCL pose param เก่า)
#   t=0s   gazebo (gzserver + gzclient)
#   t=5s   spawn_entity (URDF → Gazebo)
#   t=6s   robot_state_publisher
#   t=7s   controller_manager (ros2_control_node)
#   t=9s   spawners, joy, twist_mux
#   t=10s  lidar (Gazebo จัดการ), imu_filter_madgwick
#   t=12s  ekf_filter_node
#   t=50s  cartographer, occupancy_grid
#   t=51s  rviz
#
# หมายเหตุ:
#   - Gazebo ใช้ /clock → use_sim_time: true ทุก node
#   - cartographer delay 50s (Sim) = รอ EKF stable 38s
#     เทียบเท่า real robot 45s (EKF stable 36s)
# ─────────────────────────────────────────────────────────────────

import os
import subprocess

from launch import LaunchDescription
from launch.actions import (
    TimerAction,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share     = get_package_share_directory('joy_control')
    gazebo_ros    = get_package_share_directory('gazebo_ros')

    urdf_file               = os.path.join(pkg_share, 'urdf', 'robotimuSim.urdf.xacro')
    rviz_file               = os.path.join(pkg_share, 'rviz', 'slam_config.rviz')
    cartographer_config_dir = os.path.join(pkg_share, 'config')
    configuration_basename  = 'gbot_lidar_2d.lua'

    # [v2] ชี้ไปที่ sim-specific yaml files
    controllers_yaml = os.path.join(pkg_share, 'config', 'ros2_controllersSim.yaml')
    ekf_yaml         = os.path.join(pkg_share, 'config', 'ekf_odom_imuSim.yaml')

    twist_mux_yaml   = os.path.join(pkg_share, 'config', 'twist_mux.yaml')

    # world file อยู่ใน package หรือ absolute path
    world_file = os.path.join(pkg_share, 'worlds', 'robot_world.world')

    # ── xacro: sim_mode=true ──────────────────────────────────────
    # → เปิด gazebo_ros2_control plugin + Gazebo IMU sensor
    # → plugin ใน URDF โหลด ros2_controllersSim.yaml โดยอัตโนมัติ
    # → ปิด ros2_control hardware (esp32) + ปิด imu_gyro_integrator
    robot_desc = subprocess.check_output(
        ['xacro', urdf_file, 'sim_mode:=true']
    ).decode('utf-8')

    # ═══════════════════════════════════════════════════════════════
    # [v3][FIX] ล้าง AMCL pose param เก่าก่อน launch
    # → ป้องกัน AMCL restore pose จากการรันก่อนหน้า
    # → ไม่สนใจ error ถ้า param ยังไม่มี (node อาจยังไม่ run)
    # ═══════════════════════════════════════════════════════════════
    clear_ekf_state = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'ros2 param delete /amcl initial_pose.x 2>/dev/null || true && '
            'ros2 param delete /amcl initial_pose.y 2>/dev/null || true && '
            'ros2 param delete /amcl initial_pose.yaw 2>/dev/null || true'
        ],
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════
    # [1] Gazebo 11  (gzserver + gzclient)
    # ═══════════════════════════════════════════════════════════════
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'false',
            'pause':   'false',
        }.items(),
    )

    # ═══════════════════════════════════════════════════════════════
    # [2] Spawn robot into Gazebo  (t=5s)
    # รอ Gazebo พร้อมก่อน spawn
    # ═══════════════════════════════════════════════════════════════
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'smartfire_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05',
            '-Y', '0.0',
        ],
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════
    # [3] robot_state_publisher  (t=6s)
    # ═══════════════════════════════════════════════════════════════
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════
    # [4] controller_manager  (t=7s)
    # [v2] controllers_yaml = ros2_controllersSim.yaml
    #      → use_sim_time อยู่ใน yaml แล้ว ไม่ต้องส่ง param แยก
    #      แต่ยังคงส่ง use_sim_time=True ไว้เพื่อความปลอดภัย
    # ═══════════════════════════════════════════════════════════════
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': True},
            controllers_yaml,
        ],
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════
    # [5] Spawners + Teleop + Joy  (t=9s)
    # ═══════════════════════════════════════════════════════════════
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    spawn_diff = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
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
            'use_sim_time': True,
        }],
        prefix='xterm -e',
        output='screen',
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id':       0,
            'deadzone':        0.05,
            'autorepeat_rate': 20.0,
            'use_sim_time':    True,
        }],
        output='screen',
    )

    ps5_converter = Node(
        package='joy_control',
        executable='joy_ps5_converter',
        name='joy_ps5_converter',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_yaml, {'use_sim_time': True}],
        remappings=[('cmd_vel_out', '/diff_drive_controller/cmd_vel_unstamped')],
    )

    # ═══════════════════════════════════════════════════════════════
    # [6] IMU Filter  (t=10s)
    # Gazebo publish /imu/data_raw (raw accelerometer+gyro)
    # imu_filter_madgwick แปลงเป็น /imu/data (มี orientation quaternion)
    # แทนที่ imu_gyro_integrator ของ real robot
    # ═══════════════════════════════════════════════════════════════
    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        parameters=[{
            'use_mag':          False,
            'publish_tf':       False,
            'world_frame':      'enu',
            'gain':             0.01,       # ค่าต่ำ → trust gyro มากกว่า accel
            'zeta':             0.0,
            'use_sim_time':     True,
        }],
        remappings=[
            ('imu/data_raw', '/imu/data_raw'),
            ('imu/data',     '/imu/data'),
        ],
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════
    # [7] EKF  (t=12s)
    # [v2] ekf_yaml = ekf_odom_imuSim.yaml
    #      → use_sim_time อยู่ใน yaml แล้ว
    #      แต่ยังคงส่ง use_sim_time=True ไว้เพื่อความปลอดภัย
    # ═══════════════════════════════════════════════════════════════
    ekf_filter = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[
            ekf_yaml,
            {'use_sim_time': True},
        ],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
            ('odom0', '/diff_drive_controller/odom'),
            ('imu0',  '/imu/data'),
        ],
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════
    # [8] Cartographer SLAM  (t=50s)
    # รอ EKF run 38s → heading stable
    # ═══════════════════════════════════════════════════════════════
    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename',  configuration_basename,
        ],
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odometry/filtered'),
            ('imu',  '/imu/data'),
        ],
        output='screen',
    )

    occupancy_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        arguments=['-resolution', '0.03'],
        parameters=[{'use_sim_time': True}],
    )

    # ═══════════════════════════════════════════════════════════════
    # [9] RViz2  (t=51s)
    # ═══════════════════════════════════════════════════════════════
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True}],
    )

    # ═══════════════════════════════════════════════════════════════
    # Launch Description
    # ═══════════════════════════════════════════════════════════════
    return LaunchDescription([
        # t=0s  [v3][FIX] ล้าง AMCL pose param เก่า → หุ่นกลับ origin ทุกครั้ง
        clear_ekf_state,

        # t=0s  Gazebo 11
        gazebo,

        # t=5s  Spawn robot (รอ Gazebo init ~5s)
        TimerAction(period=5.0,  actions=[spawn_entity]),

        # t=6s  robot_state_publisher (หลัง spawn entity)
        TimerAction(period=6.0,  actions=[robot_state_publisher]),

        # t=7s  controller_manager (ros2_controllersSim.yaml)
        TimerAction(period=7.0,  actions=[controller_manager]),

        # t=9s  spawners + teleop + joy + twist_mux
        TimerAction(period=9.0, actions=[
            spawn_jsb,
            spawn_diff,
            keyboard_teleop,
            joy_node,
            ps5_converter,
            twist_mux,
        ]),

        # t=10s  IMU filter (แทน imu_gyro_integrator)
        TimerAction(period=10.0, actions=[imu_filter]),

        # t=12s  EKF (ekf_odom_imuSim.yaml, หลัง IMU filter พร้อม 2s)
        TimerAction(period=12.0, actions=[ekf_filter]),

        # t=50s  Cartographer + OccupancyGrid
        TimerAction(period=50.0, actions=[cartographer, occupancy_grid]),

        # t=51s  RViz2
        TimerAction(period=51.0, actions=[rviz]),
    ])