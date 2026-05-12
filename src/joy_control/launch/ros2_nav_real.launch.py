# launch/ros2_nav_real.launch.py  —  v6-FullFeature
# ─────────────────────────────────────────────────────────────────
# v6 changes (from v5-AutoPose):
#
#   [NEW] เพิ่ม Joy PS5 + Keyboard Teleop (จาก ros2controllReal1.launch.py)
#       - keyboard_teleop   → /cmd_vel_keyboard
#       - joy_node          → /joy
#       - ps5_converter     → แปลง joy → /cmd_vel_joy (ผ่าน twist_mux)
#
#   [NEW] เพิ่ม goal_nav_node + xy_publisher_node (จาก nav_Sim)
#       - start ที่ t=40s เหมือน Sim
#       - เหตุผล: รอ lifecycle_manager_navigation activate เสร็จ ~35-38s
#                 ถ้า start ก่อน → goal แรกหาย
#
#   [NEW] uncomment pose_saver_node
#       - save /amcl_pose → last_pose.yaml ทุก 5s
#       - start พร้อม rviz ที่ t=26s
#
#   [KEEP v5]
#       - Auto initial pose จาก last_pose.yaml
#       - velocity_smoother ถูกตัดออก (twist_mux รับ /cmd_vel_nav ตรง)
#
# Timing (v6):
#   t=0s   robot_state_publisher, serial_bridge
#   t=1s   controller_manager
#   t=3s   spawners, keyboard_teleop, joy_node, ps5_converter, twist_mux
#   t=4s   lidar, imu_gyro_integrator
#   t=8s   reset imu yaw
#   t=9s   ekf_filter_node
#   t=12s  map_server, amcl
#   t=16s  lifecycle_manager_localization
#   t=22s  nav2 servers (controller, smoother, planner, behavior, bt, waypoint)
#   t=24s  lifecycle_manager_navigation
#   t=26s  rviz2, pose_saver_node
#   t=40s  goal_nav_node, xy_publisher_node
# ─────────────────────────────────────────────────────────────────

import os
import sys
import math
import subprocess

import yaml

from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory


# ── ค่า default ถ้าไม่มีไฟล์ last_pose.yaml ──────────────────────
DEFAULT_POSE = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

# ── path ของไฟล์ที่ pose_saver_node จะ save ──────────────────────
LAST_POSE_FILE = os.path.expanduser(
    '~/SmartFireRobotReal_ws/src/joy_control/maps/last_pose.yaml'
)


def load_last_pose():
    """
    อ่าน last_pose.yaml ถ้ามี → return dict {x, y, yaw}
    ถ้าไม่มีหรืออ่านไม่ได้  → return DEFAULT_POSE
    """
    if not os.path.isfile(LAST_POSE_FILE):
        print(f'[AutoPose] ไม่พบ last_pose.yaml → ใช้ default pose (0,0,0)')
        return DEFAULT_POSE

    try:
        with open(LAST_POSE_FILE, 'r') as f:
            data = yaml.safe_load(f)

        pose = {
            'x':   float(data.get('x',   DEFAULT_POSE['x'])),
            'y':   float(data.get('y',   DEFAULT_POSE['y'])),
            'yaw': float(data.get('yaw', DEFAULT_POSE['yaw'])),
        }
        print(
            f'[AutoPose] โหลด last_pose.yaml: '
            f"x={pose['x']:.3f} y={pose['y']:.3f} "
            f"yaw={math.degrees(pose['yaw']):.1f}°"
        )
        return pose

    except Exception as e:
        print(f'[AutoPose] อ่าน last_pose.yaml ไม่ได้: {e} → ใช้ default')
        return DEFAULT_POSE


def generate_launch_description():
    pkg_share = get_package_share_directory('joy_control')

    # ── Config paths ──────────────────────────────────────────────
    urdf_file        = os.path.join(pkg_share, 'urdf',   'robotNew.urdf.xacro')
    rviz_file        = os.path.join(pkg_share, 'rviz',   'nav2_config.rviz')
    map_yaml         = os.path.join(pkg_share, 'maps',   'mapTest13.yaml')
    nav2_params_yaml = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    controllers_yaml = os.path.join(pkg_share, 'config', 'ros2_controllersReal.yaml')
    twist_mux_yaml   = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    ekf_yaml         = os.path.join(pkg_share, 'config', 'ekf_odom_imu.yaml')

    # ── Early check ───────────────────────────────────────────────
    for label, path in [
        ('nav2_params.yaml',          nav2_params_yaml),
        ('mapTest13.yaml',              map_yaml),
        ('ekf_odom_imu.yaml',         ekf_yaml),
        ('ros2_controllersReal.yaml', controllers_yaml),
    ]:
        if not os.path.isfile(path):
            print(f'\n[LAUNCH ERROR] ไม่พบไฟล์: {label}\n  path: {path}')
            sys.exit(1)

    # ── โหลด initial pose อัตโนมัติ ──────────────────────────────
    pose = load_last_pose()

    # ── Robot description ─────────────────────────────────────────
    robot_desc = subprocess.check_output(
        ['xacro', urdf_file, 'sim_mode:=false']
    ).decode('utf-8')

    # ═══════════════════════════════════════════════════════════════
    #  t=0s
    # ═══════════════════════════════════════════════════════════════
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': False}],
        output='screen',
    )

    serial_bridge = Node(
        package='serial_bridge',
        executable='serial_bridge_node',
        name='serial_bridge',
        parameters=[{'port': '/dev/ttyACM0', 'baud': 115200}],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    # ═══════════════════════════════════════════════════════════════
    #  t=1s
    # ═══════════════════════════════════════════════════════════════
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_desc}, controllers_yaml],
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════
    #  t=3s
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

    # [v6] เพิ่ม keyboard_teleop (จาก slam real)
    # keyboard_teleop = Node(
    #     package='joy_control',
    #     executable='keyboard_teleop',
    #     name='keyboard_teleop',
    #     parameters=[{
    #         'linear_vel':   0.5,
    #         'angular_vel':  1.2,
    #         'publish_rate': 20.0,
    #         'key_timeout':  0.08,
    #         'cmd_topic':    '/cmd_vel_keyboard',
    #     }],
    #     prefix='xterm -e',
    #     output='screen',
    # )

    # # [v6] เพิ่ม joy_node (จาก slam real)
    # joy_node = Node(
    #     package='joy',
    #     executable='joy_node',
    #     name='joy_node',
    #     parameters=[{
    #         'device_id':       0,
    #         'deadzone':        0.05,
    #         'autorepeat_rate': 20.0,
    #     }],
    #     output='screen',
    # )

    # # [v6] เพิ่ม ps5_converter (จาก slam real)
    # ps5_converter = Node(
    #     package='joy_control',
    #     executable='joy_ps5_converter',
    #     name='joy_ps5_converter',
    #     output='screen',
    # )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_yaml],
        remappings=[('cmd_vel_out', '/diff_drive_controller/cmd_vel_unstamped')],
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════
    #  t=4s
    # ═══════════════════════════════════════════════════════════════
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
        output='screen',
    )

    imu_gyro_integrator = Node(
        package='esp32_hardware',
        executable='imu_gyro_integrator',
        name='imu_gyro_integrator',
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════
    #  t=8s
    # ═══════════════════════════════════════════════════════════════
    reset_imu_yaw = ExecuteProcess(
        cmd=['ros2', 'service', 'call',
             '/imu_integrator/reset', 'std_srvs/srv/Empty', '{}'],
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════
    #  t=9s
    # ═══════════════════════════════════════════════════════════════
    ekf_filter = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_yaml, {'use_sim_time': False}],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
            ('odom0', '/diff_drive_controller/odom'),
            ('imu0',  '/imu/data'),
        ],
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════
    #  t=12s
    # ═══════════════════════════════════════════════════════════════
    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        parameters=[
            nav2_params_yaml,
            {
                'use_sim_time':  False,
                'yaml_filename': map_yaml,
            },
        ],
        output='screen',
    )

    amcl = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='',
        parameters=[
            nav2_params_yaml,
            {
                'use_sim_time':               False,
                'set_initial_pose':           True,
                'always_reset_initial_pose':  True,
                # ── โหลดจาก last_pose.yaml อัตโนมัติ ──
                'initial_pose.x':    pose['x'],
                'initial_pose.y':    pose['y'],
                'initial_pose.z':    0.0,
                'initial_pose.yaw':  pose['yaw'],
            },
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odometry/filtered'),
        ],
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════
    #  t=16s
    # ═══════════════════════════════════════════════════════════════
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[{
            'use_sim_time': False,
            'autostart':    True,
            'node_names':   ['map_server', 'amcl'],
            'bond_timeout': 4.0,
        }],
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════
    #  t=22s
    # ═══════════════════════════════════════════════════════════════
    controller_server = LifecycleNode(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace='',
        parameters=[
            nav2_params_yaml,
            {
                'use_sim_time': False,
                'controller_plugins': ['FollowPath'],
                'FollowPath.plugin':
                    'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
                'FollowPath.desired_linear_vel':                         0.4,
                'FollowPath.lookahead_dist':                             0.6,
                'FollowPath.min_lookahead_dist':                         0.3,
                'FollowPath.max_lookahead_dist':                         0.9,
                'FollowPath.lookahead_time':                             1.5,
                'FollowPath.rotate_to_heading_angular_vel':              1.0,
                'FollowPath.transform_tolerance':                        0.5,
                'FollowPath.use_velocity_scaled_lookahead_dist':         False,
                'FollowPath.min_approach_linear_velocity':               0.05,
                'FollowPath.approach_velocity_scaling_dist':             0.6,
                'FollowPath.use_collision_detection':                    True,
                'FollowPath.max_allowed_time_to_collision_up_to_carrot': 1.0,
                'FollowPath.use_regulated_linear_velocity_scaling':      True,
                'FollowPath.use_fixed_curvature_lookahead':              False,
                'FollowPath.curvature_feedback_gain':                    3.5,
                'FollowPath.inflation_cost_scaling_factor':              3.0,
                'FollowPath.use_cost_regulated_linear_velocity_scaling': False,
                'FollowPath.regulated_linear_scaling_min_radius':        0.9,
                'FollowPath.regulated_linear_scaling_min_speed':         0.25,
                'FollowPath.use_rotate_to_heading':                      True,
                'FollowPath.allow_reversing':                            False,
                'FollowPath.rotate_to_heading_min_angle':                0.785,
                'FollowPath.max_angular_accel':                          2.0,
                'FollowPath.max_robot_pose_search_dist':                 10.0,
            },
        ],
        remappings=[
            ('odom',    '/odometry/filtered'),
            ('cmd_vel', '/cmd_vel_nav'),
        ],
        output='screen',
    )

    smoother_server = LifecycleNode(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        namespace='',
        parameters=[nav2_params_yaml, {'use_sim_time': False}],
        output='screen',
    )

    planner_server = LifecycleNode(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace='',
        parameters=[
            nav2_params_yaml,
            {
                'use_sim_time': False,
                'global_costmap.transform_tolerance': 3.0,
            },
        ],
        output='screen',
    )

    behavior_server = LifecycleNode(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace='',
        parameters=[nav2_params_yaml, {'use_sim_time': False}],
        output='screen',
    )

    bt_navigator = LifecycleNode(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace='',
        parameters=[nav2_params_yaml, {'use_sim_time': False}],
        remappings=[('odom', '/odometry/filtered')],
        output='screen',
    )

    waypoint_follower = LifecycleNode(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        namespace='',
        parameters=[nav2_params_yaml, {'use_sim_time': False}],
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════
    #  t=24s
    # ═══════════════════════════════════════════════════════════════
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=[{
            'use_sim_time': False,
            'autostart':    True,
            'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
            ],
            'bond_timeout': 4.0,
        }],
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════
    #  t=26s
    # ═══════════════════════════════════════════════════════════════
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    # # [v6] uncomment pose_saver_node — save /amcl_pose → last_pose.yaml อัตโนมัติ
    # pose_saver = Node(
    #     package='joy_control',
    #     executable='pose_saver_node',
    #     name='pose_saver_node',
    #     output='screen',
    # )

    # ═══════════════════════════════════════════════════════════════
    #  t=40s  —  goal_nav_node + xy_publisher_node
    #  [v6] เพิ่มจาก nav_Sim
    #  เหตุผล: lifecycle_manager_navigation ใช้เวลา activate ~35-38s
    #          goal_nav_node ต้องรอให้ navigate_to_pose action server
    #          พร้อมก่อน ไม่งั้น goal แรกที่ส่งมาจะหายไป
    # ═══════════════════════════════════════════════════════════════
    goal_nav = Node(
        package='joy_control',
        executable='goal_nav_node',
        name='goal_nav_node',
        output='screen',
    )

    xy_publisher = Node(
        package='joy_control',
        executable='xy_publisher_node',
        name='xy_publisher_node',
        output='screen',
    )

    return LaunchDescription([
        # t=0s
        robot_state_publisher,
        serial_bridge,

        # t=1s
        TimerAction(period=1.0,  actions=[controller_manager]),

        # t=3s  [v6] เพิ่ม keyboard_teleop, joy_node, ps5_converter
        TimerAction(period=3.0,  actions=[
            spawn_jsb,
            spawn_diff,
            twist_mux,
        ]),

        # t=4s
        TimerAction(period=4.0,  actions=[lidar_node, imu_gyro_integrator]),

        # t=8s
        TimerAction(period=8.0,  actions=[reset_imu_yaw]),

        # t=9s
        TimerAction(period=9.0,  actions=[ekf_filter]),

        # t=12s
        TimerAction(period=12.0, actions=[map_server, amcl]),

        # t=16s
        TimerAction(period=16.0, actions=[lifecycle_manager_localization]),

        # t=22s
        TimerAction(period=22.0, actions=[
            controller_server,
            smoother_server,
            planner_server,
            behavior_server,
            bt_navigator,
            waypoint_follower,
        ]),

        # t=24s
        TimerAction(period=24.0, actions=[lifecycle_manager_navigation]),

        # t=26s  [v6] เพิ่ม pose_saver_node
        TimerAction(period=26.0, actions=[rviz]),

        # t=40s  [v6] เพิ่ม goal_nav_node + xy_publisher_node
        #        รอ Nav2 lifecycle activate เสร็จก่อน (~35-38s)
        TimerAction(period=40.0, actions=[goal_nav, xy_publisher]),
    ])