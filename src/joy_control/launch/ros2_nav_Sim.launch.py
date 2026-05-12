# launch/ros2_nav_Sim.launch.py  —  v3-FixGoalNav
# ─────────────────────────────────────────────────────────────────
# Nav2 Navigation สำหรับ Gazebo 11 Simulation
#
# สิ่งที่แก้ไขจาก v2-FixCrash:
#   - เพิ่ม goal_nav_node และ xy_publisher_node
#   - timer goal_nav_node: 40s (เพิ่มจาก 33s)
#     เพราะ lifecycle_manager_navigation ใช้เวลา activate ~35-38s
#     ถ้ารันก่อน action server พร้อม → goal แรกจะหาย
#
# Timing (Sim):
#   t=0s   gazebo (gzserver + gzclient)
#   t=5s   spawn_entity (URDF → Gazebo)
#   t=6s   robot_state_publisher
#   t=9s   spawners (jsb, diff), twist_mux
#   t=10s  imu_filter_madgwick
#   t=14s  ekf_filter_node
#   t=17s  map_server, amcl
#   t=21s  lifecycle_manager_localization
#   t=27s  controller_server, smoother_server, planner_server,
#           behavior_server, bt_navigator, waypoint_follower
#   t=29s  lifecycle_manager_navigation
#   t=31s  rviz2, pose_saver_node
#   t=40s  goal_nav_node, xy_publisher_node  ← [FIX] รอ Nav2 activate เสร็จ
# ─────────────────────────────────────────────────────────────────

import os
import sys
import math
import subprocess

import yaml

from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory


# ── ค่า default ถ้าไม่มีไฟล์ last_pose.yaml ──────────────────────
DEFAULT_POSE = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

# ── path ของไฟล์ที่ pose_saver_node จะ save ──────────────────────
LAST_POSE_FILE = os.path.expanduser(
    '~/SmartFireRobotReal_ws/src/joy_control/maps/last_pose.yaml'
)


def load_last_pose():
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
    pkg_share  = get_package_share_directory('joy_control')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    # ── Config paths ──────────────────────────────────────────────
    urdf_file        = os.path.join(pkg_share, 'urdf',   'robotimuSim.urdf.xacro')
    rviz_file        = os.path.join(pkg_share, 'rviz',   'nav2_config.rviz')
    map_yaml         = os.path.join(pkg_share, 'maps',   'mapgazeboTest.yaml')
    nav2_params_yaml = os.path.join(pkg_share, 'config', 'nav2_paramsSim.yaml')
    twist_mux_yaml   = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    ekf_yaml         = os.path.join(pkg_share, 'config', 'ekf_odom_imuSim.yaml')
    world_file       = os.path.join(pkg_share, 'worlds', 'robot_world.world')

    # ── Early check ───────────────────────────────────────────────
    for label, path in [
        ('nav2_paramsSim.yaml',  nav2_params_yaml),
        ('mapgazeboTest.yaml',   map_yaml),
        ('ekf_odom_imuSim.yaml', ekf_yaml),
    ]:
        if not os.path.isfile(path):
            print(f'\n[LAUNCH ERROR] ไม่พบไฟล์: {label}\n  path: {path}')
            sys.exit(1)

    # ── โหลด initial pose อัตโนมัติ ──────────────────────────────
    pose = load_last_pose()

    # ── Robot description (sim_mode:=true) ───────────────────────
    robot_desc = subprocess.check_output(
        ['xacro', urdf_file, 'sim_mode:=true']
    ).decode('utf-8')

    # ═══════════════════════════════════════════════════════════════
    #  t=0s  —  Gazebo 11
    # ═══════════════════════════════════════════════════════════════
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world':   world_file,
            'verbose': 'false',
            'pause':   'false',
        }.items(),
    )

    # ═══════════════════════════════════════════════════════════════
    #  t=5s  —  Spawn robot into Gazebo
    # ═══════════════════════════════════════════════════════════════
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'smartfire_robot',
            '-x', str(pose['x']),
            '-y', str(pose['y']),
            '-z', '0.05',
            '-Y', str(pose['yaw']),
        ],
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════
    #  t=6s  —  robot_state_publisher
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
    #  t=9s  —  spawners + twist_mux
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

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_yaml, {'use_sim_time': True}],
        remappings=[('cmd_vel_out', '/diff_drive_controller/cmd_vel_unstamped')],
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════
    #  t=10s  —  imu_filter_madgwick
    # ═══════════════════════════════════════════════════════════════
    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        parameters=[{
            'use_mag':      False,
            'publish_tf':   False,
            'world_frame':  'enu',
            'gain':         0.01,
            'zeta':         0.0,
            'use_sim_time': True,
        }],
        remappings=[
            ('imu/data_raw', '/imu/data_raw'),
            ('imu/data',     '/imu/data'),
        ],
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════
    #  t=14s  —  ekf_filter_node
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
    #  t=17s  —  map_server + amcl
    # ═══════════════════════════════════════════════════════════════
    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        parameters=[
            nav2_params_yaml,
            {
                'use_sim_time':  True,
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
                'use_sim_time':               True,
                'set_initial_pose':           True,
                'always_reset_initial_pose':  False,
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
    #  t=21s  —  lifecycle_manager_localization
    # ═══════════════════════════════════════════════════════════════
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[{
            'use_sim_time': True,
            'autostart':    True,
            'node_names':   ['map_server', 'amcl'],
            'bond_timeout': 4.0,
        }],
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════
    #  t=27s  —  Nav2 servers
    # ═══════════════════════════════════════════════════════════════
    controller_server = LifecycleNode(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace='',
        parameters=[
            nav2_params_yaml,
            {
                'use_sim_time': True,
                'controller_plugins': ['FollowPath'],
                'FollowPath.plugin':
                    'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
                'FollowPath.desired_linear_vel':                     0.4,
                'FollowPath.lookahead_dist':                         0.6,
                'FollowPath.min_lookahead_dist':                     0.3,
                'FollowPath.max_lookahead_dist':                     0.9,
                'FollowPath.lookahead_time':                         1.5,
                'FollowPath.rotate_to_heading_angular_vel':          1.0,
                'FollowPath.transform_tolerance':                    0.5,
                'FollowPath.use_velocity_scaled_lookahead_dist':     False,
                'FollowPath.min_approach_linear_velocity':           0.05,
                'FollowPath.approach_velocity_scaling_dist':         0.6,
                'FollowPath.use_collision_detection':                True,
                'FollowPath.max_allowed_time_to_collision_up_to_carrot': 1.0,
                'FollowPath.use_regulated_linear_velocity_scaling':  True,
                'FollowPath.use_fixed_curvature_lookahead':          False,
                'FollowPath.curvature_feedback_gain':                3.5,
                'FollowPath.inflation_cost_scaling_factor':          2.0,
                'FollowPath.use_cost_regulated_linear_velocity_scaling': True,
                'FollowPath.regulated_linear_scaling_min_radius':    0.6,
                'FollowPath.regulated_linear_scaling_min_speed':     0.15,
                'FollowPath.use_rotate_to_heading':                  True,
                'FollowPath.allow_reversing':                        False,
                'FollowPath.rotate_to_heading_min_angle':            0.785,
                'FollowPath.max_angular_accel':                      2.0,
                'FollowPath.max_robot_pose_search_dist':             10.0,
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
        parameters=[nav2_params_yaml, {'use_sim_time': True}],
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
                'use_sim_time': True,
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
        parameters=[nav2_params_yaml, {'use_sim_time': True}],
        output='screen',
    )

    bt_navigator = LifecycleNode(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace='',
        parameters=[nav2_params_yaml, {'use_sim_time': True}],
        remappings=[('odom', '/odometry/filtered')],
        output='screen',
    )

    waypoint_follower = LifecycleNode(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        namespace='',
        parameters=[nav2_params_yaml, {'use_sim_time': True}],
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════
    #  t=29s  —  lifecycle_manager_navigation
    # ═══════════════════════════════════════════════════════════════
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=[{
            'use_sim_time': True,
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
    #  t=31s  —  RViz2 + pose_saver_node
    # ═══════════════════════════════════════════════════════════════
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # pose_saver = Node(
    #     package='joy_control',
    #     executable='pose_saver_node',
    #     name='pose_saver_node',
    #     output='screen',
    # )

    # ═══════════════════════════════════════════════════════════════
    #  t=40s  —  goal_nav_node + xy_publisher_node
    #  [FIX] เพิ่มจาก 33s → 40s
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
        # t=0s   Gazebo 11
        gazebo,

        # t=5s   Spawn robot
        TimerAction(period=5.0,  actions=[spawn_entity]),

        # t=6s   robot_state_publisher
        TimerAction(period=6.0,  actions=[robot_state_publisher]),

        # t=9s   spawners + twist_mux
        TimerAction(period=9.0,  actions=[spawn_jsb, spawn_diff, twist_mux]),

        # t=10s  imu_filter_madgwick
        TimerAction(period=10.0, actions=[imu_filter]),

        # t=14s  ekf_filter
        TimerAction(period=14.0, actions=[ekf_filter]),

        # t=17s  map_server + amcl
        TimerAction(period=17.0, actions=[map_server, amcl]),

        # t=21s  lifecycle_manager_localization
        TimerAction(period=21.0, actions=[lifecycle_manager_localization]),

        # t=27s  Nav2 servers
        TimerAction(period=27.0, actions=[
            controller_server,
            smoother_server,
            planner_server,
            behavior_server,
            bt_navigator,
            waypoint_follower,
        ]),

        # t=29s  lifecycle_manager_navigation
        TimerAction(period=29.0, actions=[lifecycle_manager_navigation]),

        # t=31s  RViz2 + pose_saver
        TimerAction(period=31.0, actions=[rviz]),

        # t=40s  goal_nav_node + xy_publisher_node  [FIX] 33→40
        TimerAction(period=40.0, actions=[goal_nav, xy_publisher]),
    ])