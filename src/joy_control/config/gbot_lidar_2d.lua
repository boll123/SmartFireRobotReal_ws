include "map_builder.lua"
include "trajectory_builder.lua"

-- ──────────────────────────────────────────────────────────────────
-- gbot_lidar_2d.lua  —  v18-MapFix
--
-- v18 changes (from v17-7Hz):
--
--   [FIX MAP TILT]
--       ไม่มีการเปลี่ยนแปลงใน lua สำหรับปัญหานี้
--       แก้ที่ ekf_odom_imu.yaml (imu0_relative: false)
--       และ launch file (delay Cartographer ให้ EKF stable ก่อน)
--
--   [FIX LOOP CLOSURE - ผนังไม่ match]
--       rotation_weight     : 200  → 100   (ลดลงเพื่อให้ loop closure แก้ heading ได้)
--       linear_search_window: 0.15 → 0.20  (เพิ่มขึ้นเพื่อ match ได้กว้างขึ้น)
--       angular_search_window: 15° → 20°   (เพิ่มขึ้น)
--       num_range_data      : 60   → 80    (submap ใหญ่ขึ้น = match แม่นขึ้น)
--
--   [FIX POSE GRAPH]
--       max_constraint_distance: 3.0 → 5.0  (ครอบคลุมห้องขนาดใหญ่ขึ้น)
--       min_score             : 0.65 → 0.60  (ยอมรับ match มากขึ้น)
--       sampling_ratio        : 0.40 → 0.50  (loop close บ่อยขึ้น)
--       optimize_every_n_nodes: 20   → 15    (optimize บ่อยขึ้น)
--
-- ──────────────────────────────────────────────────────────────────

options = {
  map_builder                        = MAP_BUILDER,
  trajectory_builder                 = TRAJECTORY_BUILDER,
  map_frame                          = "map",

  -- base_footprint ใช้ได้เมื่อปิด use_imu_data
  tracking_frame                     = "base_footprint",

  published_frame                    = "odom",
  odom_frame                         = "odom",
  provide_odom_frame                 = false,
  publish_frame_projected_to_2d      = true,

  use_odometry                       = true,
  use_nav_sat                        = false,
  use_landmarks                      = false,
  num_laser_scans                    = 1,
  num_multi_echo_laser_scans         = 0,
  num_subdivisions_per_laser_scan    = 1,
  num_point_clouds                   = 0,

  lookup_transform_timeout_sec       = 0.05,
  submap_publish_period_sec          = 0.3,
  pose_publish_period_sec            = 0.02,
  trajectory_publish_period_sec      = 0.05,
  rangefinder_sampling_ratio         = 1.0,
  odometry_sampling_ratio            = 1.0,
  fixed_frame_pose_sampling_ratio    = 1.0,
  imu_sampling_ratio                 = 1.0,
  landmarks_sampling_ratio           = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads    = 4

-- ── RPLIDAR A1 @ 7.4Hz ──────────────────────────────────────────
TRAJECTORY_BUILDER_2D.min_range               = 0.6
TRAJECTORY_BUILDER_2D.max_range               = 30.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0

-- [FIX] ปิด IMU ใน Cartographer → ใช้ base_footprint เป็น tracking_frame ได้
-- IMU ยังทำงานใน EKF (robot_localization) ปกติ
TRAJECTORY_BUILDER_2D.use_imu_data           = false

-- 7.4Hz → 1 scan ต่อรอบ ไม่ต้อง accumulate
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- [FIX v18] เพิ่ม search window ให้กว้างขึ้นช่วย match ได้แม่นขึ้น
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window          = 0.20  -- จาก 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window         = math.rad(20.0)  -- จาก 15.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight    = 1e-2

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight    = 5.0
-- [FIX v18] ลดจาก 200 → 100 เพื่อให้ loop closure สามารถแก้ heading drift ได้
-- rotation_weight=200 แข็งเกินไป ทำให้ผนังซ้อนไม่ match เมื่อวนกลับมาที่เดิม
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight       = 100.0  -- จาก 200.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 5.0

-- [TUNE 7.4Hz] ปรับให้ตรงกับ scan interval จริง ~0.135s
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds    = 0.135
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians   = math.rad(0.5)

-- [FIX v18] เพิ่ม num_range_data จาก 60 → 80
-- submap ใหญ่ขึ้น (~10.8s/submap) → scan matcher มีข้อมูลมากขึ้น → loop close แม่นขึ้น
TRAJECTORY_BUILDER_2D.submaps.num_range_data             = 80  -- จาก 60
-- [FIX] ตรงกับ launch file -resolution 0.03
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.03

-- ── POSE GRAPH ────────────────────────────────────────────────────

-- [FIX v18] optimize บ่อยขึ้น → แก้ drift ได้เร็วขึ้น
POSE_GRAPH.optimize_every_n_nodes = 15  -- จาก 20

-- [FIX v18] ลด min_score เล็กน้อย + เพิ่ม constraint distance
-- ให้ครอบคลุมห้องที่ใหญ่กว่า 3m และ loop close สำเร็จมากขึ้น
POSE_GRAPH.constraint_builder.min_score                      = 0.60  -- จาก 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score  = 0.70  -- จาก 0.75
POSE_GRAPH.constraint_builder.max_constraint_distance        = 5.0   -- จาก 3.0
-- [FIX v18] เพิ่ม sampling ratio → loop close ถูก propose บ่อยขึ้น
POSE_GRAPH.constraint_builder.sampling_ratio                 = 0.50  -- จาก 0.40

POSE_GRAPH.global_sampling_ratio                             = 0.003
POSE_GRAPH.global_constraint_search_after_n_seconds          = 30.0

POSE_GRAPH.optimization_problem.odometry_translation_weight  = 100.0
POSE_GRAPH.optimization_problem.odometry_rotation_weight     = 50.0

POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e3
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight    = 1e3

POSE_GRAPH.optimization_problem.huber_scale                  = 1e1

return options