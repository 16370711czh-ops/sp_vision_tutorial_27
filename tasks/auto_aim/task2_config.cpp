#include <stdexcept>

#include "task2_pipeline.hpp"
#include "tools/yaml.hpp"

namespace auto_aim
{
Task2Settings load_task2_settings(const std::string & path)
{
  auto yaml = YAML::LoadFile(path);
  auto node = yaml["task2"];
  if (!node || !node.IsMap()) throw std::invalid_argument("Missing task2 YAML mapping.");
  Task2Settings settings;
  auto & t = settings.tracking;
  auto & f = t.filter;
  auto & a = settings.aiming;
  t.armor_num = tools::read_or<int>(node, "armor_num", t.armor_num);
  t.initial_radius = tools::read_or<double>(node, "initial_radius", t.initial_radius);
  f.radius_variance = tools::read_or<double>(node, "radius_variance", f.radius_variance);
  f.radius_diff_variance =
    tools::read_or<double>(node, "radius_diff_variance", f.radius_diff_variance);
  f.height_diff_variance =
    tools::read_or<double>(node, "height_diff_variance", f.height_diff_variance);
  f.acceleration_variance =
    tools::read_or<double>(node, "acceleration_variance", f.acceleration_variance);
  f.angular_acceleration_variance =
    tools::read_or<double>(node, "angular_acceleration_variance", f.angular_acceleration_variance);
  f.z_acceleration_variance =
    tools::read_or<double>(node, "z_acceleration_variance", f.z_acceleration_variance);
  f.max_nis = tools::read_or<double>(node, "max_nis", f.max_nis);
  f.max_prediction_dt = tools::read_or<double>(node, "max_prediction_dt", f.max_prediction_dt);
  t.lost_time = tools::read_or<double>(node, "lost_time", t.lost_time);
  t.association_distance =
    tools::read_or<double>(node, "association_distance", t.association_distance);
  t.association_angle = tools::read_or<double>(node, "association_angle", 60.0) * CV_PI / 180;
  t.max_observation_facing =
    tools::read_or<double>(node, "max_observation_facing", 80.0) * CV_PI / 180;
  t.confirm_frames = tools::read_or<int>(node, "confirm_frames", t.confirm_frames);
  t.min_updates = tools::read_or<int>(node, "min_updates", t.min_updates);
  t.warmup_time = tools::read_or<double>(node, "warmup_time", t.warmup_time);
  t.max_omega_variance = tools::read_or<double>(node, "max_omega_variance", t.max_omega_variance);
  t.min_spin_speed = tools::read_or<double>(node, "min_spin_speed", t.min_spin_speed);
  t.max_spin_speed = tools::read_or<double>(node, "max_spin_speed", t.max_spin_speed);
  a.yaw_offset = tools::read_or<double>(yaml, "yaw_offset", 0.0) * CV_PI / 180;
  a.pitch_offset = tools::read_or<double>(yaml, "pitch_offset", 0.0) * CV_PI / 180;
  a.fire_delay = tools::read_or<double>(node, "fire_delay", a.fire_delay);
  a.pitch_limit = tools::read_or<double>(node, "pitch_limit", 20.0) * CV_PI / 180;
  a.max_facing_angle = tools::read_or<double>(node, "max_facing_angle", 60.0) * CV_PI / 180;
  a.max_frame_age = tools::read_or<double>(node, "max_frame_age", a.max_frame_age);
  a.min_bullet_speed = tools::read_or<double>(node, "min_bullet_speed", a.min_bullet_speed);
  a.max_bullet_speed = tools::read_or<double>(node, "max_bullet_speed", a.max_bullet_speed);
  settings.yaw_tolerance = tools::read_or<double>(node, "yaw_tolerance", 0.8) * CV_PI / 180;
  settings.pitch_tolerance = tools::read_or<double>(node, "pitch_tolerance", 0.8) * CV_PI / 180;
  settings.fire_facing_angle =
    tools::read_or<double>(node, "fire_facing_angle", 35.0) * CV_PI / 180;
  settings.feedback_timeout =
    tools::read_or<double>(node, "feedback_timeout", settings.feedback_timeout);
  settings.camera_latency =
    tools::read_or<double>(
      node, "camera_to_pose_offset_ms", tools::read_or<double>(node, "camera_latency_ms", 0.0)) /
    1000;
  settings.pose_timeout_ms = tools::read_or<int>(node, "pose_timeout_ms", settings.pose_timeout_ms);
  settings.shot_limit = tools::read_or<int>(node, "shot_limit", settings.shot_limit);
  settings.follow_without_ballistics =
    tools::read_or<bool>(node, "follow_without_ballistics", true);
  settings.max_reprojection_error_px =
    tools::read_or<double>(node, "max_reprojection_error_px", 5.0);
  validate_task2_settings(settings);
  return settings;
}
}  // namespace auto_aim
