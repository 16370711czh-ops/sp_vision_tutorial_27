#include "task3_pipeline.hpp"

#include <stdexcept>

#include "tools/yaml.hpp"

namespace auto_aim
{
Task3Settings load_task3_settings(const std::string & path)
{
  const auto yaml = YAML::LoadFile(path);
  const auto node = yaml["task3"];
  if (!node || !node.IsMap()) throw std::invalid_argument("Missing task3 YAML mapping.");
  Task3Settings s;
  auto & t = s.tracking;
  auto & f = t.filter;
  auto & a = s.aiming;
  t.radius = tools::read_or<double>(node, "radius", t.radius);
  // Explicit values are required: the drawing's ground origin is not the gimbal origin.
  t.middle_z_min = node["middle_z_min"].as<double>();
  t.middle_z_max = node["middle_z_max"].as<double>();
  t.lost_time = tools::read_or<double>(node, "lost_time", t.lost_time);
  t.association_distance = tools::read_or<double>(node, "association_distance", t.association_distance);
  t.association_angle = tools::read_or<double>(node, "association_angle", 60.0) * CV_PI / 180;
  t.warmup_time = tools::read_or<double>(node, "warmup_time", t.warmup_time);
  t.min_updates = tools::read_or<int>(node, "min_updates", t.min_updates);
  t.max_omega_variance = tools::read_or<double>(node, "max_omega_variance", t.max_omega_variance);
  t.min_spin_speed = tools::read_or<double>(node, "min_spin_speed", t.min_spin_speed);
  t.max_spin_speed = tools::read_or<double>(node, "max_spin_speed", t.max_spin_speed);
  f.outpost_radius_variance = tools::read_or<double>(node, "radius_variance", f.outpost_radius_variance);
  f.radius_diff_variance = tools::read_or<double>(node, "height1_variance", f.radius_diff_variance);
  f.height_diff_variance = tools::read_or<double>(node, "height2_variance", f.height_diff_variance);
  f.outpost_center_noise = tools::read_or<double>(node, "center_noise", f.outpost_center_noise);
  f.outpost_angular_acceleration_variance = tools::read_or<double>(
    node, "angular_acceleration_variance", f.outpost_angular_acceleration_variance);
  f.max_nis = tools::read_or<double>(node, "max_nis", f.max_nis);
  f.max_prediction_dt = tools::read_or<double>(node, "max_prediction_dt", f.max_prediction_dt);
  a.yaw_offset = tools::read_or<double>(yaml, "yaw_offset", 0.0) * CV_PI / 180;
  a.pitch_offset = tools::read_or<double>(yaml, "pitch_offset", 0.0) * CV_PI / 180;
  a.fire_delay = tools::read_or<double>(node, "fire_delay", a.fire_delay);
  a.pitch_limit = tools::read_or<double>(node, "pitch_limit", 20.0) * CV_PI / 180;
  a.max_facing_angle = tools::read_or<double>(node, "max_facing_angle", 65.0) * CV_PI / 180;
  a.max_frame_age = tools::read_or<double>(node, "max_frame_age", a.max_frame_age);
  a.min_bullet_speed = tools::read_or<double>(node, "min_bullet_speed", a.min_bullet_speed);
  a.max_bullet_speed = tools::read_or<double>(node, "max_bullet_speed", a.max_bullet_speed);
  s.firing.yaw_tolerance = tools::read_or<double>(node, "yaw_tolerance", 0.5) * CV_PI / 180;
  s.firing.pitch_tolerance = tools::read_or<double>(node, "pitch_tolerance", 0.5) * CV_PI / 180;
  s.firing.facing_angle = tools::read_or<double>(node, "fire_facing_angle", 30.0) * CV_PI / 180;
  s.middle_height_verified = tools::read_or<bool>(node, "middle_height_verified", false);
  s.model_geometry_verified = tools::read_or<bool>(node, "model_geometry_verified", false);
  s.follow_without_ballistics = tools::read_or<bool>(node, "follow_without_ballistics", true);
  s.feedback_timeout = tools::read_or<double>(node, "feedback_timeout", s.feedback_timeout);
  s.camera_latency = tools::read_or<double>(node, "camera_to_pose_offset_ms", 0.0) / 1000;
  s.pose_timeout_ms = tools::read_or<int>(node, "pose_timeout_ms", s.pose_timeout_ms);
  s.max_reprojection_error_px = tools::read_or<double>(node, "max_reprojection_error_px", 5.0);
  validate_task3_settings(s);
  return s;
}
}  // namespace auto_aim
