#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "../src/task_1_follow.hpp"

int main(int argc, char ** argv)
{
  if (argc != 2) return 2;
  int checks = 0;
  auto expect = [&](bool condition, const char * message) {
    ++checks;
    if (!condition) throw std::runtime_error(message);
  };
  auto yaml = YAML::LoadFile(argv[1]);
  auto settings = auto_aim::load_follow_settings(yaml);
  expect(std::abs(settings.pitch_limit - 20 * std::acos(-1) / 180) < 1e-12, "degree conversion");
  expect(settings.command_timeout_ms == 100, "provided config enables command watchdog");
  expect(settings.camera_to_pose_offset == 0, "offset is an unmeasured zero starting value");
  expect(yaml["enemy_color"].as<std::string>() == "red", "provided target color");
  auto_aim::FollowContext context;
  context.auto_mode = context.feedback_valid = true;
  context.frame_age = 0.02;
  context.feedback_age = 0.01;
  auto aim = auto_aim::follow_angles(3, 0.3, 0.2, 0, 0, 0, settings.pitch_limit);
  auto command = auto_aim::follow_command(aim, context, settings);
  expect(command.control && command.yaw > 0 && command.pitch < 0, "fresh target follows");
  expect(
    !auto_aim::follow_command({}, context, settings).control,
    "lost target immediately releases control");
  expect(
    auto_aim::follow_command(aim, context, settings).control, "target recovery needs no restart");
  context.auto_mode = false;
  expect(!auto_aim::follow_command(aim, context, settings).control, "manual mode");
  context.auto_mode = true;
  context.dry_run = true;
  expect(!auto_aim::follow_command(aim, context, settings).control, "dry run never takes control");
  context.dry_run = false;
  context.feedback_valid = false;
  expect(!auto_aim::follow_command(aim, context, settings).control, "invalid feedback");
  context.feedback_valid = true;
  auto nan = std::numeric_limits<double>::quiet_NaN();
  auto inf = std::numeric_limits<double>::infinity();
  for (double age : {-0.001, settings.max_frame_age + 0.001, nan, inf}) {
    context.frame_age = age;
    expect(!auto_aim::follow_command(aim, context, settings).control, "bad image timestamp");
  }
  context.frame_age = 0.02;
  for (double age : {-0.001, settings.feedback_timeout + 0.001, nan, inf}) {
    context.feedback_age = age;
    expect(!auto_aim::follow_command(aim, context, settings).control, "bad feedback timestamp");
  }
  context.feedback_age = 0.01;
  for (const auto & invalid :
       {auto_aim::FollowAngles{true, nan, 0}, auto_aim::FollowAngles{true, inf, 0},
        auto_aim::FollowAngles{true, 1e100, 0}, auto_aim::FollowAngles{true, 0, nan},
        auto_aim::FollowAngles{true, 0, settings.pitch_limit + 0.01}}) {
    auto stop = auto_aim::follow_command(invalid, context, settings);
    expect(!stop.control && stop.yaw == 0 && stop.pitch == 0, "invalid command releases control");
  }
  auto huge = std::numeric_limits<double>::max();
  expect(
    !auto_aim::follow_angles(huge, huge, 1, 0, 0, 0, settings.pitch_limit).valid,
    "overflowing horizontal distance rejected");

  // 持续移动并跨越+/-pi；间歇丢失与模式切换不留下旧帧有效标志。
  for (int frame = 0; frame <= 720; ++frame) {
    double angle = frame * std::acos(-1) / 180;
    auto sample = auto_aim::follow_angles(
      3 * std::cos(angle), 3 * std::sin(angle), 0.8 * std::sin(angle), angle - 0.01, 0, 0,
      settings.pitch_limit);
    bool visible = frame % 29 != 0;
    context.auto_mode = frame % 31 != 0;
    auto result =
      auto_aim::follow_command(visible ? sample : auto_aim::FollowAngles{}, context, settings);
    expect(result.control == (visible && context.auto_mode), "continuous mode/loss sequence");
    expect(
      std::isfinite(result.yaw) && std::isfinite(result.pitch), "finite float protocol angles");
    expect(std::abs(result.pitch) <= settings.pitch_limit + 1e-7, "pitch limit sequence");
    if (result.control)
      expect(std::abs(result.yaw - (angle - 0.01)) < 0.02, "continuous multi-turn yaw");
  }

  auto invalid_setting = [&](const char * key, double value) {
    auto invalid = YAML::Clone(yaml);
    invalid[key] = value;
    bool rejected = false;
    try {
      auto_aim::load_follow_settings(invalid);
    } catch (const std::exception &) {
      rejected = true;
    }
    expect(rejected, "invalid YAML setting rejected by production loader");
  };
  for (double value : {0.0, -1.0, 90.0, nan, inf}) invalid_setting("task1_pitch_limit", value);
  for (double value : {0.0, -0.1, 2.0, nan}) invalid_setting("task1_max_frame_age", value);
  for (double value : {0.0, -0.1, 0.2, nan}) invalid_setting("task1_feedback_timeout", value);
  invalid_setting("task1_yaw_offset", nan);
  invalid_setting("task1_pitch_offset", inf);
  invalid_setting("task1_camera_to_pose_offset_ms", 1001);
  invalid_setting("task1_pose_timeout_ms", -1);
  invalid_setting("task1_pose_timeout_ms", 101);
  invalid_setting("command_timeout_ms", -1);
  invalid_setting("command_timeout_ms", 1001);
  auto signed_offset = YAML::Clone(yaml);
  signed_offset["task1_camera_to_pose_offset_ms"] = -12.0;
  expect(
    std::abs(auto_aim::load_follow_settings(signed_offset).camera_to_pose_offset + 0.012) < 1e-12,
    "signed millisecond offset preserved");
  signed_offset["task1_pose_timeout_ms"] = "invalid";
  bool rejected = false;
  try {
    auto_aim::load_follow_settings(signed_offset);
  } catch (const std::exception &) {
    rejected = true;
  }
  expect(rejected, "wrong YAML type rejected");
  std::cout << "PASS follow_control: " << checks << " checks\n";
}
