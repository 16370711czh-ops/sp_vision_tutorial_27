#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "task_1_follow.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/yaml.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }"
  "{show           | false | 显示调试画面，需要图形桌面 }"
  "{dry-run        | false | 仅观测，不接管云台，始终不开火 }";

using namespace std::chrono_literals;

namespace
{
int run(const std::string & config_path, bool show, bool dry_run)
{
  auto yaml = YAML::LoadFile(config_path);
  auto settings = auto_aim::load_follow_settings(yaml);
  auto enemy_color = yaml["enemy_color"].as<std::string>();
  if (enemy_color != "red" && enemy_color != "blue")
    throw std::invalid_argument("Task1 enemy_color must be red or blue.");
  auto color = enemy_color == "red" ? auto_aim::Color::red : auto_aim::Color::blue;
  if (settings.command_timeout_ms == 0)
    tools::logger()->warn("[Task1] Command watchdog disabled; use configs/task1.yaml.");

  tools::Exiter exiter;
  tools::Plotter plotter;
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);
  auto_aim::YOLO yolo(config_path, show);
  auto_aim::Solver solver(config_path);
  tools::logger()->info("[Task1] dry_run={}, fire is always disabled.", dry_run);

  cv::Mat img;
  std::chrono::steady_clock::time_point image_time;
  auto start = std::chrono::steady_clock::now();
  int frame_count = 0;
  while (!exiter.exit()) {
    auto_aim::FollowAngles aim;
    bool image_ok = false, pose_ok = false;
    int detected = 0, pnp_valid = 0;
    double distance = 0;
    uint8_t target_number = 8;
    try {
      image_ok = camera.try_read_for(img, image_time, 100ms) && !img.empty();
      if (image_ok) {
        auto pose_time =
          image_time - std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                         std::chrono::duration<double>(settings.camera_to_pose_offset));
        Eigen::Quaterniond q;
        pose_ok = gimbal.try_q(pose_time, q, std::chrono::milliseconds(settings.pose_timeout_ms));
        if (pose_ok) {
          solver.set_R_gimbal2world(q);
          auto armors = yolo.detect(img, frame_count++);
          detected = static_cast<int>(armors.size());
          for (auto it = armors.begin(); it != armors.end();) {
            bool points_ok = it->points.size() == 4;
            for (const auto & point : it->points)
              points_ok = points_ok && std::isfinite(point.x) && std::isfinite(point.y);
            if (
              it->color != color || it->name < auto_aim::ArmorName::one ||
              it->name > auto_aim::ArmorName::base || !points_ok || !std::isfinite(it->center.x) ||
              !std::isfinite(it->center.y)) {
              it = armors.erase(it);
            } else {
              ++it;
            }
          }
          cv::Point2f image_center(img.cols / 2.0f, img.rows / 2.0f);
          armors.sort([&](const auto & a, const auto & b) {
            return cv::norm(a.center - image_center) < cv::norm(b.center - image_center);
          });
          // 单块手持目标优先靠近画面中心；最近候选PnP失败时继续尝试下一块。
          for (auto & armor : armors) {
            try {
              solver.solve(armor);
              if (
                !armor.xyz_in_world.allFinite() || !armor.xyz_in_gimbal.allFinite() ||
                armor.xyz_in_gimbal[0] <= 0)
                continue;
              ++pnp_valid;
              const auto & xyz = armor.xyz_in_world;
              // 这里只求位置方向，不使用装甲板法线方向，也不添加弹道补偿。
              aim = auto_aim::follow_angles(
                xyz[0], xyz[1], xyz[2], gimbal.feedback().state.yaw, settings.yaw_offset,
                settings.pitch_offset, settings.pitch_limit);
              if (!aim.valid) continue;
              distance = xyz.norm();
              target_number = static_cast<uint8_t>(armor.name);
              if (show) tools::draw_points(img, armor.points, {0, 255, 0});
              break;
            } catch (const std::exception & e) {
              tools::logger()->debug("[Task1] PnP rejected: {}", e.what());
            }
          }
        }
      }
    } catch (const std::exception & e) {
      aim = {};
      tools::logger()->warn("[Task1] Frame skipped: {}", e.what());
    }

    auto feedback = gimbal.feedback();
    auto now = std::chrono::steady_clock::now();
    auto frame_age = image_ok ? tools::delta_time(now, image_time) : -1;
    auto feedback_age = tools::delta_time(now, feedback.timestamp);
    auto_aim::FollowContext context;
    context.auto_mode = feedback.mode == io::GimbalMode::AUTO_AIM;
    context.feedback_valid = feedback.valid;
    context.dry_run = dry_run || exiter.exit();
    context.frame_age = frame_age;
    context.feedback_age = feedback_age;
    auto command = auto_aim::follow_command(aim, context, settings);
    gimbal.send(
      command.control, false, command.yaw, command.pitch, command.control ? target_number : 8);
    auto sent = gimbal.last_command();
    // send可能因过期反馈拒绝接管；记录其最终报文与写入结果，而非仅记录期望值。
    plotter.plot({
      {"t", tools::delta_time(now, start)},
      {"image_ok", image_ok ? 1 : 0},
      {"pose_ok", pose_ok ? 1 : 0},
      {"detected", detected},
      {"pnp_valid", pnp_valid},
      {"target_valid", aim.valid ? 1 : 0},
      {"requested_control", command.control ? 1 : 0},
      {"control", sent.written && sent.command.mode != 0 ? 1 : 0},
      {"fire", sent.command.mode == 2 ? 1 : 0},
      {"command_mode", sent.command.mode},
      {"send_ok", sent.written ? 1 : 0},
      {"yaw", static_cast<double>(sent.command.yaw)},
      {"pitch", static_cast<double>(sent.command.pitch)},
      {"aim_yaw", aim.yaw},
      {"aim_pitch", aim.pitch},
      {"gimbal_yaw", feedback.state.yaw},
      {"gimbal_pitch", feedback.state.pitch},
      {"yaw_error", aim.valid ? std::remainder(aim.yaw - feedback.state.yaw, 2 * CV_PI) : 0.0},
      {"pitch_error", aim.valid ? aim.pitch - feedback.state.pitch : 0.0},
      {"distance", distance},
      {"frame_age", frame_age},
      {"feedback_age", feedback_age},
      {"dry_run", dry_run ? 1 : 0},
    });
    if (show) {
      if (image_ok) cv::imshow("task_1", img);
      auto key = cv::waitKey(1);
      if (key == 'q' || key == 27) break;
    }
  }
  gimbal.send(false, false, 0, 0, 8);
  if (show) cv::destroyAllWindows();
  return 0;
}
}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto path = cli.get<std::string>("@config-path");
  auto show = cli.get<bool>("show");
  auto dry_run = cli.get<bool>("dry-run");
  if (cli.has("help") || path.empty()) {
    cli.printMessage();
    return 0;
  }
  if (!cli.check()) {
    cli.printErrors();
    return 1;
  }
  try {
    return run(path, show, dry_run);
  } catch (const std::exception & e) {
    tools::logger()->error("[Task1] Startup/runtime failure: {}", e.what());
    return 1;
  }
}
