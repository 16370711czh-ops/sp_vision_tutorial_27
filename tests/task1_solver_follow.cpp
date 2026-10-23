#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <stdexcept>

#include "../src/task_1_follow.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tools/quaternion_history.hpp"

int main(int argc, char ** argv)
{
  if (argc != 2) return 2;
  auto yaml = YAML::LoadFile(argv[1]);
  auto settings = auto_aim::load_follow_settings(yaml);
  int checks = 0;
  auto expect = [&](bool condition, const char * message) {
    ++checks;
    if (!condition) throw std::runtime_error(message);
  };
  // 独立相机投影夹具：camera(z,-x,-y)->gimbal(x,y,z)，不使用Solver重投影生成观测。
  yaml["camera_matrix"] = std::vector<double>{600, 0, 400, 0, 600, 300, 0, 0, 1};
  yaml["distort_coeffs"] = std::vector<double>{0, 0, 0, 0, 0};
  yaml["R_gimbal2imubody"] = std::vector<double>{1, 0, 0, 0, 1, 0, 0, 0, 1};
  yaml["R_camera2gimbal"] = std::vector<double>{0, 0, 1, -1, 0, 0, 0, -1, 0};
  yaml["t_camera2gimbal"] = std::vector<double>{0.04, 0.01, 0.1};
  yaml["t_yaw2pitch"] = std::vector<double>{0, 0, 0};
  const char * fixture = "task1_solver_follow_fixture.yaml";
  {
    std::ofstream stream(fixture);
    stream << yaml;
  }
  auto_aim::Solver solver(fixture);
  std::remove(fixture);
  cv::Mat intrinsic = (cv::Mat_<double>(3, 3) << 600, 0, 400, 0, 600, 300, 0, 0, 1);
  std::vector<cv::Point3f> corners{
    {0, 0.0675f, 0.028f}, {0, -0.0675f, 0.028f}, {0, -0.0675f, -0.028f}, {0, 0.0675f, -0.028f}};
  tools::QuaternionHistory poses;
  auto t0 = std::chrono::steady_clock::now();
  double yaw = 170 * CV_PI / 180, pitch = 0;
  double max_position_error = 0, max_command_error = 0;
  for (int frame = 0; frame <= 180; ++frame) {
    double truth_yaw = (170.0 + 2 * frame) * CV_PI / 180;
    double truth_pitch = 18 * CV_PI / 180 * std::sin(frame * CV_PI / 90);
    Eigen::Vector3d truth{
      3 * std::cos(truth_pitch) * std::cos(truth_yaw),
      3 * std::cos(truth_pitch) * std::sin(truth_yaw), -3 * std::sin(truth_pitch)};
    Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
    auto t = t0 + std::chrono::milliseconds(frame * 20);
    for (int sign : {-1, 1}) {
      Eigen::Quaterniond neighbor =
        Eigen::AngleAxisd(yaw + sign * 0.001, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
      expect(
        poses.push(neighbor, t + std::chrono::milliseconds(sign * 5)), "pose timestamp ordering");
    }
    Eigen::Quaterniond sampled;
    expect(poses.sample(t, sampled, std::chrono::milliseconds(0)), "pose bracket interpolation");
    solver.set_R_gimbal2world(sampled);
    Eigen::Vector3d center_body = q.conjugate() * truth;
    std::vector<cv::Point3f> camera_points;
    for (const auto & corner : corners) {
      Eigen::Vector3d body = center_body + Eigen::AngleAxisd(CV_PI / 12, Eigen::Vector3d::UnitY()) *
                                             Eigen::Vector3d(corner.x, corner.y, corner.z);
      camera_points.emplace_back(-(body[1] - 0.01), -(body[2] - 0.1), body[0] - 0.04);
    }
    std::vector<cv::Point2f> pixels;
    cv::projectPoints(
      camera_points, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), intrinsic,
      cv::Mat::zeros(1, 5, CV_64F), pixels);
    auto_aim::Armor armor(10, 0.99f, cv::boundingRect(pixels), pixels);
    solver.solve(armor);
    double position_error = (armor.xyz_in_world - truth).norm();
    max_position_error = std::max(max_position_error, position_error);
    expect(position_error < 0.001, "independent PnP/world position error < 1mm");
    auto aim = auto_aim::follow_angles(
      armor.xyz_in_world[0], armor.xyz_in_world[1], armor.xyz_in_world[2], yaw, 0, 0,
      settings.pitch_limit);
    auto_aim::FollowContext context{true, true, false, 0.02, 0.01};
    auto command = auto_aim::follow_command(aim, context, settings);
    expect(command.control, "valid synthetic observation controls");
    double error =
      std::max(std::abs(command.yaw - truth_yaw), std::abs(command.pitch - truth_pitch));
    max_command_error = std::max(max_command_error, error);
    expect(
      error < 0.001, "absolute aim follows truth, including world-negative-x and pi crossings");
    expect(std::abs(command.yaw - yaw) < 0.15, "no full-turn yaw jump");
    expect(std::abs(command.pitch) <= settings.pitch_limit + 1e-7, "pitch stays in range");
    // 理想一阶响应只是离线夹具，不代表真实机器人的控制动力学。
    yaw += 0.5 * (command.yaw - yaw);
    pitch += 0.5 * (command.pitch - pitch);
  }
  std::cout << "PASS solver_follow: " << checks
            << " checks, max_position_error=" << max_position_error
            << " m, max_command_error=" << max_command_error << " rad\n";
}
