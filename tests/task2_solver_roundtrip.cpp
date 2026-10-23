#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "tasks/auto_aim/solver.hpp"
#include "tools/math_tools.hpp"

int main(int argc, char ** argv)
{
  if (argc != 2) return 2;
  auto_aim::Solver solver(argv[1]);
  int checks = 0;
  auto expect = [&](bool condition, const char * message) {
    ++checks;
    if (!condition) throw std::runtime_error(message);
  };
  // 独立解析轴向夹具：相机(z,-x,-y)映射到云台(x,y,z)，不调用Solver重投影生成输入。
  auto fixture_yaml = YAML::LoadFile(argv[1]);
  fixture_yaml["camera_matrix"] = std::vector<double>{600, 0, 400, 0, 600, 300, 0, 0, 1};
  fixture_yaml["distort_coeffs"] = std::vector<double>{0, 0, 0, 0, 0};
  fixture_yaml["R_gimbal2imubody"] = std::vector<double>{1, 0, 0, 0, 1, 0, 0, 0, 1};
  fixture_yaml["R_camera2gimbal"] = std::vector<double>{0, 0, 1, -1, 0, 0, 0, -1, 0};
  fixture_yaml["t_camera2gimbal"] = std::vector<double>{0.04, 0.01, 0.1};
  fixture_yaml["t_yaw2pitch"] = std::vector<double>{0, 0, 0};
  const char * fixture_path = "task2_solver_analytic_fixture.yaml";
  {
    std::ofstream stream(fixture_path);
    stream << fixture_yaml;
  }
  auto_aim::Solver analytic(fixture_path);
  Eigen::Vector3d center{3.04, 0.01, 0.1};
  std::vector<cv::Point3f> object_points{
    {0, 0.0675f, 0.028f}, {0, -0.0675f, 0.028f}, {0, -0.0675f, -0.028f}, {0, 0.0675f, -0.028f}};
  std::vector<cv::Point3f> camera_points;
  for (const auto & point : object_points) {
    Eigen::Vector3d body = center + Eigen::AngleAxisd(CV_PI / 12, Eigen::Vector3d::UnitY()) *
                                      Eigen::Vector3d(point.x, point.y, point.z);
    camera_points.emplace_back(-(body[1] - 0.01), -(body[2] - 0.1), body[0] - 0.04);
  }
  cv::Mat intrinsic = (cv::Mat_<double>(3, 3) << 600, 0, 400, 0, 600, 300, 0, 0, 1);
  std::vector<cv::Point2f> analytic_points;
  cv::projectPoints(
    camera_points, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), intrinsic, cv::Mat::zeros(1, 5, CV_64F),
    analytic_points);
  auto_aim::Armor analytic_armor(10, 0.99f, cv::boundingRect(analytic_points), analytic_points);
  analytic.set_R_gimbal2world(Eigen::Quaterniond::Identity());
  analytic.solve(analytic_armor);
  expect(
    (analytic_armor.xyz_in_world - center).norm() < 0.001,
    "independent camera-forward axis fixture");
  Eigen::Quaterniond quarter_turn(Eigen::AngleAxisd(CV_PI / 2, Eigen::Vector3d::UnitZ()));
  analytic.set_R_gimbal2world(quarter_turn);
  analytic.solve(analytic_armor);
  expect(
    (analytic_armor.xyz_in_world - Eigen::Vector3d(-0.01, 3.04, 0.1)).norm() < 0.001,
    "known world yaw rotates body-forward axis into world-left axis");
  std::remove(fixture_path);
  for (auto type : {auto_aim::ArmorType::small, auto_aim::ArmorType::big}) {
    for (auto angle : {-0.3, 0.0, 0.3}) {
      Eigen::Quaterniond q = Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()) *
                             Eigen::AngleAxisd(-0.05, Eigen::Vector3d::UnitY());
      solver.set_R_gimbal2world(q);
      Eigen::Vector3d truth{3.0, 0.2, 0.15};
      auto points = solver.reproject_armor(truth, angle, type, auto_aim::ArmorName::three);
      auto_aim::Armor armor(10, 0.99f, cv::boundingRect(points), points);
      armor.type = type;
      solver.solve(armor);
      expect(
        (armor.xyz_in_world - truth).norm() < 0.01, "PnP/camera/gimbal/world round trip failed");
      expect(
        std::abs(std::remainder(armor.ypr_in_world[0] - angle, 2 * CV_PI)) < 2 * CV_PI / 180,
        "optimized armor yaw does not match fixture");
      expect(armor.xyz_in_gimbal[0] > 0, "target must be in front of camera/gimbal");
      auto point = solver.world2pixel(truth);
      expect(point && std::isfinite(point->x) && std::isfinite(point->y), "world-to-pixel finite");
    }
  }
  Eigen::Quaterniond invalid(0, 0, 0, 0);
  bool rejected = false;
  try {
    solver.set_R_gimbal2world(invalid);
  } catch (const std::exception &) {
    rejected = true;
  }
  expect(rejected, "zero quaternion must be rejected");
  auto points = std::vector<cv::Point2f>(4, {0, 0});
  auto_aim::Armor armor(10, 0.99f, cv::Rect(0, 0, 1, 1), points);
  rejected = false;
  try {
    solver.solve(armor);
  } catch (const std::exception &) {
    rejected = true;
  }
  expect(rejected, "degenerate keypoints must be rejected");
  std::cout << "PASS solver_roundtrip: " << checks << " checks\n";
}
