#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <stdexcept>

#include "tasks/auto_aim/solver.hpp"
#include "tools/math_tools.hpp"

int main(int argc, char ** argv)
{
  if (argc != 2) return 2;
  int checks = 0;
  auto expect = [&](bool ok, const char * message) {
    ++checks;
    if (!ok) throw std::runtime_error(message);
  };
  auto yaml = YAML::LoadFile(argv[1]);
  yaml["camera_matrix"] = std::vector<double>{1000, 0, 640, 0, 1000, 360, 0, 0, 1};
  yaml["distort_coeffs"] = std::vector<double>{0, 0, 0, 0, 0};
  yaml["R_gimbal2imubody"] = std::vector<double>{1, 0, 0, 0, 1, 0, 0, 0, 1};
  yaml["R_camera2gimbal"] = std::vector<double>{0, 0, 1, -1, 0, 0, 0, -1, 0};
  yaml["t_camera2gimbal"] = std::vector<double>{0.04, 0.01, 0.1};
  yaml["t_yaw2pitch"] = std::vector<double>{0.02, 0, 0.03};
  const char * path = "task3_solver_fixture.yaml";
  { std::ofstream stream(path); stream << yaml; }
  auto_aim::Solver solver(path);
  // Independent known projection, not Solver::reproject_armor. Shared 129mm/56mm keypoint
  // geometry and -15deg outpost plate pitch are checked against known world poses.
  const Eigen::Matrix3d camera_to_body =
    (Eigen::Matrix3d() << 0, 0, 1, -1, 0, 0, 0, -1, 0).finished();
  const Eigen::Vector3d camera_origin(0.04, 0.01, 0.1);
  const cv::Mat intrinsic = (cv::Mat_<double>(3, 3) << 1000, 0, 640, 0, 1000, 360, 0, 0, 1);
  for (double body_yaw : {-0.2, 0.0, 0.2}) {
    Eigen::Quaterniond q = Eigen::AngleAxisd(body_yaw, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(-0.07, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(0.03, Eigen::Vector3d::UnitX());
    solver.set_R_gimbal2world(q);
    const Eigen::Vector3d origin_offset =
      Eigen::AngleAxisd(body_yaw, Eigen::Vector3d::UnitZ()) * Eigen::Vector3d(0.02, 0, 0.03);
    for (double armor_yaw : {-0.3, 0.1, 0.4}) {
      for (double z : {0.166, 0.266, 0.366}) {
        const Eigen::Vector3d center(3.0, 0.1, z);
        const Eigen::Matrix3d armor_rotation =
          (Eigen::AngleAxisd(armor_yaw, Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(-CV_PI / 12, Eigen::Vector3d::UnitY())).toRotationMatrix();
        std::vector<cv::Point3f> points_in_camera;
        for (const Eigen::Vector3d & corner :
             std::vector<Eigen::Vector3d>{{0, 0.0645, 0.028}, {0, -0.0645, 0.028},
                                          {0, -0.0645, -0.028}, {0, 0.0645, -0.028}}) {
          Eigen::Vector3d camera_point = camera_to_body.transpose() *
            (q.conjugate() * (center + armor_rotation * corner - origin_offset) - camera_origin);
          points_in_camera.emplace_back(camera_point[0], camera_point[1], camera_point[2]);
        }
        std::vector<cv::Point2f> pixels;
        cv::projectPoints(points_in_camera, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0),
                          intrinsic, cv::Mat::zeros(1, 5, CV_64F), pixels);
        auto_aim::Armor armor(10, 0.99f, cv::boundingRect(pixels), pixels);
        armor.name = auto_aim::ArmorName::outpost;
        armor.type = auto_aim::ArmorType::special;
        solver.solve(armor);
        expect((armor.xyz_in_world - center).norm() < 0.002,
               "outpost known world pose including camera/axis translations and quaternion");
        expect(std::abs(std::remainder(armor.ypr_in_world[0] - armor_yaw, 2 * CV_PI)) < CV_PI / 90,
               "negative 15deg outpost pitch and optimized yaw match independent projection");
      }
    }
  }
  // Ensure Task3's reprojection gate is used (not silently infinite as in the old integration).
  yaml["task3"]["max_reprojection_error_px"] = 0;
  { std::ofstream stream(path); stream << yaml; }
  bool rejected = false;
  try { auto_aim::Solver invalid(path); }
  catch (const std::invalid_argument &) { rejected = true; }
  expect(rejected, "Solver reads Task3 reprojection gate");

  // A 180deg processed image needs calibration in the SAME image convention.
  // This fixture does not decide how the user's physical camera was calibrated.
  const cv::Mat raw_k = (cv::Mat_<double>(3, 3) << 1000, 0, 610, 0, 1100, 350, 0, 0, 1);
  const cv::Mat raw_d = (cv::Mat_<double>(1, 5) << -0.02, 0.01, 0.001, -0.002, 0);
  const Eigen::Vector3d truth(3, 0.15, 0.3), axis_offset(0.02, 0, 0.03);
  const Eigen::Matrix3d board_rotation =
    (Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()) *
     Eigen::AngleAxisd(-CV_PI / 12, Eigen::Vector3d::UnitY())).toRotationMatrix();
  std::vector<cv::Point3f> raw_camera;
  for (const Eigen::Vector3d & corner :
       std::vector<Eigen::Vector3d>{{0, 0.0645, 0.028}, {0, -0.0645, 0.028},
                                    {0, -0.0645, -0.028}, {0, 0.0645, -0.028}}) {
    Eigen::Vector3d p = camera_to_body.transpose() *
      (truth + board_rotation * corner - axis_offset - camera_origin);
    raw_camera.emplace_back(p[0], p[1], p[2]);
  }
  std::vector<cv::Point2f> rotated_pixels;
  cv::projectPoints(raw_camera, cv::Vec3d(), cv::Vec3d(), raw_k, raw_d, rotated_pixels);
  for (auto & pixel : rotated_pixels) pixel = {1499 - pixel.x, 1023 - pixel.y};
  yaml["task3"]["max_reprojection_error_px"] = 5;
  yaml["camera_matrix"] = std::vector<double>{1000, 0, 610, 0, 1100, 350, 0, 0, 1};
  yaml["distort_coeffs"] = std::vector<double>{-0.02, 0.01, 0.001, -0.002, 0};
  { std::ofstream stream(path); stream << yaml; }
  auto_aim::Solver mismatched(path);
  auto_aim::Armor rotated(18, 0.99f, cv::boundingRect(rotated_pixels), rotated_pixels);
  mismatched.solve(rotated);
  expect((rotated.xyz_in_world - truth).norm() > 0.1,
         "unrotated calibration on rotated pixels can pass PnP yet give wrong world geometry");
  yaml["camera_matrix"] = std::vector<double>{1000, 0, 889, 0, 1100, 673, 0, 0, 1};
  yaml["distort_coeffs"] = std::vector<double>{-0.02, 0.01, -0.001, 0.002, 0};
  Eigen::Matrix3d rotated_camera = camera_to_body * Eigen::Vector3d(-1, -1, 1).asDiagonal();
  std::vector<double> extrinsic;
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col) extrinsic.push_back(rotated_camera(row, col));
  yaml["R_camera2gimbal"] = extrinsic;
  { std::ofstream stream(path); stream << yaml; }
  auto_aim::Solver matched(path);
  matched.solve(rotated);
  expect((rotated.xyz_in_world - truth).norm() < 0.002,
         "consistent rotated K, tangential distortion, extrinsic and physical corner ids recover truth");
  std::remove(path);
  std::cout << "PASS task3 solver: " << checks << " checks\n";
}
