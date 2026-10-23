#include <iostream>
#include <stdexcept>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/task3_pipeline.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/plotter.hpp"

int main(int argc, char ** argv)
{
  if (argc != 2) return 2;
  auto_aim::YOLO detector(argv[1], false);  // CPU model only; no Camera/Gimbal object.
  auto_aim::Solver solver(argv[1]);
  solver.set_R_gimbal2world(Eigen::Quaterniond::Identity());
  int checks = 0;
  auto expect = [&](bool ok, const char * message) {
    ++checks;
    if (!ok) throw std::runtime_error(message);
  };
  const auto pixels = solver.reproject_armor(
    {3, 0.1, 0.25}, 0.1, auto_aim::ArmorType::special, auto_aim::ArmorName::outpost);
  cv::Mat image = cv::Mat::zeros(1024, 1500, CV_8UC3);
  Eigen::Vector4d observation;
  for (int color : {0, 1}) {
    // Supply a positive network-output fixture to the real public postprocess API.
    // This proves decoder->Armor->type gate->Solver integration, not model recognition.
    cv::Mat output = cv::Mat::zeros(1, 22, CV_32F);
    for (int i = 0; i < 4; ++i) {
      const int wire_index = (4 - i) % 4;
      output.at<float>(0, 2 * wire_index) = pixels[i].x;
      output.at<float>(0, 2 * wire_index + 1) = pixels[i].y;
    }
    output.at<float>(0, 8) = 10;
    output.at<float>(0, 9 + color) = 10;
    output.at<float>(0, 13 + 6) = 10;  // actual YOLOV5 class index for outpost
    auto armors = detector.postprocess(1.0, output, image, 0);
    expect(armors.size() == 1, "positive outpost row survives actual detector filters");
    auto armor = armors.front();
    expect(armor.name == auto_aim::ArmorName::outpost && armor.type == auto_aim::ArmorType::special,
           "actual decoded object satisfies Task3's class/type filter");
    expect(armor.color == (color ? auto_aim::Color::red : auto_aim::Color::blue),
           "positive row preserves enemy color");
    solver.solve(armor);
    observation = {armor.xyz_in_world[0], armor.xyz_in_world[1], armor.xyz_in_world[2],
                   armor.ypr_in_world[0]};
    expect((armor.xyz_in_world - Eigen::Vector3d(3, 0.1, 0.25)).norm() < 0.002,
           "decoded outpost uses special dimensions in PnP");
  }
  expect(detector.detect(cv::Mat()).empty(), "empty model input stays empty");
  expect(detector.detect(image, 1).empty(), "real CPU model forward on black frame has no detections");
  auto_aim::Task3Pipeline pipeline(auto_aim::load_task3_settings(argv[1]), false, true);
  auto_aim::Task3Frame frame;
  frame.image_valid = frame.pose_valid = frame.auto_mode = frame.feedback_fresh = true;
  frame.bullet_speed = 23;
  frame.observations = {observation};
  auto_aim::Task3Result result;
  for (int i = 0; i < 4; ++i) {
    frame.image_time = std::chrono::steady_clock::time_point{} + std::chrono::milliseconds(i * 20);
    frame.now = frame.image_time + std::chrono::milliseconds(10);
    result = pipeline.step(frame);
  }
  expect(result.observed && !result.control && !result.fire, "decoded PnP pose enters actual dry-run pipeline");
  const int receiver = ::socket(AF_INET, SOCK_DGRAM, 0);
  expect(receiver >= 0, "create UDP receiver");
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  expect(::bind(receiver, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0, "bind loopback");
  socklen_t length = sizeof(address);
  expect(::getsockname(receiver, reinterpret_cast<sockaddr *>(&address), &length) == 0, "ephemeral port");
  timeval timeout{1, 0};
  expect(::setsockopt(receiver, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == 0, "bounded receive");
  tools::Plotter plotter("127.0.0.1", ntohs(address.sin_port));
  const nlohmann::json packet{{"yaw", result.aim.yaw}, {"pitch", result.aim.pitch},
    {"angular_velocity", pipeline.tracker().target()->ekf_x()[7]},
    {"requested_control", 0}, {"control", 0}, {"fire", 0}, {"send_ok", 0}};
  plotter.plot(packet);
  char bytes[2048];
  const auto size = ::recv(receiver, bytes, sizeof(bytes), 0);
  expect(size > 0, "real Plotter datagram received");
  expect(nlohmann::json::parse(bytes, bytes + size) == packet, "Task3 numeric fields survive UDP/JSON");
  ::close(receiver);
  std::cout << "PLOTTER_PACKET=" << packet.dump() << '\n';
  std::cout << "PASS detector_contract: " << checks << " checks\n";
}
