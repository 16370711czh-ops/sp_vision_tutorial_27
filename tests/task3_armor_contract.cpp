#include <iostream>
#include <stdexcept>

#include "tasks/auto_aim/armor.hpp"

int main()
{
  const std::vector<cv::Point2f> points{{0, 0}, {129, 0}, {129, 56}, {0, 56}};
  int checks = 0;
  auto expect = [&](bool ok, const char * message) {
    ++checks;
    if (!ok) throw std::runtime_error(message);
  };
  for (int color : {0, 1}) {
    // Exercise the actual YOLOV5 overloads, not the single-class-id constructor used
    // by previous PnP fixtures. YOLOV5 class 6 must satisfy Task3's input contract.
    auto_aim::Armor normal(color, 6, 0.99f, cv::Rect(0, 0, 129, 56), points);
    auto_aim::Armor roi(color, 6, 0.99f, cv::Rect(0, 0, 129, 56), points, {37, 19});
    for (const auto * armor : {&normal, &roi}) {
      expect(armor->name == auto_aim::ArmorName::outpost, "YOLOV5 class6 outpost label");
      expect(armor->type == auto_aim::ArmorType::special,
             "YOLOV5 outpost must use special type accepted by Task3 and Solver");
      expect(armor->color == (color == 0 ? auto_aim::Color::blue : auto_aim::Color::red),
             "YOLOV5 red/blue contract");
    }
    expect(roi.points[0] == cv::Point2f(37, 19), "ROI offset retains corresponding corner");
    for (int num = 0; num <= 5; ++num) {
      auto_aim::Armor car(color, num, 0.99f, cv::Rect(0, 0, 129, 56), points);
      expect(car.type == (num == 1 ? auto_aim::ArmorType::big : auto_aim::ArmorType::small),
             "Task1/2 normal car types unchanged");
    }
  }
  std::cout << "PASS armor_contract: " << checks << " checks\n";
}
