#include "../src/task_1_angles.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace
{
constexpr double PI = 3.14159265358979323846;
constexpr double LIMIT = 20 * PI / 180;
int checks = 0;

void expect(bool condition, const char * message)
{
  ++checks;
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

bool near(double a, double b) { return std::abs(a - b) < 1e-9; }
}  // namespace

int main()
{
  using auto_aim::follow_angles;

  auto front = follow_angles(3, 0, 0, 0, 0, 0, LIMIT);
  expect(front.valid && near(front.yaw, 0) && near(front.pitch, 0), "front target");

  auto left = follow_angles(3, 3, 0, 0, 0, 0, LIMIT);
  auto right = follow_angles(3, -3, 0, 0, 0, 0, LIMIT);
  expect(left.valid && near(left.yaw, PI / 4), "left target yaw positive");
  expect(right.valid && near(right.yaw, -PI / 4), "right target yaw negative");

  auto up = follow_angles(3, 0, 0.3, 0, 0, 0, LIMIT);
  auto down = follow_angles(3, 0, -0.3, 0, 0, 0, LIMIT);
  expect(up.valid && near(up.pitch, -std::atan2(0.3, 3.0)), "up means negative pitch");
  expect(down.valid && near(down.pitch, std::atan2(0.3, 3.0)), "down means positive pitch");

  auto high = follow_angles(1, 0, 10, 0, 0, 0, LIMIT);
  auto low = follow_angles(1, 0, -10, 0, 0, 0, LIMIT);
  expect(high.valid && near(high.pitch, -LIMIT), "upper pitch limit");
  expect(low.valid && near(low.pitch, LIMIT), "lower pitch limit");

  // 目标在-179度，当前在+179度：应发+181度，而不是造成-358度的差。
  auto current = 179 * PI / 180;
  auto target = -179 * PI / 180;
  auto wrap = follow_angles(std::cos(target), std::sin(target), 0, current, 0, 0, LIMIT);
  expect(wrap.valid && near(wrap.yaw - current, 2 * PI / 180), "positive pi crossing");

  current = -179 * PI / 180;
  target = 179 * PI / 180;
  wrap = follow_angles(std::cos(target), std::sin(target), 0, current, 0, 0, LIMIT);
  expect(wrap.valid && near(wrap.yaw - current, -2 * PI / 180), "negative pi crossing");

  auto turns = follow_angles(3, 0, 0, 8 * PI + 0.1, 0, 0, LIMIT);
  expect(turns.valid && near(turns.yaw, 8 * PI), "multi-turn yaw feedback");

  auto offset = follow_angles(3, 0, 0, 0, 0.02, -0.03, LIMIT);
  expect(offset.valid && near(offset.yaw, 0.02) && near(offset.pitch, -0.03), "offset signs");

  auto rear = follow_angles(-3, 0.01, 0, PI, 0, 0, LIMIT);
  expect(rear.valid && std::abs(rear.yaw - PI) < 0.01, "world x negative is valid");

  expect(!follow_angles(0, 0, 0, 0, 0, 0, LIMIT).valid, "zero distance rejected");
  expect(!follow_angles(0, 0, 3, 0, 0, 0, LIMIT).valid, "undefined yaw rejected");
  expect(!follow_angles(3, 0, 0, 0, 0, 0, 0).valid, "invalid pitch limit rejected");
  expect(!follow_angles(3, 0, 0, 0, 0, 0, PI / 2).valid, "vertical pitch limit rejected");

  const auto nan = std::numeric_limits<double>::quiet_NaN();
  const auto inf = std::numeric_limits<double>::infinity();
  expect(!follow_angles(nan, 0, 0, 0, 0, 0, LIMIT).valid, "NaN xyz rejected");
  expect(!follow_angles(3, inf, 0, 0, 0, 0, LIMIT).valid, "infinite xyz rejected");
  expect(!follow_angles(3, 0, 0, nan, 0, 0, LIMIT).valid, "NaN feedback rejected");
  expect(!follow_angles(3, 0, 0, 0, inf, 0, LIMIT).valid, "invalid offset rejected");

  // 在完整yaw范围扫描，验证指令与目标方向一致，且离当前yaw不超过pi。
  for (int degree = -180; degree <= 180; ++degree) {
    auto angle = degree * PI / 180;
    auto command = follow_angles(3 * std::cos(angle), 3 * std::sin(angle), 0.2, 5, 0, 0, LIMIT);
    expect(command.valid, "yaw sweep valid");
    expect(std::abs(command.yaw - 5) <= PI + 1e-9, "yaw sweep shortest equivalent");
    expect(near(std::sin(command.yaw), std::sin(angle)), "yaw sweep sine");
    expect(near(std::cos(command.yaw), std::cos(angle)), "yaw sweep cosine");
  }

  std::cout << "PASS: " << checks << " angle checks\n";
  return 0;
}
