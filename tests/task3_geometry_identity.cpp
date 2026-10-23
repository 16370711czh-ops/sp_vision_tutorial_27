#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>

#include "tasks/auto_aim/outpost_tracker.hpp"
#include "tools/math_tools.hpp"

namespace
{
int checks = 0;
double largest_position_error = 0, largest_height_error = 0, largest_omega_error = 0;
void expect(bool ok, const std::string & message)
{
  ++checks;
  if (!ok) throw std::runtime_error(message);
}
auto stamp(double t)
{
  return std::chrono::steady_clock::time_point{} +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(t));
}

// Independent rigid-body fixture: explicit body-fixed vertices and inward normals,
// rotated with a 3D rotation matrix. Heights/radius are SIMULATION PARAMETERS.
// The supplied raster drawing does NOT prove that these numbers are plate centers.
struct Fixture
{
  double radius = 0.2765;
  std::array<double, 3> heights;
  Eigen::Vector3d center;
  Eigen::Vector4d plate(double phase, int physical_id) const
  {
    const double root = std::sqrt(3.0) / 2;
    const Eigen::Vector3d body[3] = {
      {-radius, 0, heights[0]}, {radius / 2, -root * radius, heights[1]},
      {radius / 2, root * radius, heights[2]}};
    const Eigen::Vector3d normal[3] = {{1, 0, 0}, {-0.5, root, 0}, {-0.5, -root, 0}};
    const Eigen::Matrix3d rotation =
      Eigen::AngleAxisd(phase, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d p = center + rotation * body[physical_id];
    const Eigen::Vector3d n = rotation * normal[physical_id];
    return {p[0], p[1], p[2], std::atan2(n[1], n[0])};
  }
};

void scenario(std::array<double, 3> heights, int initial_id, double omega, double bearing, int seed)
{
  Fixture fixture{0.2765, heights, {3 * std::cos(bearing), 3 * std::sin(bearing), 0}};
  const double phase0 = bearing - initial_id * 2 * CV_PI / 3 + 0.08;
  auto_aim::OutpostTracker tracker;
  std::mt19937 rng(seed);
  std::normal_distribution<double> position_noise(0, 0.003), angle_noise(0, 0.008);
  int accepted = 0, identity_samples = 0, switches = 0, previous = -1;
  for (int frame = 0; frame < 600; ++frame) {
    const double t = frame * 0.02;
    std::vector<Eigen::Vector4d> observations;
    if (frame == 0 || frame % 31 != 0) {
      for (int id = 0; id < 3; ++id) {
        Eigen::Vector4d p = fixture.plate(phase0 + omega * t, id);
        const double facing = std::abs(std::remainder(p[3] - std::atan2(p[1], p[0]), 2 * CV_PI));
        if (facing > 55 * CV_PI / 180) continue;
        if (frame != 0) {
          for (int axis = 0; axis < 3; ++axis) p[axis] += position_noise(rng);
          p[3] = std::remainder(p[3] + angle_noise(rng), 2 * CV_PI);
        }
        observations.push_back(p);
      }
    }
    if (frame == 0) {
      expect(observations.size() == 1, "known physical initial plate is uniquely visible");
      expect((observations.front() - fixture.plate(phase0, initial_id)).norm() < 1e-10,
             "fixture initial id is correct before tracking");
    }
    // Top clutter explicitly outside configured window, plus wrong pose and wrong location.
    auto top = fixture.plate(phase0 + omega * t, 0);
    top[2] = 0.9;
    observations.push_back(top);
    observations.push_back({8, 4, 0.25, bearing});
    const bool ok = tracker.update(observations, stamp(t));
    accepted += ok;
    expect(tracker.target().has_value(), "short omissions do not lose the same target");
    if (ok && tracker.target()->last_id != previous) ++switches;
    if (ok) previous = tracker.target()->last_id;
    if (frame < 250) continue;
    ++identity_samples;
    const auto predicted = tracker.target()->armor_xyza_list();
    const double omega_error = std::abs(tracker.target()->ekf_x()[7] - omega);
    largest_omega_error = std::max(largest_omega_error, omega_error);
    expect(omega_error < 0.4, "omega matches independent known signed truth");
    for (int model_id = 0; model_id < 3; ++model_id) {
      // This mapping is fixed at initialization, NEVER re-sorted or rematched by height.
      const int physical_id = (initial_id + model_id) % 3;
      const auto actual = fixture.plate(phase0 + omega * t, physical_id);
      const double position_error = (predicted[model_id].head<3>() - actual.head<3>()).norm();
      const double height_error = std::abs(predicted[model_id][2] - actual[2]);
      largest_position_error = std::max(largest_position_error, position_error);
      largest_height_error = std::max(largest_height_error, height_error);
      expect(position_error < 0.025, "predicted position belongs to the SAME physical plate id");
      expect(height_error < 0.02, "height stays attached to its physical phase, not sorted set");
      expect(std::abs(std::remainder(predicted[model_id][3] - actual[3], 2 * CV_PI)) < 0.06,
             "plate phase ordering remains positive in space for either spin direction");
    }
  }
  expect(accepted > 400 && identity_samples == 350 && switches > 8,
         "scenario exercises sustained tracking and repeated phase switches");
  expect(tracker.seen_mask() == 7 && tracker.ready(), "all physical ids observed and warmed up");
}

void jacobian_observability()
{
  for (double bearing : {-CV_PI + 1e-7, 0.0, CV_PI - 1e-7}) {
    Fixture fixture{0.2765, {0.22, 0.13, 0.34}, {3 * std::cos(bearing), 3 * std::sin(bearing), 0}};
    auto_aim::Target target(fixture.plate(bearing, 0), auto_aim::ArmorName::outpost,
                            auto_aim::ArmorType::special, stamp(0), fixture.radius, 3);
    auto state = target.ekf_x();
    state[9] = -0.09;
    state[10] = 0.12;
    for (double phase : {-CV_PI + 1e-7, CV_PI - 1e-7, 0.0}) {
      state[6] = phase;
      for (int id = 0; id < 3; ++id) {
        const auto h = target.observation_jacobian(state, id);
        for (int column = 0; column < 11; ++column) {
          auto plus = state, minus = state;
          plus[column] += 1e-6;
          minus[column] -= 1e-6;
          Eigen::Vector4d delta = target.observation_model(plus, id) - target.observation_model(minus, id);
          for (int row : {0, 1, 3}) delta[row] = std::remainder(delta[row], 2 * CV_PI);
          expect((delta / 2e-6 - h.col(column)).norm() < 2e-6, "full H central difference across wrap");
        }
      }
    }
    // Linearized local observability with known plate ids at two times.
    Eigen::MatrixXd stacked(24, 11);
    int row = 0;
    for (double dt : {0.0, 0.1}) {
      Eigen::MatrixXd f = Eigen::MatrixXd::Identity(11, 11);
      f(6, 7) = dt;
      auto future = state;
      future[6] += 2.5 * dt;
      for (int id = 0; id < 3; ++id) {
        stacked.middleRows(row, 4) = target.observation_jacobian(future, id) * f;
        row += 4;
      }
    }
    Eigen::MatrixXd active(24, 8);
    const int indices[] = {0, 2, 4, 6, 7, 8, 9, 10};
    for (int i = 0; i < 8; ++i) active.col(i) = stacked.col(indices[i]);
    expect(active.colPivHouseholderQr().rank() == 8, "eight active quantities locally observable with ids/times");
    expect(stacked.col(1).isZero() && stacked.col(3).isZero() && stacked.col(5).isZero(),
           "three reserved velocity states are explicitly unobservable and decoupled");
  }
}

void limitations_are_visible()
{
  // Negative control: an omega-perfect but phase/height-swapped predictor must fail our oracle.
  Fixture fixture{0.2765, {0.22, 0.13, 0.34}, {3, 0, 0}};
  std::array<double, 3> heights{0.22, 0.13, 0.34}, swapped{0.13, 0.22, 0.34};
  auto sorted_a = heights, sorted_b = swapped;
  std::sort(sorted_a.begin(), sorted_a.end());
  std::sort(sorted_b.begin(), sorted_b.end());
  expect(sorted_a == sorted_b && std::abs(heights[0] - swapped[0]) > 0.08,
         "height sorting would hide a large per-id geometry error at perfect omega");
  auto_aim::OutpostTracker tracker;
  expect(!tracker.update({{2.72, 0, 0.9, 0}}, stamp(0)), "outside-window top rejected");
  expect(tracker.update({{2.72, 0, 0.3, 0}}, stamp(0.02)),
         "counterexample: top misregistered INSIDE window is indistinguishable to height-only gate");
  expect(!tracker.ready(), "single ambiguous board is not a ready three-board estimate");

  // Deliberately wrong radius prior: a counterexample to omega accuracy implying
  // correct full geometry. This is not a measurement of the actual outpost radius.
  Fixture wrong_radius_truth{0.24, {0.22, 0.13, 0.34}, {3, 0, 0}};
  auto_aim::OutpostTracker biased;
  for (int frame = 0; frame < 600; ++frame) {
    const double t = frame * 0.02;
    std::vector<Eigen::Vector4d> measurements;
    for (int id = 0; id < 3; ++id) {
      auto p = wrong_radius_truth.plate(0.08 + 2.5 * t, id);
      if (std::abs(std::remainder(p[3] - std::atan2(p[1], p[0]), 2 * CV_PI)) < 55 * CV_PI / 180)
        measurements.push_back(p);
    }
    biased.update(measurements, stamp(t));
  }
  expect(biased.target().has_value(), "wrong-radius demonstration maintains target");
  double worst_position = 0;
  const auto estimate = biased.target()->armor_xyza_list();
  for (int id = 0; id < 3; ++id)
    worst_position = std::max(worst_position,
      (estimate[id].head<3>() - wrong_radius_truth.plate(0.08 + 2.5 * 11.98, id).head<3>()).norm());
  const double omega_error = std::abs(biased.target()->ekf_x()[7] - 2.5);
  expect(omega_error < 0.4 && worst_position > 0.02,
         "accurate omega can coexist with wrong plate positions when strong radius prior is wrong");
  std::cout << "RADIUS_COUNTEREXAMPLE true_r=0.24 prior_r=0.2765 estimated_r="
            << biased.target()->ekf_x()[8] << " omega_error=" << omega_error
            << " max_plate_error=" << worst_position << " ready=" << biased.ready() << '\n';
}
}  // namespace

int main()
{
  std::array<double, 3> heights{0.13, 0.22, 0.34};
  int scenarios = 0;
  do {
    for (int initial_id = 0; initial_id < 3; ++initial_id)
      for (double omega : {-2.5, 2.5})
        for (double bearing : {-2.4, 0.0, 2.4})
          scenario(heights, initial_id, omega, bearing, ++scenarios);
  } while (std::next_permutation(heights.begin(), heights.end()));
  jacobian_observability();
  limitations_are_visible();
  std::cout << "PASS geometry_identity: scenarios=" << scenarios << " checks=" << checks
            << " max_position_error=" << largest_position_error
            << " max_height_error=" << largest_height_error
            << " max_omega_error=" << largest_omega_error << '\n';
}
