// Helpers for the property tests: fixed-seed random inputs, so every run checks the same cases.

#ifndef DELIVERY_BOT_GEOMETRY_TEST_UTIL_HPP_
#define DELIVERY_BOT_GEOMETRY_TEST_UTIL_HPP_

#include <cstdint>
#include <random>

#include "delivery_bot_geometry/angle.hpp"
#include "delivery_bot_geometry/se2.hpp"

namespace delivery_bot_geometry::test
{

class RandomPoses
{
public:
  explicit RandomPoses(std::uint32_t seed)
  : engine_(seed)
  {
  }

  double uniform(double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(engine_); }

  /// Position in [-extent, extent]^2, yaw in [-pi, pi).
  Pose2 pose(double extent) { return {uniform(-extent, extent), uniform(-extent, extent), uniform(-kPi, kPi)}; }

private:
  std::mt19937 engine_;
};

}  // namespace delivery_bot_geometry::test

#endif  // DELIVERY_BOT_GEOMETRY_TEST_UTIL_HPP_
