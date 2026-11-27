#include "delivery_bot_geometry/dubins.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "curve_util.hpp"
#include "delivery_bot_geometry/angle.hpp"

namespace delivery_bot_geometry
{

namespace
{

// Start and goal in the frame where the start-goal line is the x axis, distances divided by the radius.
struct Normalized
{
  double d{0.0};  // distance / radius
  double alpha{0.0};
  double beta{0.0};
  double sa{0.0};
  double sb{0.0};
  double ca{0.0};
  double cb{0.0};
  double c_ab{0.0};
};

using Lengths = std::array<double, 3>;  // normalised: arcs in rad, the straight in units of the radius

// Rounding can push p^2 of a zero-length straight (an exact C or CC path) or the cosine of a CCC word just outside
// its valid range, which would drop the optimal word. Values within this tolerance count as on the boundary.
constexpr double kFeasibilityTol = 1e-10;

// sqrt(p^2), or nullopt if the word is infeasible.
std::optional<double> straight_length(double p_sq)
{
  if (p_sq < -kFeasibilityTol) {
    return std::nullopt;
  }
  return std::sqrt(std::max(p_sq, 0.0));
}

// The cosine of the middle arc of a CCC word clamped to [-1, 1], or nullopt if the word is infeasible.
std::optional<double> middle_cosine(double c)
{
  if (std::abs(c) > 1.0 + kFeasibilityTol) {
    return std::nullopt;
  }
  return std::clamp(c, -1.0, 1.0);
}

std::optional<Lengths> lsl(const Normalized & n)
{
  const std::optional<double> p = straight_length(2.0 + n.d * n.d - 2.0 * n.c_ab + 2.0 * n.d * (n.sa - n.sb));
  if (!p) {
    return std::nullopt;
  }
  const double tmp = std::atan2(n.cb - n.ca, n.d + n.sa - n.sb);
  return Lengths{normalize_angle_positive(tmp - n.alpha), *p, normalize_angle_positive(n.beta - tmp)};
}

std::optional<Lengths> rsr(const Normalized & n)
{
  const std::optional<double> p = straight_length(2.0 + n.d * n.d - 2.0 * n.c_ab + 2.0 * n.d * (n.sb - n.sa));
  if (!p) {
    return std::nullopt;
  }
  const double tmp = std::atan2(n.ca - n.cb, n.d - n.sa + n.sb);
  return Lengths{normalize_angle_positive(n.alpha - tmp), *p, normalize_angle_positive(tmp - n.beta)};
}

std::optional<Lengths> lsr(const Normalized & n)
{
  const std::optional<double> p = straight_length(-2.0 + n.d * n.d + 2.0 * n.c_ab + 2.0 * n.d * (n.sa + n.sb));
  if (!p) {
    return std::nullopt;
  }
  const double tmp = std::atan2(-n.ca - n.cb, n.d + n.sa + n.sb) - std::atan2(-2.0, *p);
  return Lengths{normalize_angle_positive(tmp - n.alpha), *p, normalize_angle_positive(tmp - n.beta)};
}

std::optional<Lengths> rsl(const Normalized & n)
{
  const std::optional<double> p = straight_length(-2.0 + n.d * n.d + 2.0 * n.c_ab - 2.0 * n.d * (n.sa + n.sb));
  if (!p) {
    return std::nullopt;
  }
  const double tmp = std::atan2(n.ca + n.cb, n.d - n.sa - n.sb) - std::atan2(2.0, *p);
  return Lengths{normalize_angle_positive(n.alpha - tmp), *p, normalize_angle_positive(n.beta - tmp)};
}

std::optional<Lengths> rlr(const Normalized & n)
{
  const std::optional<double> c = middle_cosine((6.0 - n.d * n.d + 2.0 * n.c_ab + 2.0 * n.d * (n.sa - n.sb)) / 8.0);
  if (!c) {
    return std::nullopt;
  }
  const double phi = std::atan2(n.ca - n.cb, n.d - n.sa + n.sb);
  const double p = normalize_angle_positive(kTwoPi - std::acos(*c));
  const double t = normalize_angle_positive(n.alpha - phi + normalize_angle_positive(p / 2.0));
  return Lengths{t, p, normalize_angle_positive(n.alpha - n.beta - t + p)};
}

std::optional<Lengths> lrl(const Normalized & n)
{
  const std::optional<double> c = middle_cosine((6.0 - n.d * n.d + 2.0 * n.c_ab + 2.0 * n.d * (n.sb - n.sa)) / 8.0);
  if (!c) {
    return std::nullopt;
  }
  const double phi = std::atan2(n.ca - n.cb, n.d + n.sa - n.sb);
  const double p = normalize_angle_positive(kTwoPi - std::acos(*c));
  const double t = normalize_angle_positive(-n.alpha - phi + p / 2.0);
  return Lengths{t, p, normalize_angle_positive(n.beta - n.alpha - t + p)};
}

}  // namespace

std::vector<CurveCandidate> dubins_all(const Pose2 & start, const Pose2 & goal, double radius)
{
  detail::check_radius(radius);
  const double dx = goal.x - start.x;
  const double dy = goal.y - start.y;
  const double distance = std::hypot(dx, dy);
  const double theta = distance > 0.0 ? std::atan2(dy, dx) : 0.0;

  Normalized n;
  n.d = distance / radius;
  n.alpha = normalize_angle_positive(start.yaw - theta);
  n.beta = normalize_angle_positive(goal.yaw - theta);
  n.sa = std::sin(n.alpha);
  n.sb = std::sin(n.beta);
  n.ca = std::cos(n.alpha);
  n.cb = std::cos(n.beta);
  n.c_ab = std::cos(n.alpha - n.beta);

  struct Word
  {
    const char * letters;
    std::optional<Lengths> (*solve)(const Normalized &);
  };
  static constexpr std::array<Word, 6> kWords{
    {{"LSL", &lsl}, {"RSR", &rsr}, {"LSR", &lsr}, {"RSL", &rsl}, {"RLR", &rlr}, {"LRL", &lrl}}};

  std::vector<CurveCandidate> candidates;
  for (const Word & word : kWords) {
    const std::optional<Lengths> lengths = word.solve(n);
    if (lengths) {
      candidates.push_back(detail::make_candidate(start, radius, word.letters, lengths->data(), lengths->size()));
    }
  }
  return candidates;
}

CurveCandidate dubins_shortest(const Pose2 & start, const Pose2 & goal, double radius)
{
  std::vector<CurveCandidate> candidates = dubins_all(start, goal, radius);
  if (candidates.empty()) {
    // LSL is always feasible: its p^2 is a sum of two squares.
    throw std::logic_error("dubins_shortest: no feasible word");
  }
  return detail::shortest(std::move(candidates));
}

}  // namespace delivery_bot_geometry
