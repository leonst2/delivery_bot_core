#include "delivery_bot_geometry/reeds_shepp.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "curve_util.hpp"
#include "delivery_bot_geometry/angle.hpp"

// The base formulas are numbered as in Reeds & Shepp (1990), section 8. All of them work on the goal pose (x, y, phi)
// in the start frame with distances divided by the turning radius, and return the normalised signed segment lengths
// (t, u, v): arcs in rad, straights in units of the radius. "p" / "m" in a name = forward / reverse segment.

namespace delivery_bot_geometry
{

namespace
{

constexpr double kZero = 1e-10;  // sign conditions and range boundaries (rounding tolerance)
constexpr double kHalfPi = kPi / 2.0;

using Tuv = std::array<double, 3>;

struct Polar
{
  double r{0.0};
  double theta{0.0};
};

// Polar coordinates; the angle of a (numerically) zero vector is 0 instead of whatever atan2 makes of rounding noise.
Polar polar(double x, double y)
{
  const double r = std::hypot(x, y);
  return {r, r < 1e-12 ? 0.0 : std::atan2(y, x)};
}

// 8.1: L+ S+ L+
std::optional<Tuv> lp_sp_lp(double x, double y, double phi)
{
  const Polar p = polar(x - std::sin(phi), y - 1.0 + std::cos(phi));
  const double t = p.theta;
  const double v = normalize_angle(phi - t);
  if (t >= -kZero && v >= -kZero) {
    return Tuv{t, p.r, v};
  }
  return std::nullopt;
}

// 8.2: L+ S+ R+
std::optional<Tuv> lp_sp_rp(double x, double y, double phi)
{
  const Polar p = polar(x + std::sin(phi), y - 1.0 - std::cos(phi));
  const double r_sq = p.r * p.r;
  if (r_sq < 4.0 - kZero) {
    return std::nullopt;
  }
  const double u = std::sqrt(std::max(r_sq - 4.0, 0.0));
  const double t = normalize_angle(p.theta + std::atan2(2.0, u));
  const double v = normalize_angle(t - phi);
  if (t >= -kZero && v >= -kZero) {
    return Tuv{t, u, v};
  }
  return std::nullopt;
}

// 8.3: L+ R- L+ (the paper has a typo here; this is the corrected form)
std::optional<Tuv> lp_rm_l(double x, double y, double phi)
{
  const Polar p = polar(x - std::sin(phi), y - 1.0 + std::cos(phi));
  if (p.r > 4.0 + kZero) {
    return std::nullopt;
  }
  const double u = -2.0 * std::asin(std::min(0.25 * p.r, 1.0));
  const double t = normalize_angle(p.theta + 0.5 * u + kPi);
  const double v = normalize_angle(phi - t + u);
  if (t >= -kZero && u <= kZero) {
    return Tuv{t, u, v};
  }
  return std::nullopt;
}

struct TauOmega
{
  double tau{0.0};
  double omega{0.0};
};

// Helper of 8.7 / 8.8. Returns a struct rather than a std::pair: GCC notes an ABI change for pairs of doubles.
TauOmega tau_omega(double u, double v, double xi, double eta, double phi)
{
  const double delta = normalize_angle(u - v);
  const double a = std::sin(u) - std::sin(delta);
  const double b = std::cos(u) - std::cos(delta) - 1.0;
  const double t1 = std::atan2(eta * a - xi * b, xi * a + eta * b);
  const double t2 = 2.0 * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3.0;
  const double tau = t2 < 0.0 ? normalize_angle(t1 + kPi) : normalize_angle(t1);
  const double omega = normalize_angle(tau - u + v - phi);
  return {tau, omega};
}

// 8.7: L+ R+ L- R- (values t, u, -u, v)
std::optional<Tuv> lp_rup_lum_rm(double x, double y, double phi)
{
  const double xi = x + std::sin(phi);
  const double eta = y - 1.0 - std::cos(phi);
  const double rho = 0.25 * (2.0 + std::hypot(xi, eta));
  if (rho > 1.0 + kZero) {
    return std::nullopt;
  }
  const double u = std::acos(std::min(rho, 1.0));
  const auto [t, v] = tau_omega(u, -u, xi, eta, phi);
  if (t >= -kZero && v <= kZero) {
    return Tuv{t, u, v};
  }
  return std::nullopt;
}

// 8.8: L+ R- L- R+ (values t, u, u, v)
std::optional<Tuv> lp_rum_lum_rp(double x, double y, double phi)
{
  const double xi = x + std::sin(phi);
  const double eta = y - 1.0 - std::cos(phi);
  const double rho = (20.0 - xi * xi - eta * eta) / 16.0;
  if (rho < -kZero || rho > 1.0 + kZero) {
    return std::nullopt;
  }
  const double u = -std::acos(std::clamp(rho, -1.0, 1.0));
  if (u < -kHalfPi - kZero) {
    return std::nullopt;
  }
  const auto [t, v] = tau_omega(u, u, xi, eta, phi);
  if (t >= -kZero && v >= -kZero) {
    return Tuv{t, u, v};
  }
  return std::nullopt;
}

// 8.9: L+ R-(pi/2) S- L- (values t, -pi/2, u, v)
std::optional<Tuv> lp_rm_sm_lm(double x, double y, double phi)
{
  const Polar p = polar(x - std::sin(phi), y - 1.0 + std::cos(phi));
  if (p.r < 2.0 - kZero) {
    return std::nullopt;
  }
  const double r = std::sqrt(std::max(p.r * p.r - 4.0, 0.0));
  const double u = 2.0 - r;
  const double t = normalize_angle(p.theta + std::atan2(r, -2.0));
  const double v = normalize_angle(phi - kHalfPi - t);
  if (t >= -kZero && u <= kZero && v <= kZero) {
    return Tuv{t, u, v};
  }
  return std::nullopt;
}

// 8.10: L+ R-(pi/2) S- R- (values t, -pi/2, u, v)
std::optional<Tuv> lp_rm_sm_rm(double x, double y, double phi)
{
  const double xi = x + std::sin(phi);
  const double eta = y - 1.0 - std::cos(phi);
  const Polar p = polar(-eta, xi);
  if (p.r < 2.0 - kZero) {
    return std::nullopt;
  }
  const double t = p.theta;
  const double u = 2.0 - p.r;
  const double v = normalize_angle(t + kHalfPi - phi);
  if (t >= -kZero && u <= kZero && v <= kZero) {
    return Tuv{t, u, v};
  }
  return std::nullopt;
}

// 8.11: L+ R-(pi/2) S- L-(pi/2) R+ (values t, -pi/2, u, -pi/2, v)
std::optional<Tuv> lp_rm_slm_rp(double x, double y, double phi)
{
  const double xi = x + std::sin(phi);
  const double eta = y - 1.0 - std::cos(phi);
  const Polar p = polar(xi, eta);
  if (p.r < 2.0 - kZero) {
    return std::nullopt;
  }
  const double u = 4.0 - std::sqrt(std::max(p.r * p.r - 4.0, 0.0));
  if (u > kZero) {
    return std::nullopt;
  }
  const double t = normalize_angle(std::atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta));
  const double v = normalize_angle(t - phi);
  if (t >= -kZero && v >= -kZero) {
    return Tuv{t, u, v};
  }
  return std::nullopt;
}

// A word with its normalised signed lengths, at most five segments.
struct Word
{
  std::string letters;
  std::array<double, 5> values{};
  std::size_t count{0};
};

struct Formula
{
  const char * letters;  // base word
  std::optional<Tuv> (*solve)(double, double, double);
  Word (*expand)(const char *, const Tuv &);  // base word values from (t, u, v)
  bool backwards;                             // also apply to the backwards (time-reversed) problem
};

Word word3(const char * letters, const Tuv & tuv) { return {letters, {tuv[0], tuv[1], tuv[2], 0.0, 0.0}, 3}; }

Word word_ccccu(const char * letters, const Tuv & tuv)  // t, u, -u, v
{
  return {letters, {tuv[0], tuv[1], -tuv[1], tuv[2], 0.0}, 4};
}

Word word_cccc(const char * letters, const Tuv & tuv)  // t, u, u, v
{
  return {letters, {tuv[0], tuv[1], tuv[1], tuv[2], 0.0}, 4};
}

Word word_ccsc(const char * letters, const Tuv & tuv)  // t, -pi/2, u, v
{
  return {letters, {tuv[0], -kHalfPi, tuv[1], tuv[2], 0.0}, 4};
}

Word word_ccscc(const char * letters, const Tuv & tuv)  // t, -pi/2, u, -pi/2, v
{
  return {letters, {tuv[0], -kHalfPi, tuv[1], -kHalfPi, tuv[2]}, 5};
}

void time_flip(Word & word)
{
  for (std::size_t i = 0; i < word.count; ++i) {
    word.values.at(i) = -word.values.at(i);
  }
}

void reflect(Word & word)
{
  for (char & letter : word.letters) {
    if (letter == 'L') {
      letter = 'R';
    } else if (letter == 'R') {
      letter = 'L';
    }
  }
}

void reverse_order(Word & word)
{
  std::reverse(word.letters.begin(), word.letters.end());
  std::reverse(word.values.begin(), word.values.begin() + static_cast<std::ptrdiff_t>(word.count));
}

// Fixed order: the order of the candidates and with it the tie-breaking of reeds_shepp_shortest.
const std::array<Formula, 7> kFormulas{{
  {"LSL", &lp_sp_lp, &word3, false},
  {"LSR", &lp_sp_rp, &word3, false},
  {"LRL", &lp_rm_l, &word3, true},
  {"LRLR", &lp_rup_lum_rm, &word_ccccu, false},
  {"LRLR", &lp_rum_lum_rp, &word_cccc, false},
  {"LRSL", &lp_rm_sm_lm, &word_ccsc, true},
  {"LRSR", &lp_rm_sm_rm, &word_ccsc, true},
}};
const Formula kCcscc{"LRSLR", &lp_rm_slm_rp, &word_ccscc, false};

// Plain, time-flipped, reflected and both, for the problem (x, y, phi); reversed word order if `backwards`. Calls
// `sink(word)` for every feasible one.
template<typename Sink>
void add_variants(const Formula & formula, double x, double y, double phi, bool backwards, Sink & sink)
{
  struct Variant
  {
    double sx;
    double sy;
    double sphi;
    bool flip;
    bool mirror;
  };
  static constexpr std::array<Variant, 4> kVariants{{
    {1.0, 1.0, 1.0, false, false},
    {-1.0, 1.0, -1.0, true, false},
    {1.0, -1.0, -1.0, false, true},
    {-1.0, -1.0, 1.0, true, true},
  }};
  for (const Variant & variant : kVariants) {
    const std::optional<Tuv> tuv = formula.solve(variant.sx * x, variant.sy * y, variant.sphi * phi);
    if (!tuv) {
      continue;
    }
    Word word = formula.expand(formula.letters, *tuv);
    if (variant.flip) {
      time_flip(word);
    }
    if (variant.mirror) {
      reflect(word);
    }
    if (backwards) {
      reverse_order(word);
    }
    sink(word);
  }
}

// Every feasible word from start to goal, in the fixed candidate order.
template<typename Sink>
void for_each_word(const Pose2 & start, const Pose2 & goal, double radius, Sink && sink)
{
  detail::check_radius(radius);
  const Pose2 rel = between(start, goal);
  const double x = rel.x / radius;
  const double y = rel.y / radius;
  const double phi = rel.yaw;
  // The backwards problem: drive from the goal to the start in reverse time.
  const double xb = x * std::cos(phi) + y * std::sin(phi);
  const double yb = x * std::sin(phi) - y * std::cos(phi);
  for (const Formula & formula : kFormulas) {
    add_variants(formula, x, y, phi, false, sink);
    if (formula.backwards) {
      add_variants(formula, xb, yb, phi, true, sink);
    }
  }
  add_variants(kCcscc, x, y, phi, false, sink);
}

// The length Path would report for this word: the same segment threshold and summation order as Path's
// constructor, so comparisons (and with them the tie-breaking) are bitwise identical to comparing built paths.
double word_length(const Word & word, double radius)
{
  constexpr double kMinSegmentLength = 1e-12;  // as in path.cpp
  double total = 0.0;
  for (std::size_t i = 0; i < word.count; ++i) {
    const double length = word.values.at(i) * radius;
    if (std::abs(length) >= kMinSegmentLength) {
      total = total + std::abs(length);
    }
  }
  return total;
}

}  // namespace

std::vector<CurveCandidate> reeds_shepp_all(const Pose2 & start, const Pose2 & goal, double radius)
{
  std::vector<CurveCandidate> candidates;
  for_each_word(start, goal, radius, [&](const Word & word) {
    candidates.push_back(detail::make_candidate(start, radius, word.letters, word.values.data(), word.count));
  });
  return candidates;
}

CurveCandidate reeds_shepp_shortest(const Pose2 & start, const Pose2 & goal, double radius)
{
  // Pick by length first and build only the winner: same choice as building every candidate, far less work.
  Word best;
  double best_length = std::numeric_limits<double>::infinity();
  for_each_word(start, goal, radius, [&](const Word & word) {
    const double length = word_length(word, radius);
    if (length < best_length) {
      best_length = length;
      best = word;
    }
  });
  if (best.count == 0) {
    // Some word reaches every goal (Reeds & Shepp 1990); only rounding could get here.
    throw std::logic_error("reeds_shepp_shortest: no feasible word");
  }
  return detail::make_candidate(start, radius, best.letters, best.values.data(), best.count);
}

double reeds_shepp_length(const Pose2 & start, const Pose2 & goal, double radius)
{
  double best_length = std::numeric_limits<double>::infinity();
  for_each_word(
    start, goal, radius, [&](const Word & word) { best_length = std::min(best_length, word_length(word, radius)); });
  if (!std::isfinite(best_length)) {
    throw std::logic_error("reeds_shepp_length: no feasible word");
  }
  return best_length;
}

}  // namespace delivery_bot_geometry
