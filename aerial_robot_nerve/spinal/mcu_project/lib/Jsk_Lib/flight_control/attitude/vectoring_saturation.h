#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace spinal_saturation
{
struct VectoringScales
{
  float roll_pitch = 1.0f;
  float base = 1.0f;
  float yaw = 1.0f;
  bool valid = false;
};

namespace detail
{
// Intersect [lo, hi] with ||offset_scale * offset + s * direction|| <= limit.
// Both roots matter: RP alone can exceed the limit while base cancels it.
inline bool intersectBall(const float* offset, float offset_scale,
                          const float* direction, int dimensions, float limit,
                          float& lo, float& hi)
{
  float a = 0.0f, b = 0.0f, c = -limit * limit;
  for (int j = 0; j < dimensions; ++j) {
    const float value = offset_scale * offset[j];
    a += direction[j] * direction[j];
    b += value * direction[j];
    c += value * value;
  }
  if (!std::isfinite(a) || !std::isfinite(b) || !std::isfinite(c)) return false;
  if (a == 0.0f) return c <= 0.0f;
  const float discriminant = b * b - a * c;
  if (!std::isfinite(discriminant) || discriminant < 0.0f) return false;
  // Stable quadratic roots (avoid cancellation near s=0).
  const float q = -b - std::copysign(std::sqrt(discriminant), b);
  float root_lo, root_hi;
  if (q == 0.0f) {
    root_lo = root_hi = -b / a;
  } else {
    root_lo = q / a;
    root_hi = c / q;
    if (root_lo > root_hi) std::swap(root_lo, root_hi);
  }
  lo = std::max(lo, root_lo);
  hi = std::min(hi, root_hi);
  return lo <= hi;
}

inline bool commonBaseScale(const float* base, const float* rp, int rotors,
                            int dimensions, float limit, float rp_scale, float& scale)
{
  float lo = 0.0f, hi = 1.0f;
  for (int i = 0; i < rotors; ++i) {
    if (!intersectBall(rp + i * dimensions, rp_scale, base + i * dimensions,
                       dimensions, limit, lo, hi)) return false;
  }
  scale = hi;
  return true;
}
}  // namespace detail

// Vectoring rotors only (2 or 3 force components per rotor). Limits are in
// rotor-vector units: caller multiplies per-propeller limits by rotor_devider_.
// Priority: RP > base > separately transmitted yaw. A single scale is shared
// by all rotors, and all scales stay in [0,1]. No allocation or ROS dependency.
inline VectoringScales vectoringScales(const float* base, const float* rp,
                                       const float* yaw, int rotors, int dimensions,
                                       float max_thrust, float min_thrust)
{
  VectoringScales result;
  if (!base || !rp || !yaw || rotors <= 0 || dimensions < 2 || dimensions > 3 ||
      !std::isfinite(max_thrust) || max_thrust <= 0.0f ||
      !std::isfinite(min_thrust) || min_thrust < 0.0f || min_thrust > max_thrust)
    return result;
  float max_rp_squared = 0.0f;
  for (int i = 0; i < rotors; ++i) {
    float squared = 0.0f;
    for (int j = 0; j < dimensions; ++j) {
      const int k = i * dimensions + j;
      if (!std::isfinite(base[k]) || !std::isfinite(rp[k]) || !std::isfinite(yaw[k]))
        return result;
      squared += rp[k] * rp[k];
    }
    max_rp_squared = std::max(max_rp_squared, squared);
  }
  if (!std::isfinite(max_rp_squared)) return result;

  if (!detail::commonBaseScale(base, rp, rotors, dimensions, max_thrust,
                               result.roll_pitch, result.base)) {
    if (max_rp_squared == 0.0f) return result;  // arithmetic overflow, not RP infeasibility
    // No common base scale can accommodate RP. Only in this case reduce RP;
    // this makes base=0 feasible for every rotor. Leave float rounding margin.
    const float margin = 1.0f - 8.0f * std::numeric_limits<float>::epsilon();
    result.roll_pitch = std::min(1.0f, max_thrust / std::sqrt(max_rp_squared)) * margin;
    if (!detail::commonBaseScale(base, rp, rotors, dimensions, max_thrust,
                                 result.roll_pitch, result.base)) return result;
  }

  // Shed separate yaw completely when base/RP was reduced, as in the legacy
  // priority policy. Otherwise include yaw's actual vector in the limit check.
  result.yaw = 0.0f;
  if (result.base == 1.0f && result.roll_pitch == 1.0f) {
    float lo = 0.0f, hi = 1.0f;
    for (int i = 0; i < rotors; ++i) {
      float baseline[3] = {};
      float a = 0.0f, b = 0.0f, c = -min_thrust * min_thrust;
      for (int j = 0; j < dimensions; ++j) {
        const int k = i * dimensions + j;
        baseline[j] = base[k] + rp[k];
        a += yaw[k] * yaw[k];
        b += baseline[j] * yaw[k];
        c += baseline[j] * baseline[j];
      }
      if (!detail::intersectBall(baseline, 1.0f, yaw + i * dimensions,
                                 dimensions, max_thrust, lo, hi)) return result;
      // Keep the feasible interval connected to yaw=0: do not let opposing
      // yaw pass through zero thrust and emerge with a reversed vector.
      // If baseline is already below minimum, the existing PWM floor applies.
      const float discriminant = b * b - a * c;
      if (min_thrust > 0.0f && c >= 0.0f && b < 0.0f && discriminant > 0.0f) {
        const float entry = c / (-b + std::sqrt(discriminant));
        hi = std::min(hi, entry);
      }
    }
    result.yaw = std::max(0.0f, hi);
  }
  result.valid = true;
  return result;
}
}  // namespace spinal_saturation
