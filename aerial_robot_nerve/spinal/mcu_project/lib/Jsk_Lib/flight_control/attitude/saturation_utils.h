#pragma once

#include <cmath>
#include <cstdint>

namespace spinal_saturation
{
inline float maxBaseThrustScale(const float* base,
                                const float* roll_pitch,
                                uint8_t rotor_coef,
                                float thrust_limit)
{
  if (!base || !roll_pitch || rotor_coef == 0 ||
      !std::isfinite(thrust_limit) || thrust_limit <= 0.0f) {
    return 0.0f;
  }

  float a = 0.0f;
  float b = 0.0f;
  float c = -thrust_limit * thrust_limit;
  for (uint8_t i = 0; i < rotor_coef; ++i) {
    if (!std::isfinite(base[i]) || !std::isfinite(roll_pitch[i])) return 0.0f;
    a += base[i] * base[i];
    b += 2.0f * base[i] * roll_pitch[i];
    c += roll_pitch[i] * roll_pitch[i];
  }

  if (a + b + c <= 0.0f) return 1.0f;
  if (a <= 1.0e-12f) return 0.0f;

  const float discriminant = b * b - 4.0f * a * c;
  if (discriminant < 0.0f) return 0.0f;

  const float sqrt_discriminant = std::sqrt(discriminant);
  const float root_low = (-b - sqrt_discriminant) / (2.0f * a);
  const float root_high = (-b + sqrt_discriminant) / (2.0f * a);
  const float feasible_low = root_low > 0.0f ? root_low : 0.0f;
  const float feasible_high = root_high < 1.0f ? root_high : 1.0f;
  if (feasible_high < feasible_low || feasible_high < 0.0f) return 0.0f;
  return feasible_high;
}
}  // namespace spinal_saturation
