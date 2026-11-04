#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace hih_control
{
using Vector4 = std::array<double, 4>;

// Translational entries: N, m. Fourth entry: world-Z torque (Nm), yaw (rad).
struct AdmittanceParameters
{
  Vector4 mass{{1.5, 1.5, 1.0, 1.0}};
  Vector4 damping{{20.0, 20.0, 5.0, 2.0}};
  Vector4 stiffness{{0.0, 0.0, 0.0, 0.0}};
  Vector4 max_offset{{0.15, 0.15, 0.10, 0.15}};
  Vector4 max_velocity{{0.10, 0.10, 0.08, 0.15}};
  Vector4 max_acceleration{{0.30, 0.30, 0.20, 0.40}};
  Vector4 max_wrench{{10.0, 10.0, 10.0, 1.0}};
  Vector4 deadband{{0.5, 0.5, 0.5, 0.05}};
  Vector4 bias{{0.0, 0.0, 0.0, 0.0}};
  Vector4 gain{{1.0, 1.0, 1.0, 1.0}};
  double filter_cutoff_hz = 2.0;
  double max_dt = 0.1;
  bool moving_reference = false;
  Vector4 max_travel{{1.0, 1.0, 0.5, 2.0}}; // Total displacement from contact acquisition.

  void validate() const
  {
    for (int i = 0; i < 4; ++i)
    {
      const double values[] = {mass[i], damping[i], stiffness[i], max_offset[i],
        max_velocity[i], max_acceleration[i], max_wrench[i], deadband[i], bias[i], gain[i], max_travel[i]};
      for (double v : values)
        if (!std::isfinite(v)) throw std::invalid_argument("Admittance parameters must be finite");
      if (mass[i] <= 0 || damping[i] < 0 || stiffness[i] < 0 || max_offset[i] < 0 ||
          max_velocity[i] <= 0 || max_acceleration[i] <= 0 || max_wrench[i] <= 0 || deadband[i] < 0)
        throw std::invalid_argument("Invalid Admittance mass/damping/stiffness/limit");
      if (moving_reference && (stiffness[i] != 0 || max_travel[i] < max_offset[i] || max_travel[i] <= 0))
        throw std::invalid_argument("Moving reference requires K=0 and max_travel >= max_offset > limit");
    }
    if (!std::isfinite(filter_cutoff_hz) || filter_cutoff_hz < 0 ||
        !std::isfinite(max_dt) || max_dt <= 0)
      throw std::invalid_argument("Invalid Admittance filter or time limit");
  }
};

// Independent of ROS so the same numerical code can be tested without an aircraft.
class Admittance4D
{
public:
  explicit Admittance4D(const AdmittanceParameters& p = AdmittanceParameters()) : p_(p)
  { p_.validate(); reset(); }
  void configure(const AdmittanceParameters& p) { p.validate(); p_ = p; reset(); }
  void reset() { offset_.fill(0); reference_.fill(0); velocity_.fill(0); filtered_.fill(0); }
  void hold(bool clear_input=false) { velocity_.fill(0); if (clear_input) filtered_.fill(0); }
  const Vector4& offset() const { return offset_; }
  const Vector4& velocity() const { return velocity_; }
  const Vector4& filteredWrench() const { return filtered_; }
  const Vector4& referenceOffset() const { return reference_; }
  Vector4 totalOffset() const
  { Vector4 total; for (int i=0; i<4; ++i) total[i]=reference_[i]+offset_[i]; return total; }

  // M*xdd + D*xd + K*x = deadband(gain * contact-wrench-error).
  // Backward Euler with small substeps; bounds prevent integrator windup.
  bool step(const Vector4& wrench, double dt, bool valid_sample = true)
  {
    if (!std::isfinite(dt) || dt <= 0 || dt > p_.max_dt) return false;
    for (double f : wrench) if (!std::isfinite(f)) return false;
    const auto old_offset=offset_, old_velocity=velocity_, old_filter=filtered_;
    const int steps = std::max(1, static_cast<int>(std::ceil(dt / 0.005)));
    const double h = dt / steps;
    for (int s = 0; s < steps; ++s)
      for (int i = 0; i < 4; ++i)
      {
        if (p_.max_offset[i] == 0) { offset_[i] = velocity_[i] = filtered_[i] = 0; continue; }
        double input = valid_sample ? p_.gain[i] * (wrench[i] - p_.bias[i]) : 0.0;
        input = std::copysign(std::max(0.0, std::abs(input) - p_.deadband[i]), input);
        input = std::clamp(input, -p_.max_wrench[i], p_.max_wrench[i]);
        const double alpha = p_.filter_cutoff_hz == 0 ? 1.0 :
          h / (h + 1.0 / (2.0 * 3.14159265358979323846 * p_.filter_cutoff_hz));
        filtered_[i] += alpha * (input - filtered_[i]);
        double v = (p_.mass[i] * velocity_[i] + h * (filtered_[i] - p_.stiffness[i] * offset_[i])) /
          (p_.mass[i] + h * p_.damping[i] + h * h * p_.stiffness[i]);
        v = std::clamp(v, velocity_[i] - p_.max_acceleration[i] * h,
                         velocity_[i] + p_.max_acceleration[i] * h);
        v = std::clamp(v, -p_.max_velocity[i], p_.max_velocity[i]);
        const double x = offset_[i] + h * v;
        offset_[i] = std::clamp(x, -p_.max_offset[i], p_.max_offset[i]);
        // An outward velocity at the displacement bound must not keep integrating.
        velocity_[i] = ((x >= p_.max_offset[i] && v > 0) || (x <= -p_.max_offset[i] && v < 0)) ? 0.0 : v;
      }
    if (p_.moving_reference)
    {
      const auto total = totalOffset();
      for (int i=0; i<4; ++i)
        if (std::abs(total[i]) >= p_.max_travel[i])
        { offset_=old_offset; velocity_=old_velocity; filtered_=old_filter; return false; }
      for (int i=0; i<4; ++i)
        if (p_.max_offset[i] > 0 && std::abs(offset_[i]) >= 0.5*p_.max_offset[i])
        { reference_[i] += offset_[i]; offset_[i]=0; }
    }
    return true;
  }

private:
  AdmittanceParameters p_;
  Vector4 offset_, reference_, velocity_, filtered_;
};
}  // namespace hih_control
