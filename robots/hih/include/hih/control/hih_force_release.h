#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace hih_control
{
struct ForceReleaseParameters
{
  double rate=1, normal_tolerance=.3, force_tolerance=.5, torque_tolerance=.1;
  double speed_tolerance=.05, omega_tolerance=.05;
  double command_speed_tolerance=.005, command_omega_tolerance=.01;
  double settle_time=1, timeout=30, data_timeout=.5;
  double max_distance=.15, max_yaw=.15;
  void validate() const
  {
    for (double x : {rate,normal_tolerance,force_tolerance,torque_tolerance,
      speed_tolerance,omega_tolerance,command_speed_tolerance,command_omega_tolerance,
      settle_time,timeout,data_timeout,max_distance,max_yaw})
      if (!std::isfinite(x) || x<=0) throw std::invalid_argument("Invalid force-release parameter");
    if (settle_time>=timeout || data_timeout>=timeout)
      throw std::invalid_argument("Force-release timeout must exceed dwell/data timeout");
  }
};

// Pure local release logic. Zero requested force alone never means completion.
class ForceRelease
{
public:
  enum Phase { OFF, RELEASING, READY, HOVER, FAILED };
  void configure(const ForceReleaseParameters& p) { p.validate(); p_=p; reset(); }
  void reset() { phase_=OFF; reference_=0; started_=0; stable_since_=bad_since_=-1; status_="idle"; }
  bool request(double now, double current_reference, double measured_normal)
  {
    if (phase_!=OFF || !std::isfinite(now) || !std::isfinite(current_reference) ||
        !std::isfinite(measured_normal)) return false;
    reference_=std::max(0.0,std::min(current_reference,measured_normal));
    started_=now; stable_since_=bad_since_=-1;
    phase_=RELEASING; status_="releasing"; return true;
  }
  bool advance(double now, double dt, bool fresh, double normal, double max_force, double max_dt)
  {
    if (phase_!=RELEASING) return false;
    if (!std::isfinite(now)) { fail("release_invalid_clock_hold"); return false; }
    if (now<started_ || now-started_>p_.timeout)
    { fail("release_timeout_hold"); return false; }
    if (fresh && std::isfinite(normal) && std::abs(normal)>=max_force)
    { fail("release_force_limit_hold"); return false; }
    if (!fresh || !std::isfinite(normal) || !std::isfinite(dt) || dt<=0 || dt>max_dt)
    {
      stable_since_=-1;
      if (bad_since_<0) bad_since_=now;
      status_="release_data_or_control_gap_held";
      if (now-bad_since_>=p_.data_timeout) fail("release_data_timeout_hold");
      return false;
    }
    bad_since_=-1; status_="releasing";
    // Do not chase a retreating hand with a larger requested compression.
    reference_=std::max(0.0,std::min(reference_,normal)-p_.rate*dt); return true;
  }
  void observe(double now, double normal, double force, double torque,
               bool motion_valid, double speed, double omega,
               double command_speed, double command_omega, double distance, double yaw)
  {
    if (phase_!=RELEASING) return;
    if (!std::isfinite(distance) || !std::isfinite(yaw) ||
        distance>p_.max_distance || std::abs(yaw)>p_.max_yaw)
    { fail("release_motion_limit_hold"); return; }
    bool finite=true;
    for(double x : {normal,force,torque,speed,omega,command_speed,command_omega})
      finite=finite && std::isfinite(x);
    const bool settled=finite && motion_valid && reference_==0 &&
      std::abs(normal)<=p_.normal_tolerance && force<=p_.force_tolerance &&
      torque<=p_.torque_tolerance && speed<=p_.speed_tolerance && omega<=p_.omega_tolerance &&
      command_speed<=p_.command_speed_tolerance && command_omega<=p_.command_omega_tolerance;
    if (!settled) stable_since_=-1;
    else if (stable_since_<0) stable_since_=now;
    else if (now-stable_since_>=p_.settle_time) { phase_=READY; status_="release_handoff_pending"; }
  }
  void complete() { if (phase_==READY) { phase_=HOVER; status_="released_hover"; } }
  void fail(const std::string& reason) { phase_=FAILED; status_=reason; stable_since_=-1; }
  Phase phase() const { return phase_; }
  bool pending() const { return phase_==RELEASING || phase_==READY; }
  double reference() const { return reference_; }
  const std::string& status() const { return status_; }
private:
  ForceReleaseParameters p_;
  Phase phase_=OFF;
  double reference_=0,started_=0,stable_since_=-1,bad_since_=-1;
  std::string status_="idle";
};
}
