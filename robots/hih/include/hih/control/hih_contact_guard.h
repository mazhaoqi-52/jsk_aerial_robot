#pragma once
#include <cmath>
#include <stdexcept>
#include <string>

namespace hih_control
{
// Local contact protection only. It never arms, lands or commands the peer.
struct ContactGuardParameters
{
  double acquire_force=0.8, release_force=0.4, max_force=20;
  double loss_timeout=0.2, stale_timeout=0.5;
  void validate() const
  {
    for (double x : {acquire_force,release_force,max_force,loss_timeout,stale_timeout})
      if (!std::isfinite(x) || x<=0) throw std::invalid_argument("Invalid contact guard parameter");
    if (release_force>=acquire_force || acquire_force>=max_force)
      throw std::invalid_argument("Contact thresholds must satisfy release < acquire < max");
  }
};

class ContactGuard
{
public:
  enum State { WAITING, ACTIVE, HOLD, FAULT };
  void configure(const ContactGuardParameters& p) { p.validate(); p_=p; reset(); }
  void reset() { state_=WAITING; acquired_=false; bad_since_=-1; status_="waiting_for_contact"; }
  void fault(const std::string& reason) { state_=FAULT; status_=reason; }
  State state() const { return state_; }
  bool acquired() const { return acquired_; }
  const std::string& status() const { return status_; }
  State step(double now, bool interval_ok, bool fresh, double normal_force)
  {
    if (state_==FAULT) return state_;
    if (fresh && std::isfinite(normal_force) && std::abs(normal_force)>=p_.max_force)
    { fault("normal_force_limit_hold"); return state_; }
    if (!interval_ok || !std::isfinite(now))
    { state_=acquired_ ? HOLD : WAITING; status_="control_gap_held"; return state_; }
    fresh = fresh && std::isfinite(normal_force);
    if (!acquired_)
    {
      if (!fresh || normal_force<p_.acquire_force)
      { status_="waiting_for_contact"; return state_=WAITING; }
      acquired_=true;
    }
    if (!fresh || normal_force<p_.release_force)
    {
      if (bad_since_<0 || now<bad_since_) bad_since_=now;
      state_=HOLD; status_=fresh ? "contact_loss_held" : "stale_input_held";
      if (now-bad_since_ >= (fresh ? p_.loss_timeout : p_.stale_timeout))
        fault(fresh ? "contact_lost_hold" : "mob_timeout_hold");
      return state_;
    }
    bad_since_=-1; state_=ACTIVE; status_="active"; return state_;
  }
private:
  ContactGuardParameters p_;
  State state_=WAITING;
  bool acquired_=false;
  double bad_since_=-1;
  std::string status_="waiting_for_contact";
};
}
