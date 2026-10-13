#include "annin_ar4_driver/gripper_overcurrent_protection.hpp"

#include <algorithm>

namespace annin_ar4_driver {

GripperOverCurrentProtection::GripperOverCurrentProtection(
    const rclcpp::Logger& logger, const rclcpp::Clock& clock)
    : logger_(logger), clock_(clock) {}

void GripperOverCurrentProtection::AddCurrentSample(
    const rclcpp::Time& /*time*/, double current) {
  samples_[sample_idx_] = current;
  sample_idx_ = (sample_idx_ + 1) % kFilterWindow;
  if (sample_count_ < kFilterWindow) {
    ++sample_count_;
  }

  // Median of the window rather than the raw sample: one ACS712 spike must not
  // be able to read as contact. Sorting 3 doubles every read cycle is free.
  std::array<double, kFilterWindow> sorted;
  std::copy_n(samples_.begin(), sample_count_, sorted.begin());
  std::sort(sorted.begin(), sorted.begin() + sample_count_);
  prev_current_ = current_;
  current_ = sorted[sample_count_ / 2];
}

double GripperOverCurrentProtection::AdjustGripperPosition(
    double position_command, double min_position, double max_position,
    const rclcpp::Time& now) {
  // Seed the output tracker on the first call so the ramp starts from wherever
  // the gripper already is rather than snapping.
  if (!output_init_) {
    output_pos_ = position_command;
    last_time_ = now;
    output_init_ = true;
    return output_pos_;
  }

  double dt = (now - last_time_).seconds();
  last_time_ = now;
  // Guard against clock jumps / the very first cycles: fall back to a nominal
  // step so a bad dt can't fling the command across the whole range at once.
  if (dt <= 0.0 || dt > 0.5) {
    dt = 0.02;
  }

  const double range = max_position - min_position;
  const bool commanded_closing =
      (position_command - min_position) < grip_intent_fraction_ * range;

  // Opening, or a command looser than where we already are: follow it straight
  // through and re-arm the grasp. "Normal when opening."
  if (position_command >= output_pos_ || !commanded_closing) {
    output_pos_ = position_command;
    grip_frozen_ = false;
    high_streak_ = 0;
    return output_pos_;
  }

  // Commanded to close further than we are. Watch the current for contact,
  // either as an absolute level or as a sharp rise into a rigid object.
  const bool over_threshold = current_ > contact_current_threshold_;
  // The rise test needs a full window behind it: while the filter is still
  // priming, prev_current_ is seeded at 0 and every sample looks like a jump.
  const bool sharp_rise = sample_count_ == kFilterWindow &&
                          (current_ - prev_current_) >= contact_rise_per_cycle_ &&
                          current_ >= contact_rise_floor_;
  // A sustained over-threshold reading needs the debounce; a sharp rise fires on
  // the spot (see the header — a slope test cannot survive a debounce).
  high_streak_ = over_threshold ? high_streak_ + 1 : 0;
  if (!grip_frozen_ && (sharp_rise || high_streak_ >= contact_debounce_)) {
    grip_frozen_ = true;
    // Relax off the trip point: detection lags the touch, so output_pos_ is
    // already past what the jaws can reach around the object. Commanding that
    // blocked angle is the stall that browns out the rail — back off to an
    // angle the servo can actually hold and let it clamp there.
    output_pos_ = std::min(max_position, output_pos_ + contact_backoff_);
    RCLCPP_INFO(logger_,
                "Gripper contact at %.4f m (%.2f A, %s) - holding grasp here "
                "instead of driving to full close (avoids rail brownout).",
                output_pos_, current_,
                over_threshold ? "over threshold" : "sharp rise");
  }

  // Clamped onto an object: hold this gentle contact position. Driving further
  // closed is exactly what browns out the rail, so we don't.
  if (grip_frozen_) {
    return output_pos_;
  }

  // No contact yet: creep closed at the rate limit (never faster), so the
  // current is sampled as it rises into contact rather than spiking past it.
  const double step = close_velocity_ * dt;
  output_pos_ = std::max(position_command, output_pos_ - step);
  return output_pos_;
}

}  // namespace annin_ar4_driver
