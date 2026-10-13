#pragma once

#include <array>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace annin_ar4_driver {

// Current-limited grasp for the servo gripper.
//
// Background (2026-07-24): the Hiwonder HPS-3518SG coreless servo has very low
// winding resistance, so a hard stall demands more current than the fixed 5 V RC
// supply can deliver through its leads (~0.6 Ohm of droop). Commanding the jaws
// straight to full-close against an object collapses the rail from 5.4 V to <4 V,
// the servo browns out, releases, recovers and retries in a ~2 s loop — it never
// grips. A held, gentle clamp draws a steady ~1.75 A with NO sag, though.
//
// So instead of "drive to full close and rely on an overcurrent cutout", this
// policy CREEPS the jaws closed at a rate limit and FREEZES the commanded
// position the instant the current rises past a contact threshold. That keeps
// the servo in its stable ~1.75 A holding regime and out of the brownout regime,
// and it is size-agnostic — it stops wherever contact happens. It is also gentler
// on any servo (the cored DS3230MG replacement included), since it never hard-
// stalls, which is what cooks a servo.
class GripperOverCurrentProtection {
 public:
  GripperOverCurrentProtection(const rclcpp::Logger& logger,
                               const rclcpp::Clock& clock);

  // Feed the latest measured servo current (Amps) each read cycle.
  void AddCurrentSample(const rclcpp::Time& time, double current);

  // Returns the position to actually command to the servo this cycle.
  //
  // Opening (or any command looser than the current output) passes straight
  // through and re-arms the grasp. A closing command creeps toward the target at
  // close_velocity_ until the current exceeds contact_current_threshold_ for
  // contact_debounce_ consecutive cycles, then holds the jaws at that contact
  // position instead of driving on to full close.
  double AdjustGripperPosition(double position_command, double min_position,
                               double max_position, const rclcpp::Time& now);

 private:
  double current_ = 0.0;  // median-filtered, see AddCurrentSample

  // Median filter over the raw current samples. The ACS712 5 A sensor on the
  // Nano throws isolated spikes, and three of them in a row is all
  // contact_debounce_ asks for — which is how the jaws came to declare contact
  // in free air. A median discards a lone spike outright, where a mean would
  // still drag the estimate over the threshold.
  //
  // The window is deliberately SHORT. Every sample of filtering is a sample of
  // lag, and lag here is measured in stall: a 5-wide window plus a 3-cycle
  // debounce put ~8 cycles between the jaws touching the cube and the freeze,
  // which is long enough for the servo to drag the rail down and brown out.
  // 3-wide still rejects an isolated spike (one sample cannot move a median of
  // three) at roughly half the delay.
  static constexpr int kFilterWindow = 3;
  std::array<double, kFilterWindow> samples_{};
  int sample_idx_ = 0;
  int sample_count_ = 0;
  double prev_current_ = 0.0;  // previous filtered value, for the rise-rate trip

  // Rate-limited-close + contact-freeze state.
  bool output_init_ = false;      // output_pos_ seeded yet?
  double output_pos_ = 0.0;       // the position we actually command (ramps)
  rclcpp::Time last_time_;        // for dt in the rate limit
  bool grip_frozen_ = false;      // jaws have clamped an object; hold here
  int high_streak_ = 0;           // consecutive high-current cycles (debounce)

  rclcpp::Logger logger_;
  rclcpp::Clock clock_;

  // ---- Tunables (hardcoded for now; easily promoted to yaml params) ----
  // Contact current: above this the jaws are pressing something. Set BELOW the
  // ~2.2 A brownout point and above the draw of simply MOVING the jaws, so the
  // grasp freezes at contact and never demands the stall current that collapses
  // the rail.
  //
  // Measured on the HPS-3518SG, 2026-07-27, closing on empty jaws:
  //   idle, jaws open  0.61-0.79 A   <- the "free-run" figure this was first set
  //                                     against, but that is the RESTING draw
  //   driving closed   peaks ~1.5 A  <- the servo working its own linkage
  //   holding a grasp  1.08-1.43 A
  //   brownout         ~2.2 A
  // At 1.3 A the motion peaks alone tripped contact_debounce_, so a close on
  // nothing froze at 0.0027 m (and mid-episode at 0.0050 m — ~35% open, well
  // short of the 23 mm cube).
  //
  // The ceiling is NOT a free choice: closing on the cube, 2026-07-27, the draw
  // peaked at 2.19 A and then collapsed to ~0.5 A as the rail gave out, over and
  // over on a ~2 s period. The servo cannot pull more than ~2.2 A through this
  // supply — it browns out first — so any threshold at or above that can never
  // fire, and the jaws sit in the close-release-retry loop instead of gripping.
  // 1.9 A is the operating point: above the 1.61 A peak of merely moving the
  // jaws, below the 2.19 A collapse. The window is narrow because the rail is
  // marginal; the cored DS3230MG is the real fix.
  double contact_current_threshold_ = 1.9;  // Amps
  int contact_debounce_ = 2;                // consecutive high cycles to confirm

  // Rise-rate trip — the safety net for the absolute threshold.
  //
  // An absolute threshold alone races the brownout: if the rail collapses at a
  // current below contact_current_threshold_, the servo resets and releases
  // before contact is ever declared, the still-closing command drives it into
  // the cube again, and the jaws sit in a ~2 s close-release-retry loop instead
  // of gripping. Hitting a rigid object also shows up as a SHARP rise long
  // before the absolute peak, so trip on the slope too and catch contact early
  // regardless of where the rail gives out. Both conditions are required so
  // ordinary creep (which climbs gradually from ~0.7 A) cannot fire it.
  //
  // This trip bypasses contact_debounce_ deliberately. A rise does not persist:
  // once the current is high and flat the slope is zero again, so requiring two
  // consecutive rising cycles means the streak resets every cycle and the trip
  // can never fire at all. The median filter is what protects it from noise
  // here — a lone spike cannot move a median of three.
  double contact_rise_per_cycle_ = 0.5;  // Amps of jump between cycles
  double contact_rise_floor_ = 1.5;      // ...only above this absolute current

  // How fast the jaws close (m/s). Slow enough that the current is sampled as it
  // rises into contact, before it can spike straight into the brownout. Halved
  // from 0.02 after the cube stalled the servo: at 0.01 m/s the full ~0.014 m
  // travel takes ~1.4 s and contact spans several samples rather than one.
  double close_velocity_ = 0.01;  // m/s

  // On contact, RELAX the command by this much instead of holding exactly where
  // the trip fired. Detection always lags the touch, so the frozen position is
  // already deeper than the jaws can physically reach with the object in the
  // way — and commanding a servo to an angle an obstacle blocks is precisely
  // the sustained stall that collapses the rail. Backing off a hair gives the
  // servo a target it can actually hold, which turns a stall into a clamp.
  double contact_backoff_ = 0.0008;  // metres

  // Commands closer to closed than this fraction of the closed->open travel are
  // treated as a grasp (intent to close). At/above it the command is "opening".
  double grip_intent_fraction_ = 0.5;
};

}  // namespace annin_ar4_driver
