// Mapper: gamepad + keyboard + touch -> ControlInput and one-shot events (docs/PLAN_v001.md section 3).
// Pure logic (no OS calls) so the whole control table is unit-tested on any platform.
#pragma once

#include <cstdint>

#include "input/pad.hpp"
#include "robot/core.hpp"

namespace ally::input {

// Keyboard state for the keys the controller uses (true = held this frame)
struct KeyState {
  bool w{}, a{}, s{}, d{}, q{}, e{};   // drive
  bool i{}, j{}, k{}, l{}, u{}, o{};   // arm x / y / z
  bool r{}, f{}, g{}, h{};             // pitch up/down, gripper close/open
  bool shift{}, ctrl{}, space{};
  int digit{-1};                       // 0..9 pressed this frame (edge), -1 none
};

// On-screen touch controls (filled by the UI each frame)
struct TouchControls {
  float stick_x{0}, stick_y{0};  // virtual stick -1..1 (+x right, +y up)
  float turn{0};                 // turn slider -1..1 (+ = right)
  float arm[5]{0, 0, 0, 0, 0};   // arm buttons as rates (x, y, z, pitch, gripper)
  bool slow{false};
};

struct MapperSettings {
  float curve{1.6f};                 // v' = sign(v)|v|^curve
  bool allow_drive_while_arm{false}; // LB: left stick still drives, right stick + d-pad move the arm
  double clear_hold_s{0.8};
  double speed_step{0.1};
};

struct MapperEvents {
  bool estop{false};          // B / Space (edge)
  bool clear_estop{false};    // View or Ctrl+Space held clear_hold_s (fires once per hold)
  float clear_progress{0};    // 0..1 while holding
  int speed_delta{0};         // D-pad up/down in drive mode: +1 / -1 (x speed_step)
  int speed_set_percent{-1};  // number keys: 10..100
  bool camera_toggle{false};  // X
  bool camera_big{false};     // Y
  bool arm_sync{false};       // left stick click
  bool menu{false};           // Menu button
  double z_step{0};           // D-pad up/down in arm mode, m
  double pitch_step{0};       // D-pad left/right in arm mode, rad
};

struct MapResult {
  robot::ControlInput input;
  bool arm_hold{false};  // dead-man held (LB or Shift)
  MapperEvents ev;
};

float shape_axis(float v, float curve);

class Mapper {
 public:
  explicit Mapper(MapperSettings s = {}) : s_(s) {}
  // focused=false: everything neutral, only the emergency stop still works.
  MapResult map(const PadState& pad, const KeyState& keys, const TouchControls& touch, double now, bool focused);
  MapperSettings& settings() { return s_; }

 private:
  MapperSettings s_;
  uint16_t prev_buttons_{0};
  bool prev_space_{false};
  bool pad_seen_neutral_{false};
  double clear_start_{-1.0};
  bool clear_fired_{false};
};

}  // namespace ally::input
