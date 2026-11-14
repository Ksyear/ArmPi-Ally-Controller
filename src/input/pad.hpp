// Gamepad input + rumble. Windows: XInput (ROG Ally X built-in controller in gamepad mode),
// loaded at run time from xinput1_4.dll / xinput9_1_0.dll so the exe has no hard dependency.
// Other platforms: a stub that reports "no pad" (core/unit tests still build).
#pragma once

#include <cstdint>
#include <deque>
#include <string>

namespace ally::input {

enum PadButton : uint16_t {
  kDpadUp = 0x0001,
  kDpadDown = 0x0002,
  kDpadLeft = 0x0004,
  kDpadRight = 0x0008,
  kStart = 0x0010,  // "Menu" (≡)
  kBack = 0x0020,   // "View" (⧉)
  kLThumb = 0x0040,
  kRThumb = 0x0080,
  kLB = 0x0100,
  kRB = 0x0200,
  kA = 0x1000,
  kB = 0x2000,
  kX = 0x4000,
  kY = 0x8000,
};

struct PadState {
  bool connected{false};
  int index{-1};
  float lx{0}, ly{0}, rx{0}, ry{0};  // -1..1 after dead zone (+y = up)
  float lt{0}, rt{0};                // 0..1 after threshold
  uint16_t buttons{0};
  uint32_t packet{0};
  bool down(uint16_t b) const { return (buttons & b) != 0; }
};

// Radial dead zone with rescale so output starts at 0 right after the dead zone.
void apply_radial_deadzone(float* x, float* y, float deadzone);
float apply_trigger_threshold(float v, float threshold);

class Gamepad {
 public:
  Gamepad();
  ~Gamepad();
  Gamepad(const Gamepad&) = delete;
  Gamepad& operator=(const Gamepad&) = delete;

  bool available() const;        // XInput library loaded
  std::string backend() const;   // e.g. "xinput1_4.dll" / "없음"
  PadState poll();               // first connected pad (index cached, rescanned every 2 s when none)
  PadState poll_index(int i);    // a specific slot 0..3
  void set_rumble(float low, float high);  // 0..1, current pad
  float stick_deadzone{0.12f};
  float trigger_threshold{0.08f};

 private:
  struct Impl;
  Impl* impl_;
};

// Vibration patterns (portable timing logic; output goes to Gamepad::set_rumble)
enum class Haptic { kTick, kArmOn, kLimit, kReject, kLinkLost, kLinkOk, kEstop, kServiceOk, kServiceFail };

class HapticPlayer {
 public:
  void play(Haptic h, double now);
  // Returns (low, high) motor strengths for this moment.
  void sample(double now, float* low, float* high);
  void set_enabled(bool on) { enabled_ = on; }
  bool enabled() const { return enabled_; }
  void set_strength(float s) { strength_ = s; }
  float strength() const { return strength_; }

 private:
  struct Seg {
    double start, end;
    float low, high;
  };
  std::deque<Seg> q_;
  bool enabled_{true};
  float strength_{1.0f};
};

}  // namespace ally::input
