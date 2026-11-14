#include "input/mapper.hpp"

#include <algorithm>
#include <cmath>

#include "common/util.hpp"

namespace ally::input {

float shape_axis(float v, float curve) {
  if (!std::isfinite(v)) return 0.0f;
  const float a = std::min(1.0f, std::fabs(v));
  return std::copysign(std::pow(a, curve), v);
}

namespace {
inline double key_axis(bool plus, bool minus) { return (plus ? 1.0 : 0.0) - (minus ? 1.0 : 0.0); }
inline double clamp1(double v) { return v < -1.0 ? -1.0 : (v > 1.0 ? 1.0 : v); }
}  // namespace

MapResult Mapper::map(const PadState& pad_in, const KeyState& k, const TouchControls& t, double now, bool focused) {
  MapResult out;
  PadState pad = pad_in;
  if (!pad.connected) {
    pad = PadState{};
    pad_seen_neutral_ = false;
  }
  const uint16_t pressed = static_cast<uint16_t>(pad.buttons & ~prev_buttons_);
  prev_buttons_ = pad.buttons;
  auto edge = [&](uint16_t b) { return (pressed & b) != 0; };

  // Emergency stop works even without focus: stopping is always safe.
  if (edge(kB) || (k.space && !prev_space_ && !k.ctrl)) out.ev.estop = true;
  prev_space_ = k.space;

  // Clear e-stop: hold View (or Ctrl+Space) for clear_hold_s; fires once per hold.
  const bool clear_held = focused && (pad.down(kBack) || (k.ctrl && k.space));
  if (clear_held) {
    if (clear_start_ < 0) {
      clear_start_ = now;
      clear_fired_ = false;
    }
    const double held = now - clear_start_;
    out.ev.clear_progress = static_cast<float>(std::min(1.0, held / s_.clear_hold_s));
    if (held >= s_.clear_hold_s && !clear_fired_) {
      out.ev.clear_estop = true;
      clear_fired_ = true;
    }
  } else {
    clear_start_ = -1.0;
    clear_fired_ = false;
  }

  if (!focused) {
    pad_seen_neutral_ = false;
    return out;  // neutral input, arm_hold false
  }

  // After (re)connect or focus return, ignore the sticks until they are centered once.
  const bool sticks_centered = std::fabs(pad.lx) + std::fabs(pad.ly) + std::fabs(pad.rx) + std::fabs(pad.ry) < 1e-3 &&
                               pad.lt < 1e-3 && pad.rt < 1e-3;
  if (sticks_centered) pad_seen_neutral_ = true;
  const bool use_sticks = pad.connected && pad_seen_neutral_;

  const float lx = use_sticks ? shape_axis(pad.lx, s_.curve) : 0.0f;
  const float ly = use_sticks ? shape_axis(pad.ly, s_.curve) : 0.0f;
  const float rx = use_sticks ? shape_axis(pad.rx, s_.curve) : 0.0f;
  const float ry = use_sticks ? shape_axis(pad.ry, s_.curve) : 0.0f;
  const float lt = use_sticks ? pad.lt : 0.0f;
  const float rt = use_sticks ? pad.rt : 0.0f;

  const bool arm_mode = pad.down(kLB) || k.shift;
  out.arm_hold = arm_mode;
  out.input.precision = pad.down(kRB) || t.slow;

  auto& in = out.input;
  // REP-103: +strafe = left, +turn = counter-clockwise (stick left), +y = left.
  const double key_f = key_axis(k.w, k.s), key_s = key_axis(k.a, k.d), key_t = key_axis(k.q, k.e);
  const double touch_f = shape_axis(t.stick_y, s_.curve), touch_s = -shape_axis(t.stick_x, s_.curve);
  const double touch_t = -shape_axis(t.turn, s_.curve);
  const bool pad_drives = !arm_mode || s_.allow_drive_while_arm;
  in.forward = clamp1((pad_drives ? ly : 0.0) + key_f + touch_f);
  in.strafe = clamp1((pad_drives ? -lx : 0.0) + key_s + touch_s);
  in.turn = clamp1((!arm_mode ? -rx : 0.0) + key_t + touch_t);

  double ax = key_axis(k.i, k.k), ay = key_axis(k.j, k.l), az = key_axis(k.u, k.o);
  double ap = key_axis(k.r, k.f), ag = key_axis(k.g, k.h);
  if (arm_mode) {
    if (!s_.allow_drive_while_arm) {
      ax += ly;
      ay += -lx;
    }
    az += ry;
    ap += -rx;  // stick left = pitch up (+)
    ag += rt - lt;
  }
  ax += t.arm[0];
  ay += t.arm[1];
  az += t.arm[2];
  ap += t.arm[3];
  ag += t.arm[4];
  in.arm[0] = clamp1(ax);
  in.arm[1] = clamp1(ay);
  in.arm[2] = clamp1(az);
  in.arm[3] = clamp1(ap);
  in.arm[4] = clamp1(ag);

  // One-shot buttons
  if (arm_mode) {
    if (edge(kDpadUp)) out.ev.z_step = 0.01;
    if (edge(kDpadDown)) out.ev.z_step = -0.01;
    if (edge(kDpadLeft)) out.ev.pitch_step = rad(5.0);
    if (edge(kDpadRight)) out.ev.pitch_step = -rad(5.0);
  } else {
    if (edge(kDpadUp)) out.ev.speed_delta = 1;
    if (edge(kDpadDown)) out.ev.speed_delta = -1;
  }
  if (k.digit >= 0 && k.digit <= 9) out.ev.speed_set_percent = k.digit == 0 ? 100 : k.digit * 10;
  out.ev.camera_toggle = edge(kX);
  out.ev.camera_big = edge(kY);
  out.ev.arm_sync = edge(kLThumb);
  out.ev.menu = edge(kStart);
  return out;
}

}  // namespace ally::input
