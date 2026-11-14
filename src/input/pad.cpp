#include "input/pad.hpp"

#include <algorithm>
#include <cmath>

#include "common/util.hpp"

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <xinput.h>
#endif

namespace ally::input {

void apply_radial_deadzone(float* x, float* y, float dz) {
  const float m = std::sqrt(*x * *x + *y * *y);
  if (m <= dz || m <= 0.0f) {
    *x = *y = 0.0f;
    return;
  }
  const float scaled = std::min(1.0f, (m - dz) / (1.0f - dz));
  *x = *x / m * scaled;
  *y = *y / m * scaled;
}

float apply_trigger_threshold(float v, float t) {
  if (v <= t) return 0.0f;
  return std::min(1.0f, (v - t) / (1.0f - t));
}

#ifdef _WIN32

struct Gamepad::Impl {
  using GetStateFn = DWORD(WINAPI*)(DWORD, XINPUT_STATE*);
  using SetStateFn = DWORD(WINAPI*)(DWORD, XINPUT_VIBRATION*);
  HMODULE dll{nullptr};
  GetStateFn get{nullptr};
  SetStateFn set{nullptr};
  std::string name{"없음"};
  int index{-1};
  double next_scan{0.0};
  float last_low{-1}, last_high{-1};
};

Gamepad::Gamepad() : impl_(new Impl) {
  const wchar_t* dlls[] = {L"xinput1_4.dll", L"xinput1_3.dll", L"xinput9_1_0.dll"};
  const char* names[] = {"xinput1_4.dll", "xinput1_3.dll", "xinput9_1_0.dll"};
  for (int i = 0; i < 3 && !impl_->dll; ++i) {
    // System32 only: a same-named DLL dropped next to the exe must never be loaded.
    impl_->dll = ::LoadLibraryExW(dlls[i], nullptr, LOAD_LIBRARY_SEARCH_SYSTEM32);
    if (impl_->dll) impl_->name = names[i];
  }
  if (impl_->dll) {
    impl_->get = reinterpret_cast<Impl::GetStateFn>(reinterpret_cast<void*>(::GetProcAddress(impl_->dll, "XInputGetState")));
    impl_->set = reinterpret_cast<Impl::SetStateFn>(reinterpret_cast<void*>(::GetProcAddress(impl_->dll, "XInputSetState")));
  }
}

Gamepad::~Gamepad() {
  set_rumble(0, 0);
  if (impl_->dll) ::FreeLibrary(impl_->dll);
  delete impl_;
}

bool Gamepad::available() const { return impl_->get != nullptr; }
std::string Gamepad::backend() const { return impl_->name; }

PadState Gamepad::poll_index(int i) {
  PadState p;
  if (!impl_->get || i < 0 || i > 3) return p;
  XINPUT_STATE s{};
  if (impl_->get(static_cast<DWORD>(i), &s) != ERROR_SUCCESS) return p;
  p.connected = true;
  p.index = i;
  p.packet = s.dwPacketNumber;
  p.buttons = s.Gamepad.wButtons;
  auto ax = [](SHORT v) { return std::max(-1.0f, static_cast<float>(v) / 32767.0f); };
  p.lx = ax(s.Gamepad.sThumbLX);
  p.ly = ax(s.Gamepad.sThumbLY);
  p.rx = ax(s.Gamepad.sThumbRX);
  p.ry = ax(s.Gamepad.sThumbRY);
  apply_radial_deadzone(&p.lx, &p.ly, stick_deadzone);
  apply_radial_deadzone(&p.rx, &p.ry, stick_deadzone);
  p.lt = apply_trigger_threshold(s.Gamepad.bLeftTrigger / 255.0f, trigger_threshold);
  p.rt = apply_trigger_threshold(s.Gamepad.bRightTrigger / 255.0f, trigger_threshold);
  return p;
}

PadState Gamepad::poll() {
  if (!impl_->get) return {};
  if (impl_->index >= 0) {
    PadState p = poll_index(impl_->index);
    if (p.connected) return p;
    impl_->index = -1;
  }
  // XInputGetState on empty slots is slow (~ms), so scan at most every 2 s.
  const double now = mono_now();
  if (now < impl_->next_scan) return {};
  impl_->next_scan = now + 2.0;
  for (int i = 0; i < 4; ++i) {
    PadState p = poll_index(i);
    if (p.connected) {
      impl_->index = i;
      return p;
    }
  }
  return {};
}

void Gamepad::set_rumble(float low, float high) {
  if (!impl_->set || impl_->index < 0) return;
  low = clampd(low, 0, 1);
  high = clampd(high, 0, 1);
  if (std::fabs(low - impl_->last_low) < 0.01f && std::fabs(high - impl_->last_high) < 0.01f) return;
  XINPUT_VIBRATION v{};
  v.wLeftMotorSpeed = static_cast<WORD>(low * 65535.0f);
  v.wRightMotorSpeed = static_cast<WORD>(high * 65535.0f);
  impl_->set(static_cast<DWORD>(impl_->index), &v);
  impl_->last_low = low;
  impl_->last_high = high;
}

#else

struct Gamepad::Impl {};
Gamepad::Gamepad() : impl_(new Impl) {}
Gamepad::~Gamepad() { delete impl_; }
bool Gamepad::available() const { return false; }
std::string Gamepad::backend() const { return "없음(Windows 전용)"; }
PadState Gamepad::poll() { return {}; }
PadState Gamepad::poll_index(int) { return {}; }
void Gamepad::set_rumble(float, float) {}

#endif

void HapticPlayer::play(Haptic h, double now) {
  if (!enabled_) return;
  // Keep the strongest pattern: an e-stop clears everything queued before it.
  if (h == Haptic::kEstop) q_.clear();
  double t = q_.empty() ? now : std::max(now, q_.back().end);
  auto seg = [&](double ms, float low, float high) {
    q_.push_back(Seg{t, t + ms / 1000.0, low, high});
    t += ms / 1000.0;
  };
  auto gap = [&](double ms) { t += ms / 1000.0; };
  switch (h) {
    case Haptic::kTick: seg(25, 0.0f, 0.35f); break;
    case Haptic::kArmOn: seg(40, 0.0f, 0.5f); gap(60); seg(40, 0.0f, 0.5f); break;
    case Haptic::kLimit: seg(70, 0.45f, 0.0f); break;
    case Haptic::kReject: seg(120, 0.6f, 0.2f); break;
    case Haptic::kLinkLost: seg(150, 0.7f, 0.0f); gap(100); seg(150, 0.7f, 0.0f); break;
    case Haptic::kLinkOk: seg(60, 0.0f, 0.4f); break;
    case Haptic::kEstop: seg(450, 1.0f, 1.0f); break;
    case Haptic::kServiceOk: seg(50, 0.0f, 0.45f); break;
    case Haptic::kServiceFail: seg(90, 0.5f, 0.0f); gap(70); seg(90, 0.5f, 0.0f); break;
  }
  while (q_.size() > 32) q_.pop_front();
}

void HapticPlayer::sample(double now, float* low, float* high) {
  while (!q_.empty() && q_.front().end <= now) q_.pop_front();
  *low = *high = 0.0f;
  if (!enabled_ || q_.empty() || q_.front().start > now) return;
  *low = q_.front().low * strength_;
  *high = q_.front().high * strength_;
}

}  // namespace ally::input
