// Design tokens (docs/DESIGN_PLAN_v001.md section 4). All sizes are 1920x1080 pixels before scaling.
// Colour is reserved for state and selection; structure is grey steps and 1 px borders.
#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include "imgui.h"

namespace ally::ui {

// ── colour ──
namespace col {
constexpr ImU32 rgb(unsigned v, unsigned a = 255) { return IM_COL32((v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF, a); }
inline constexpr ImU32 bg0 = rgb(0x0B0D10);       // camera canvas
inline constexpr ImU32 bg1 = rgb(0x13161B);       // app background (also the D3D clear colour)
inline constexpr ImU32 surface1 = rgb(0x1B1F26);  // panels, cards
inline constexpr ImU32 surface2 = rgb(0x242932);  // buttons, inputs
inline constexpr ImU32 surface3 = rgb(0x2E3440);  // hover / pressed / selected background
inline constexpr ImU32 border1 = rgb(0x363D49);
inline constexpr ImU32 border2 = rgb(0x4B5566);
inline constexpr ImU32 text1 = rgb(0xEDF0F4);
inline constexpr ImU32 text2 = rgb(0xA7B0BD);
inline constexpr ImU32 text3 = rgb(0x6F7A8A);
inline constexpr ImU32 accent = rgb(0x5EA2FF);    // selection / current mode / active input (no state meaning)
inline constexpr ImU32 accentInk = rgb(0x0B1A2E);
inline constexpr ImU32 ok = rgb(0x3DBE6A);
inline constexpr ImU32 warn = rgb(0xE8B83A);
inline constexpr ImU32 warnInk = rgb(0x2A1E02);
inline constexpr ImU32 danger = rgb(0xE5484D);
inline constexpr ImU32 stop = rgb(0xD7262C);      // e-stop fill (white text 5.0:1)
inline constexpr ImU32 stopPressed = rgb(0xB01F24);
inline constexpr ImU32 stopLatched = rgb(0x5A1416);
inline constexpr ImU32 info = rgb(0x8FB5FF);
inline constexpr ImU32 disabled = rgb(0x59616E);
inline constexpr ImU32 white = rgb(0xFFFFFF);
inline constexpr ImU32 black = rgb(0x000000);
ImU32 alpha(ImU32 c, float a);  // replace alpha (0..1)
ImU32 mix(ImU32 a, ImU32 b, float t);
ImVec4 v4(ImU32 c);
}  // namespace col

// ── type scale (px, unscaled; pass to PushFont as the size) ──
namespace fs {
inline constexpr float display = 44, title = 30, body = 26, label = 22, micro = 18;
inline constexpr float num = 26, numL = 34, numXL = 56;
}  // namespace fs

// ── spacing / shape ──
namespace sp {
inline constexpr float s1 = 4, s2 = 8, s3 = 12, s4 = 16, s5 = 24, s6 = 32;
inline constexpr float gutter = 16;
inline constexpr float radiusS = 6, radiusM = 10, radiusL = 16;
inline constexpr float hair = 1, stroke = 2, strokeBold = 4;
inline constexpr float touchMin = 64, touch = 88, touchL = 112;
}  // namespace sp

// ── motion (seconds) ──
namespace tm {
inline constexpr float fast = 0.08f, base = 0.12f, slow = 0.20f, rewind = 0.12f;
}

enum class Status { kOk, kIdle, kWarn, kDanger, kOff };
ImU32 status_color(Status s);

// Fonts: Pretendard (text, with Lucide icons merged) and JetBrains Mono (numbers).
struct Fonts {
  ImFont* regular{nullptr};
  ImFont* semibold{nullptr};
  ImFont* bold{nullptr};
  ImFont* mono{nullptr};
  ImFont* mono_medium{nullptr};
  bool embedded{false};  // false = fell back to system Malgun Gothic
};

// Per-frame UI context shared by the widgets.
// Extra touch contacts. ImGui follows one pointer as its mouse (the first finger); every other finger is listed here
// so that sticks, hold buttons and the e-stop also answer a second finger.
struct TouchPoint {
  uint32_t id{0};
  ImVec2 pos{}, down_pos{};  // window client pixels
  bool just_down{false};     // touched down since the previous frame
};

struct UiContext {
  Fonts fonts;
  float scale{1.0f};       // display height / 1080 (layout)
  float text_scale{1.0f};  // scale x user text size setting (1.0-1.3)
  double now{0.0};
  float dt{0.016f};
  std::vector<TouchPoint> touches;                     // extra fingers this frame
  std::vector<std::pair<uint32_t, ImGuiID>> touch_owner;  // finger -> widget that claimed it
};
UiContext& ctx();
inline float S(float px) { return px * ctx().scale; }

void apply_theme(ImGuiStyle& st, float scale);

// Small easing helpers backed by ImGui state storage (id-keyed floats).
float ease_out(float t);
// Moves the stored value toward target over `seconds` (exponential-like approach, frame-rate independent).
float tween(ImGuiID id, float target, float seconds);

}  // namespace ally::ui
