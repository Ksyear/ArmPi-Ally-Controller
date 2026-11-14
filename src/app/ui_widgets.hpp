// Custom widgets (docs/DESIGN_PLAN_v001.md section 6), drawn with ImDrawList on top of InvisibleButton.
// Every size is in 1080p pixels and scaled with S(); text sizes are type-scale tokens (fs::*).
#pragma once

#include <string>
#include <vector>

#include "app/ui_theme.hpp"

namespace ally::ui::w {

// ── text ──
float text_px(float size);  // final pixel size for a type token (layout scale x user text scale)
ImVec2 text_size(ImFont* f, float size, const char* t);
void text(ImDrawList* dl, ImVec2 pos, ImFont* f, float size, ImU32 c, const char* t);
void text_center(ImDrawList* dl, ImVec2 center, ImFont* f, float size, ImU32 c, const char* t);
// Camera overlay text: 8-direction dark outline under the glyphs so it reads on any image.
void osd_text(ImDrawList* dl, ImVec2 pos, ImFont* f, float size, ImU32 c, const char* t);

// ── status ── (shape + colour + word: ● ok, ○ idle/off, ▲ warn, ■ danger)
void status_symbol(ImDrawList* dl, ImVec2 center, float r, Status s);
float status_item_width(const char* label, const char* value);
void status_item(ImDrawList* dl, ImVec2 pos, float h, const char* label, Status s, const char* value);

// ── gamepad badges ── (drawn shapes + letters; no trademark logos or button colours)
enum class PadKey { kA, kB, kX, kY, kLB, kRB, kLT, kRT, kView, kMenu, kDpadUD, kDpadLR, kLStick, kRStick };
ImVec2 pad_badge(ImDrawList* dl, ImVec2 pos, PadKey k, float h, bool pressed = false);
float hint(ImDrawList* dl, ImVec2 pos, PadKey k, const char* text, float h);  // badge + label, returns width

// ── buttons ──
enum class Variant { kDefault, kPrimary, kDanger, kGhost, kSelected };
// Click on release (normal buttons).
bool button(const char* id, const char* label, ImVec2 size, Variant v = Variant::kDefault, const char* icon = nullptr,
            bool enabled = true);
// Returns true while held (continuous arm/gripper buttons).
bool hold_button_active(const char* id, const char* label, ImVec2 size, const char* icon = nullptr);
bool icon_button(const char* id, const char* icon, ImVec2 size, bool selected = false);

enum class EstopVisual { kReady, kLatched, kDisabled };
// Fires on touch-down (press), not on release.
bool estop_button(const char* id, ImVec2 size, EstopVisual v);
// Hold for hold_s to fire (once). external_progress (0..1) mirrors the gamepad View hold on the same ring.
bool hold_ring_button(const char* id, const char* label, ImVec2 size, double hold_s, float external_progress,
                      bool enabled);
// Returns the clicked index or -1. Selected item: accent fill (vertical) or surface3 + accent underline (horizontal).
int segmented(const char* id, const char* const* items, const char* const* icons, int n, int current, ImVec2 item,
              bool vertical);

// ── containers / values ──
bool begin_card(const char* id, const char* title, ImVec2 size, bool scroll = false);
void end_card();
void card_title(const char* title, const char* right_text = nullptr, ImU32 right_color = col::text2);
void number_tile(const char* label, const char* value, const char* unit, ImVec2 size, Status s = Status::kOk);
void trigger_bar(ImDrawList* dl, ImVec2 pos, float w, float value, bool connected);
bool virtual_stick(const char* id, float radius, float* x, float* y);
// Large touch slider (track 16, knob 56). Returns true while changed.
bool slider(const char* id, float* v, float lo, float hi, ImVec2 size, const char* left_label, const char* right_label);

// ── toasts ──
struct Toast {
  std::string text;
  Status kind;
  double t0;
  double until;
};
void toast_stack(ImDrawList* dl, ImVec2 top_center, float max_w, const std::vector<Toast>& toasts, double now);

}  // namespace ally::ui::w
