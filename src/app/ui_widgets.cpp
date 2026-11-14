#include "app/ui_widgets.hpp"

#include <algorithm>
#include <cfloat>
#include <cmath>

namespace ally::ui::w {

namespace {
constexpr float kPi = 3.14159265358979f;
ImFont* body_font() { return ctx().fonts.regular; }
ImFont* strong_font() { return ctx().fonts.semibold; }
}  // namespace

// ───────── text ─────────

float text_px(float size) { return size * ctx().text_scale; }

ImVec2 text_size(ImFont* f, float size, const char* t) {
  if (!f) f = ImGui::GetFont();
  return f->CalcTextSizeA(text_px(size), FLT_MAX, 0.0f, t);
}

void text(ImDrawList* dl, ImVec2 pos, ImFont* f, float size, ImU32 c, const char* t) {
  dl->AddText(f ? f : ImGui::GetFont(), text_px(size), pos, c, t);
}

void text_center(ImDrawList* dl, ImVec2 center, ImFont* f, float size, ImU32 c, const char* t) {
  const ImVec2 ts = text_size(f, size, t);
  text(dl, ImVec2(center.x - ts.x * 0.5f, center.y - ts.y * 0.5f), f, size, c, t);
}

void osd_text(ImDrawList* dl, ImVec2 pos, ImFont* f, float size, ImU32 c, const char* t) {
  const float o = std::max(1.0f, S(1.0f));
  const ImU32 shadow = col::alpha(col::black, 0.7f);
  for (int dx = -1; dx <= 1; ++dx)
    for (int dy = -1; dy <= 1; ++dy)
      if (dx || dy) text(dl, ImVec2(pos.x + dx * o, pos.y + dy * o), f, size, shadow, t);
  text(dl, pos, f, size, c, t);
}

// ───────── status ─────────

void status_symbol(ImDrawList* dl, ImVec2 c, float r, Status s) {
  const ImU32 color = status_color(s);
  switch (s) {
    case Status::kOk: dl->AddCircleFilled(c, r, color, 16); break;
    case Status::kIdle:
    case Status::kOff: dl->AddCircle(c, r - S(1), color, 16, S(2)); break;
    case Status::kWarn:
      dl->AddTriangleFilled(ImVec2(c.x, c.y - r * 1.1f), ImVec2(c.x + r * 1.1f, c.y + r * 0.9f),
                            ImVec2(c.x - r * 1.1f, c.y + r * 0.9f), color);
      break;
    case Status::kDanger: dl->AddRectFilled(ImVec2(c.x - r * 0.9f, c.y - r * 0.9f), ImVec2(c.x + r * 0.9f, c.y + r * 0.9f), color, S(1)); break;
  }
}

float status_item_width(const char* label, const char* value) {
  float wdt = S(10) + S(sp::s2) + text_size(body_font(), fs::label, label).x;
  if (value && *value) wdt += S(sp::s2) + text_size(ctx().fonts.mono, fs::label, value).x;
  return wdt;
}

void status_item(ImDrawList* dl, ImVec2 pos, float h, const char* label, Status s, const char* value) {
  const float cy = pos.y + h * 0.5f;
  status_symbol(dl, ImVec2(pos.x + S(5), cy), S(5), s);
  float x = pos.x + S(10) + S(sp::s2);
  const ImVec2 ls = text_size(body_font(), fs::label, label);
  text(dl, ImVec2(x, cy - ls.y * 0.5f), body_font(), fs::label, s == Status::kDanger ? col::danger : col::text2, label);
  x += ls.x;
  if (value && *value) {
    x += S(sp::s2);
    const ImVec2 vs = text_size(ctx().fonts.mono, fs::label, value);
    const ImU32 vc = s == Status::kWarn ? col::warn : s == Status::kDanger ? col::danger : col::text1;
    text(dl, ImVec2(x, cy - vs.y * 0.5f), ctx().fonts.mono, fs::label, vc, value);
  }
}

// ───────── pad badges ─────────

ImVec2 pad_badge(ImDrawList* dl, ImVec2 p, PadKey k, float h, bool pressed) {
  const ImU32 fill = pressed ? col::accent : col::surface3;
  const ImU32 ink = pressed ? col::accentInk : col::text1;
  const ImU32 edge = col::border2;
  auto letters = [&](const char* t, float wdt) {
    text_center(dl, ImVec2(p.x + wdt * 0.5f, p.y + h * 0.5f), ctx().fonts.bold, fs::micro, ink, t);
  };
  switch (k) {
    case PadKey::kA:
    case PadKey::kB:
    case PadKey::kX:
    case PadKey::kY: {
      const ImVec2 c(p.x + h * 0.5f, p.y + h * 0.5f);
      dl->AddCircleFilled(c, h * 0.5f, fill, 24);
      dl->AddCircle(c, h * 0.5f, edge, 24, S(1));
      letters(k == PadKey::kA ? "A" : k == PadKey::kB ? "B" : k == PadKey::kX ? "X" : "Y", h);
      return ImVec2(h, h);
    }
    case PadKey::kLB:
    case PadKey::kRB:
    case PadKey::kLT:
    case PadKey::kRT: {
      const float wdt = h * 1.6f;
      dl->AddRectFilled(p, ImVec2(p.x + wdt, p.y + h), fill, S(sp::radiusS));
      dl->AddRect(p, ImVec2(p.x + wdt, p.y + h), edge, S(sp::radiusS), 0, S(1));
      letters(k == PadKey::kLB ? "LB" : k == PadKey::kRB ? "RB" : k == PadKey::kLT ? "LT" : "RT", wdt);
      return ImVec2(wdt, h);
    }
    case PadKey::kView:
    case PadKey::kMenu:
    case PadKey::kDpadUD:
    case PadKey::kDpadLR:
    case PadKey::kLStick:
    case PadKey::kRStick: {
      const ImVec2 c(p.x + h * 0.5f, p.y + h * 0.5f);
      dl->AddCircleFilled(c, h * 0.5f, fill, 24);
      dl->AddCircle(c, h * 0.5f, edge, 24, S(1));
      const float u = h * 0.16f;
      if (k == PadKey::kView) {
        dl->AddRect(ImVec2(c.x - u * 1.6f, c.y - u * 1.2f), ImVec2(c.x + u * 0.4f, c.y + u * 0.6f), ink, 0, 0, S(1.5f));
        dl->AddRect(ImVec2(c.x - u * 0.4f, c.y - u * 0.6f), ImVec2(c.x + u * 1.6f, c.y + u * 1.2f), ink, 0, 0, S(1.5f));
      } else if (k == PadKey::kMenu) {
        for (int i = -1; i <= 1; ++i)
          dl->AddLine(ImVec2(c.x - u * 1.5f, c.y + i * u), ImVec2(c.x + u * 1.5f, c.y + i * u), ink, S(1.5f));
      } else if (k == PadKey::kDpadUD || k == PadKey::kDpadLR) {
        const bool ud = k == PadKey::kDpadUD;
        auto tri = [&](float dir) {
          if (ud)
            dl->AddTriangleFilled(ImVec2(c.x, c.y + dir * u * 2.0f), ImVec2(c.x - u, c.y + dir * u * 0.7f),
                                  ImVec2(c.x + u, c.y + dir * u * 0.7f), ink);
          else
            dl->AddTriangleFilled(ImVec2(c.x + dir * u * 2.0f, c.y), ImVec2(c.x + dir * u * 0.7f, c.y - u),
                                  ImVec2(c.x + dir * u * 0.7f, c.y + u), ink);
        };
        tri(-1);
        tri(1);
      } else {
        dl->AddCircle(c, h * 0.28f, ink, 16, S(1.5f));
        letters(k == PadKey::kLStick ? "L" : "R", h);
      }
      return ImVec2(h, h);
    }
  }
  return ImVec2(h, h);
}

float hint(ImDrawList* dl, ImVec2 pos, PadKey k, const char* t, float h) {
  const ImVec2 b = pad_badge(dl, pos, k, h);
  const ImVec2 ts = text_size(body_font(), fs::micro, t);
  text(dl, ImVec2(pos.x + b.x + S(sp::s2), pos.y + (h - ts.y) * 0.5f), body_font(), fs::micro, col::text2, t);
  return b.x + S(sp::s2) + ts.x;
}

// ───────── buttons ─────────

namespace {
void draw_button_face(ImDrawList* dl, ImVec2 p, ImVec2 size, Variant v, bool active, bool hovered, bool enabled,
                      const char* label, const char* icon) {
  ImU32 fill = col::surface2, edge = col::border1, ink = col::text1;
  switch (v) {
    case Variant::kPrimary: fill = active ? col::mix(col::accent, col::black, 0.15f) : col::accent; edge = fill; ink = col::accentInk; break;
    case Variant::kDanger: fill = active ? col::alpha(col::danger, 0.18f) : col::surface2; edge = col::danger; ink = col::danger; break;
    case Variant::kGhost: fill = active ? col::surface3 : col::alpha(col::surface2, 0); edge = col::alpha(col::border1, 0); break;
    case Variant::kSelected: fill = col::surface3; edge = col::accent; break;
    case Variant::kDefault: fill = active ? col::surface3 : (hovered ? col::mix(col::surface2, col::surface3, 0.5f) : col::surface2); break;
  }
  if (!enabled) {
    fill = col::alpha(col::surface2, 0.5f);
    edge = col::border1;
    ink = col::disabled;
  }
  const float r = S(sp::radiusM);
  dl->AddRectFilled(p, ImVec2(p.x + size.x, p.y + size.y), fill, r);
  dl->AddRect(p, ImVec2(p.x + size.x, p.y + size.y), edge, r, 0, v == Variant::kDanger || v == Variant::kSelected ? S(2) : S(1));
  if (v == Variant::kSelected)
    dl->AddRectFilled(ImVec2(p.x + S(4), p.y + size.y - S(4)), ImVec2(p.x + size.x - S(4), p.y + size.y - S(1)), col::accent, S(2));
  // icon above label when the button is tall, otherwise inline
  ImFont* f = strong_font();
  const ImVec2 ls = label && *label ? text_size(f, fs::body, label) : ImVec2(0, 0);
  if (icon && *icon) {
    const ImVec2 is = text_size(f, fs::title, icon);
    if (size.y >= S(96) && label && *label) {
      const float total = is.y + S(sp::s1) + ls.y;
      const float y0 = p.y + (size.y - total) * 0.5f;
      text(dl, ImVec2(p.x + (size.x - is.x) * 0.5f, y0), f, fs::title, ink, icon);
      text(dl, ImVec2(p.x + (size.x - ls.x) * 0.5f, y0 + is.y + S(sp::s1)), f, fs::body, ink, label);
    } else {
      const float total = is.x + (ls.x > 0 ? S(sp::s2) + ls.x : 0);
      const float x0 = p.x + (size.x - total) * 0.5f;
      text(dl, ImVec2(x0, p.y + (size.y - is.y) * 0.5f), f, fs::title, ink, icon);
      if (ls.x > 0) text(dl, ImVec2(x0 + is.x + S(sp::s2), p.y + (size.y - ls.y) * 0.5f), f, fs::body, ink, label);
    }
  } else if (label && *label) {
    // multi-line labels are centred line by line
    const ImVec2 ms = ImGui::CalcTextSize(label);  // only used for line count
    (void)ms;
    text(dl, ImVec2(p.x + (size.x - ls.x) * 0.5f, p.y + (size.y - ls.y) * 0.5f), f, fs::body, ink, label);
  }
}
}  // namespace

bool button(const char* id, const char* label, ImVec2 size, Variant v, const char* icon, bool enabled) {
  ImGui::PushID(id);
  const ImVec2 p = ImGui::GetCursorScreenPos();
  ImGui::BeginDisabled(!enabled);
  const bool clicked = ImGui::InvisibleButton("b", size);
  ImGui::EndDisabled();
  draw_button_face(ImGui::GetWindowDrawList(), p, size, v, ImGui::IsItemActive(), ImGui::IsItemHovered(), enabled, label, icon);
  ImGui::PopID();
  return clicked && enabled;
}

bool hold_button_active(const char* id, const char* label, ImVec2 size, const char* icon) {
  ImGui::PushID(id);
  const ImVec2 p = ImGui::GetCursorScreenPos();
  ImGui::InvisibleButton("h", size);
  const bool active = ImGui::IsItemActive();
  draw_button_face(ImGui::GetWindowDrawList(), p, size, active ? Variant::kSelected : Variant::kDefault, active,
                   ImGui::IsItemHovered(), true, label, icon);
  ImGui::PopID();
  return active;
}

bool icon_button(const char* id, const char* icon, ImVec2 size, bool selected) {
  ImGui::PushID(id);
  const ImVec2 p = ImGui::GetCursorScreenPos();
  const bool clicked = ImGui::InvisibleButton("i", size);
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const bool active = ImGui::IsItemActive();
  dl->AddRectFilled(p, ImVec2(p.x + size.x, p.y + size.y),
                    active ? col::alpha(col::surface3, 0.95f) : col::alpha(col::bg0, selected ? 0.85f : 0.6f), S(sp::radiusM));
  dl->AddRect(p, ImVec2(p.x + size.x, p.y + size.y), selected ? col::accent : col::alpha(col::border2, 0.8f), S(sp::radiusM), 0, S(1));
  text_center(dl, ImVec2(p.x + size.x * 0.5f, p.y + size.y * 0.5f), body_font(), fs::title, selected ? col::accent : col::text1, icon);
  ImGui::PopID();
  return clicked;
}

bool estop_button(const char* id, ImVec2 size, EstopVisual v) {
  ImGui::PushID(id);
  const ImVec2 p = ImGui::GetCursorScreenPos();
  ImGui::InvisibleButton("e", size);
  const bool pressed_now = ImGui::IsItemActivated();  // touch-down fires (safety: no waiting for release)
  const bool active = ImGui::IsItemActive();
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const ImGuiID sid = ImGui::GetItemID();
  const float shrink = tween(sid, active && v != EstopVisual::kDisabled ? 0.03f : 0.0f, tm::fast);
  const ImVec2 c(p.x + size.x * 0.5f, p.y + size.y * 0.5f);
  const ImVec2 hs(size.x * 0.5f * (1.0f - shrink), size.y * 0.5f * (1.0f - shrink));
  const ImVec2 a(c.x - hs.x, c.y - hs.y), b(c.x + hs.x, c.y + hs.y);
  const float r = S(sp::radiusL);
  if (v == EstopVisual::kDisabled) {
    dl->AddRect(a, b, col::border2, r, 0, S(2));
    text_center(dl, c, ctx().fonts.bold, fs::title, col::text3, "비상정지");
  } else if (v == EstopVisual::kLatched) {
    const float breath = 0.7f + 0.3f * std::sin(static_cast<float>(ctx().now) * 2.0f * kPi);
    dl->AddRectFilled(a, b, col::stopLatched, r);
    dl->AddRect(a, b, col::alpha(col::stop, breath), r, 0, S(3));
    text_center(dl, ImVec2(c.x, c.y - S(14)), ctx().fonts.bold, fs::title, col::white, "정지됨");
    text_center(dl, ImVec2(c.x, c.y + S(24)), body_font(), fs::micro, col::alpha(col::white, 0.8f), "다시 눌러도 정지");
  } else {
    dl->AddRectFilled(a, b, active ? col::stopPressed : col::stop, r);
    dl->AddRect(ImVec2(a.x + S(8), a.y + S(8)), ImVec2(b.x - S(8), b.y - S(8)), col::alpha(col::white, 0.4f), r - S(6), 0, S(1.5f));
    text_center(dl, ImVec2(c.x, c.y - S(30)), ctx().fonts.bold, fs::display, col::white, "비상");
    text_center(dl, ImVec2(c.x, c.y + S(16)), ctx().fonts.bold, fs::display, col::white, "정지");
    text_center(dl, ImVec2(c.x, b.y - S(26)), body_font(), fs::micro, col::alpha(col::white, 0.75f), "소프트웨어 정지");
  }
  ImGui::PopID();
  return pressed_now && v != EstopVisual::kDisabled;
}

bool hold_ring_button(const char* id, const char* label, ImVec2 size, double hold_s, float external, bool enabled) {
  ImGui::PushID(id);
  const ImVec2 p = ImGui::GetCursorScreenPos();
  ImGui::BeginDisabled(!enabled);
  ImGui::InvisibleButton("r", size);
  ImGui::EndDisabled();
  const bool active = enabled && ImGui::IsItemActive();
  const ImGuiID sid = ImGui::GetItemID();
  ImGuiStorage* st = ImGui::GetStateStorage();
  const float held = active ? ImGui::GetIO().MouseDownDuration[0] : 0.0f;
  float frac = std::max(std::min(1.0f, static_cast<float>(held / hold_s)), external);
  // rewind smoothly when released early
  const float shown = frac > 0 ? frac : tween(sid + 1, 0.0f, tm::rewind);
  if (frac > 0) st->SetFloat(sid + 1, frac);
  bool fire = false;
  const bool fired = st->GetBool(sid + 2, false);
  if (active && held >= hold_s && !fired) {
    fire = true;
    st->SetBool(sid + 2, true);
    st->SetFloat(sid + 3, static_cast<float>(ctx().now));
  }
  if (!active) st->SetBool(sid + 2, false);
  ImDrawList* dl = ImGui::GetWindowDrawList();
  dl->AddRectFilled(p, ImVec2(p.x + size.x, p.y + size.y), active ? col::surface3 : col::surface2, S(sp::radiusM));
  dl->AddRect(p, ImVec2(p.x + size.x, p.y + size.y), col::border1, S(sp::radiusM), 0, S(1));
  const ImVec2 rc(p.x + S(sp::s4) + S(20), p.y + size.y * 0.5f);
  dl->AddCircle(rc, S(18), col::border2, 32, S(2));
  const bool flash_ok = ctx().now - st->GetFloat(sid + 3, -10.0f) < 0.25;
  if (shown > 0.001f || flash_ok) {
    const float a0 = -kPi * 0.5f;
    dl->PathArcTo(rc, S(18), a0, a0 + 2.0f * kPi * (flash_ok ? 1.0f : shown), 40);
    dl->PathStroke(flash_ok ? col::ok : col::warn, 0, S(4));
  }
  const ImVec2 ls = text_size(strong_font(), fs::body, label);
  text(dl, ImVec2(rc.x + S(32), p.y + (size.y - ls.y) * 0.5f), strong_font(), fs::body, enabled ? (active ? col::text1 : col::text2) : col::disabled, label);
  ImGui::PopID();
  return fire;
}

int segmented(const char* id, const char* const* items, const char* const* icons, int n, int current, ImVec2 item,
              bool vertical) {
  ImGui::PushID(id);
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const ImVec2 p0 = ImGui::GetCursorScreenPos();
  const float gap = vertical ? S(sp::s2) : 0.0f;
  const ImVec2 total = vertical ? ImVec2(item.x, item.y * n + gap * (n - 1)) : ImVec2(item.x * n, item.y);
  if (!vertical) {
    dl->AddRectFilled(p0, ImVec2(p0.x + total.x, p0.y + total.y), col::surface1, S(sp::radiusM));
    dl->AddRect(p0, ImVec2(p0.x + total.x, p0.y + total.y), col::border1, S(sp::radiusM), 0, S(1));
  }
  int clicked = -1;
  // animated selection position
  const float sel = tween(ImGui::GetID("sel"), static_cast<float>(std::max(0, current)), tm::base);
  if (!vertical && current >= 0) {
    const float x = p0.x + item.x * sel;
    dl->AddRectFilled(ImVec2(x + S(3), p0.y + S(3)), ImVec2(x + item.x - S(3), p0.y + item.y - S(3)), col::surface3, S(sp::radiusS));
    dl->AddRectFilled(ImVec2(x + S(12), p0.y + item.y - S(6)), ImVec2(x + item.x - S(12), p0.y + item.y - S(3)), col::accent, S(2));
  }
  for (int i = 0; i < n; ++i) {
    const ImVec2 ip = vertical ? ImVec2(p0.x, p0.y + i * (item.y + gap)) : ImVec2(p0.x + i * item.x, p0.y);
    ImGui::SetCursorScreenPos(ip);
    ImGui::PushID(i);
    if (ImGui::InvisibleButton("s", item)) clicked = i;
    const bool active = ImGui::IsItemActive();
    ImGui::PopID();
    const bool selected = i == current;
    ImU32 ink = selected ? col::text1 : col::text2;
    if (vertical) {
      const ImU32 fill = selected ? col::accent : (active ? col::surface3 : col::surface2);
      dl->AddRectFilled(ip, ImVec2(ip.x + item.x, ip.y + item.y), fill, S(sp::radiusM));
      if (!selected) dl->AddRect(ip, ImVec2(ip.x + item.x, ip.y + item.y), col::border1, S(sp::radiusM), 0, S(1));
      ink = selected ? col::accentInk : col::text1;
    }
    const char* icon = icons ? icons[i] : nullptr;
    const ImVec2 ls = text_size(strong_font(), fs::label, items[i]);
    float x = ip.x + (item.x - ls.x) * 0.5f;
    if (icon && *icon) {
      const ImVec2 is = text_size(strong_font(), fs::label, icon);
      x = ip.x + (item.x - (is.x + S(sp::s2) + ls.x)) * 0.5f;
      text(dl, ImVec2(x, ip.y + (item.y - is.y) * 0.5f), strong_font(), fs::label, ink, icon);
      x += is.x + S(sp::s2);
    }
    text(dl, ImVec2(x, ip.y + (item.y - ls.y) * 0.5f), strong_font(), fs::label, ink, items[i]);
  }
  ImGui::SetCursorScreenPos(p0);
  ImGui::Dummy(total);
  ImGui::PopID();
  return clicked;
}

// ───────── containers / values ─────────

bool begin_card(const char* id, const char* title, ImVec2 size, bool scroll) {
  ImGui::PushStyleColor(ImGuiCol_ChildBg, col::v4(col::surface1));
  ImGui::PushStyleColor(ImGuiCol_Border, col::v4(col::border1));
  ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, S(sp::radiusM));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(S(sp::s4), S(sp::s4)));
  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(S(sp::s3), S(sp::s2)));
  const bool open = ImGui::BeginChild(id, size, ImGuiChildFlags_Borders | ImGuiChildFlags_AlwaysUseWindowPadding,
                                      scroll ? 0 : (ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse));
  if (title && *title) card_title(title);
  return open;
}

void end_card() {
  ImGui::EndChild();
  ImGui::PopStyleVar(3);
  ImGui::PopStyleColor(2);
}

void card_title(const char* title, const char* right_text, ImU32 right_color) {
  ImGui::PushFont(ctx().fonts.semibold, fs::label);
  ImGui::PushStyleColor(ImGuiCol_Text, col::v4(col::text2));
  ImGui::TextUnformatted(title);
  ImGui::PopStyleColor();
  if (right_text && *right_text) {
    const float wdt = ImGui::CalcTextSize(right_text).x;
    ImGui::SameLine(ImGui::GetContentRegionMax().x - wdt);
    ImGui::PushStyleColor(ImGuiCol_Text, col::v4(right_color));
    ImGui::TextUnformatted(right_text);
    ImGui::PopStyleColor();
  }
  ImGui::PopFont();
}

void number_tile(const char* label, const char* value, const char* unit, ImVec2 size, Status s) {
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const ImVec2 p = ImGui::GetCursorScreenPos();
  ImGui::Dummy(size);
  dl->AddRectFilled(p, ImVec2(p.x + size.x, p.y + size.y), col::surface2, S(sp::radiusM));
  text(dl, ImVec2(p.x + S(sp::s3), p.y + S(sp::s2)), body_font(), fs::micro, col::text2, label);
  const ImU32 vc = s == Status::kWarn ? col::warn : s == Status::kDanger ? col::danger : s == Status::kOff ? col::disabled : col::text1;
  const ImVec2 vs = text_size(ctx().fonts.mono_medium, fs::numL, value);
  const ImVec2 us = unit && *unit ? text_size(body_font(), fs::micro, unit) : ImVec2(0, 0);
  const float x = p.x + (size.x - vs.x - (us.x > 0 ? us.x + S(sp::s1) : 0)) * 0.5f;
  const float y = p.y + size.y - vs.y - S(sp::s2);
  text(dl, ImVec2(x, y), ctx().fonts.mono_medium, fs::numL, vc, value);
  if (us.x > 0) text(dl, ImVec2(x + vs.x + S(sp::s1), y + vs.y - us.y - S(3)), body_font(), fs::micro, col::text3, unit);
}

void trigger_bar(ImDrawList* dl, ImVec2 pos, float wdt, float value, bool connected) {
  const float h = S(8);
  dl->AddRectFilled(pos, ImVec2(pos.x + wdt, pos.y + h), col::border2, h * 0.5f);
  if (value > 0.001f && connected) dl->AddRectFilled(pos, ImVec2(pos.x + wdt * std::min(1.0f, value), pos.y + h), col::accent, h * 0.5f);
}

bool virtual_stick(const char* id, float radius, float* x, float* y) {
  const ImVec2 p = ImGui::GetCursorScreenPos();
  ImGui::InvisibleButton(id, ImVec2(radius * 2, radius * 2));
  const bool active = ImGui::IsItemActive();
  const ImVec2 c(p.x + radius, p.y + radius);
  *x = *y = 0;
  if (active) {
    const ImVec2 m = ImGui::GetIO().MousePos;
    float dx = (m.x - c.x) / radius, dy = -(m.y - c.y) / radius;
    const float len = std::sqrt(dx * dx + dy * dy);
    if (len > 1.0f) {
      dx /= len;
      dy /= len;
    }
    *x = dx;
    *y = dy;
  }
  ImDrawList* dl = ImGui::GetWindowDrawList();
  dl->AddCircleFilled(c, radius, col::surface2, 64);
  dl->AddCircle(c, radius, active ? col::accent : col::border2, 64, S(2));
  dl->AddLine(ImVec2(c.x - radius * 0.7f, c.y), ImVec2(c.x + radius * 0.7f, c.y), col::border1, S(1));
  dl->AddLine(ImVec2(c.x, c.y - radius * 0.7f), ImVec2(c.x, c.y + radius * 0.7f), col::border1, S(1));
  dl->AddCircleFilled(ImVec2(c.x + *x * radius * 0.68f, c.y - *y * radius * 0.68f), S(32), active ? col::accent : col::text2, 40);
  return active;
}

bool slider(const char* id, float* v, float lo, float hi, ImVec2 size, const char* left_label, const char* right_label) {
  ImGui::PushID(id);
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const ImVec2 p = ImGui::GetCursorScreenPos();
  const float lw = left_label ? text_size(body_font(), fs::micro, left_label).x + S(sp::s3) : 0;
  const float rw = right_label ? text_size(body_font(), fs::micro, right_label).x + S(sp::s3) : 0;
  const float x0 = p.x + lw + S(28), x1 = p.x + size.x - rw - S(28);
  ImGui::InvisibleButton("sl", size);
  const bool active = ImGui::IsItemActive();
  bool changed = false;
  if (active && x1 > x0) {
    const float t = std::clamp((ImGui::GetIO().MousePos.x - x0) / (x1 - x0), 0.0f, 1.0f);
    const float nv = lo + (hi - lo) * t;
    changed = std::fabs(nv - *v) > 1e-4f;
    *v = nv;
  }
  const float cy = p.y + size.y * 0.5f;
  if (left_label) text(dl, ImVec2(p.x, cy - text_size(body_font(), fs::micro, left_label).y * 0.5f), body_font(), fs::micro, col::text2, left_label);
  if (right_label)
    text(dl, ImVec2(p.x + size.x - rw + S(sp::s3), cy - text_size(body_font(), fs::micro, right_label).y * 0.5f), body_font(), fs::micro,
         col::text2, right_label);
  dl->AddRectFilled(ImVec2(x0, cy - S(8)), ImVec2(x1, cy + S(8)), col::border2, S(8));
  const float t = hi > lo ? std::clamp((*v - lo) / (hi - lo), 0.0f, 1.0f) : 0.0f;
  const float kx = x0 + (x1 - x0) * t;
  dl->AddRectFilled(ImVec2(x0, cy - S(8)), ImVec2(kx, cy + S(8)), col::alpha(col::accent, 0.55f), S(8));
  dl->AddCircleFilled(ImVec2(kx, cy), S(28), active ? col::mix(col::accent, col::white, 0.2f) : col::accent, 40);
  ImGui::PopID();
  return changed;
}

// ───────── toasts ─────────

void toast_stack(ImDrawList* dl, ImVec2 top_center, float max_w, const std::vector<Toast>& toasts, double now) {
  float y = top_center.y;
  const size_t first = toasts.size() > 3 ? toasts.size() - 3 : 0;
  for (size_t i = first; i < toasts.size(); ++i) {
    const Toast& t = toasts[i];
    const float in = ease_out(static_cast<float>((now - t.t0) / tm::base));
    const float out = std::clamp(static_cast<float>((t.until - now) / 0.15), 0.0f, 1.0f);
    const float a = std::min(in, out);
    const ImU32 band = status_color(t.kind == Status::kIdle ? Status::kIdle : t.kind);
    const ImU32 bandc = t.kind == Status::kIdle ? col::info : band;
    const ImVec2 ts = text_size(body_font(), fs::body, t.text.c_str());
    const float wdt = std::min(max_w, ts.x + S(56) + S(sp::s5));
    const float h = S(56);
    const float yy = y - S(16) * (1.0f - in);
    const ImVec2 p(top_center.x - wdt * 0.5f, yy);
    dl->AddRectFilled(p, ImVec2(p.x + wdt, p.y + h), col::alpha(col::surface2, 0.92f * a), S(sp::radiusM));
    dl->AddRect(p, ImVec2(p.x + wdt, p.y + h), col::alpha(col::border1, a), S(sp::radiusM), 0, S(1));
    dl->AddRectFilled(p, ImVec2(p.x + S(4), p.y + h), col::alpha(bandc, a), S(2));
    status_symbol(dl, ImVec2(p.x + S(28), p.y + h * 0.5f), S(7), t.kind == Status::kIdle ? Status::kOk : t.kind);
    dl->PushClipRect(p, ImVec2(p.x + wdt - S(8), p.y + h), true);
    text(dl, ImVec2(p.x + S(48), p.y + (h - ts.y) * 0.5f), body_font(), fs::body, col::alpha(col::text1, a), t.text.c_str());
    dl->PopClipRect();
    y += h + S(sp::s2);
  }
}


// ───────── stepper ─────────

void stepper(const char* id, const char* const* labels, int n, int current, int failed, ImVec2 size) {
  if (n <= 0) return;
  ImGui::PushID(id);
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const ImVec2 p0 = ImGui::GetCursorScreenPos();
  ImGui::Dummy(size);
  const float cw = size.x / static_cast<float>(n);
  const float bar_y = p0.y + size.y - S(4);
  dl->AddRectFilled(ImVec2(p0.x, bar_y), ImVec2(p0.x + size.x, bar_y + S(2)), col::border1, S(1));
  // the accent bar glides to the current step
  const float pos = tween(ImGui::GetID("pos"), static_cast<float>(std::clamp(current, 0, n - 1)), tm::base);
  for (int i = 0; i < n; ++i) {
    const float x0 = p0.x + cw * i;
    const bool done = current >= 0 && i < current && i != failed;
    const bool cur = i == current && i != failed;
    const bool bad = i == failed;
    ImU32 ink = col::text3;
    if (done) ink = col::text2;
    if (cur) ink = col::text1;
    if (bad) ink = col::danger;
    const ImVec2 ls = text_size(cur ? strong_font() : ctx().fonts.regular, fs::micro, labels[i]);
    const float mark = S(10);
    const float tx = x0 + (cw - ls.x - ((done || bad) ? mark + S(sp::s1) : 0.0f)) * 0.5f;
    float x = tx;
    if (done || bad) {
      status_symbol(dl, ImVec2(x + mark * 0.5f, p0.y + (size.y - S(6)) * 0.5f), S(4), bad ? Status::kDanger : Status::kOk);
      x += mark + S(sp::s1);
    }
    text(dl, ImVec2(x, p0.y + (size.y - S(6) - ls.y) * 0.5f), cur ? strong_font() : ctx().fonts.regular, fs::micro, ink, labels[i]);
    if (bad) dl->AddRectFilled(ImVec2(x0 + S(6), bar_y - S(1)), ImVec2(x0 + cw - S(6), bar_y + S(3)), col::danger, S(2));
    if (done) dl->AddRectFilled(ImVec2(x0 + S(6), bar_y), ImVec2(x0 + cw - S(6), bar_y + S(2)), col::alpha(col::ok, 0.6f), S(1));
  }
  if (current >= 0 && current < n && current != failed) {
    const float x0 = p0.x + cw * pos;
    dl->AddRectFilled(ImVec2(x0 + S(6), bar_y - S(1)), ImVec2(x0 + cw - S(6), bar_y + S(3)), col::accent, S(2));
  }
  ImGui::PopID();
}

}  // namespace ally::ui::w
