#include "app/ui_theme.hpp"

#include <algorithm>
#include <cmath>

namespace ally::ui {

namespace col {
ImU32 alpha(ImU32 c, float a) {
  const unsigned al = static_cast<unsigned>(std::clamp(a, 0.0f, 1.0f) * 255.0f + 0.5f);
  return (c & ~IM_COL32_A_MASK) | (al << IM_COL32_A_SHIFT);
}
ImU32 mix(ImU32 a, ImU32 b, float t) {
  t = std::clamp(t, 0.0f, 1.0f);
  auto ch = [&](int shift) {
    const float x = static_cast<float>((a >> shift) & 0xFF), y = static_cast<float>((b >> shift) & 0xFF);
    return static_cast<ImU32>(x + (y - x) * t + 0.5f) << shift;
  };
  return ch(IM_COL32_R_SHIFT) | ch(IM_COL32_G_SHIFT) | ch(IM_COL32_B_SHIFT) | ch(IM_COL32_A_SHIFT);
}
ImVec4 v4(ImU32 c) { return ImGui::ColorConvertU32ToFloat4(c); }
}  // namespace col

ImU32 status_color(Status s) {
  switch (s) {
    case Status::kOk: return col::ok;
    case Status::kIdle: return col::text2;
    case Status::kWarn: return col::warn;
    case Status::kDanger: return col::danger;
    case Status::kOff: return col::disabled;
  }
  return col::text2;
}

UiContext& ctx() {
  static UiContext c;
  return c;
}

void apply_theme(ImGuiStyle& st, float scale) {
  ImGui::StyleColorsDark(&st);
  st.WindowRounding = 0;
  st.ChildRounding = sp::radiusM;
  st.FrameRounding = sp::radiusM;
  st.GrabRounding = 999;
  st.PopupRounding = sp::radiusM;
  st.ScrollbarRounding = 999;
  st.WindowPadding = ImVec2(sp::s4, sp::s4);
  st.FramePadding = ImVec2(sp::s4, sp::s3);
  st.ItemSpacing = ImVec2(sp::s3, sp::s3);
  st.ItemInnerSpacing = ImVec2(sp::s2, sp::s2);
  st.CellPadding = ImVec2(sp::s3, sp::s2);
  st.ScrollbarSize = 16;
  st.GrabMinSize = 40;
  st.FrameBorderSize = 1;
  st.ChildBorderSize = 1;
  st.WindowBorderSize = 0;
  st.TabRounding = sp::radiusS;
  ImVec4* c = st.Colors;
  c[ImGuiCol_WindowBg] = col::v4(col::bg1);
  c[ImGuiCol_ChildBg] = col::v4(col::surface1);
  c[ImGuiCol_PopupBg] = col::v4(col::surface1);
  c[ImGuiCol_FrameBg] = col::v4(col::surface2);
  c[ImGuiCol_FrameBgHovered] = col::v4(col::surface3);
  c[ImGuiCol_FrameBgActive] = col::v4(col::surface3);
  c[ImGuiCol_Button] = col::v4(col::surface2);
  c[ImGuiCol_ButtonHovered] = col::v4(col::surface3);
  c[ImGuiCol_ButtonActive] = col::v4(col::mix(col::surface3, col::accent, 0.35f));
  c[ImGuiCol_Border] = col::v4(col::border1);
  c[ImGuiCol_Separator] = col::v4(col::border1);
  c[ImGuiCol_Text] = col::v4(col::text1);
  c[ImGuiCol_TextDisabled] = col::v4(col::disabled);
  c[ImGuiCol_SliderGrab] = col::v4(col::accent);
  c[ImGuiCol_SliderGrabActive] = col::v4(col::mix(col::accent, col::white, 0.2f));
  c[ImGuiCol_CheckMark] = col::v4(col::accent);
  c[ImGuiCol_Header] = col::v4(col::surface2);
  c[ImGuiCol_HeaderHovered] = col::v4(col::surface3);
  c[ImGuiCol_HeaderActive] = col::v4(col::surface3);
  c[ImGuiCol_TableHeaderBg] = col::v4(col::surface1);
  c[ImGuiCol_TableBorderLight] = col::v4(col::border1);
  c[ImGuiCol_TableBorderStrong] = col::v4(col::border1);
  c[ImGuiCol_TableRowBg] = col::v4(col::alpha(col::surface1, 0));
  c[ImGuiCol_TableRowBgAlt] = col::v4(col::alpha(col::surface2, 0.4f));
  c[ImGuiCol_PlotHistogram] = col::v4(col::text2);
  c[ImGuiCol_ScrollbarBg] = col::v4(col::alpha(col::bg1, 0));
  c[ImGuiCol_ScrollbarGrab] = col::v4(col::border2);
  c[ImGuiCol_ScrollbarGrabHovered] = col::v4(col::text3);
  c[ImGuiCol_ScrollbarGrabActive] = col::v4(col::text2);
  c[ImGuiCol_TextSelectedBg] = col::v4(col::alpha(col::accent, 0.35f));
  c[ImGuiCol_NavCursor] = col::v4(col::accent);
  st.ScaleAllSizes(scale);
}

float ease_out(float t) {
  t = std::clamp(t, 0.0f, 1.0f);
  return 1.0f - (1.0f - t) * (1.0f - t) * (1.0f - t);
}

float tween(ImGuiID id, float target, float seconds) {
  ImGuiStorage* st = ImGui::GetStateStorage();
  float v = st->GetFloat(id, target);
  const float dt = ctx().dt;
  const float k = seconds <= 0 ? 1.0f : std::clamp(dt / seconds * 3.0f, 0.0f, 1.0f);  // ~95 % in `seconds`
  v += (target - v) * k;
  if (std::fabs(target - v) < 1e-3f) v = target;
  st->SetFloat(id, v);
  return v;
}

}  // namespace ally::ui
