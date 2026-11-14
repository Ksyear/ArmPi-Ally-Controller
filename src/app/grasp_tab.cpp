// Grasp tab (docs/CONTROLS_KO.md): candidates, hold-to-run, progress, calibration,
// plus the camera overlay and the per-frame sequencer tick. Part of GuiApp, split out of gui_app.cpp.
#include <algorithm>
#include <cmath>

#include "app/gui_app.hpp"
#include "app/ui_icons.hpp"

namespace ally::ui {

namespace {

const Fonts& F() { return ctx().fonts; }

using GS = robot::GraspState;

constexpr float kPickRadiusPx = 40.0f;  // camera tap -> nearest candidate within this (image pixels)

ImU32 rgb_u32(uint32_t rgb) { return IM_COL32((rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF, 255); }

Status grasp_status(GS s) {
  switch (s) {
    case GS::kDone: return Status::kOk;
    case GS::kFailed:
    case GS::kAborted: return Status::kDanger;
    case GS::kPaused:
    case GS::kConfirm: return Status::kWarn;
    case GS::kIdle: return Status::kOff;
    default: return Status::kIdle;
  }
}

void wrapped(const std::string& t, ImU32 c, float size) {
  ImGui::PushFont(F().regular, size);
  ImGui::PushStyleColor(ImGuiCol_Text, col::v4(c));
  ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x);
  ImGui::TextUnformatted(t.c_str());
  ImGui::PopTextWrapPos();
  ImGui::PopStyleColor();
  ImGui::PopFont();
}

double dist3(const robot::ArmTarget& a, const robot::Pose5& b) {
  return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
}

robot::ArmTarget pose_target(const robot::Pose5& p, double gripper) { return robot::ArmTarget{p.x, p.y, p.z, p.pitch, gripper}; }

}  // namespace

// ───────────────────────── lifecycle ─────────────────────────

void GuiApp::set_demo_calib(const std::string& path) {
  vision::VisionCalib c;
  std::string err;
  if (vision::load_calib(path, &c, &err)) {
    demo_calib_ = true;
    set_calib(c, false);
    log_.info("데모 보정 파일을 이번 실행에만 씀");
  } else {
    log_.warn("데모 보정 파일: " + err);
  }
}

void GuiApp::set_calib(const vision::VisionCalib& c, bool save_file) {
  calib_ = c;
  reach_band_dirty_ = true;
  if (grasp_) grasp_->set_calib(calib_);
  if (!save_file) return;
  if (demo_calib_ || !demo_host_.empty()) {
    notify("데모 실행에서는 보정을 저장하지 않습니다", Status::kIdle);
    return;
  }
  std::string err;
  if (vision::save_calib(vision::calib_path(), calib_, &err)) {
    log_.ok("카메라 보정 저장");
    notify("보정 저장됨", Status::kOk);
  } else {
    log_.warn("보정 저장 실패: " + err);
    notify("보정 저장 실패: " + err, Status::kDanger, 4.0);
  }
}

void GuiApp::start_grasp() {
  if (!core_) return;
  grasp_robot_ = std::make_unique<robot::CoreGraspRobot>(*core_);
  robot::GraspOptions o;
  o.confirm_before_descend = settings_.autograsp_confirm;
  o.max_retries = settings_.autograsp_retry;
  grasp_ = std::make_unique<robot::GraspSequencer>(
      *grasp_robot_,
      robot::GraspVision{[this] { return vision_.latest(); }, [this] { return vision_.detector().classes(); }}, log_, o);
  grasp_->set_options(o);
  grasp_->set_calib(calib_);
  grasp_done_seen_ = grasp_fail_seen_ = 0;
  reach_band_dirty_ = true;
}

void GuiApp::stop_grasp() {
  if (grasp_) grasp_->cancel();
  grasp_.reset();
  grasp_robot_.reset();
  calib_mode_ = false;
  vision_.set_enabled(false);
  vision_.clear();
}

// ───────────────────────── per frame ─────────────────────────

void GuiApp::grasp_frame(double now) {
  const bool on_tab = core_ && tab_ == Tab::kGrasp && !show_connect_;
  const bool want_vision = on_tab && settings_.camera_on;
  if (want_vision != vision_.enabled()) {
    vision_.set_enabled(want_vision);
    if (!want_vision) vision_.clear();
  }
  if (!grasp_) return;
  robot::GraspOptions o = grasp_->options();
  if (o.confirm_before_descend != settings_.autograsp_confirm || o.max_retries != settings_.autograsp_retry) {
    o.confirm_before_descend = settings_.autograsp_confirm;
    o.max_retries = settings_.autograsp_retry;
    grasp_->set_options(o);
  }
  // Screenshot helper: select the first detection (selection only; nothing moves without the dead-man).
  if (demo_state_ == "grasp" && !demo_state_done_ && on_tab) {
    const auto ds = vision_.latest();
    if (ds && !ds->items.empty() && grasp_->select(ds->items.front().track_id, now)) demo_state_done_ = true;
  }
  // Dead-man: A (or the on-screen hold button) on the grasp tab, window focused, not driving the base.
  const auto snap = core_->snapshot();
  const bool driving = std::fabs(snap.applied_f) + std::fabs(snap.applied_s) + std::fabs(snap.applied_t) > 1e-3;
  const bool deadman = on_tab && focused_ && !calib_mode_ && !driving && (last_pad_.down(input::kA) || grasp_hold_touch_);
  grasp_->tick(now, deadman);
  const auto& st = grasp_->status();
  if (st.done_events > grasp_done_seen_) {
    haptics_.play(input::Haptic::kServiceOk, now);
    notify("집기 완료 — 물체를 들고 있습니다", Status::kOk, 4.0);
  }
  if (st.fail_events > grasp_fail_seen_) {
    haptics_.play(input::Haptic::kServiceFail, now);
    notify(st.message, st.state == GS::kAborted ? Status::kDanger : Status::kWarn, 4.0);
  }
  grasp_done_seen_ = st.done_events;
  grasp_fail_seen_ = st.fail_events;
}

void GuiApp::grasp_pad(uint16_t pressed) {
  if (!grasp_ || calib_mode_) return;
  const auto& st = grasp_->status();
  if (st.state == GS::kConfirm) {
    // Fine-tune the grasp point: up/down = forward/back (x), left/right = y. 5 mm per press.
    if (pressed & input::kDpadUp) grasp_->nudge(0.005, 0, 0);
    if (pressed & input::kDpadDown) grasp_->nudge(-0.005, 0, 0);
    if (pressed & input::kDpadLeft) grasp_->nudge(0, 0.005, 0);
    if (pressed & input::kDpadRight) grasp_->nudge(0, -0.005, 0);
    return;
  }
  const int dir = (pressed & input::kDpadRight) ? 1 : (pressed & input::kDpadLeft) ? -1 : 0;
  if (dir == 0) return;
  const auto ds = vision_.latest();
  if (!ds || ds->items.empty()) return;
  // Cycle left-to-right through what is on screen.
  std::vector<const vision::Detection*> items;
  for (const auto& d : ds->items) items.push_back(&d);
  std::sort(items.begin(), items.end(), [](const vision::Detection* a, const vision::Detection* b) { return a->u < b->u; });
  int cur = -1;
  for (size_t i = 0; i < items.size(); ++i)
    if (items[i]->track_id == st.selected_track) cur = static_cast<int>(i);
  const int n = static_cast<int>(items.size());
  const int next = cur < 0 ? (dir > 0 ? 0 : n - 1) : (cur + dir + n) % n;
  grasp_->select(items[static_cast<size_t>(next)]->track_id, mono_now());
}

void GuiApp::grasp_pick(float u, float v, double now) {
  if (!core_) return;
  if (calib_mode_) {
    if (calib_step_ != 2) return;
    if (dist3(core_->snapshot().target, calib_edit_.observe_pose) > 0.01) {
      notify("먼저 '관측 자세로 이동' — 표시는 관측 자세의 영상에서 누릅니다", Status::kWarn, 3.0);
      return;
    }
    calib_tap_set_ = true;
    calib_tap_u_ = u;
    calib_tap_v_ = v;
    calib_edit_.image_w = cam_w_;
    calib_edit_.image_h = cam_h_;
    return;
  }
  if (!grasp_) return;
  const auto ds = vision_.latest();
  const vision::Detection* best = nullptr;
  float bd = kPickRadiusPx;
  if (ds) {
    for (const auto& d : ds->items) {
      const float dd = std::hypot(d.u - u, d.v - v);
      if (dd <= bd) {
        bd = dd;
        best = &d;
      }
    }
  }
  if (best) {
    if (!grasp_->select(best->track_id, now)) notify("집기 중에는 다른 물체를 고를 수 없습니다", Status::kWarn, 2.0);
    grasp_hint_.clear();
    return;
  }
  // Empty spot: show where that is on the table (handy to check a calibration).
  double x = 0, y = 0;
  std::string why;
  if (calib_.usable(cam_w_, cam_h_, &why) && calib_.pixel_to_table(u, v, 0, &x, &y))
    grasp_hint_ = strf("누른 곳: 책상 x %.1f cm, y %.1f cm", x * 100, y * 100);
  else
    grasp_hint_ = "누른 곳에 인식된 물체가 없습니다";
}

// ───────────────────────── camera overlay ─────────────────────────

void GuiApp::draw_grasp_overlay(ImDrawList* dl, ImVec2 p0, ImVec2 p1) {
  if (!grasp_ || cam_w_ <= 0 || cam_h_ <= 0) return;
  const float sx = (p1.x - p0.x) / static_cast<float>(cam_w_), sy = (p1.y - p0.y) / static_cast<float>(cam_h_);
  auto to_screen = [&](float u, float v) { return ImVec2(p0.x + u * sx, p0.y + v * sy); };
  dl->PushClipRect(p0, p1, true);  // nothing spills into the letterbox
  std::string why;
  const bool calib_ok = calib_.usable(cam_w_, cam_h_, &why);
  // Reachable table area (computed once per calibration).
  if (calib_ok) {
    if (reach_band_dirty_) {
      reach_band_ = grasp_->planner().reach_band_pixels(calib_, 48);
      reach_band_dirty_ = false;
    }
    if (reach_band_.size() >= 3) {
      for (const auto& p : reach_band_) dl->PathLineTo(to_screen(p.first, p.second));
      dl->PathStroke(col::alpha(col::ok, 0.5f), ImDrawFlags_Closed, S(2));
    }
  }
  const auto& st = grasp_->status();
  const auto ds = vision_.latest();
  if (ds && !calib_mode_) {
    const auto classes = vision_.detector().classes();
    for (const auto& d : ds->items) {
      const bool sel = d.track_id == st.selected_track;
      bool reach = false;
      if (calib_ok && d.class_index >= 0 && d.class_index < static_cast<int>(classes.size())) {
        std::string e;
        reach = grasp_->planner().plan(calib_, d.u, d.v, classes[static_cast<size_t>(d.class_index)], &e).has_value();
      }
      const ImVec2 a = to_screen(d.bbox[0], d.bbox[1]), b = to_screen(d.bbox[2], d.bbox[3]);
      const ImU32 c = sel ? col::accent : (calib_ok && !reach) ? col::warn : col::alpha(col::white, 0.75f);
      if (sel) {
        // corner brackets + centre cross
        const float k = std::min(S(16), std::min(b.x - a.x, b.y - a.y) * 0.45f), t = S(3);
        dl->AddLine(a, ImVec2(a.x + k, a.y), c, t);
        dl->AddLine(a, ImVec2(a.x, a.y + k), c, t);
        dl->AddLine(ImVec2(b.x, a.y), ImVec2(b.x - k, a.y), c, t);
        dl->AddLine(ImVec2(b.x, a.y), ImVec2(b.x, a.y + k), c, t);
        dl->AddLine(ImVec2(a.x, b.y), ImVec2(a.x + k, b.y), c, t);
        dl->AddLine(ImVec2(a.x, b.y), ImVec2(a.x, b.y - k), c, t);
        dl->AddLine(b, ImVec2(b.x - k, b.y), c, t);
        dl->AddLine(b, ImVec2(b.x, b.y - k), c, t);
        const ImVec2 m = to_screen(d.u, d.v);
        dl->AddLine(ImVec2(m.x - S(8), m.y), ImVec2(m.x + S(8), m.y), c, S(2));
        dl->AddLine(ImVec2(m.x, m.y - S(8)), ImVec2(m.x, m.y + S(8)), c, S(2));
      } else {
        dl->AddRect(a, b, c, S(2), 0, S(1.5f));
      }
      if (d.class_index >= 0 && d.class_index < static_cast<int>(classes.size())) {
        std::string label = classes[static_cast<size_t>(d.class_index)].name;
        if (calib_ok && !reach) label += " · 닿지 않음";
        const ImVec2 ls = w::text_size(F().semibold, fs::micro, label.c_str());
        w::osd_text(dl, ImVec2(a.x, std::max(p0.y + S(2), a.y - ls.y - S(4))), F().semibold, fs::micro, c, label.c_str());
      }
    }
  }
  // Planned grasp point (table plane): filled dot = grasp, ring = hover above it.
  if (st.plan && calib_ok && st.state != GS::kIdle && st.state != GS::kSelected) {
    double u = 0, v = 0;
    if (calib_.table_to_pixel(st.plan->grasp.x, st.plan->grasp.y, &u, &v)) {
      const ImVec2 m = to_screen(static_cast<float>(u), static_cast<float>(v));
      dl->AddCircleFilled(m, S(6), col::accent, 20);
      dl->AddCircle(m, S(14), col::accent, 28, S(2));
    }
  }
  // Calibration: collected points and the pending tap.
  if (calib_mode_) {
    int i = 1;
    for (const auto& p : calib_edit_.points) {
      const ImVec2 m = to_screen(static_cast<float>(p.u), static_cast<float>(p.v));
      dl->AddCircleFilled(m, S(9), col::info, 20);
      const std::string n = std::to_string(i++);
      w::text_center(dl, m, F().semibold, fs::micro, col::accentInk, n.c_str());
    }
    if (calib_tap_set_) {
      const ImVec2 m = to_screen(calib_tap_u_, calib_tap_v_);
      dl->AddCircle(m, S(16), col::accent, 28, S(2));
      dl->AddLine(ImVec2(m.x - S(24), m.y), ImVec2(m.x + S(24), m.y), col::accent, S(2));
      dl->AddLine(ImVec2(m.x, m.y - S(24)), ImVec2(m.x, m.y + S(24)), col::accent, S(2));
    }
  }
  dl->PopClipRect();
}

// ───────────────────────── tab panel ─────────────────────────

void GuiApp::draw_tab_grasp(ImVec2 size) {
  if (!core_ || !grasp_) return;
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const float wdt = size.x - S(20);  // scrollable panel: room for the scrollbar
  const auto& st = grasp_->status();
  const auto snap = core_->snapshot();
  // title row: name + one state word
  {
    const ImVec2 p = ImGui::GetCursorScreenPos();
    w::text(dl, p, F().semibold, fs::title, col::text1, "집기");
    const float tx = p.x + w::text_size(F().semibold, fs::title, "집기").x + S(sp::s4);
    const Status s = calib_mode_ ? Status::kIdle : grasp_status(st.state);
    const char* word = calib_mode_ ? "보정 중" : robot::to_string_ko(st.state == GS::kPaused ? GS::kPaused : st.state);
    w::status_symbol(dl, ImVec2(tx + S(6), p.y + S(20)), S(6), s);
    w::text(dl, ImVec2(tx + S(20), p.y + S(8)), F().regular, fs::label,
            s == Status::kDanger ? col::danger : s == Status::kWarn ? col::warn : s == Status::kOk ? col::ok : col::text2, word);
    if (st.attempt > 0) {
      const std::string a = strf("재시도 %d", st.attempt);
      const ImVec2 as = w::text_size(F().regular, fs::micro, a.c_str());
      w::text(dl, ImVec2(p.x + wdt - as.x, p.y + S(10)), F().regular, fs::micro, col::text2, a.c_str());
    }
    ImGui::Dummy(ImVec2(wdt, S(34)));
  }
  if (calib_mode_) {
    draw_grasp_calibration(wdt);
    return;
  }
  // progress
  int current = robot::grasp_step_index(st.state);
  int failed = -1;
  if (st.state == GS::kPaused) current = robot::grasp_step_index(st.resume_state);
  if (st.state == GS::kFailed || st.state == GS::kAborted) {
    failed = robot::grasp_step_index(st.failed_state);
    current = failed;
  }
  w::stepper("steps", robot::kGraspStepLabels, robot::kGraspSteps, current, failed, ImVec2(wdt, S(40)));
  // one message line (what to do next / why it stopped)
  {
    std::string why;
    std::string msg = st.message;
    if (msg.empty()) {
      if (!settings_.camera_on) msg = "영상이 꺼져 있습니다 — 영상을 켜세요";
      else if (!calib_.usable(cam_w_, cam_h_, &why)) msg = "먼저 카메라 보정이 필요합니다: " + why;
      else msg = "영상이나 목록에서 물체를 고르세요(D-pad ←→)";
    }
    const ImU32 c = st.state == GS::kFailed || st.state == GS::kAborted ? col::danger
                    : st.state == GS::kPaused || st.state == GS::kConfirm ? col::warn
                                                                            : col::text2;
    ImGui::BeginChild("msg", ImVec2(wdt, S(56)), ImGuiChildFlags_None, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoBackground);
    wrapped(msg, c, fs::label);
    ImGui::EndChild();
  }
  // hold-to-run (dead-man) + cancel
  {
    const float cw = S(150), gap = S(sp::s3);
    const char* label = "A 누르고 있는 동안 진행";
    if (st.state == GS::kSelected || st.state == GS::kFailed || st.state == GS::kAborted) label = "A 누르고 있으면 시작";
    if (st.state == GS::kConfirm) label = "A 다시 눌러 집기";
    if (st.state == GS::kPaused) label = "A 다시 눌러 이어서";
    if (st.state == GS::kDone) label = "완료";
    if (st.state == GS::kIdle) label = "먼저 물체 고르기";
    grasp_hold_touch_ = w::hold_button_active("go", label, ImVec2(wdt - cw - gap, S(96)), icon::kHand) ||
                        grasp_hold_touch_;
    const ImVec2 bp = ImGui::GetItemRectMin();
    w::pad_badge(dl, ImVec2(bp.x + S(10), bp.y + S(10)), w::PadKey::kA, S(28), last_pad_.down(input::kA));
    ImGui::SameLine(0, gap);
    if (w::button("cancel", "취소", ImVec2(cw, S(96)), w::Variant::kDanger, icon::kX, st.state != GS::kIdle)) {
      grasp_->cancel();
      grasp_hint_.clear();
    }
  }
  // fine-tune before the descent
  if (st.state == GS::kConfirm && st.plan) {
    const char* labels[] = {"앞", "뒤", "왼", "오른", "위", "아래"};
    const double d[6][3] = {{0.005, 0, 0}, {-0.005, 0, 0}, {0, 0.005, 0}, {0, -0.005, 0}, {0, 0, 0.005}, {0, 0, -0.005}};
    const float bw = (wdt - 5 * S(sp::s2)) / 6;
    for (int i = 0; i < 6; ++i) {
      if (i) ImGui::SameLine(0, S(sp::s2));
      ImGui::PushID(i);
      if (w::button("n", labels[i], ImVec2(bw, S(56)), w::Variant::kDefault, i % 2 ? icon::kMinus : icon::kPlus))
        grasp_->nudge(d[i][0], d[i][1], d[i][2]);
      ImGui::PopID();
    }
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const float hx = p.x + w::hint(dl, p, w::PadKey::kDpadUD, "앞뒤", S(24)) + S(sp::s4);
    w::hint(dl, ImVec2(hx, p.y), w::PadKey::kDpadLR, "좌우 · 한 번에 5 mm", S(24));
    ImGui::Dummy(ImVec2(wdt, S(28)));
  }
  ImGui::Dummy(ImVec2(0, S(sp::s2)));
  draw_grasp_candidates(wdt);
  ImGui::Dummy(ImVec2(0, S(sp::s2)));
  // calibration summary
  if (w::begin_card("calib", "카메라 보정", ImVec2(wdt, S(200)))) {
    std::string why;
    const bool ok = calib_.usable(cam_w_, cam_h_, &why);
    const float tw = (ImGui::GetContentRegionAvail().x - 2 * S(sp::s3)) / 3;
    w::number_tile("잔차", calib_.H.valid ? strf("%.1f", calib_.res.rms_m * 1000).c_str() : "-", "mm", ImVec2(tw, S(76)),
                   !calib_.H.valid ? Status::kOff : calib_.res.rms_m > 0.008 ? Status::kWarn : Status::kOk);
    ImGui::SameLine(0, S(sp::s3));
    w::number_tile("책상 높이", calib_.z_table ? strf("%.1f", *calib_.z_table * 100).c_str() : "-", "cm", ImVec2(tw, S(76)),
                   calib_.z_table ? Status::kOk : Status::kOff);
    ImGui::SameLine(0, S(sp::s3));
    w::number_tile("대응점", std::to_string(calib_.points.size()).c_str(), "개", ImVec2(tw, S(76)),
                   calib_.points.size() >= 4 ? Status::kOk : Status::kOff);
    const float bw = S(180);
    if (w::button("calib_start", ok ? "다시 보정" : "보정 시작", ImVec2(bw, S(56)), ok ? w::Variant::kDefault : w::Variant::kPrimary,
                  icon::kCrosshair)) {
      grasp_->cancel();
      calib_mode_ = true;
      calib_step_ = 0;
      calib_edit_ = calib_;
      calib_edit_.points.clear();
      calib_edit_.H = vision::Homography{};
      calib_edit_.res = vision::Residual{};
      calib_tap_set_ = false;
    }
    ImGui::SameLine(0, S(sp::s3));
    ImGui::BeginGroup();
    ImGui::PushFont(F().regular, fs::micro);
    ImGui::TextColored(col::v4(ok ? col::text2 : col::warn), "%s", ok ? "사용 가능" : why.c_str());
    if (!calib_.saved_at.empty()) ImGui::TextColored(col::v4(col::text3), "저장 %s", calib_.saved_at.c_str());
    ImGui::PopFont();
    ImGui::EndGroup();
  }
  w::end_card();
  // options
  ImGui::Checkbox("하강 전에 한 번 더 확인(권장)", &settings_.autograsp_confirm);
  bool retry = settings_.autograsp_retry > 0;
  if (ImGui::Checkbox("못 집으면 한 번 더 시도", &retry)) settings_.autograsp_retry = retry ? 1 : 0;
  if (!grasp_hint_.empty()) wrapped(grasp_hint_, col::text3, fs::micro);
  (void)snap;
}

void GuiApp::draw_grasp_candidates(float wdt) {
  const auto ds = vision_.latest();
  const auto classes = vision_.detector().classes();
  const auto& st = grasp_->status();
  const int n = ds ? std::min<int>(4, static_cast<int>(ds->items.size())) : 0;
  const float row_h = S(52);
  const std::string right = ds ? strf("%d개 · %.0f ms", static_cast<int>(ds->items.size()), vision_.avg_ms()) : std::string();
  if (w::begin_card("cand", nullptr, ImVec2(wdt, S(56) + row_h * 4 + S(sp::s2) * 3 + S(sp::s4)))) {
    w::card_title("인식된 물체", right.c_str());
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const float rw = ImGui::GetContentRegionAvail().x;
    std::string why;
    const bool calib_ok = calib_.usable(cam_w_, cam_h_, &why);
    if (n == 0) {
      wrapped(!settings_.camera_on ? "영상이 꺼져 있습니다" :
              !ds ? "영상을 기다리는 중" : "물체가 보이지 않습니다 — 색 블록이나 공을 팔 앞 책상에 두세요",
              col::text3, fs::label);
    }
    for (int i = 0; i < n; ++i) {
      const auto& d = ds->items[static_cast<size_t>(i)];
      const bool known = d.class_index >= 0 && d.class_index < static_cast<int>(classes.size());
      const bool sel = d.track_id == st.selected_track;
      ImGui::PushID(d.track_id);
      const ImVec2 p = ImGui::GetCursorScreenPos();
      if (ImGui::InvisibleButton("row", ImVec2(rw, row_h)) && !grasp_->select(d.track_id, mono_now()))
        notify("집기 중에는 다른 물체를 고를 수 없습니다", Status::kWarn, 2.0);
      const bool act = ImGui::IsItemActive();
      ImGui::PopID();
      dl->AddRectFilled(p, ImVec2(p.x + rw, p.y + row_h), sel ? col::surface3 : act ? col::surface3 : col::surface2, S(sp::radiusS));
      if (sel) dl->AddRectFilled(p, ImVec2(p.x + S(4), p.y + row_h), col::accent, S(2));
      const float cy = p.y + row_h * 0.5f;
      if (known) dl->AddCircleFilled(ImVec2(p.x + S(24), cy), S(9), rgb_u32(vision::class_display_rgb(classes[static_cast<size_t>(d.class_index)])), 20);
      const char* name = known ? classes[static_cast<size_t>(d.class_index)].name.c_str() : "?";
      const ImVec2 ns = w::text_size(F().semibold, fs::label, name);
      w::text(dl, ImVec2(p.x + S(44), cy - ns.y * 0.5f), F().semibold, fs::label, sel ? col::text1 : col::text1, name);
      double x = 0, y = 0;
      std::string pos = "보정 필요";
      Status rs = Status::kOff;
      const char* rword = "";
      if (calib_ok && calib_.pixel_to_table(d.u, d.v, 0, &x, &y)) {
        pos = strf("x %.1f  y %+.1f cm", x * 100, y * 100);
        std::string e;
        const bool reach = known && grasp_->planner().plan(calib_, d.u, d.v, classes[static_cast<size_t>(d.class_index)], &e).has_value();
        rs = reach ? Status::kOk : Status::kWarn;
        rword = reach ? "닿음" : "닿지 않음";
      }
      const ImVec2 ps = w::text_size(F().mono, fs::micro, pos.c_str());
      const float rx = p.x + rw - S(sp::s3);
      const ImVec2 rws = w::text_size(F().regular, fs::micro, rword);
      w::text(dl, ImVec2(rx - rws.x, cy - rws.y * 0.5f), F().regular, fs::micro, rs == Status::kWarn ? col::warn : col::text2, rword);
      if (*rword) w::status_symbol(dl, ImVec2(rx - rws.x - S(12), cy), S(5), rs);
      w::text(dl, ImVec2(rx - rws.x - S(28) - ps.x, cy - ps.y * 0.5f), F().mono, fs::micro, col::text2, pos.c_str());
    }
    if (n > 0) {
      const ImVec2 p = ImGui::GetCursorScreenPos();
      const float hx = p.x + w::hint(dl, p, w::PadKey::kDpadLR, "고르기", S(24)) + S(sp::s4);
      w::text(dl, ImVec2(hx, p.y + S(2)), F().regular, fs::micro, col::text3, "또는 영상에서 누르기");
      ImGui::Dummy(ImVec2(rw, S(24)));
    }
  }
  w::end_card();
}

// Calibration (docs/ARCHITECTURE.md; works whether the camera sits on the arm or on the chassis):
// every image point is tapped at the observe pose, then the arm tip is placed on that same table mark.
void GuiApp::draw_grasp_calibration(float wdt) {
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const auto snap = core_->snapshot();
  const char* titles[] = {"1/4 관측 자세", "2/4 책상 높이", "3/4 대응점", "4/4 확인·저장"};
  const char* labels[] = {"관측", "높이", "점", "저장"};
  w::stepper("csteps", labels, 4, calib_step_, -1, ImVec2(wdt, S(40)));
  const float bw = (wdt - S(sp::s3)) * 0.5f;
  if (w::begin_card("cal", titles[calib_step_], ImVec2(wdt, S(400)))) {
    const float cw = ImGui::GetContentRegionAvail().x;
    const float hb = (cw - S(sp::s3)) * 0.5f;
    switch (calib_step_) {
      case 0:
        wrapped("물체 인식은 언제나 이 자세에서 합니다. 팔이 영상을 가리지 않는 자세가 좋습니다. 기본 자세를 권장합니다.", col::text2, fs::label);
        if (w::button("obs_def", "기본 자세로 정하기", ImVec2(hb, S(64)), w::Variant::kPrimary, icon::kHouse)) {
          const auto& h = armpi_protocol::kDefaultHome;
          calib_edit_.observe_pose = robot::Pose5{h.x, h.y, h.z, h.pitch, 0.0};
          calib_step_ = 1;  // only stores the pose: the arm moves when '관측 자세로 이동' is pressed
        }
        ImGui::SameLine(0, S(sp::s3));
        if (w::button("obs_cur", "지금 자세로 정하기", ImVec2(hb, S(64)))) {
          const auto& t = snap.target;
          calib_edit_.observe_pose = robot::Pose5{t.x, t.y, t.z, t.pitch, 0.0};
          calib_step_ = 1;
        }
        break;
      case 1:
        wrapped("팔 끝(집게 끝)을 책상 면에 닿을 듯 말 듯 내리세요. LB를 누른 채 오른 스틱, 또는 아래 '아래' 버튼을 씁니다. 그다음 '여기가 책상'.",
                col::text2, fs::label);
        w::text(dl, ImGui::GetCursorScreenPos(), F().mono, fs::label, col::text1, strf("지금 z = %.3f m", snap.target.z).c_str());
        ImGui::Dummy(ImVec2(cw, S(32)));
        if (w::button("ztab", "여기가 책상", ImVec2(hb, S(64)), w::Variant::kPrimary)) {
          calib_edit_.z_table = snap.target.z;
          calib_step_ = 2;
          calib_tap_set_ = false;
        }
        break;
      case 2: {
        const bool at_obs = dist3(snap.target, calib_edit_.observe_pose) <= 0.01;
        std::string guide;
        if (!calib_tap_set_)
          guide = at_obs ? "책상 위 표시(종이 모서리, 스티커)를 영상에서 누르세요." : "먼저 '관측 자세로 이동'을 누르세요. 표시는 관측 자세의 영상에서 누릅니다.";
        else
          guide = "팔 끝을 그 표시 바로 위(약 1 cm)에 맞춘 뒤 '이 점 저장'. 4–6개를 넓게 퍼뜨려 찍습니다.";
        wrapped(guide, col::text2, fs::label);
        if (w::button("goobs", "관측 자세로 이동", ImVec2(hb, S(64)), at_obs ? w::Variant::kDefault : w::Variant::kPrimary, icon::kHouse))
          core_->arm_goto(pose_target(calib_edit_.observe_pose, snap.target.gripper));
        ImGui::SameLine(0, S(sp::s3));
        if (w::button("savept", "이 점 저장", ImVec2(hb, S(64)), calib_tap_set_ ? w::Variant::kPrimary : w::Variant::kDefault,
                      icon::kPlus, calib_tap_set_)) {
          vision::PointPair pp;
          pp.u = calib_tap_u_;
          pp.v = calib_tap_v_;
          pp.x = snap.target.x;
          pp.y = snap.target.y;
          calib_edit_.points.push_back(pp);
          calib_tap_set_ = false;
          std::string err;
          if (calib_edit_.points.size() >= 4) calib_edit_.refit(&err);
        }
        if (w::button("undo", "마지막 점 지우기", ImVec2(hb, S(56)), w::Variant::kDefault, icon::kMinus, !calib_edit_.points.empty())) {
          calib_edit_.points.pop_back();
          calib_edit_.H = vision::Homography{};
          std::string err;
          if (calib_edit_.points.size() >= 4) calib_edit_.refit(&err);
        }
        ImGui::SameLine(0, S(sp::s3));
        if (w::button("next", "다음", ImVec2(hb, S(56)), w::Variant::kPrimary, icon::kChevronRight, calib_edit_.H.valid))
          calib_step_ = 3;
        w::text(dl, ImGui::GetCursorScreenPos(), F().regular, fs::micro, col::text2,
                strf("찍은 점 %d개%s", static_cast<int>(calib_edit_.points.size()),
                     calib_edit_.H.valid ? strf(" · 잔차 %.1f mm", calib_edit_.res.rms_m * 1000).c_str() : " (4개 이상 필요)")
                    .c_str());
        ImGui::Dummy(ImVec2(cw, S(24)));
        break;
      }
      case 3: {
        const float tw = (cw - S(sp::s3)) * 0.5f;
        const bool big = calib_edit_.res.rms_m > 0.008;
        w::number_tile("잔차(책상)", strf("%.1f", calib_edit_.res.rms_m * 1000).c_str(), "mm", ImVec2(tw, S(84)),
                       big ? Status::kWarn : Status::kOk);
        ImGui::SameLine(0, S(sp::s3));
        w::number_tile("잔차(영상)", strf("%.1f", calib_edit_.res.rms_px).c_str(), "px", ImVec2(tw, S(84)));
        wrapped(big ? "잔차가 큽니다(8 mm 초과). 점을 더 넓게, 정확히 찍어 보세요." : "저장하면 바로 집기에 씁니다. 빈 곳을 눌러 책상 좌표가 맞는지 확인할 수 있습니다.",
                big ? col::warn : col::text2, fs::label);
        if (w::button("savecal", "저장", ImVec2(hb, S(64)), w::Variant::kPrimary, icon::kCircleCheck, cam_w_ > 0 && cam_h_ > 0)) {
          calib_edit_.image_w = cam_w_;
          calib_edit_.image_h = cam_h_;
          set_calib(calib_edit_, true);
          calib_mode_ = false;
        }
        ImGui::SameLine(0, S(sp::s3));
        if (w::button("redo", "점 다시 찍기", ImVec2(hb, S(64)), w::Variant::kDefault, icon::kRefreshCw)) {
          calib_edit_.points.clear();
          calib_edit_.H = vision::Homography{};
          calib_step_ = 2;
        }
        break;
      }
      default: calib_step_ = 0; break;
    }
  }
  w::end_card();
  if (w::button("calcancel", "보정 취소", ImVec2(bw, S(56)), w::Variant::kGhost, icon::kX)) {
    calib_mode_ = false;
    calib_tap_set_ = false;
  }
  if (calib_step_ > 0) {
    ImGui::SameLine(0, S(sp::s3));
    if (w::button("calback", "이전 단계", ImVec2(bw, S(56)), w::Variant::kGhost, icon::kChevronLeft)) {
      --calib_step_;
      calib_tap_set_ = false;
    }
  }
}

}  // namespace ally::ui
