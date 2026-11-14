#include "app/gui_app.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "version.hpp"

namespace ally::ui {

namespace {

constexpr ImU32 kRed = IM_COL32(214, 48, 49, 255);
constexpr ImU32 kRedDark = IM_COL32(120, 20, 22, 255);
constexpr ImU32 kGreen = IM_COL32(39, 174, 96, 255);
constexpr ImU32 kYellow = IM_COL32(230, 170, 30, 255);
constexpr ImU32 kBlue = IM_COL32(52, 132, 228, 255);
constexpr ImU32 kGray = IM_COL32(78, 84, 96, 255);
constexpr ImU32 kDark = IM_COL32(24, 26, 31, 255);
constexpr ImU32 kPanel = IM_COL32(34, 37, 44, 255);
constexpr ImU32 kAccent = IM_COL32(255, 140, 40, 255);
constexpr ImU32 kWhite = IM_COL32(255, 255, 255, 255);
constexpr ImU32 kDim = IM_COL32(160, 166, 178, 255);

// Workspace box from the robot config (robot clamps again; this is only for drawing and the reach filter).
constexpr double kWsXMin = 0.05, kWsXMax = 0.32, kWsYMin = -0.25, kWsYMax = 0.25, kWsZMin = -0.08, kWsZMax = 0.32;

ImVec4 col4(ImU32 c) { return ImGui::ColorConvertU32ToFloat4(c); }

ImU32 state_color(const std::string& s) {
  if (s == "active") return kGreen;
  if (s == "idle") return kBlue;
  if (s == "emergency_stop" || s == "fault") return kRed;
  if (s == "no_ack") return kGray;
  return kYellow;  // awaiting_neutral, timeout
}

const char* state_ko(const std::string& s) {
  if (s == "active") return "활성";
  if (s == "idle") return "대기";
  if (s == "awaiting_neutral") return "중립 확인 중";
  if (s == "timeout") return "시간 초과";
  if (s == "emergency_stop") return "비상정지";
  if (s == "fault") return "고장";
  if (s == "no_ack") return "응답 없음";
  return s.c_str();
}

const char* reason_ko(const std::string& r) {
  if (r == "ok") return "정상";
  if (r == "not_owner") return "다른 조종자";
  if (r == "neutral_required") return "중립 필요";
  if (r == "emergency_stop") return "비상정지";
  if (r == "workspace_limit") return "작업 공간 밖";
  if (r == "ik_unreachable") return "닿을 수 없음";
  if (r == "invalid_packet") return "형식 오류";
  if (r == "out_of_order") return "순서 오류";
  if (r == "no_ack") return "응답 없음";
  return r.c_str();
}

std::string fmt_ms(const std::optional<double>& v) { return v ? strf("%.0f", *v) : std::string("-"); }

}  // namespace

GuiApp::GuiApp(Platform p, Settings s, Fonts fonts) : plat_(std::move(p)), settings_(std::move(s)), fonts_(fonts) {
  std::snprintf(ip_buf_, sizeof(ip_buf_), "%s", settings_.robot_host.c_str());
  haptics_.set_enabled(settings_.haptics);
  haptics_.set_strength(settings_.haptic_strength);
  log_.info(std::string(kAppName) + " " + kVersion + " 시작. XInput: " + pad_.backend());
  std::string err;
  prune_logs(settings_dir(), 20);
  if (log_.open_file(settings_dir() + "/ally_" + wall_stamp() + ".log", &err)) log_.info("기록 파일: " + settings_dir());
}

GuiApp::~GuiApp() { shutdown(); }

void GuiApp::shutdown() {
  if (discover_thread_.joinable()) discover_thread_.join();
  disconnect();
  pad_.set_rumble(0, 0);
  if (plat_.keep_awake && keep_awake_on_) plat_.keep_awake(false);
  keep_awake_on_ = false;
  save();
}

void GuiApp::set_demo_connect(const std::string& host, int port_base, const std::string& tab) {
  demo_host_ = host;
  demo_base_ = port_base;
  demo_pending_ = !host.empty();
  if (tab == "arm") tab_ = Tab::kArm;
  else if (tab == "diag") tab_ = Tab::kDiag;
  else if (tab == "settings") tab_ = Tab::kSettings;
}

void GuiApp::save() {
  if (!demo_host_.empty()) return;  // demo/test runs never touch the user's settings file
  std::string err;
  if (!save_settings(settings_path(), settings_, &err)) log_.warn(err);
}

void GuiApp::notify(const std::string& text, ImU32 color, double secs) {
  notes_.push_back(Note{utf8_truncate(text, 160), color, mono_now() + secs});
  while (notes_.size() > 4) notes_.pop_front();
}

void GuiApp::set_focused(bool focused) {
  if (focused == focused_) return;
  focused_ = focused;
  if (!focused) {
    // Safety rule: losing focus drops every intent; nothing resumes by itself.
    if (core_) {
      core_->clear_input();
      core_->set_arm_enabled(false);
    }
    touch_arm_toggle_ = false;
    touch_ = input::TouchControls{};
    pad_.set_rumble(0, 0);
    log_.info("창 포커스 잃음 → 중립");
  } else {
    notify("포커스 복귀 — 스틱을 놓았다가 다시 조작하세요", kYellow, 2.5);
  }
}

// ───────────────────────── connection ─────────────────────────

void GuiApp::connect(const std::string& host_in) {
  const std::string host = trim(host_in);
  if (host.empty()) {
    notify("로봇 주소를 입력하세요", kYellow);
    return;
  }
  bool ok_chars = host.size() <= 253;
  for (char c : host) ok_chars = ok_chars && ((c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '.' || c == '-');
  if (!ok_chars) {
    notify("로봇 주소 형식 오류(숫자·영문·점·하이픈만)", kRed);
    return;
  }
  disconnect();
  robot::CoreOptions opt;
  opt.host = host;
  opt.control_port = settings_.control_port;
  opt.rosbridge_port = settings_.rosbridge_port;
  opt.camera_port = settings_.camera_port;
  if (!demo_host_.empty() && host == demo_host_ && demo_base_ > 0) {
    opt.control_port = demo_base_ + 1;
    opt.rosbridge_port = demo_base_ + 4090;
    opt.camera_port = demo_base_ + 3080;
  }
  opt.camera_url = settings_.camera_url;
  opt.ack_timeout_s = settings_.ack_timeout_s;
  opt.speed = settings_.speed;
  auto core = std::make_unique<robot::RobotCore>(opt, log_, comm_);
  std::string err;
  if (!core->start(&err)) {
    notify("연결 실패: " + err, kRed, 5.0);
    log_.error("연결 실패: " + err);
    return;
  }
  core_ = std::move(core);
  cues_init_ = false;
  cam_seq_ = 0;
  settings_.robot_host = host;
  save();
  show_connect_ = false;
  notify(demo_host_.empty() ? "연결: " + host : std::string("연결: 데모 로봇"), kGreen);
}

void GuiApp::disconnect() {
  if (!core_) return;
  core_->stop();
  core_.reset();
  cam_tex_ = ImTextureID{};
  cam_w_ = cam_h_ = 0;
  touch_arm_toggle_ = false;
  log_.info("연결 끊음");
}

void GuiApp::start_discovery() {
  if (discovering_) return;
  if (discover_thread_.joinable()) discover_thread_.join();
  discovering_ = true;
  const int port = settings_.discovery_port;
  discover_thread_ = std::thread([this, port] {
    std::string err;
    auto found = net::discover(1.5, {}, &comm_, &err, port);
    std::lock_guard<std::mutex> lk(discover_mu_);
    discovered_ = std::move(found);
    discover_error_ = err;
    discovering_ = false;
  });
}

// ───────────────────────── input → core ─────────────────────────

void GuiApp::handle_input(double now) {
  ImGuiIO& io = ImGui::GetIO();
  pad_.stick_deadzone = settings_.deadzone;
  mapper_.settings().curve = settings_.curve;
  mapper_.settings().allow_drive_while_arm = settings_.allow_drive_while_arm;

  const input::PadState pad = pad_.poll();
  if (pad_was_connected_ && !pad.connected) {
    notify("게임패드 연결 끊김 — 중립", kRed, 4.0);
    log_.warn("게임패드 분리");
  } else if (!pad_was_connected_ && pad.connected && now > 2.0) {
    notify("게임패드 연결됨", kGreen, 2.0);
  }
  pad_was_connected_ = pad.connected;
  last_pad_ = pad;

  input::KeyState k;
  k.space = ImGui::IsKeyDown(ImGuiKey_Space);
  k.ctrl = io.KeyCtrl;
  if (!io.WantTextInput) {
    k.w = ImGui::IsKeyDown(ImGuiKey_W);
    k.a = ImGui::IsKeyDown(ImGuiKey_A);
    k.s = ImGui::IsKeyDown(ImGuiKey_S);
    k.d = ImGui::IsKeyDown(ImGuiKey_D);
    k.q = ImGui::IsKeyDown(ImGuiKey_Q);
    k.e = ImGui::IsKeyDown(ImGuiKey_E);
    k.i = ImGui::IsKeyDown(ImGuiKey_I);
    k.j = ImGui::IsKeyDown(ImGuiKey_J);
    k.k = ImGui::IsKeyDown(ImGuiKey_K);
    k.l = ImGui::IsKeyDown(ImGuiKey_L);
    k.u = ImGui::IsKeyDown(ImGuiKey_U);
    k.o = ImGui::IsKeyDown(ImGuiKey_O);
    k.r = ImGui::IsKeyDown(ImGuiKey_R);
    k.f = ImGui::IsKeyDown(ImGuiKey_F);
    k.g = ImGui::IsKeyDown(ImGuiKey_G);
    k.h = ImGui::IsKeyDown(ImGuiKey_H);
    k.shift = io.KeyShift;
    for (int d = 0; d <= 9; ++d) {
      if (ImGui::IsKeyPressed(static_cast<ImGuiKey>(ImGuiKey_0 + d), false)) k.digit = d;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_F1, false)) tab_ = Tab::kDrive;
    if (ImGui::IsKeyPressed(ImGuiKey_F2, false)) tab_ = Tab::kArm;
    if (ImGui::IsKeyPressed(ImGuiKey_F3, false)) tab_ = Tab::kDiag;
    if (ImGui::IsKeyPressed(ImGuiKey_F4, false)) tab_ = Tab::kSettings;
    if (ImGui::IsKeyPressed(ImGuiKey_F11, false) && plat_.set_fullscreen) {
      settings_.fullscreen = !settings_.fullscreen;
      plat_.set_fullscreen(settings_.fullscreen);
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
      if (tab_ == Tab::kSettings) tab_ = Tab::kDrive;
      else if (show_connect_ && core_) show_connect_ = false;
    }
  }

  const auto r = mapper_.map(pad, k, touch_, now, focused_);
  clear_progress_pad_ = r.ev.clear_progress;
  if (r.ev.estop) {
    if (core_) core_->estop();
    haptics_.play(input::Haptic::kEstop, now);
    notify("비상정지", kRed, 4.0);
  }
  if (!core_) {
    prev_arm_hold_ = r.arm_hold;
    if (r.ev.menu) tab_ = tab_ == Tab::kSettings ? Tab::kDrive : Tab::kSettings;
    return;
  }
  if (r.ev.clear_estop) {
    core_->clear_estop();
    notify("비상정지 해제 요청 — 스틱을 놓았다가 다시", kYellow);
  }
  if (r.ev.speed_delta != 0) core_->set_speed(core_->speed() + 0.1 * r.ev.speed_delta);
  if (r.ev.speed_set_percent > 0) core_->set_speed(r.ev.speed_set_percent / 100.0);
  if (r.ev.speed_delta != 0 || r.ev.speed_set_percent > 0) {
    settings_.speed = core_->speed();
    notify(strf("속도 상한 %.0f %%", settings_.speed * 100), kBlue, 1.2);
  }
  if (r.ev.camera_toggle) settings_.camera_on = !settings_.camera_on;
  if (r.ev.camera_big) camera_big_ = !camera_big_;
  if (r.ev.arm_sync) {
    core_->arm_sync();
    notify("팔 목표를 로봇 현재 자세로 맞춤", kBlue);
  }
  if (r.ev.menu) tab_ = tab_ == Tab::kSettings ? Tab::kDrive : Tab::kSettings;
  if (r.ev.z_step != 0 || r.ev.pitch_step != 0) core_->arm_nudge(0, 0, r.ev.z_step, r.ev.pitch_step);

  // Arm dead-man: LB / Shift held. A touch toggle (latched) keeps the arm on after release.
  const auto snap = core_->snapshot();
  if (touch_arm_toggle_ && !snap.arm_latched && !r.arm_hold) touch_arm_toggle_ = false;  // core turned it off
  if (r.arm_hold && !prev_arm_hold_) core_->set_arm_enabled(true, false);
  if (!r.arm_hold && prev_arm_hold_ && !touch_arm_toggle_) core_->set_arm_enabled(false);
  prev_arm_hold_ = r.arm_hold;

  // The settings tab (Menu) counts as "not driving": all intents neutral while it is open.
  if (focused_ && tab_ != Tab::kSettings) core_->set_input(r.input);
  else core_->clear_input();
}

void GuiApp::handle_cues(double now) {
  if (!core_) return;
  const auto snap = core_->snapshot();
  const auto& c = snap.cues;
  if (!cues_init_) {
    last_cues_ = c;
    cues_init_ = true;
  }
  if (c.estop > last_cues_.estop) {
    haptics_.play(input::Haptic::kEstop, now);
    notify("로봇 비상정지 latch", kRed, 4.0);
  }
  if (c.limit > last_cues_.limit) {
    haptics_.play(input::Haptic::kLimit, now);
    notify("팔: 닿는 범위의 끝", kYellow, 1.5);
  }
  if (c.rejected > last_cues_.rejected) {
    haptics_.play(input::Haptic::kReject, now);
    notify("로봇이 팔 목표를 거부함 → 마지막 수락값으로", kYellow);
  }
  if (c.link_lost > last_cues_.link_lost) {
    haptics_.play(input::Haptic::kLinkLost, now);
    notify("통신 끊김 — 다시 붙는 중", kYellow, 3.0);
  }
  if (c.link_ok > last_cues_.link_ok && last_cues_.link_ok > 0) haptics_.play(input::Haptic::kLinkOk, now);
  if (c.arm_on > last_cues_.arm_on) haptics_.play(input::Haptic::kArmOn, now);
  last_cues_ = c;

  if (const auto res = core_->ros().last_result(); res && res->mono > last_service_seen_) {
    last_service_seen_ = res->mono;
    haptics_.play(res->ok ? input::Haptic::kServiceOk : input::Haptic::kServiceFail, now);
    notify((res->ok ? "완료: " : "실패: ") + res->service + (res->message.empty() ? "" : " — " + res->message),
           res->ok ? kGreen : kRed, 3.5);
  }
}

// ───────────────────────── frame ─────────────────────────

void GuiApp::frame(double now) {
  ImGuiIO& io = ImGui::GetIO();
  scale_ = std::max(0.6f, io.DisplaySize.y / 1080.0f);

  if (demo_pending_) {
    demo_pending_ = false;
    connect(demo_host_);
  }
  handle_input(now);
  handle_cues(now);

  float lo = 0, hi = 0;
  haptics_.set_enabled(settings_.haptics);
  haptics_.set_strength(settings_.haptic_strength);
  haptics_.sample(now, &lo, &hi);
  pad_.set_rumble(focused_ ? lo : 0.0f, focused_ ? hi : 0.0f);

  if (now >= next_power_poll_) {
    next_power_poll_ = now + 5.0;
    if (plat_.ally_battery_percent) ally_batt_ = plat_.ally_battery_percent();
    if (plat_.ally_on_ac) ally_ac_ = plat_.ally_on_ac();
  }
  const bool want_awake = core_ && core_->snapshot().connected;
  if (plat_.keep_awake && want_awake != keep_awake_on_) {
    plat_.keep_awake(want_awake);
    keep_awake_on_ = want_awake;
  }

  // camera stream follows the setting
  if (core_) {
    if (settings_.camera_on && !core_->camera().running()) core_->start_camera();
    if (!settings_.camera_on && core_->camera().running()) core_->stop_camera();
    if (const auto f = core_->camera().latest(cam_seq_); f && plat_.upload_camera) {
      cam_tex_ = plat_.upload_camera(f->width, f->height, f->rgba.data());
      cam_w_ = f->width;
      cam_h_ = f->height;
      cam_seq_ = f->seq;
      cam_last_frame_ = now;
    }
  }
  while (!notes_.empty() && notes_.front().until < now) notes_.pop_front();

  ImGui::SetNextWindowPos(ImVec2(0, 0));
  ImGui::SetNextWindowSize(io.DisplaySize);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
  ImGui::PushStyleColor(ImGuiCol_WindowBg, col4(kDark));
  ImGui::Begin("root", nullptr,
               ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBringToFrontOnFocus |
                   ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoScrollWithMouse);
  ImGui::PopStyleColor();
  ImGui::PopStyleVar(2);

  const float W = io.DisplaySize.x, H = io.DisplaySize.y;
  const float top_h = S(70), bottom_h = S(250), side_w = std::max(S(200), W * 0.11f);
  const float body_h = H - top_h - bottom_h;

  ImGui::BeginChild("top", ImVec2(W, top_h), ImGuiChildFlags_None, ImGuiWindowFlags_NoScrollbar);
  draw_top_bar(now);
  ImGui::EndChild();

  const float main_w = W - side_w;
  if (show_connect_ || !core_) {
    ImGui::BeginChild("connect", ImVec2(main_w, body_h), ImGuiChildFlags_None, ImGuiWindowFlags_NoScrollbar);
    draw_connect_panel(ImVec2(main_w, body_h));
    ImGui::EndChild();
  } else {
    const float cam_w = camera_big_ ? main_w : main_w * 0.60f;
    ImGui::BeginChild("camera", ImVec2(cam_w, body_h), ImGuiChildFlags_None, ImGuiWindowFlags_NoScrollbar);
    draw_camera(ImVec2(cam_w, body_h), now);
    ImGui::EndChild();
    if (!camera_big_) {
      ImGui::SameLine(0, 0);
      ImGui::PushStyleColor(ImGuiCol_ChildBg, col4(kPanel));
      ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(S(14), S(10)));
      ImGui::BeginChild("tab", ImVec2(main_w - cam_w, body_h), ImGuiChildFlags_AlwaysUseWindowPadding);
      switch (tab_) {
        case Tab::kDrive: draw_tab_drive(); break;
        case Tab::kArm: draw_tab_arm(); break;
        case Tab::kDiag: draw_tab_diag(); break;
        case Tab::kSettings: draw_tab_settings(); break;
      }
      ImGui::EndChild();
      ImGui::PopStyleVar();
      ImGui::PopStyleColor();
    }
  }
  ImGui::SameLine(0, 0);
  ImGui::BeginChild("side", ImVec2(side_w, body_h), ImGuiChildFlags_None, ImGuiWindowFlags_NoScrollbar);
  draw_side_column(ImVec2(side_w, body_h), now);
  ImGui::EndChild();

  ImGui::BeginChild("bottom", ImVec2(W, bottom_h), ImGuiChildFlags_None, ImGuiWindowFlags_NoScrollbar);
  draw_bottom_bar(ImVec2(W, bottom_h));
  ImGui::EndChild();

  ImGui::End();
}

// ───────────────────────── widgets ─────────────────────────

void GuiApp::chip(const char* text, ImU32 bg, ImU32 fg) {
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const ImVec2 ts = ImGui::CalcTextSize(text);
  const ImVec2 p = ImGui::GetCursorScreenPos();
  const ImVec2 pad(S(10), S(5));
  const ImVec2 q(p.x + ts.x + pad.x * 2, p.y + ts.y + pad.y * 2);
  if (chip_limit_x_ > 0 && q.x > chip_limit_x_) {  // no room left before the tab buttons: drop the rest
    ImGui::Dummy(ImVec2(0, 0));
    return;
  }
  dl->AddRectFilled(p, q, bg, S(8));
  dl->AddText(ImVec2(p.x + pad.x, p.y + pad.y), fg, text);
  ImGui::Dummy(ImVec2(q.x - p.x, q.y - p.y));
}

bool GuiApp::hold_button(const char* id, const char* label, ImVec2 size, double hold_s, ImU32 color, float* progress) {
  ImGui::PushID(id);
  const ImVec2 p = ImGui::GetCursorScreenPos();
  ImGui::InvisibleButton("hb", size);
  const bool active = ImGui::IsItemActive();
  const float held = active ? static_cast<float>(ImGui::GetIO().MouseDownDuration[0]) : 0.0f;
  const float frac = std::min(1.0f, static_cast<float>(held / hold_s));
  if (progress) *progress = frac;
  ImDrawList* dl = ImGui::GetWindowDrawList();
  dl->AddRectFilled(p, ImVec2(p.x + size.x, p.y + size.y), active ? color : kGray, S(10));
  if (frac > 0) dl->AddRectFilled(p, ImVec2(p.x + size.x * frac, p.y + size.y), IM_COL32(255, 255, 255, 60), S(10));
  const ImVec2 ts = ImGui::CalcTextSize(label);
  dl->AddText(ImVec2(p.x + (size.x - ts.x) / 2, p.y + (size.y - ts.y) / 2), kWhite, label);
  // fire once when the hold completes
  static ImGuiID fired_for = 0;
  const ImGuiID me = ImGui::GetItemID();
  bool fire = false;
  if (active && held >= hold_s && fired_for != me) {
    fired_for = me;
    fire = true;
  }
  if (!active && fired_for == me) fired_for = 0;
  ImGui::PopID();
  return fire;
}

bool GuiApp::virtual_stick(const char* id, float radius, float* x, float* y) {
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
  dl->AddCircleFilled(c, radius, IM_COL32(50, 54, 63, 255), 48);
  dl->AddCircle(c, radius, active ? kAccent : kGray, 48, S(3));
  dl->AddLine(ImVec2(c.x - radius * 0.8f, c.y), ImVec2(c.x + radius * 0.8f, c.y), IM_COL32(90, 95, 105, 160), S(1));
  dl->AddLine(ImVec2(c.x, c.y - radius * 0.8f), ImVec2(c.x, c.y + radius * 0.8f), IM_COL32(90, 95, 105, 160), S(1));
  dl->AddCircleFilled(ImVec2(c.x + *x * radius * 0.75f, c.y - *y * radius * 0.75f), radius * 0.32f,
                      active ? kAccent : IM_COL32(120, 126, 138, 255), 32);
  return active;
}

// ───────────────────────── top bar ─────────────────────────

void GuiApp::draw_top_bar(double now) {
  const float tw = S(112);
  chip_limit_x_ = ImGui::GetWindowPos().x + ImGui::GetWindowWidth() - tw * 4 - S(36);
  ImGui::SetCursorPos(ImVec2(S(12), S(10)));
  ImGui::PushFont(fonts_.bold, 0.0f);
  const robot::CoreSnapshot snap = core_ ? core_->snapshot() : robot::CoreSnapshot{};
  const bool ros_ok = core_ && core_->ros().connected();
  const auto st = core_ ? core_->ros().status() : robot::RobotStatus{};
  const bool st_fresh = core_ && core_->ros().status_fresh();
  const bool estop = snap.emergency || (st_fresh && st.emergency_stop);

  chip(estop ? "비상정지" : (core_ ? "정상" : "연결 안 됨"), estop ? kRed : (core_ ? kGreen : kGray));
  ImGui::SameLine(0, S(8));
  if (snap.faulted) {  // fault latches until a new session: say so plainly
    chip("고장 보고됨 — 조종 탭에서 새 세션", kRed);
    ImGui::SameLine(0, S(8));
  }
  if (core_) {
    // Demo/screenshot runs hide the address so captured images never carry a real network address.
    const std::string where = demo_host_.empty() ? snap.robot : std::string("데모 로봇");
    chip(snap.connected ? ("UDP " + where).c_str() : "응답 없음", snap.connected ? kGreen : kYellow);
    ImGui::SameLine(0, S(8));
    chip(strf("차체 %s", state_ko(snap.base.server_state)).c_str(), state_color(snap.base.server_state));
    ImGui::SameLine(0, S(8));
    chip(strf("팔 %s%s", state_ko(snap.arm.server_state), snap.arm_enabled ? " ·켬" : "").c_str(),
         state_color(snap.arm.server_state));
    ImGui::SameLine(0, S(8));
    std::string owner = "조종권 -";
    ImU32 oc = kGray;
    if (st_fresh) {
      if (st.owner.empty()) owner = "조종권 없음";
      else if (st.owner == snap.client_id) {
        owner = "조종권: 나";
        oc = kGreen;
      } else {
        owner = "다른 조종기";
        oc = kRed;
      }
    }
    chip(owner.c_str(), oc);
    ImGui::SameLine(0, S(8));
    chip(ros_ok ? "rosbridge" : "rosbridge 없음", ros_ok ? kGreen : kYellow);
    ImGui::SameLine(0, S(8));
  }
  chip(last_pad_.connected ? "패드" : "패드 없음", last_pad_.connected ? kGreen : (pad_.available() ? kRed : kGray));
  ImGui::SameLine(0, S(8));
  if (st_fresh && st.battery_v) {
    chip(strf("로봇 %.2f V", *st.battery_v).c_str(), st.battery_low ? kYellow : kGray);
    ImGui::SameLine(0, S(8));
  }
  if (ally_batt_ >= 0) {
    chip(strf("Ally %d %%%s", ally_batt_, ally_ac_ ? " 충전" : "").c_str(),
         ally_batt_ <= 10 ? kRed : (ally_batt_ <= 20 ? kYellow : kGray));
    ImGui::SameLine(0, S(8));
  }
  if (core_) {
    chip(strf("ACK %s/%s ms", fmt_ms(snap.ack_ms_p50).c_str(), fmt_ms(snap.ack_ms_max).c_str()).c_str(), kGray);
    ImGui::SameLine(0, S(8));
    const double fps = core_->camera().fps();
    if (settings_.camera_on) {
      chip(fps > 0 ? strf("영상 %.0f fps", fps).c_str() : "영상 없음", fps > 0 ? kGray : kYellow);
      ImGui::SameLine(0, S(8));
    }
    chip(strf("속도 %.0f %%%s", snap.speed * 100, snap.precision ? " 저속" : "").c_str(), kBlue);
  }
  ImGui::PopFont();
  chip_limit_x_ = 0;

  // tabs on the right
  const char* names[] = {"조종 F1", "팔 F2", "진단 F3", "설정 F4"};
  ImGui::SetCursorPos(ImVec2(ImGui::GetWindowWidth() - tw * 4 - S(16), S(10)));
  for (int i = 0; i < 4; ++i) {
    if (i) ImGui::SameLine(0, S(4));
    const bool sel = static_cast<int>(tab_) == i && !show_connect_;
    ImGui::PushStyleColor(ImGuiCol_Button, col4(sel ? kAccent : kGray));
    if (ImGui::Button(names[i], ImVec2(tw, S(48)))) {
      tab_ = static_cast<Tab>(i);
      if (core_) show_connect_ = false;
      camera_big_ = false;
    }
    ImGui::PopStyleColor();
  }
  (void)now;
}

// ───────────────────────── camera + HUD ─────────────────────────

void GuiApp::draw_camera(ImVec2 size, double now) {
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const ImVec2 o = ImGui::GetCursorScreenPos();
  dl->AddRectFilled(o, ImVec2(o.x + size.x, o.y + size.y), IM_COL32(12, 13, 16, 255));
  const float aspect = (cam_w_ > 0 && cam_h_ > 0) ? static_cast<float>(cam_w_) / static_cast<float>(cam_h_) : 4.0f / 3.0f;
  const float avail_h = size.y - S(64);
  float iw = size.x - S(16), ih = iw / aspect;
  if (ih > avail_h) {
    ih = avail_h;
    iw = ih * aspect;
  }
  const ImVec2 p0(o.x + (size.x - iw) / 2, o.y + S(8));
  const ImVec2 p1(p0.x + iw, p0.y + ih);
  const bool fresh = cam_tex_ && now - cam_last_frame_ < 2.0;
  if (cam_tex_) {
    dl->AddImage(ImTextureRef(cam_tex_), p0, p1);
    if (!fresh) dl->AddRectFilled(p0, p1, IM_COL32(0, 0, 0, 140));
  } else {
    dl->AddRect(p0, p1, kGray, S(6), 0, S(2));
  }
  const std::string cam_state = !settings_.camera_on ? "영상 꺼짐 (X 버튼)" : (core_ ? core_->camera().status() : "연결 안 됨");
  if (!fresh) {
    const ImVec2 ts = ImGui::CalcTextSize(cam_state.c_str());
    dl->AddText(ImVec2((p0.x + p1.x - ts.x) / 2, (p0.y + p1.y - ts.y) / 2), kDim, cam_state.c_str());
  }
  if (settings_.show_crosshair) {
    const ImVec2 c((p0.x + p1.x) / 2, (p0.y + p1.y) / 2);
    dl->AddLine(ImVec2(c.x - S(18), c.y), ImVec2(c.x + S(18), c.y), IM_COL32(255, 255, 255, 120), S(2));
    dl->AddLine(ImVec2(c.x, c.y - S(18)), ImVec2(c.x, c.y + S(18)), IM_COL32(255, 255, 255, 120), S(2));
  }

  // HUD: state (top-left), arm target (top-right)
  if (core_) {
    const auto snap = core_->snapshot();
    const bool estop = snap.emergency;
    const char* label = estop ? "비상정지" : (snap.base.server_state == "active" || snap.arm.server_state == "active") ? "조종 중" : "중립";
    const ImU32 lc = estop ? kRed : (std::string(label) == "조종 중" ? kGreen : kDim);
    ImGui::PushFont(fonts_.bold, 30.0f);
    dl->AddRectFilled(ImVec2(p0.x + S(10), p0.y + S(10)), ImVec2(p0.x + S(10) + ImGui::CalcTextSize(label).x + S(24), p0.y + S(56)),
                      IM_COL32(0, 0, 0, 150), S(8));
    dl->AddText(ImVec2(p0.x + S(22), p0.y + S(14)), lc, label);
    ImGui::PopFont();
    const auto& t = snap.target;
    const std::string arm = strf("x %.3f  y %.3f  z %.3f m\npitch %.0f°  그리퍼 %.2f%s", t.x, t.y, t.z, deg(t.pitch), t.gripper,
                                 snap.arm_enabled ? "" : "  (팔 꺼짐)");
    const ImVec2 as = ImGui::CalcTextSize(arm.c_str());
    dl->AddRectFilled(ImVec2(p1.x - as.x - S(30), p0.y + S(10)), ImVec2(p1.x - S(10), p0.y + S(20) + as.y),
                      IM_COL32(0, 0, 0, 150), S(8));
    dl->AddText(ImVec2(p1.x - as.x - S(20), p0.y + S(15)), snap.limit_active ? kYellow : kWhite, arm.c_str());
    // drive vector
    const ImVec2 dc(p0.x + S(70), p1.y - S(70));
    dl->AddCircle(dc, S(50), IM_COL32(255, 255, 255, 70), 32, S(2));
    dl->AddLine(dc, ImVec2(dc.x - static_cast<float>(snap.applied_s) * S(50), dc.y - static_cast<float>(snap.applied_f) * S(50)),
                kAccent, S(4));
    if (std::fabs(snap.applied_t) > 1e-3)
      dl->PathArcTo(dc, S(40), -kPi / 2, -kPi / 2 - static_cast<float>(snap.applied_t) * 1.5f, 16);
    dl->PathStroke(kBlue, 0, S(3));
  }
  // notification strip
  if (!notes_.empty()) {
    const auto& n = notes_.back();
    ImGui::PushFont(fonts_.bold, 26.0f);
    const ImVec2 ts = ImGui::CalcTextSize(n.text.c_str());
    const ImVec2 a((p0.x + p1.x - ts.x) / 2 - S(20), p1.y - ts.y - S(30));
    dl->AddRectFilled(a, ImVec2(a.x + ts.x + S(40), a.y + ts.y + S(16)), IM_COL32(0, 0, 0, 190), S(10));
    dl->AddRect(a, ImVec2(a.x + ts.x + S(40), a.y + ts.y + S(16)), n.color, S(10), 0, S(2));
    dl->AddText(ImVec2(a.x + S(20), a.y + S(8)), n.color, n.text.c_str());
    ImGui::PopFont();
  }
  // buttons under the image
  ImGui::SetCursorScreenPos(ImVec2(o.x + S(12), o.y + size.y - S(52)));
  if (ImGui::Button(settings_.camera_on ? "영상 끄기 (X)" : "영상 켜기 (X)", ImVec2(S(200), S(44)))) settings_.camera_on = !settings_.camera_on;
  ImGui::SameLine();
  if (ImGui::Button(camera_big_ ? "작게 (Y)" : "크게 (Y)", ImVec2(S(150), S(44)))) camera_big_ = !camera_big_;
  if (core_ && !cam_state.empty() && settings_.camera_on) {
    ImGui::SameLine();
    ImGui::TextColored(col4(kDim), "%s", core_->camera().status().c_str());
  }
}

// ───────────────────────── side column (always visible) ─────────────────────────

void GuiApp::draw_side_column(ImVec2 size, double now) {
  const float w = size.x - S(20);
  ImGui::SetCursorPos(ImVec2(S(10), S(10)));
  ImGui::PushFont(fonts_.bold, 34.0f);
  ImGui::PushStyleColor(ImGuiCol_Button, col4(kRed));
  ImGui::PushStyleColor(ImGuiCol_ButtonHovered, col4(IM_COL32(235, 60, 60, 255)));
  ImGui::PushStyleColor(ImGuiCol_ButtonActive, col4(kRedDark));
  ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, S(16));
  if (ImGui::Button("비상\n정지", ImVec2(w, w))) {
    if (core_) core_->estop();
    haptics_.play(input::Haptic::kEstop, now);
    notify("비상정지", kRed, 4.0);
  }
  ImGui::PopStyleVar();
  ImGui::PopStyleColor(3);
  ImGui::PopFont();
  ImGui::SetCursorPosX(S(10));
  ImGui::TextColored(col4(kDim), "B · Space");

  ImGui::SetCursorPosX(S(10));
  if (hold_button("clear", "길게 눌러 해제", ImVec2(w, S(64)), 0.8, kYellow, &clear_progress_touch_) && core_) {
    core_->clear_estop();
    notify("비상정지 해제 요청 — 스틱을 놓았다가 다시", kYellow);
  }
  const float prog = std::max(clear_progress_touch_, clear_progress_pad_);
  if (prog > 0) {
    ImGui::SetCursorPosX(S(10));
    ImGui::ProgressBar(prog, ImVec2(w, S(10)), "");
  }
  ImGui::SetCursorPosX(S(10));
  ImGui::TextColored(col4(kDim), "View 0.8초");

  ImGui::Dummy(ImVec2(0, S(10)));
  ImGui::SetCursorPosX(S(10));
  ImGui::TextUnformatted("속도 상한");
  const int presets[] = {30, 60, 100};
  for (int i = 0; i < 3; ++i) {
    ImGui::SetCursorPosX(S(10));
    const bool sel = core_ && std::lround(core_->speed() * 100) == presets[i];
    ImGui::PushStyleColor(ImGuiCol_Button, col4(sel ? kBlue : kGray));
    if (ImGui::Button(strf("%d %%", presets[i]).c_str(), ImVec2(w, S(52))) && core_) {
      core_->set_speed(presets[i] / 100.0);
      settings_.speed = core_->speed();
    }
    ImGui::PopStyleColor();
  }
  ImGui::SetCursorPosX(S(10));
  ImGui::PushStyleColor(ImGuiCol_Button, col4(touch_.slow ? kAccent : kGray));
  if (ImGui::Button(touch_.slow ? "저속 켬" : "저속 (RB)", ImVec2(w, S(52)))) touch_.slow = !touch_.slow;
  ImGui::PopStyleColor();
}

// ───────────────────────── bottom bar (touch driving) ─────────────────────────

void GuiApp::draw_bottom_bar(ImVec2 size) {
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const ImVec2 o = ImGui::GetCursorScreenPos();
  dl->AddRectFilled(o, ImVec2(o.x + size.x, o.y + size.y), kPanel);
  const float r = std::min(S(110), size.y / 2 - S(12));
  ImGui::SetCursorPos(ImVec2(S(20), (size.y - 2 * r) / 2));
  virtual_stick("drive_stick", r, &touch_.stick_x, &touch_.stick_y);

  // turn slider
  ImGui::SameLine(0, S(30));
  const float sw = S(300), sh = S(80);
  ImGui::SetCursorPosY((size.y - sh) / 2);
  const ImVec2 sp = ImGui::GetCursorScreenPos();
  ImGui::InvisibleButton("turn", ImVec2(sw, sh));
  touch_.turn = 0;
  if (ImGui::IsItemActive()) touch_.turn = clampd((ImGui::GetIO().MousePos.x - (sp.x + sw / 2)) / (sw / 2), -1, 1);
  dl->AddRectFilled(ImVec2(sp.x, sp.y + sh / 2 - S(8)), ImVec2(sp.x + sw, sp.y + sh / 2 + S(8)), IM_COL32(50, 54, 63, 255), S(8));
  const float kx = sp.x + sw / 2 + touch_.turn * (sw / 2 - S(26));
  dl->AddCircleFilled(ImVec2(kx, sp.y + sh / 2), S(26), ImGui::IsItemActive() ? kAccent : IM_COL32(120, 126, 138, 255));
  dl->AddText(ImVec2(sp.x, sp.y - S(4)), kDim, "회전 ◀ ▶");

  // arm toggle + arm buttons
  ImGui::SameLine(0, S(30));
  const float bx = ImGui::GetCursorPosX();
  const auto snap = core_ ? core_->snapshot() : robot::CoreSnapshot{};
  ImGui::SetCursorPos(ImVec2(bx, S(14)));
  ImGui::PushStyleColor(ImGuiCol_Button, col4(touch_arm_toggle_ ? kAccent : kGray));
  if (ImGui::Button(touch_arm_toggle_ ? "팔 조종 켬\n(60초 무입력 시 꺼짐)" : "팔 조종 켜기\n(또는 LB 누른 채)", ImVec2(S(230), S(100))) && core_) {
    touch_arm_toggle_ = !touch_arm_toggle_;
    core_->set_arm_enabled(touch_arm_toggle_, true);
  }
  ImGui::PopStyleColor();
  ImGui::SetCursorPos(ImVec2(bx, S(124)));
  ImGui::TextColored(col4(snap.arm_enabled ? kGreen : kDim), snap.arm_enabled ? "팔: 켜짐" : "팔: 꺼짐");

  struct ArmBtn {
    const char* label;
    int axis;
    float dir;
  };
  const ArmBtn btns[] = {{"앞", 0, 1}, {"왼", 1, 1}, {"위", 2, 1}, {"손목↑", 3, 1},
                         {"뒤", 0, -1}, {"오른", 1, -1}, {"아래", 2, -1}, {"손목↓", 3, -1}};
  for (auto& a : touch_.arm) a = 0;
  const float abw = S(104), abh = S(92);
  const float ax0 = bx + S(250);
  for (int i = 0; i < 8; ++i) {
    ImGui::SetCursorPos(ImVec2(ax0 + (i % 4) * (abw + S(8)), S(16) + (i / 4) * (abh + S(10))));
    ImGui::Button(btns[i].label, ImVec2(abw, abh));
    if (ImGui::IsItemActive()) touch_.arm[btns[i].axis] = btns[i].dir;
  }
  // gripper
  const float gx = ax0 + 4 * (abw + S(8)) + S(16);
  ImGui::SetCursorPos(ImVec2(gx, S(16)));
  ImGui::PushStyleColor(ImGuiCol_Button, col4(IM_COL32(70, 110, 160, 255)));
  ImGui::Button("그리퍼\n닫기 (RT)", ImVec2(S(150), abh));
  if (ImGui::IsItemActive()) touch_.arm[4] = 1;
  ImGui::SetCursorPos(ImVec2(gx, S(16) + abh + S(10)));
  ImGui::Button("그리퍼\n열기 (LT)", ImVec2(S(150), abh));
  if (ImGui::IsItemActive()) touch_.arm[4] = -1;
  ImGui::PopStyleColor();

  // live gamepad mini view
  const float mx = gx + S(180);
  if (mx + S(260) < size.x) {
    const ImVec2 c1(o.x + mx + S(60), o.y + size.y / 2), c2(o.x + mx + S(190), o.y + size.y / 2);
    for (int s = 0; s < 2; ++s) {
      const ImVec2 c = s == 0 ? c1 : c2;
      const float px = s == 0 ? last_pad_.lx : last_pad_.rx, py = s == 0 ? last_pad_.ly : last_pad_.ry;
      dl->AddCircle(c, S(50), kGray, 32, S(2));
      dl->AddCircleFilled(ImVec2(c.x + px * S(40), c.y - py * S(40)), S(12), last_pad_.connected ? kAccent : kGray);
    }
    const bool lb = last_pad_.down(input::kLB), rb = last_pad_.down(input::kRB);
    dl->AddText(ImVec2(c1.x - S(40), c1.y - S(90)), lb ? kAccent : kDim, lb ? "LB 팔 모드" : "LB");
    dl->AddText(ImVec2(c2.x - S(20), c2.y - S(90)), rb ? kAccent : kDim, rb ? "RB 저속" : "RB");
    dl->AddRectFilled(ImVec2(c1.x - S(50), c1.y + S(64)), ImVec2(c1.x - S(50) + last_pad_.lt * S(100), c1.y + S(74)), kBlue);
    dl->AddRectFilled(ImVec2(c2.x - S(50), c2.y + S(64)), ImVec2(c2.x - S(50) + last_pad_.rt * S(100), c2.y + S(74)), kBlue);
    dl->AddText(ImVec2(c1.x - S(50), c1.y + S(78)), kDim, last_pad_.connected ? "LT" : "패드 없음");
    dl->AddText(ImVec2(c2.x - S(50), c2.y + S(78)), kDim, "RT");
  }
}

// ───────────────────────── tabs ─────────────────────────

void GuiApp::draw_tab_drive() {
  if (!core_) return;
  const auto snap = core_->snapshot();
  ImGui::PushFont(fonts_.bold, 26.0f);
  ImGui::TextUnformatted("조종 상태");
  ImGui::PopFont();
  if (ImGui::BeginTable("ch", 3, ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_RowBg)) {
    ImGui::TableSetupColumn("");
    ImGui::TableSetupColumn("차체");
    ImGui::TableSetupColumn("팔");
    ImGui::TableHeadersRow();
    auto row = [](const char* name, const std::string& a, const std::string& b) {
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::TextUnformatted(name);
      ImGui::TableSetColumnIndex(1);
      ImGui::TextUnformatted(a.c_str());
      ImGui::TableSetColumnIndex(2);
      ImGui::TextUnformatted(b.c_str());
    };
    row("상태", state_ko(snap.base.server_state), state_ko(snap.arm.server_state));
    row("마지막 응답", reason_ko(snap.base.server_reason), reason_ko(snap.arm.server_reason));
    row("무장", snap.base.armed ? "예" : "아니오", snap.arm.armed ? "예" : "아니오");
    row("자동 재무장", std::to_string(snap.base.auto_rearms), std::to_string(snap.arm.auto_rearms));
    row("마지막 차단", snap.base.last_block_reason, snap.arm.last_block_reason);
    ImGui::EndTable();
  }
  ImGui::Spacing();
  ImGui::Text("ACK 지연  마지막 %s · 중앙 %s · 최대 %s ms", fmt_ms(snap.ack_ms_last).c_str(), fmt_ms(snap.ack_ms_p50).c_str(),
              fmt_ms(snap.ack_ms_max).c_str());
  ImGui::Text("패킷  보냄 %llu · ACK %llu · 늦음 %llu · 오류 %llu", static_cast<unsigned long long>(snap.packets_sent),
              static_cast<unsigned long long>(snap.acks), static_cast<unsigned long long>(snap.late_acks),
              static_cast<unsigned long long>(snap.send_errors));
  ImGui::Text("조종 루프 %.1f Hz · 밀림 %llu", snap.loop_hz, static_cast<unsigned long long>(snap.loop_overruns));
  if (!snap.last_error.empty()) ImGui::TextColored(col4(kYellow), "최근 오류: %s", snap.last_error.c_str());
  ImGui::Text("보내는 값  앞 %+.2f · 옆 %+.2f · 회전 %+.2f", snap.applied_f, snap.applied_s, snap.applied_t);
  const auto st = core_->ros().status();
  if (core_->ros().status_fresh()) {
    ImGui::Text("로봇 차체 명령  vx %+.3f m/s · vy %+.3f m/s · wz %+.2f rad/s", st.base_vx, st.base_vy, st.base_wz);
    if (!st.estop_sources.empty()) {
      std::string s;
      for (const auto& e : st.estop_sources) s += e + " ";
      ImGui::TextColored(col4(kRed), "비상정지 출처: %s", s.c_str());
    }
    if (st.fault) ImGui::TextColored(col4(kRed), "고장: %s (로봇 재시작 필요)", st.fault_reason.c_str());
  } else {
    ImGui::TextColored(col4(kDim), "로봇 상태 미수신(rosbridge)");
  }
  ImGui::Spacing();
  float sp = static_cast<float>(core_->speed() * 100);
  ImGui::SetNextItemWidth(-1);
  if (ImGui::SliderFloat("##speed", &sp, 10, 100, "속도 상한 %.0f %%")) {
    core_->set_speed(sp / 100.0);
    settings_.speed = core_->speed();
  }
  ImGui::Spacing();
  if (ImGui::Button("연결 바꾸기", ImVec2(S(180), S(52)))) show_connect_ = true;
  ImGui::SameLine();
  if (snap.session_released) {
    if (ImGui::Button("새 세션", ImVec2(S(160), S(52)))) core_->new_session();
  } else {
    if (ImGui::Button("세션 놓기", ImVec2(S(160), S(52)))) core_->release_session();
  }
  ImGui::SameLine();
  if (ImGui::Button("연결 끊기", ImVec2(S(160), S(52)))) {
    disconnect();
    show_connect_ = true;
  }
  ImGui::TextColored(col4(kDim), "세션 놓기: 다른 조종기(휴대폰·VR)가 로봇을 쓸 수 있게 하고, 관절 직접 조종을 켭니다.");
}

// Top view: up = +x (forward), left = +y. Side view: right = +x, up = +z. Tap/drag sets a ramped goal.
void GuiApp::arm_plot(const char* id, ImVec2 size, bool top) {
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const ImVec2 p = ImGui::GetCursorScreenPos();
  ImGui::InvisibleButton(id, size);
  const bool active = ImGui::IsItemActive();
  dl->AddRectFilled(p, ImVec2(p.x + size.x, p.y + size.y), IM_COL32(20, 22, 27, 255), S(8));
  dl->PushClipRect(p, ImVec2(p.x + size.x, p.y + size.y), true);  // keep circles/markers inside the plot
  // world ranges (m)
  const double h_min = top ? -0.30 : 0.0, h_max = top ? 0.30 : 0.36;   // horizontal: y (top, flipped) or x
  const double v_min = top ? 0.0 : -0.12, v_max = top ? 0.36 : 0.36;   // vertical: x (top) or z
  const float m = S(14);
  const float sx = (size.x - 2 * m) / static_cast<float>(h_max - h_min);
  const float sy = (size.y - 2 * m) / static_cast<float>(v_max - v_min);
  const float sc = std::min(sx, sy);
  const float cx = p.x + size.x / 2, cy = p.y + size.y - m;
  // screen <-> world
  auto to_screen = [&](double h, double v) {
    const double hh = top ? -h : h - (h_min + h_max) / 2;  // top view: +y drawn to the left
    return ImVec2(cx + static_cast<float>(hh) * sc, cy - static_cast<float>(v - v_min) * sc);
  };
  auto to_world = [&](ImVec2 s, double* h, double* v) {
    const double hh = (s.x - cx) / sc;
    *h = top ? -hh : hh + (h_min + h_max) / 2;
    *v = (cy - s.y) / sc + v_min;
  };
  // workspace box
  const ImVec2 b0 = top ? to_screen(kWsYMax, kWsXMax) : to_screen(kWsXMin, kWsZMax);
  const ImVec2 b1 = top ? to_screen(kWsYMin, kWsXMin) : to_screen(kWsXMax, kWsZMin);
  dl->AddRect(ImVec2(std::min(b0.x, b1.x), std::min(b0.y, b1.y)), ImVec2(std::max(b0.x, b1.x), std::max(b0.y, b1.y)),
              IM_COL32(255, 140, 40, 110), 0, 0, S(2));
  // reach circle (shoulder reach, top view) / ground line (side view)
  if (top) {
    dl->AddCircle(to_screen(0, 0), static_cast<float>(0.36 * sc), IM_COL32(255, 255, 255, 30), 64, S(1));
  } else {
    const ImVec2 g0 = to_screen(0, 0), g1 = to_screen(h_max, 0);
    dl->AddLine(ImVec2(p.x + m, g0.y), ImVec2(g1.x, g1.y), IM_COL32(255, 255, 255, 50), S(1));
  }
  const ImVec2 base = top ? to_screen(0, 0) : to_screen(0, 0);
  dl->AddRectFilled(ImVec2(base.x - S(10), base.y - S(6)), ImVec2(base.x + S(10), base.y + S(6)), kGray, S(2));
  dl->AddText(ImVec2(p.x + S(8), p.y + S(6)), kDim, top ? "위에서 (x 앞 · y 왼쪽)" : "옆에서 (x 앞 · z 위)");

  if (core_) {
    const auto snap = core_->snapshot();
    auto dot = [&](double x, double y, double z, ImU32 c, float r, bool ring) {
      const ImVec2 s = top ? to_screen(y, x) : to_screen(x, z);
      if (ring) dl->AddCircle(s, r, c, 24, S(3));
      else dl->AddCircleFilled(s, r, c, 24);
    };
    const auto st = core_->ros().status();
    if (core_->ros().status_fresh() && st.arm_current)
      dot(st.arm_current->x, st.arm_current->y, st.arm_current->z, IM_COL32(170, 170, 170, 255), S(9), false);
    if (const auto ap = core_->ros().applied_arm_target()) dot(ap->x, ap->y, ap->z, kGreen, S(13), true);
    const auto& t = snap.target;
    dot(t.x, t.y, t.z, snap.limit_active ? kYellow : kBlue, S(8), false);
    if (!top) {  // pitch direction at the tip
      const ImVec2 s = to_screen(t.x, t.z);
      dl->AddLine(s, ImVec2(s.x + std::cos(static_cast<float>(t.pitch)) * S(30), s.y - std::sin(static_cast<float>(t.pitch)) * S(30)),
                  kBlue, S(3));
    }
    if (active) {
      if (!snap.arm_enabled) {
        if (ImGui::IsItemActivated()) notify("팔 조종을 먼저 켜세요(LB 또는 '팔 조종 켜기')", kYellow, 2.0);
      } else {
        double h, v;
        to_world(ImGui::GetIO().MousePos, &h, &v);
        if (top) core_->arm_goto_partial(clampd(v, kWsXMin, kWsXMax), clampd(h, kWsYMin, kWsYMax), std::nullopt, std::nullopt);
        else core_->arm_goto_partial(clampd(h, kWsXMin, kWsXMax), std::nullopt, clampd(v, kWsZMin, kWsZMax), std::nullopt);
        const ImVec2 mp = ImGui::GetIO().MousePos;
        dl->AddCircle(mp, S(18), kAccent, 24, S(3));
      }
    }
  }
  dl->PopClipRect();
}

void GuiApp::draw_tab_arm() {
  if (!core_) return;
  const auto snap = core_->snapshot();
  ImGui::PushFont(fonts_.bold, 26.0f);
  ImGui::TextUnformatted("팔");
  ImGui::PopFont();
  ImGui::SameLine();
  ImGui::TextColored(col4(snap.arm_enabled ? kGreen : kDim), snap.arm_enabled ? "조종 켜짐" : "조종 꺼짐 — LB를 누르거나 아래 '팔 조종 켜기'");
  const float w = ImGui::GetContentRegionAvail().x;
  const float pw = (w - S(10)) / 2;
  arm_plot("top", ImVec2(pw, S(300)), true);
  ImGui::SameLine(0, S(10));
  arm_plot("side", ImVec2(pw, S(300)), false);
  ImGui::PushStyleColor(ImGuiCol_Text, col4(kDim));
  ImGui::TextWrapped("파랑 = 내 목표, 초록 고리 = 로봇이 쓰는 목표, 회색 = 로봇 현재. 누르고 끌면 그 점으로 천천히 이동.");
  ImGui::PopStyleColor();

  ImGui::PushItemWidth(S(150));
  ImGui::InputFloat("x (m)", &num_x_, 0.01f, 0.05f, "%.3f");
  ImGui::SameLine();
  ImGui::InputFloat("y (m)", &num_y_, 0.01f, 0.05f, "%.3f");
  ImGui::InputFloat("z (m)", &num_z_, 0.01f, 0.05f, "%.3f");
  ImGui::SameLine();
  ImGui::InputFloat("pitch (°)", &num_pitch_deg_, 5.0f, 15.0f, "%.0f");
  ImGui::PopItemWidth();
  if (ImGui::Button("숫자 목표로 이동", ImVec2(S(220), S(52)))) {
    core_->arm_goto(robot::ArmTarget{num_x_, num_y_, num_z_, rad(num_pitch_deg_), snap.target.gripper});
    notify("팔: 숫자 목표로 이동(속도 제한)", kBlue, 2.0);
  }
  ImGui::SameLine();
  if (ImGui::Button("지금 목표 불러오기", ImVec2(S(220), S(52)))) {
    num_x_ = static_cast<float>(snap.target.x);
    num_y_ = static_cast<float>(snap.target.y);
    num_z_ = static_cast<float>(snap.target.z);
    num_pitch_deg_ = static_cast<float>(deg(snap.target.pitch));
  }
  ImGui::SameLine();
  if (ImGui::Button("기본 자세", ImVec2(S(150), S(52)))) {
    std::string err;
    if (!core_->arm_home(&err)) notify("기본 자세 실패: " + err, kRed);
  }
  num_grip_ = static_cast<float>(snap.target.gripper);
  ImGui::SetNextItemWidth(-1);
  if (ImGui::SliderFloat("##grip", &num_grip_, 0.0f, 1.0f, "그리퍼 %.2f (0 열림 · 1 닫힘)")) core_->set_gripper(num_grip_);

  const auto st = core_->ros().status();
  const auto ap = core_->ros().applied_arm_target();
  if (ImGui::BeginTable("armtbl", 6, ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_RowBg)) {
    ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, S(190));
    for (const char* h : {"x", "y", "z", "pitch°", "그리퍼"}) ImGui::TableSetupColumn(h);
    ImGui::TableHeadersRow();
    auto row = [](const char* name, const std::optional<robot::Pose5>& p) {
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::TextUnformatted(name);
      if (!p) return;
      const double v[5] = {p->x, p->y, p->z, deg(p->pitch), p->gripper};
      for (int i = 0; i < 5; ++i) {
        ImGui::TableSetColumnIndex(i + 1);
        ImGui::Text(i == 3 ? "%.0f" : "%.3f", v[i]);
      }
    };
    const auto& t = snap.target;
    row("내 목표", robot::Pose5{t.x, t.y, t.z, t.pitch, t.gripper});
    row("로봇이 쓰는 목표", ap);
    row(st.position_source == "measured" ? "로봇 현재(측정)" : "로봇 현재(명령값)",
        core_->ros().status_fresh() ? st.arm_current : std::nullopt);
    ImGui::EndTable();
  }
  const auto js = core_->ros().joints();
  if (!js.position.empty()) {
    ImGui::TextUnformatted("관절(°)");
    for (size_t i = 0; i < js.position.size() && i < js.names.size(); ++i) {
      const float d = static_cast<float>(deg(js.position[i]));
      ImGui::ProgressBar((d + 120.0f) / 240.0f, ImVec2(S(260), S(22)), strf("%s %.0f°", js.names[i].c_str(), d).c_str());
      if (i % 2 == 0) ImGui::SameLine();
    }
  }
}

void GuiApp::draw_tab_diag() {
  if (!core_) return;
  ImGui::PushFont(fonts_.bold, 26.0f);
  ImGui::TextUnformatted("진단");
  ImGui::PopFont();
  ImGui::SameLine();
  ImGui::TextColored(col4(kDim), "화면 %.0f fps · 이 앱이 주고받은 패킷만 보입니다", ImGui::GetIO().Framerate);

  bool keep_ros = comm_.keep_rosbridge();
  if (ImGui::Checkbox("rosbridge 메시지도 보기", &keep_ros)) comm_.set_keep_rosbridge(keep_ros);
  ImGui::SameLine();
  ImGui::Text("보냄 %llu · 받음 %llu", static_cast<unsigned long long>(comm_.tx()), static_cast<unsigned long long>(comm_.rx()));
  ImGui::BeginChild("pk", ImVec2(0, S(230)), ImGuiChildFlags_Borders);
  for (const auto& r : comm_.last(120)) {
    ImGui::TextColored(col4(r.outgoing ? kBlue : kGreen), "%s %s %s", r.time.c_str(), r.outgoing ? "→" : "←",
                       net::channel_name(r.channel));
    ImGui::SameLine();
    ImGui::TextUnformatted(printable(r.data, 160).c_str());
  }
  if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 5) ImGui::SetScrollHereY(1.0f);
  ImGui::EndChild();

  const auto st = core_->ros().status();
  if (ImGui::CollapsingHeader("로봇 상태(/armpi/status)", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (!core_->ros().status_fresh()) {
      ImGui::TextColored(col4(kYellow), "미수신: %s", core_->ros().last_error().c_str());
    } else {
      ImGui::Text("하드웨어 %s · 커널 %s · 위치 출처 %s · 팔 모드 %s", st.hardware.c_str(), st.kernel.c_str(),
                  st.position_source.c_str(), st.arm_mode.c_str());
      ImGui::Text("UDP 허용 %s · IK 막힘 %s · 마지막 팔 목표 결과 %s", st.udp_allowed ? "예" : "아니오",
                  st.arm_ik_blocked ? "예" : "아니오", st.arm_last_target_result.c_str());
      ImGui::Text("조종자 %s (%s)", st.owner.empty() ? "-" : st.owner.c_str(), st.owner_addr.empty() ? "-" : st.owner_addr.c_str());
      if (!st.warnings.empty()) ImGui::TextColored(col4(kYellow), "경고: %s", st.warnings.c_str());
      for (const auto& w : st.boot_warnings) ImGui::TextColored(col4(kYellow), "부팅 점검: %s", w.c_str());
      const auto bs = core_->ros().bridge_state();
      if (bs.valid) ImGui::Text("수신기: 차체 %s · 팔 %s · 목표 보정 누적 %llu", bs.base_state.c_str(), bs.arm_state.c_str(),
                                static_cast<unsigned long long>(bs.arm_adjust_total));
    }
  }

  if (ImGui::CollapsingHeader("관절 직접 조종")) {
    std::string why;
    const bool allowed = core_->joint_debug_allowed(&why);
    ImGui::TextColored(col4(allowed ? kGreen : kDim), "%s", allowed ? "사용 가능" : why.c_str());
    ImGui::PushItemWidth(S(300));
    for (int i = 0; i < 6; ++i) ImGui::SliderFloat(robot::kJointNames[static_cast<size_t>(i)], &joints_deg_[i], -120.0f, 120.0f, "%.0f°");
    ImGui::PopItemWidth();
    if (ImGui::Button("현재 관절값 불러오기", ImVec2(S(240), S(48)))) {
      const auto js = core_->ros().joints();
      for (size_t i = 0; i < js.position.size() && i < 6; ++i) joints_deg_[i] = static_cast<float>(deg(js.position[i]));
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(!allowed);
    if (ImGui::Button("관절 보내기", ImVec2(S(180), S(48)))) {
      std::array<double, 6> r{};
      for (int i = 0; i < 6; ++i) r[static_cast<size_t>(i)] = rad(joints_deg_[i]);
      std::string err;
      if (core_->joint_debug(r, &err)) notify("관절 목표 보냄", kBlue, 2.0);
      else notify("관절 보내기 실패: " + err, kRed);
    }
    ImGui::EndDisabled();
    ImGui::TextColored(col4(kDim), "UDP 조종 세션을 놓아야 쓸 수 있습니다(조종 탭 '세션 놓기').");
  }

  if (ImGui::CollapsingHeader("로봇 전체 정지·해제 (rosbridge)")) {
    ImGui::PushStyleColor(ImGuiCol_Button, col4(kRed));
    if (ImGui::Button("로봇 전체 정지", ImVec2(S(220), S(52)))) {
      std::string err;
      if (!core_->robot_estop(&err)) notify("전체 정지 요청 실패: " + err, kRed);
    }
    ImGui::PopStyleColor();
    ImGui::SameLine();
    float prog = 0;
    if (hold_button("reset", "길게: 전체 해제", ImVec2(S(220), S(52)), 0.8, kYellow, &prog)) confirm_reset_ = true;
    if (confirm_reset_) {
      ImGui::TextColored(col4(kYellow), "로봇의 모든 비상정지(버튼·서비스 포함)를 풉니다. 주변이 안전합니까?");
      if (ImGui::Button("확인 — 해제", ImVec2(S(200), S(48)))) {
        std::string err;
        if (!core_->robot_reset(&err)) notify("전체 해제 실패: " + err, kRed);
        confirm_reset_ = false;
      }
      ImGui::SameLine();
      if (ImGui::Button("취소", ImVec2(S(120), S(48)))) confirm_reset_ = false;
    }
  }

  if (ImGui::CollapsingHeader("사건 기록", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::BeginChild("ev", ImVec2(0, S(180)), ImGuiChildFlags_Borders);
    for (const auto& e : log_.last(60)) {
      const ImU32 c = e.level == Level::kError ? kRed : e.level == Level::kWarn ? kYellow : e.level == Level::kOk ? kGreen : kDim;
      ImGui::TextColored(col4(c), "%s %s", e.time.c_str(), e.text.c_str());
    }
    if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 5) ImGui::SetScrollHereY(1.0f);
    ImGui::EndChild();
    if (ImGui::Button("기록 폴더 열기", ImVec2(S(200), S(44))) && plat_.open_folder) plat_.open_folder(settings_dir());
  }
}

void GuiApp::draw_tab_settings() {
  ImGui::PushFont(fonts_.bold, 26.0f);
  ImGui::TextUnformatted("설정");
  ImGui::PopFont();
  ImGui::TextColored(col4(kDim), "설정 탭이 열려 있는 동안은 조종 입력이 모두 중립입니다(비상정지는 됨).");
  ImGui::PushItemWidth(S(200));
  ImGui::InputInt("제어 포트(UDP)", &settings_.control_port, 0);
  ImGui::InputInt("검색 포트(UDP)", &settings_.discovery_port, 0);
  ImGui::InputInt("rosbridge 포트(0=끔)", &settings_.rosbridge_port, 0);
  ImGui::InputInt("카메라 포트", &settings_.camera_port, 0);
  float ack = static_cast<float>(settings_.ack_timeout_s);
  if (ImGui::SliderFloat("ACK 허용 지연(s)", &ack, 0.15f, 1.0f, "%.2f")) settings_.ack_timeout_s = ack;
  ImGui::PopItemWidth();
  settings_.control_port = std::clamp(settings_.control_port, 1, 65535);
  settings_.discovery_port = std::clamp(settings_.discovery_port, 1, 65535);
  settings_.rosbridge_port = std::clamp(settings_.rosbridge_port, 0, 65535);
  settings_.camera_port = std::clamp(settings_.camera_port, 1, 65535);
  ImGui::TextColored(col4(kDim), "포트·지연은 다음 연결부터 적용됩니다.");
  ImGui::Separator();
  ImGui::PushItemWidth(S(260));
  ImGui::SliderFloat("스틱 데드존", &settings_.deadzone, 0.05f, 0.35f, "%.2f");
  ImGui::SliderFloat("스틱 곡선(1=직선)", &settings_.curve, 1.0f, 2.5f, "%.1f");
  ImGui::PopItemWidth();
  ImGui::Checkbox("LB 중에도 왼스틱으로 주행(고급)", &settings_.allow_drive_while_arm);
  ImGui::Separator();
  ImGui::Checkbox("진동", &settings_.haptics);
  ImGui::SameLine();
  ImGui::SetNextItemWidth(S(200));
  ImGui::SliderFloat("세기", &settings_.haptic_strength, 0.2f, 1.0f, "%.1f");
  ImGui::SameLine();
  if (ImGui::Button("진동 시험", ImVec2(S(140), S(44)))) haptics_.play(input::Haptic::kArmOn, mono_now());
  ImGui::Checkbox("영상 십자선", &settings_.show_crosshair);
  ImGui::SameLine();
  if (ImGui::Checkbox("전체 화면(F11)", &settings_.fullscreen) && plat_.set_fullscreen) plat_.set_fullscreen(settings_.fullscreen);
  ImGui::TextUnformatted("화면 주사율 상한");
  for (int f : {30, 60, 120}) {
    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button, col4(settings_.fps_cap == f ? kBlue : kGray));
    if (ImGui::Button(strf("%d", f).c_str(), ImVec2(S(80), S(44)))) settings_.fps_cap = f;
    ImGui::PopStyleColor();
  }
  ImGui::Separator();
  if (ImGui::Button("설정 저장", ImVec2(S(180), S(52)))) {
    save();
    notify("설정 저장됨", kGreen, 1.5);
  }
  ImGui::SameLine();
  if (ImGui::Button("앱 종료", ImVec2(S(160), S(52))) && plat_.quit) plat_.quit();
  ImGui::Spacing();
  ImGui::TextColored(col4(kDim), "%s %s · Dear ImGui %s · stb_image 2.30", kAppName, kVersion, IMGUI_VERSION);
  ImGui::TextColored(col4(kDim), "설정 파일: %s", settings_path().c_str());
}

void GuiApp::draw_connect_panel(ImVec2 size) {
  ImGui::SetCursorPos(ImVec2(S(40), S(30)));
  ImGui::BeginGroup();
  ImGui::PushFont(fonts_.bold, 40.0f);
  ImGui::TextUnformatted("로봇 연결");
  ImGui::PopFont();
  ImGui::TextColored(col4(kDim), "ArmPi Pro와 같은 와이파이에 있어야 합니다. 처음에는 바퀴를 띄우고 속도 10–30 %%로 시작하세요.");
  ImGui::Spacing();
  ImGui::PushFont(nullptr, 30.0f);
  ImGui::SetNextItemWidth(S(420));
  ImGui::InputTextWithHint("##ip", "로봇 IP 주소", ip_buf_, sizeof(ip_buf_), ImGuiInputTextFlags_CharsDecimal);
  ImGui::PopFont();
  ImGui::SameLine();
  ImGui::PushStyleColor(ImGuiCol_Button, col4(kGreen));
  if (ImGui::Button("연결", ImVec2(S(160), S(56)))) connect(ip_buf_);
  ImGui::PopStyleColor();
  ImGui::SameLine();
  ImGui::BeginDisabled(discovering_);
  if (ImGui::Button(discovering_ ? "찾는 중..." : "로봇 찾기", ImVec2(S(180), S(56)))) start_discovery();
  ImGui::EndDisabled();
  if (core_) {
    ImGui::SameLine();
    if (ImGui::Button("돌아가기", ImVec2(S(160), S(56)))) show_connect_ = false;
  }

  // touch keypad (the Windows touch keyboard does not appear for ImGui text fields)
  const char* keys[] = {"7", "8", "9", "4", "5", "6", "1", "2", "3", ".", "0", "⌫"};
  const float kw = S(130), kh = S(76);
  for (int i = 0; i < 12; ++i) {
    if (i % 3) ImGui::SameLine(0, S(8));
    if (ImGui::Button(keys[i], ImVec2(kw, kh))) {
      const size_t n = std::strlen(ip_buf_);
      if (i == 11) {
        if (n) ip_buf_[n - 1] = '\0';
      } else if (n + 1 < sizeof(ip_buf_)) {
        ip_buf_[n] = keys[i][0];
        ip_buf_[n + 1] = '\0';
      }
    }
  }
  if (ImGui::Button("지우기", ImVec2(kw * 3 + S(16), S(56)))) ip_buf_[0] = '\0';
  ImGui::EndGroup();

  ImGui::SameLine(0, S(60));
  ImGui::BeginGroup();
  ImGui::PushFont(fonts_.bold, 28.0f);
  ImGui::TextUnformatted("찾은 로봇");
  ImGui::PopFont();
  std::vector<net::RobotInfo> found;
  std::string derr;
  {
    std::lock_guard<std::mutex> lk(discover_mu_);
    found = discovered_;
    derr = discover_error_;
  }
  if (found.empty())
    ImGui::TextColored(col4(kDim), "%s", discovering_ ? "찾는 중..." : (derr.empty() ? "아직 없음 — '로봇 찾기'" : derr.c_str()));
  for (const auto& r : found) {
    // The name comes from the network: printable, short, and no "##" (ImGui ID separator).
    std::string name = printable(r.name, 48);
    for (size_t p = name.find("##"); p != std::string::npos; p = name.find("##")) name.replace(p, 2, "#");
    const std::string label = strf("%s\n%s · 포트 %d · ID %.8s · %.0f ms", name.c_str(), r.address.c_str(), r.control_port,
                                   r.device_id.c_str(), r.rtt_ms);
    ImGui::PushID((r.address + "|" + r.device_id).c_str());
    const bool pressed = ImGui::Button(label.c_str(), ImVec2(S(460), S(90)));
    ImGui::PopID();
    if (pressed) {
      settings_.control_port = r.control_port;
      std::snprintf(ip_buf_, sizeof(ip_buf_), "%s", r.address.c_str());
      connect(r.address);
    }
  }
  if (!settings_.robot_host.empty()) {
    ImGui::Spacing();
    ImGui::TextColored(col4(kDim), "최근 연결");
    if (ImGui::Button(settings_.robot_host.c_str(), ImVec2(S(460), S(64)))) connect(settings_.robot_host);
  }
  ImGui::Spacing();
  ImGui::TextColored(col4(kDim), "패드 %s · %s", last_pad_.connected ? "연결됨" : "없음", pad_.backend().c_str());
  ImGui::TextColored(col4(kDim), "Armoury Crate가 '게임패드' 모드여야 내장 패드가 잡힙니다.");
  ImGui::EndGroup();
  (void)size;
}

}  // namespace ally::ui
