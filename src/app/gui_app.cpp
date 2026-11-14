#include "app/gui_app.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "app/ui_icons.hpp"
#include "version.hpp"

namespace ally::ui {

namespace {

// Workspace box from the robot config (the robot clamps again; used here for drawing and plot limits).
constexpr double kWsXMin = 0.05, kWsXMax = 0.32, kWsYMin = -0.25, kWsYMax = 0.25, kWsZMin = -0.08, kWsZMax = 0.32;
constexpr float kPi = 3.14159265358979f;

Status channel_status(const std::string& s) {
  if (s == "active") return Status::kOk;
  if (s == "idle") return Status::kIdle;
  if (s == "emergency_stop" || s == "fault") return Status::kDanger;
  return Status::kWarn;  // awaiting_neutral, timeout, no_ack
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

const Fonts& F() { return ctx().fonts; }

// A full-size child window used as a layout region (no padding, no background).
bool begin_region(const char* id, ImVec2 p0, ImVec2 size, bool scroll = false, ImU32 bg = 0) {
  ImGui::SetCursorScreenPos(p0);
  ImGui::PushStyleColor(ImGuiCol_ChildBg, col::v4(bg));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 0.0f);
  return ImGui::BeginChild(id, size, ImGuiChildFlags_None,
                           scroll ? 0 : (ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse));
}

void end_region() {
  ImGui::EndChild();
  ImGui::PopStyleVar(3);
  ImGui::PopStyleColor();
}

// Label column for the 2-column settings form.
void form_label(const char* label, float label_w) {
  ImGui::AlignTextToFramePadding();
  ImGui::PushStyleColor(ImGuiCol_Text, col::v4(col::text2));
  ImGui::TextUnformatted(label);
  ImGui::PopStyleColor();
  ImGui::SameLine(label_w);
}

}  // namespace

GuiApp::GuiApp(Platform p, Settings s, Fonts fonts) : plat_(std::move(p)), settings_(std::move(s)) {
  ctx().fonts = fonts;
  std::snprintf(ip_buf_, sizeof(ip_buf_), "%s", settings_.robot_host.c_str());
  haptics_.set_enabled(settings_.haptics);
  haptics_.set_strength(settings_.haptic_strength);
  log_.info(std::string(kAppName) + " " + kVersion + " 시작. XInput: " + pad_.backend());
  std::string err;
  prune_logs(settings_dir(), 20);
  if (log_.open_file(settings_dir() + "/ally_" + wall_stamp() + ".log", &err)) log_.info("기록 파일: " + settings_dir());
  if (!fonts.embedded) log_.warn("내장 글꼴을 읽지 못해 시스템 글꼴(맑은 고딕)을 씀");
  // autograsp: optional colour dictionary and camera calibration from the settings folder
  std::string text;
  std::string cerr;
  if (read_text_file(settings_dir() + "/vision_colors.json", &text, &cerr)) {
    std::vector<vision::ObjectClass> cls;
    if (vision::classes_from_json(text, &cls, &cerr)) {
      vision_.detector().set_classes(std::move(cls));
      log_.info("물체 색 사전: vision_colors.json");
    } else {
      log_.warn(cerr);
    }
  }
  vision::VisionCalib calib;
  if (vision::load_calib(vision::calib_path(), &calib, &cerr)) calib_ = calib;
  vision_.set_max_hz(10.0);
  vision_.start();
}

GuiApp::~GuiApp() { shutdown(); }

void GuiApp::shutdown() {
  if (discover_thread_.joinable()) discover_thread_.join();
  disconnect();
  vision_.stop();
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
  else if (tab == "grasp") tab_ = Tab::kGrasp;
  else if (tab == "diag") tab_ = Tab::kDiag;
  else if (tab == "settings") tab_ = Tab::kSettings;
}

void GuiApp::save() {
  if (!demo_host_.empty()) return;  // demo/test runs never touch the user's settings file
  std::string err;
  if (!save_settings(settings_path(), settings_, &err)) log_.warn(err);
}

std::vector<w::Toast> GuiApp::shown_toasts() const {
  if (!privacy_) return toasts_;
  std::vector<w::Toast> t = toasts_;
  for (auto& x : t) x.text = mask_addresses(x.text);
  return t;
}

void GuiApp::notify(const std::string& text, Status kind, double secs) {
  const double now = mono_now();
  toasts_.push_back(w::Toast{utf8_truncate(text, 160), kind, now, now + secs});
  if (toasts_.size() > 6) toasts_.erase(toasts_.begin());
}

void GuiApp::trigger_estop(double now) {
  if (core_) core_->estop();
  haptics_.play(input::Haptic::kEstop, now);
  estop_flash_until_ = now + 0.3;
  notify("비상정지", Status::kDanger, 4.0);
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
    notify("포커스 복귀 — 스틱을 놓았다가 다시 조작하세요", Status::kWarn, 2.5);
  }
}

// ───────────────────────── connection ─────────────────────────

void GuiApp::connect(const std::string& host_in) {
  const std::string host = trim(host_in);
  if (host.empty()) {
    notify("로봇 주소를 입력하세요", Status::kWarn);
    return;
  }
  bool ok_chars = host.size() <= 253;
  for (char c : host)
    ok_chars = ok_chars && ((c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '.' || c == '-');
  if (!ok_chars) {
    notify("로봇 주소 형식 오류(숫자·영문·점·하이픈만)", Status::kDanger);
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
    notify("연결 실패: " + err, Status::kDanger, 5.0);
    log_.error("연결 실패: " + err);
    return;
  }
  core_ = std::move(core);
  start_grasp();
  cues_init_ = false;
  cam_seq_ = 0;
  settings_.robot_host = host;
  save();
  show_connect_ = false;
  notify(demo_host_.empty() ? "연결: " + host : std::string("연결: 데모 로봇"), Status::kOk);
}

void GuiApp::disconnect() {
  if (!core_) return;
  stop_grasp();  // holds a reference to core_
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
    notify("게임패드 연결 끊김 — 중립", Status::kDanger, 4.0);
    log_.warn("게임패드 분리");
  } else if (!pad_was_connected_ && pad.connected && now > 2.0) {
    notify("게임패드 연결됨", Status::kOk, 2.0);
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
    if (ImGui::IsKeyPressed(ImGuiKey_F5, false)) tab_ = Tab::kGrasp;
    if (ImGui::IsKeyPressed(ImGuiKey_F12, false) && plat_.capture) plat_.capture(1, 0.0);
    if (ImGui::IsKeyPressed(ImGuiKey_F11, false) && plat_.set_fullscreen) {
      settings_.fullscreen = !settings_.fullscreen;
      plat_.set_fullscreen(settings_.fullscreen);
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
      if (tab_ == Tab::kSettings) tab_ = Tab::kDrive;
      else if (show_connect_ && core_) show_connect_ = false;
    }
  }

  auto r = mapper_.map(pad, k, touch_, now, focused_);
  // Tilt driving: only while RT is held (and not the arm dead-man), relative to the pose at the moment of the press.
  tilt_out_ = input::TiltDrive{};
  // RT must be seen released (focused, pad present) before a press counts: no tilt driving straight after a focus
  // return or a pad reconnect with RT still held.
  if (!focused_ || !pad.connected) tilt_armed_ = false;
  else if (pad.rt < 0.1f) tilt_armed_ = true;
  const bool tilt_want = settings_.tilt_drive && focused_ && pad.connected && tilt_armed_ && pad.rt > 0.5f && !r.arm_hold;
  if (settings_.tilt_drive && !tilt_tried_) {
    tilt_tried_ = true;
    tilt_.start();  // background thread; never blocks this frame
  }
  if (tilt_.state() < 0 && !tilt_warned_ && settings_.tilt_drive) {
    tilt_warned_ = true;
    notify("기울기 주행: 가속도계 없음", Status::kWarn, 3.0);
  }
  if (tilt_want) {
    if (!tilt_on_) {
      tilt_on_ = true;
      tilt_ref_set_ = false;
      tilt_press_t_ = now;
    }
    input::Gravity g;
    double stamp = 0;
    if (tilt_.read(&g, &stamp)) {
      const double p = input::tilt_pitch_deg(g), rl = input::tilt_roll_deg(g);
      if (!tilt_ref_set_ && stamp > tilt_press_t_) {  // reference = the first answer after the press, not an old cache
        tilt_ref_set_ = true;
        tilt_ref_pitch_ = p;
        tilt_ref_roll_ = rl;
      }
      if (tilt_ref_set_) {
        tilt_out_ = input::tilt_to_drive(p, rl, tilt_ref_pitch_, tilt_ref_roll_, 5.0,
                                         std::clamp(static_cast<double>(settings_.tilt_full_deg), 12.0, 45.0));
        r.input.forward = std::clamp(r.input.forward + tilt_out_.forward, -1.0, 1.0);
        r.input.turn = std::clamp(r.input.turn + tilt_out_.turn, -1.0, 1.0);
      }
    }
  } else {
    tilt_on_ = false;
  }
  user_active_ = std::fabs(r.input.forward) > 1e-3 || std::fabs(r.input.strafe) > 1e-3 || std::fabs(r.input.turn) > 1e-3 ||
                 r.arm_hold;
  for (double v : r.input.arm) user_active_ = user_active_ || std::fabs(v) > 1e-3;
  const uint16_t pressed = static_cast<uint16_t>(pad.buttons & ~prev_buttons_);
  prev_buttons_ = pad.buttons;
  // Grasp tab: the D-pad picks candidates / fine-tunes the grasp point instead of speed and arm steps.
  const bool grasp_tab = tab_ == Tab::kGrasp && !show_connect_;
  if (grasp_tab && focused_) grasp_pad(pressed);
  clear_progress_pad_ = r.ev.clear_progress;
  if (r.ev.estop) trigger_estop(now);
  if (!core_) {
    prev_arm_hold_ = r.arm_hold;
    if (r.ev.menu) tab_ = tab_ == Tab::kSettings ? Tab::kDrive : Tab::kSettings;
    return;
  }
  if (r.ev.clear_estop) {
    core_->clear_estop();
    notify("비상정지 해제 요청 — 스틱을 놓았다가 다시", Status::kWarn);
  }
  const int speed_delta = grasp_tab ? 0 : r.ev.speed_delta;
  if (speed_delta != 0) core_->set_speed(core_->speed() + 0.1 * speed_delta);
  if (r.ev.speed_set_percent > 0) core_->set_speed(r.ev.speed_set_percent / 100.0);
  if (speed_delta != 0 || r.ev.speed_set_percent > 0) {
    settings_.speed = core_->speed();
    notify(strf("속도 상한 %.0f %%", settings_.speed * 100), Status::kIdle, 1.2);
  }
  if (r.ev.camera_toggle) settings_.camera_on = !settings_.camera_on;
  if (r.ev.camera_big) camera_big_ = !camera_big_;
  if (r.ev.arm_sync) {
    core_->arm_sync();
    notify("팔 목표를 로봇 현재 자세로 맞춤", Status::kIdle);
  }
  if (r.ev.menu) tab_ = tab_ == Tab::kSettings ? Tab::kDrive : Tab::kSettings;
  if (!grasp_tab && (r.ev.z_step != 0 || r.ev.pitch_step != 0)) core_->arm_nudge(0, 0, r.ev.z_step, r.ev.pitch_step);

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
    estop_flash_until_ = now + 0.3;
    notify("로봇 비상정지 상태", Status::kDanger, 4.0);
  }
  if (c.limit > last_cues_.limit) {
    haptics_.play(input::Haptic::kLimit, now);
    notify("팔: 닿는 범위의 끝", Status::kWarn, 1.5);
  }
  if (c.rejected > last_cues_.rejected) {
    haptics_.play(input::Haptic::kReject, now);
    notify("로봇이 팔 목표를 거부함 → 마지막 수락값으로", Status::kWarn);
  }
  if (c.link_lost > last_cues_.link_lost) {
    haptics_.play(input::Haptic::kLinkLost, now);
    notify("통신 끊김 — 다시 붙는 중", Status::kWarn, 3.0);
  }
  if (c.link_ok > last_cues_.link_ok && last_cues_.link_ok > 0) haptics_.play(input::Haptic::kLinkOk, now);
  if (c.arm_on > last_cues_.arm_on) haptics_.play(input::Haptic::kArmOn, now);
  last_cues_ = c;

  if (const auto res = core_->ros().last_result(); res && res->mono > last_service_seen_) {
    last_service_seen_ = res->mono;
    haptics_.play(res->ok ? input::Haptic::kServiceOk : input::Haptic::kServiceFail, now);
    notify((res->ok ? "완료: " : "실패: ") + res->service + (res->message.empty() ? "" : " — " + res->message),
           res->ok ? Status::kOk : Status::kDanger, 3.5);
  }
}

// ───────────────────────── frame ─────────────────────────

void GuiApp::frame(double now) {
  ImGuiIO& io = ImGui::GetIO();
  UiContext& c = ctx();
  c.scale = std::max(0.6f, io.DisplaySize.y / 1080.0f);
  c.text_scale = c.scale * std::clamp(settings_.font_scale, 1.0f, 1.3f);
  c.now = now;
  c.dt = std::max(1e-4f, io.DeltaTime);
  ImGui::GetStyle().FontScaleMain = c.text_scale;
  c.touches = plat_.touches ? plat_.touches() : std::vector<TouchPoint>{};
  w::touch_new_frame();

  if (demo_pending_) {
    demo_pending_ = false;
    connect(demo_host_);
  }
  if (!demo_state_.empty() && !demo_state_done_ && core_) {
    if (demo_state_ == "cambig") {
      camera_big_ = true;
      demo_state_done_ = true;
    } else if (demo_state_ == "estop" && core_->snapshot().connected) {
      trigger_estop(now);
      demo_state_done_ = true;
    }
  }
  handle_input(now);
  handle_cues(now);
  grasp_frame(now);
  grasp_hold_touch_ = false;  // the hold button sets it again while drawn and held

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
      vision_.submit(f);  // shared frame, no copy; ignored unless the grasp tab is open
    }
  }
  toasts_.erase(std::remove_if(toasts_.begin(), toasts_.end(), [&](const w::Toast& t) { return t.until < now; }), toasts_.end());

  ImGui::SetNextWindowPos(ImVec2(0, 0));
  ImGui::SetNextWindowSize(io.DisplaySize);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  ImGui::PushStyleColor(ImGuiCol_WindowBg, col::v4(col::bg1));
  ImGui::Begin("root", nullptr,
               ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBringToFrontOnFocus |
                   ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoScrollWithMouse);
  ImGui::PopStyleColor();
  ImGui::PopStyleVar();

  // Layout grid (1080p): top 56, bottom 304, gutter 16; camera | tab panel 508 | safety column 208.
  const float W = io.DisplaySize.x, H = io.DisplaySize.y;
  const float g = S(sp::gutter), top_h = S(56), bottom_h = S(304);
  const float safety_w = S(208), panel_w = S(508);
  const float body_y = top_h + g, body_h = H - top_h - bottom_h - 2 * g;
  const float safety_x = W - g - safety_w;
  const bool connect_screen = show_connect_ || !core_;
  const bool big = camera_big_ && !connect_screen;
  const float panel_x = safety_x - g - panel_w;
  const float cam_w = (big ? safety_x - g : panel_x - g) - g;

  draw_top_bar(ImVec2(0, 0), ImVec2(W, top_h));
  if (connect_screen) {
    draw_connect_left(ImVec2(g, body_y), ImVec2(panel_x - 2 * g, body_h));
    // Settings work without a robot (ports, input, tilt driving...); every other tab needs a connection.
    if (!core_ && tab_ == Tab::kSettings) draw_tab_panel(ImVec2(panel_x, body_y), ImVec2(panel_w, body_h));
    else draw_connect_right(ImVec2(panel_x, body_y), ImVec2(panel_w, body_h));
    w::toast_stack(ImGui::GetWindowDrawList(), ImVec2((g + panel_x - g) * 0.5f, body_y + S(sp::s5) + S(48)), S(720), shown_toasts(), now);
  } else {
    draw_camera(ImVec2(g, body_y), ImVec2(cam_w, body_h));
    if (!big) draw_tab_panel(ImVec2(panel_x, body_y), ImVec2(panel_w, body_h));
  }
  draw_safety_column(ImVec2(safety_x, body_y), ImVec2(safety_w, body_h));
  draw_bottom_bar(ImVec2(0, H - bottom_h), ImVec2(W, bottom_h));

  ImGui::End();
}

// ───────────────────────── top status bar ─────────────────────────

void GuiApp::draw_top_bar(ImVec2 p0, ImVec2 size) {
  ImDrawList* dl = ImGui::GetWindowDrawList();
  dl->AddRectFilled(p0, ImVec2(p0.x + size.x, p0.y + size.y), col::bg1);
  dl->AddLine(ImVec2(p0.x, p0.y + size.y - 1), ImVec2(p0.x + size.x, p0.y + size.y - 1), col::border1, S(1));

  const robot::CoreSnapshot snap = core_ ? core_->snapshot() : robot::CoreSnapshot{};
  const bool ros_ok = core_ && core_->ros().connected();
  const auto st = core_ ? core_->ros().status() : robot::RobotStatus{};
  const bool st_fresh = core_ && core_->ros().status_fresh();
  const bool estop = snap.emergency || (st_fresh && st.emergency_stop);

  struct Item {
    std::string label;
    Status s;
    std::string value;
  };
  std::vector<Item> left;
  if (estop) left.push_back({"비상정지", Status::kDanger, ""});
  if (!core_) {
    left.push_back({"연결 안 됨", Status::kOff, ""});
  } else {
    const std::string name = !demo_host_.empty() ? std::string("데모 로봇")
                             : privacy_          ? std::string("로봇")
                                                 : snap.robot.substr(0, snap.robot.find(':'));
    left.push_back({snap.connected ? name : "응답 없음", snap.connected ? Status::kOk : Status::kWarn, ""});
    left.push_back({"차체", channel_status(snap.base.server_state), state_ko(snap.base.server_state)});
    left.push_back({"팔", channel_status(snap.arm.server_state),
                    std::string(state_ko(snap.arm.server_state)) + (snap.arm_enabled ? " · 켬" : "")});
    if (st_fresh) {
      if (st.owner.empty()) left.push_back({"조종권", Status::kIdle, "없음"});
      else if (st.owner == snap.client_id) left.push_back({"조종권", Status::kOk, "나"});
      else left.push_back({"다른 조종기", Status::kDanger, ""});
    }
    if (snap.faulted) left.push_back({"고장 — 새 세션 필요", Status::kDanger, ""});
  }
  // right-side gauges: icon + mono value
  struct Gauge {
    const char* icon;
    std::string value;
    Status s;
  };
  std::vector<Gauge> right;
  if (st_fresh && st.battery_v)
    right.push_back({st.battery_low ? icon::kBatteryLow : icon::kBattery, strf("%.2f V", *st.battery_v), st.battery_low ? Status::kWarn : Status::kOk});
  if (ally_batt_ >= 0)
    right.push_back({ally_ac_ ? icon::kBatteryCharging : icon::kBattery, strf("Ally %d%%", ally_batt_),
                     ally_batt_ <= 10 ? Status::kDanger : ally_batt_ <= 20 ? Status::kWarn : Status::kOk});
  if (core_) {
    const bool slow = !snap.ack_ms_p50 || *snap.ack_ms_p50 > 100.0;
    right.push_back({icon::kActivity, snap.ack_ms_p50 ? strf("%s ms", fmt_ms(snap.ack_ms_p50).c_str()) : std::string("- ms"),
                     snap.connected ? (slow ? Status::kWarn : Status::kOk) : Status::kDanger});
    if (settings_.camera_on) {
      const double fps = core_->camera().fps();
      right.push_back({icon::kCamera, fps > 0 ? strf("%.0f fps", fps) : std::string("없음"), fps > 0 ? Status::kOk : Status::kWarn});
    } else {
      right.push_back({icon::kCameraOff, "꺼짐", Status::kOff});
    }
    right.push_back({icon::kRadio, ros_ok ? "ROS" : "ROS 없음", ros_ok ? Status::kOk : Status::kWarn});
  }
  right.push_back({icon::kGamepad2, last_pad_.connected ? "패드" : "패드 없음",
                   last_pad_.connected ? Status::kOk : (pad_.available() ? Status::kDanger : Status::kOff)});

  // mode segment (right edge)
  const char* tabs[] = {"주행", "팔", "집기", "진단", "설정"};
  const char* tab_icons[] = {icon::kGauge, icon::kBot, icon::kBox, icon::kActivity, icon::kSettings};
  const ImVec2 seg_item(S(100), S(40));
  const ImVec2 quit_sz(S(48), S(40));
  const float quit_x = p0.x + size.x - S(sp::gutter) - quit_sz.x;
  const float seg_x = quit_x - S(sp::s3) - seg_item.x * 5;
  const float cy = p0.y + size.y * 0.5f;

  // left group
  float x = p0.x + S(sp::gutter);
  const float h = S(32);
  for (const auto& it : left) {
    const float wdt = w::status_item_width(it.label.c_str(), it.value.c_str());
    w::status_item(dl, ImVec2(x, cy - h * 0.5f), h, it.label.c_str(), it.s, it.value.c_str());
    x += wdt + S(sp::s5);
  }
  // right group (right-aligned against the mode segment, compact = icon + value only)
  float rx = seg_x - S(sp::s5);
  for (auto it = right.rbegin(); it != right.rend(); ++it) {
    const ImVec2 vs = w::text_size(F().mono, fs::label, it->value.c_str());
    const ImVec2 is = w::text_size(F().regular, fs::label, it->icon);
    const float wdt = is.x + S(sp::s2) + vs.x;
    if (rx - wdt < x + S(sp::s5)) break;  // never overlap the left group
    rx -= wdt;
    const ImU32 ic = it->s == Status::kOk ? col::text2 : status_color(it->s);
    w::text(dl, ImVec2(rx, cy - is.y * 0.5f), F().regular, fs::label, ic, it->icon);
    const ImU32 vc = it->s == Status::kWarn ? col::warn : it->s == Status::kDanger ? col::danger : it->s == Status::kOff ? col::disabled : col::text1;
    w::text(dl, ImVec2(rx + is.x + S(sp::s2), cy - vs.y * 0.5f), F().mono, fs::label, vc, it->value.c_str());
    rx -= S(sp::s5);
  }
  ImGui::SetCursorScreenPos(ImVec2(seg_x, cy - seg_item.y * 0.5f));
  const int current = core_ ? (show_connect_ ? -1 : static_cast<int>(tab_)) : (tab_ == Tab::kSettings ? static_cast<int>(Tab::kSettings) : -1);
  const int clicked = w::segmented("modes", tabs, tab_icons, 5, current, seg_item, false);
  if (clicked >= 0) {
    const Tab t = static_cast<Tab>(clicked);
    if (!core_ && t != Tab::kSettings) {
      tab_ = Tab::kDrive;  // back to the connect screen
      notify(std::string(tabs[clicked]) + " 탭은 로봇에 연결하면 쓸 수 있습니다 — 로봇 찾기 또는 주소 입력", Status::kWarn, 3.0);
    } else {
      tab_ = t;
      if (core_) show_connect_ = false;
      camera_big_ = false;
    }
  }
  // Quit: two taps within 3 s (a single stray tap never closes the app). Quitting sends neutral + disconnect first.
  const double now = ctx().now;
  const bool armed = now < quit_armed_until_;
  ImGui::SetCursorScreenPos(ImVec2(quit_x, cy - quit_sz.y * 0.5f));
  if (w::button("quit_top", "", quit_sz, armed ? w::Variant::kDanger : w::Variant::kGhost, icon::kPower)) {
    if (armed) {
      log_.info("상단 전원 버튼으로 종료");
      if (plat_.quit) plat_.quit();
    } else {
      quit_armed_until_ = now + 3.0;
      notify("한 번 더 누르면 앱을 끕니다(로봇은 중립으로 두고 연결을 끊음)", Status::kWarn, 3.0);
    }
  }
}

// ───────────────────────── camera + HUD ─────────────────────────

void GuiApp::draw_camera(ImVec2 o, ImVec2 size) {
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const ImVec2 o1(o.x + size.x, o.y + size.y);
  dl->AddRectFilled(o, o1, col::bg0, S(sp::radiusL));
  dl->PushClipRect(o, o1, true);
  const float aspect = (cam_w_ > 0 && cam_h_ > 0) ? static_cast<float>(cam_w_) / static_cast<float>(cam_h_) : 4.0f / 3.0f;
  float iw = size.x, ih = iw / aspect;
  if (ih > size.y) {
    ih = size.y;
    iw = ih * aspect;
  }
  const ImVec2 p0(o.x + (size.x - iw) * 0.5f, o.y + (size.y - ih) * 0.5f);
  const ImVec2 p1(p0.x + iw, p0.y + ih);
  const double now = ctx().now;
  const bool fresh = cam_tex_ && now - cam_last_frame_ < 2.0;
  if (cam_tex_) {
    dl->AddImage(ImTextureRef(cam_tex_), p0, p1);
    if (!fresh) dl->AddRectFilled(p0, p1, col::alpha(col::black, 0.55f));
  }
  if (!fresh) {
    const std::string cam_state = !settings_.camera_on ? "영상 꺼짐" : (core_ ? core_->camera().status() : "연결 안 됨");
    w::text_center(dl, ImVec2((o.x + o1.x) * 0.5f, (o.y + o1.y) * 0.5f - S(20)), F().regular, fs::title, col::text3,
                   settings_.camera_on ? icon::kCamera : icon::kCameraOff);
    w::text_center(dl, ImVec2((o.x + o1.x) * 0.5f, (o.y + o1.y) * 0.5f + S(24)), F().regular, fs::body, col::text2, cam_state.c_str());
  }
  // edge scrims keep overlay text readable on bright images
  const float sh = S(96);
  dl->AddRectFilledMultiColor(o, ImVec2(o1.x, o.y + sh), col::alpha(col::black, 0.35f), col::alpha(col::black, 0.35f),
                              col::alpha(col::black, 0), col::alpha(col::black, 0));
  dl->AddRectFilledMultiColor(ImVec2(o.x, o1.y - sh), o1, col::alpha(col::black, 0), col::alpha(col::black, 0),
                              col::alpha(col::black, 0.35f), col::alpha(col::black, 0.35f));
  if (settings_.show_crosshair) {
    const ImVec2 c((p0.x + p1.x) * 0.5f, (p0.y + p1.y) * 0.5f);
    dl->AddLine(ImVec2(c.x - S(18), c.y), ImVec2(c.x - S(6), c.y), col::alpha(col::white, 0.55f), S(2));
    dl->AddLine(ImVec2(c.x + S(6), c.y), ImVec2(c.x + S(18), c.y), col::alpha(col::white, 0.55f), S(2));
    dl->AddLine(ImVec2(c.x, c.y - S(18)), ImVec2(c.x, c.y - S(6)), col::alpha(col::white, 0.55f), S(2));
    dl->AddLine(ImVec2(c.x, c.y + S(6)), ImVec2(c.x, c.y + S(18)), col::alpha(col::white, 0.55f), S(2));
  }
  const float m = S(sp::s6);
  const bool grasp_view = core_ && grasp_ && tab_ == Tab::kGrasp && !show_connect_;
  if (grasp_view && fresh) draw_grasp_overlay(dl, p0, p1);
  if (core_) {
    const auto snap = core_->snapshot();
    // top-left: one state word (e-stop first, then automation, then driving)
    const bool estop = snap.emergency;
    const bool driving = snap.base.server_state == "active" || snap.arm.server_state == "active";
    const bool auto_grasp = grasp_ && grasp_->active();
    const std::string word_s = estop        ? std::string("비상정지")
                               : auto_grasp ? std::string("자동 집기 · ") + robot::to_string_ko(grasp_->status().state)
                               : driving    ? std::string("조종 중")
                                            : std::string("중립");
    const char* word = word_s.c_str();
    if (estop) {
      const ImVec2 ws = w::text_size(F().bold, fs::title, word);
      dl->AddRectFilled(ImVec2(o.x + m - S(12), o.y + m - S(6)), ImVec2(o.x + m + ws.x + S(12), o.y + m + ws.y + S(6)), col::stop, S(sp::radiusS));
      w::text(dl, ImVec2(o.x + m, o.y + m), F().bold, fs::title, col::white, word);
    } else {
      w::osd_text(dl, ImVec2(o.x + m, o.y + m), F().bold, fs::title, auto_grasp ? col::accent : driving ? col::ok : col::text1, word);
    }
    // top-right: arm target while the arm is on
    if (snap.arm_enabled) {
      const auto& t = snap.target;
      const std::string l1 = strf("x %.3f  y %.3f  z %.3f", t.x, t.y, t.z);
      const std::string l2 = strf("pitch %+.0f°  그리퍼 %.2f", deg(t.pitch), t.gripper);
      const ImU32 c = snap.limit_active ? col::warn : col::text1;
      const ImVec2 s1 = w::text_size(F().mono, fs::label, l1.c_str());
      const ImVec2 s2 = w::text_size(F().mono, fs::label, l2.c_str());
      w::osd_text(dl, ImVec2(o1.x - m - s1.x, o.y + m), F().mono, fs::label, c, l1.c_str());
      w::osd_text(dl, ImVec2(o1.x - m - s2.x, o.y + m + s1.y + S(4)), F().mono, fs::label, c, l2.c_str());
    }
    // bottom-left: drive vector (+ tilt driving tag)
    const ImVec2 dc(o.x + m + S(44), o1.y - m - S(44));
    if (tilt_on_) w::osd_text(dl, ImVec2(o.x + m, o1.y - m - S(88) - S(30)), F().semibold, fs::label, col::accent, "기울기 주행 · RT");
    dl->AddCircle(dc, S(44), col::alpha(col::white, 0.7f), 40, S(2));
    dl->AddLine(dc, ImVec2(dc.x - static_cast<float>(snap.applied_s) * S(44), dc.y - static_cast<float>(snap.applied_f) * S(44)), col::accent, S(4));
    if (std::fabs(snap.applied_t) > 1e-3) {
      dl->PathArcTo(dc, S(36), -kPi / 2, -kPi / 2 - static_cast<float>(snap.applied_t) * 1.5f, 16);
      dl->PathStroke(col::info, 0, S(3));
    }
    // bottom-right: speed limit
    const std::string spd = strf("%.0f%%", snap.speed * 100);
    const ImVec2 ss = w::text_size(F().mono_medium, fs::numL, spd.c_str());
    w::osd_text(dl, ImVec2(o1.x - m - ss.x, o1.y - m - ss.y), F().mono_medium, fs::numL, col::text1, spd.c_str());
    const char* tag = snap.precision ? "속도 상한 · 저속" : "속도 상한";
    const ImVec2 ts = w::text_size(F().regular, fs::micro, tag);
    w::osd_text(dl, ImVec2(o1.x - m - ts.x, o1.y - m - ss.y - ts.y), F().regular, fs::micro, snap.precision ? col::warn : col::text2, tag);
    // grasp tab: tap the image to pick an object (or a calibration point). Submitted before the icon
    // buttons and marked overlappable so those buttons still win where they sit.
    if (grasp_view && fresh) {
      ImGui::SetCursorScreenPos(p0);
      ImGui::SetNextItemAllowOverlap();
      if (ImGui::InvisibleButton("cam_pick", ImVec2(iw, ih)) && cam_w_ > 0 && cam_h_ > 0) {
        const ImVec2 mp = ImGui::GetIO().MousePos;
        grasp_pick((mp.x - p0.x) / iw * static_cast<float>(cam_w_), (mp.y - p0.y) / ih * static_cast<float>(cam_h_), now);
      }
    }
    // icon buttons above the speed
    const ImVec2 ib(S(64), S(64));
    ImGui::SetCursorScreenPos(ImVec2(o1.x - m - ib.x * 2 - S(sp::s2), o1.y - m - ss.y - ts.y - S(sp::s3) - ib.y));
    if (w::icon_button("cam_toggle", settings_.camera_on ? icon::kCamera : icon::kCameraOff, ib, false))
      settings_.camera_on = !settings_.camera_on;
    ImGui::SameLine(0, S(sp::s2));
    if (w::icon_button("cam_big", camera_big_ ? icon::kMinimize2 : icon::kMaximize2, ib, camera_big_)) camera_big_ = !camera_big_;
  }
  dl->PopClipRect();
  // toasts at the top centre of the camera (bottom centre is busy with the drive vector and buttons)
  w::toast_stack(dl, ImVec2((o.x + o1.x) * 0.5f, o.y + S(sp::s5) + S(48)), std::min(size.x - S(64), S(720)), shown_toasts(), now);
}

// ───────────────────────── safety column (always visible) ─────────────────────────

void GuiApp::draw_safety_column(ImVec2 p0, ImVec2 size) {
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const double now = ctx().now;
  const robot::CoreSnapshot snap = core_ ? core_->snapshot() : robot::CoreSnapshot{};
  const bool latched = core_ && (snap.emergency || (core_->ros().status_fresh() && core_->ros().status().emergency_stop));
  // brief flash of the column on an e-stop, then the latched look
  if (now < estop_flash_until_) dl->AddRectFilled(p0, ImVec2(p0.x + size.x, p0.y + size.y), col::alpha(col::stop, 0.12f), S(sp::radiusL));
  else if (latched) dl->AddRectFilled(p0, ImVec2(p0.x + size.x, p0.y + size.y), col::alpha(col::stop, 0.06f), S(sp::radiusL));

  const float bw = S(192);
  const float x = p0.x + (size.x - bw) * 0.5f;
  float y = p0.y;
  touch_targets_[1] = ImVec2(x + bw * 0.5f, y + bw * 0.5f);
  ImGui::SetCursorScreenPos(ImVec2(x, y));
  const w::EstopVisual v = !core_ ? w::EstopVisual::kDisabled : latched ? w::EstopVisual::kLatched : w::EstopVisual::kReady;
  // Even when latched the button stays live: pressing again re-sends the stop.
  if (w::estop_button("estop", ImVec2(bw, bw), v) || (latched && ImGui::IsItemActivated())) trigger_estop(now);
  y += bw + S(6);
  float hx = x + w::hint(dl, ImVec2(x, y), w::PadKey::kB, "", S(24)) + S(sp::s1);
  w::text(dl, ImVec2(hx, y + S(2)), F().regular, fs::micro, col::text2, "또는 Space");
  y += S(24) + S(sp::s3);

  ImGui::SetCursorScreenPos(ImVec2(x, y));
  if (w::hold_ring_button("clear", "길게 눌러 해제", ImVec2(bw, S(72)), 0.8, clear_progress_pad_, core_ != nullptr) && core_) {
    core_->clear_estop();
    notify("비상정지 해제 요청 — 스틱을 놓았다가 다시", Status::kWarn);
  }
  y += S(72) + S(6);
  hx = x + w::hint(dl, ImVec2(x, y), w::PadKey::kView, "", S(24)) + S(sp::s1);
  w::text(dl, ImVec2(hx, y + S(2)), F().regular, fs::micro, col::text2, "0.8초 누름");
  y += S(24) + S(sp::s4);

  // speed limit
  const float spd = core_ ? static_cast<float>(core_->speed()) : static_cast<float>(settings_.speed);
  w::text(dl, ImVec2(x, y), F().regular, fs::label, col::text2, "속도 상한");
  const std::string sv = strf("%.0f%%", spd * 100);
  const ImVec2 svs = w::text_size(F().mono_medium, fs::numL, sv.c_str());
  w::text(dl, ImVec2(x + bw - svs.x, y - S(6)), F().mono_medium, fs::numL, col::text1, sv.c_str());
  y += S(36);
  const char* speeds[] = {"30%", "60%", "100%"};
  const int presets[] = {30, 60, 100};
  int cur = -1;
  for (int i = 0; i < 3; ++i)
    if (std::lround(spd * 100) == presets[i]) cur = i;
  ImGui::SetCursorScreenPos(ImVec2(x, y));
  const int pick = w::segmented("speed", speeds, nullptr, 3, cur, ImVec2(bw, S(56)), true);
  if (pick >= 0 && core_) {
    core_->set_speed(presets[pick] / 100.0);
    settings_.speed = core_->speed();
  }
  y += 3 * S(56) + 2 * S(sp::s2) + S(6);
  hx = x + w::hint(dl, ImVec2(x, y), w::PadKey::kDpadUD, "", S(24)) + S(sp::s1);
  w::text(dl, ImVec2(hx, y + S(2)), F().regular, fs::micro, col::text2, "±10%");
  y += S(24) + S(sp::s3);
  // slow (precision) toggle — fills what is left, at least 56 px
  const float slow_h = std::max(S(56), std::min(S(64), p0.y + size.y - y));
  ImGui::SetCursorScreenPos(ImVec2(x, y));
  if (w::button("slow", touch_.slow ? "저속 켜짐" : "저속", ImVec2(bw, slow_h), touch_.slow ? w::Variant::kSelected : w::Variant::kDefault))
    touch_.slow = !touch_.slow;
  w::pad_badge(dl, ImVec2(x + bw - S(24) * 1.6f - S(10), y + (slow_h - S(24)) * 0.5f), w::PadKey::kRB, S(24), last_pad_.down(input::kRB));
}

// ───────────────────────── bottom bar (touch driving) ─────────────────────────

void GuiApp::draw_bottom_bar(ImVec2 p0, ImVec2 size) {
  ImDrawList* dl = ImGui::GetWindowDrawList();
  dl->AddRectFilled(p0, ImVec2(p0.x + size.x, p0.y + size.y), col::bg1);
  dl->AddLine(p0, ImVec2(p0.x + size.x, p0.y), col::border1, S(1));
  const float cy = p0.y + size.y * 0.5f;

  // virtual stick (drive)
  const float r = S(100);
  touch_targets_[0] = ImVec2(p0.x + S(36) + r, cy);
  ImGui::SetCursorScreenPos(ImVec2(p0.x + S(36), cy - r));
  w::virtual_stick("drive_stick", r, &touch_.stick_x, &touch_.stick_y);
  w::text_center(dl, ImVec2(p0.x + S(36) + r, cy + r + S(16)), F().regular, fs::micro, col::text3, "주행");

  // turn slider
  const float sx0 = p0.x + S(272), sw = S(320);
  w::text(dl, ImVec2(sx0, cy - S(70)), F().regular, fs::label, col::text2, "회전");
  w::text(dl, ImVec2(sx0 + sw - S(60), cy - S(70)), F().regular, fs::label, col::text3, icon::kRotateCcw);
  w::text(dl, ImVec2(sx0 + sw - S(28), cy - S(70)), F().regular, fs::label, col::text3, icon::kRotateCw);
  ImGui::SetCursorScreenPos(ImVec2(sx0, cy - S(28)));
  ImGui::InvisibleButton("turn", ImVec2(sw, S(56)));
  bool turning = ImGui::IsItemActive();
  float turn_x = ImGui::GetIO().MousePos.x;
  if (!turning) {
    if (const TouchPoint* tp = w::touch_claim(ImGui::GetItemID(), ImVec2(sx0, cy - S(28)), ImVec2(sx0 + sw, cy + S(28)))) {
      turning = true;
      turn_x = tp->pos.x;
    }
  }
  touch_.turn = turning ? clampd((turn_x - (sx0 + sw * 0.5f)) / (sw * 0.5f - S(28)), -1, 1) : 0.0f;
  dl->AddRectFilled(ImVec2(sx0, cy - S(8)), ImVec2(sx0 + sw, cy + S(8)), col::border2, S(8));
  dl->AddCircleFilled(ImVec2(sx0 + sw * 0.5f + touch_.turn * (sw * 0.5f - S(28)), cy), S(28), turning ? col::accent : col::text2, 40);

  // arm group
  const robot::CoreSnapshot snap = core_ ? core_->snapshot() : robot::CoreSnapshot{};
  float ax = p0.x + S(640);
  ImGui::SetCursorScreenPos(ImVec2(ax, cy - S(56)));
  const bool on = touch_arm_toggle_;
  if (w::button("arm_toggle", on ? "팔 켜짐" : "팔 켜기", ImVec2(S(160), S(112)), on ? w::Variant::kSelected : w::Variant::kDefault, icon::kHand) && core_) {
    touch_arm_toggle_ = !touch_arm_toggle_;
    core_->set_arm_enabled(touch_arm_toggle_, true);
  }
  const float bx = ax;
  w::pad_badge(dl, ImVec2(bx, cy + S(64)), w::PadKey::kLB, S(28), last_pad_.down(input::kLB));
  w::text(dl, ImVec2(bx + S(28) * 1.6f + S(sp::s2), cy + S(68)), F().regular, fs::micro, snap.arm_enabled ? col::ok : col::text3,
          snap.arm_enabled ? "팔 조종 중" : "누른 채도 됨");
  ax += S(160) + S(sp::s4);

  for (auto& a : touch_.arm) a = 0;
  const ImVec2 b(S(112), S(112));
  const float pair_gap = S(4);
  struct Pair {
    const char* id;
    const char* l1;
    const char* i1;
    const char* l2;
    const char* i2;
    int axis;
    bool horizontal;
  };
  const Pair pairs[] = {{"fb", "앞", icon::kArrowUp, "뒤", icon::kArrowDown, 0, false},
                        {"lr", "왼", icon::kArrowLeft, "오른", icon::kArrowRight, 1, true},
                        {"ud", "위", icon::kChevronLeft, "아래", icon::kChevronRight, 2, false},
                        {"wr", "손목↑", icon::kRotateCcw, "손목↓", icon::kRotateCw, 3, false}};
  for (const auto& pr : pairs) {
    ImGui::PushID(pr.id);
    if (pr.horizontal) {
      ImGui::SetCursorScreenPos(ImVec2(ax, cy - b.y * 0.5f));
      if (w::hold_button_active("a", pr.l1, b, pr.i1)) touch_.arm[pr.axis] = 1;
      ImGui::SetCursorScreenPos(ImVec2(ax + b.x + pair_gap, cy - b.y * 0.5f));
      if (w::hold_button_active("b", pr.l2, b, pr.i2)) touch_.arm[pr.axis] = -1;
      ax += 2 * b.x + pair_gap + S(sp::s4);
    } else {
      ImGui::SetCursorScreenPos(ImVec2(ax, cy - b.y - pair_gap * 0.5f));
      if (w::hold_button_active("a", pr.l1, b, pr.axis == 2 ? nullptr : pr.i1)) touch_.arm[pr.axis] = 1;
      ImGui::SetCursorScreenPos(ImVec2(ax, cy + pair_gap * 0.5f));
      if (w::hold_button_active("b", pr.l2, b, pr.axis == 2 ? nullptr : pr.i2)) touch_.arm[pr.axis] = -1;
      ax += b.x + S(sp::s4);
    }
    ImGui::PopID();
  }
  // gripper pair
  const ImVec2 gb(S(144), S(112));
  ImGui::SetCursorScreenPos(ImVec2(ax, cy - gb.y - pair_gap * 0.5f));
  if (w::hold_button_active("grip_close", "닫기 RT", gb)) touch_.arm[4] = 1;
  ImGui::SetCursorScreenPos(ImVec2(ax, cy + pair_gap * 0.5f));
  if (w::hold_button_active("grip_open", "열기 LT", gb)) touch_.arm[4] = -1;
  ax += gb.x;
  w::text_center(dl, ImVec2(ax - gb.x * 0.5f, cy - gb.y - S(18)), F().regular, fs::micro, col::text3, "그리퍼");

  // gamepad monitor (right edge)
  const float mx1 = p0.x + size.x - S(sp::gutter);
  if (mx1 - ax > S(280)) {
    const float mw = std::min(S(300), mx1 - ax - S(sp::s5));
    const float mx0 = mx1 - mw;
    const ImVec2 c1(mx0 + mw * 0.27f, cy), c2(mx0 + mw * 0.73f, cy);
    const bool con = last_pad_.connected;
    for (int s = 0; s < 2; ++s) {
      const ImVec2 c = s == 0 ? c1 : c2;
      const float px = s == 0 ? last_pad_.lx : last_pad_.rx, py = s == 0 ? last_pad_.ly : last_pad_.ry;
      dl->AddCircle(c, S(44), col::border2, 40, S(2));
      dl->AddCircleFilled(ImVec2(c.x + px * S(34), c.y - py * S(34)), S(10), con ? col::text1 : col::disabled, 20);
      const float bh = S(28);
      const float bw2 = bh * 1.6f;
      w::pad_badge(dl, ImVec2(c.x - bw2 * 0.5f, c.y - S(44) - bh - S(12)), s == 0 ? w::PadKey::kLB : w::PadKey::kRB, bh,
                   last_pad_.down(s == 0 ? input::kLB : input::kRB));
      w::trigger_bar(dl, ImVec2(c.x - S(44), c.y + S(44) + S(16)), S(88), s == 0 ? last_pad_.lt : last_pad_.rt, con);
      w::text_center(dl, ImVec2(c.x, c.y + S(44) + S(40)), F().regular, fs::micro, col::text3, s == 0 ? "LT" : "RT");
    }
    if (!con) w::text_center(dl, ImVec2((c1.x + c2.x) * 0.5f, cy), F().semibold, fs::label, col::danger, "패드 없음");
  }
}

// ───────────────────────── tab panel ─────────────────────────

void GuiApp::draw_tab_panel(ImVec2 p0, ImVec2 size) {
  const bool scroll = tab_ == Tab::kDiag || tab_ == Tab::kSettings || tab_ == Tab::kGrasp;
  if (begin_region("tab_panel", p0, size, scroll)) {
    switch (tab_) {
      case Tab::kDrive: draw_tab_drive(size); break;
      case Tab::kArm: draw_tab_arm(size); break;
      case Tab::kGrasp: draw_tab_grasp(size); break;
      case Tab::kDiag: draw_tab_diag(size); break;
      case Tab::kSettings: draw_tab_settings(size); break;
    }
  }
  end_region();
}

void GuiApp::draw_tab_drive(ImVec2 size) {
  if (!core_) return;
  const auto snap = core_->snapshot();
  const float wdt = size.x;
  // channels
  if (w::begin_card("ch", "채널", ImVec2(wdt, S(176)))) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const float col_w = (ImGui::GetContentRegionAvail().x) * 0.5f;
    const char* names[] = {"차체", "팔"};
    const armpi_protocol::ChannelStatus* ch[] = {&snap.base, &snap.arm};
    for (int i = 0; i < 2; ++i) {
      const float x = p.x + col_w * i;
      w::text(dl, ImVec2(x, p.y), F().regular, fs::micro, col::text2, names[i]);
      const Status s = channel_status(ch[i]->server_state);
      w::status_symbol(dl, ImVec2(x + S(8), p.y + S(44)), S(7), s);
      w::text(dl, ImVec2(x + S(24), p.y + S(28)), F().semibold, fs::title,
              s == Status::kDanger ? col::danger : s == Status::kWarn ? col::warn : col::text1, state_ko(ch[i]->server_state));
      w::text(dl, ImVec2(x, p.y + S(72)), F().regular, fs::label, col::text2,
              strf("응답: %s", reason_ko(ch[i]->server_reason)).c_str());
    }
    ImGui::Dummy(ImVec2(wdt, S(100)));
  }
  w::end_card();
  ImGui::Dummy(ImVec2(0, S(sp::s4) - ImGui::GetStyle().ItemSpacing.y));
  // link
  if (w::begin_card("link", "연결 품질", ImVec2(wdt, S(156)))) {
    const float tw = (ImGui::GetContentRegionAvail().x - 2 * S(sp::s3)) / 3;
    const bool slow = snap.ack_ms_p50 && *snap.ack_ms_p50 > 100.0;
    w::number_tile("ACK 중앙", fmt_ms(snap.ack_ms_p50).c_str(), "ms", ImVec2(tw, S(84)), slow ? Status::kWarn : Status::kOk);
    ImGui::SameLine(0, S(sp::s3));
    w::number_tile("ACK 최대", fmt_ms(snap.ack_ms_max).c_str(), "ms", ImVec2(tw, S(84)));
    ImGui::SameLine(0, S(sp::s3));
    w::number_tile("조종 루프", strf("%.1f", snap.loop_hz).c_str(), "Hz", ImVec2(tw, S(84)),
                   snap.loop_hz < 18.0 ? Status::kWarn : Status::kOk);
  }
  w::end_card();
  ImGui::Dummy(ImVec2(0, S(sp::s4) - ImGui::GetStyle().ItemSpacing.y));
  // session
  if (w::begin_card("session", "세션", ImVec2(wdt, S(196)))) {
    const float bw = (ImGui::GetContentRegionAvail().x - 2 * S(sp::s2)) / 3;
    if (w::button("change", "연결 바꾸기", ImVec2(bw, S(64)), w::Variant::kDefault, icon::kLink)) show_connect_ = true;
    ImGui::SameLine(0, S(sp::s2));
    if (snap.session_released) {
      if (w::button("newsess", "새 세션", ImVec2(bw, S(64)), w::Variant::kPrimary, icon::kRefreshCw)) core_->new_session();
    } else {
      if (w::button("release", "세션 놓기", ImVec2(bw, S(64)), w::Variant::kDefault, icon::kUnplug)) core_->release_session();
    }
    ImGui::SameLine(0, S(sp::s2));
    if (w::button("disconnect", "연결 끊기", ImVec2(bw, S(64)), w::Variant::kDanger, icon::kPower)) {
      disconnect();
      show_connect_ = true;
    }
    ImGui::PushStyleColor(ImGuiCol_Text, col::v4(col::text2));
    ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x);
    ImGui::TextUnformatted("세션 놓기: 휴대폰·VR 같은 다른 조종기가 로봇을 쓸 수 있게 하고, 관절 직접 조종을 켭니다.");
    ImGui::PopTextWrapPos();
    ImGui::PopStyleColor();
  }
  w::end_card();
  if (!snap.last_error.empty()) {
    ImGui::PushStyleColor(ImGuiCol_Text, col::v4(col::warn));
    ImGui::TextWrapped("최근 오류: %s", snap.last_error.c_str());
    ImGui::PopStyleColor();
  }
}

// Top view: up = +x (forward), left = +y. Side view: right = +x, up = +z. Tap/drag sets a ramped goal.
void GuiApp::arm_plot(const char* id, ImVec2 size, bool top) {
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const ImVec2 p = ImGui::GetCursorScreenPos();
  ImGui::InvisibleButton(id, size);
  const bool active = ImGui::IsItemActive();
  dl->AddRectFilled(p, ImVec2(p.x + size.x, p.y + size.y), col::bg0, S(sp::radiusM));
  dl->AddRect(p, ImVec2(p.x + size.x, p.y + size.y), col::border1, S(sp::radiusM), 0, S(1));
  dl->PushClipRect(p, ImVec2(p.x + size.x, p.y + size.y), true);
  const double h_min = top ? -0.30 : 0.0, h_max = top ? 0.30 : 0.36;
  const double v_min = top ? 0.0 : -0.12, v_max = 0.36;
  const float m = S(14);
  const float sc = std::min((size.x - 2 * m) / static_cast<float>(h_max - h_min), (size.y - 2 * m) / static_cast<float>(v_max - v_min));
  const float cx = p.x + size.x / 2, cy = p.y + size.y - m;
  auto to_screen = [&](double hh, double v) {
    const double x = top ? -hh : hh - (h_min + h_max) / 2;
    return ImVec2(cx + static_cast<float>(x) * sc, cy - static_cast<float>(v - v_min) * sc);
  };
  auto to_world = [&](ImVec2 s, double* hh, double* v) {
    const double x = (s.x - cx) / sc;
    *hh = top ? -x : x + (h_min + h_max) / 2;
    *v = (cy - s.y) / sc + v_min;
  };
  // 5 cm grid
  for (double gx = std::ceil(h_min / 0.05) * 0.05; gx <= h_max + 1e-9; gx += 0.05) {
    const ImVec2 a = to_screen(gx, v_min), b2 = to_screen(gx, v_max);
    dl->AddLine(a, b2, col::alpha(col::border1, 0.6f), S(1));
  }
  for (double gy = std::ceil(v_min / 0.05) * 0.05; gy <= v_max + 1e-9; gy += 0.05) {
    const ImVec2 a = to_screen(h_min, gy), b2 = to_screen(h_max, gy);
    dl->AddLine(a, b2, col::alpha(col::border1, 0.6f), S(1));
  }
  // workspace box
  const ImVec2 b0 = top ? to_screen(kWsYMax, kWsXMax) : to_screen(kWsXMin, kWsZMax);
  const ImVec2 b1 = top ? to_screen(kWsYMin, kWsXMin) : to_screen(kWsXMax, kWsZMin);
  const ImVec2 bmin(std::min(b0.x, b1.x), std::min(b0.y, b1.y)), bmax(std::max(b0.x, b1.x), std::max(b0.y, b1.y));
  dl->AddRectFilled(bmin, bmax, col::alpha(col::surface2, 0.5f));
  dl->AddRect(bmin, bmax, col::border2, 0, 0, S(1));
  const ImVec2 base = to_screen(0, 0);
  dl->AddRectFilled(ImVec2(base.x - S(10), base.y - S(5)), ImVec2(base.x + S(10), base.y + S(5)), col::text3, S(2));
  w::text(dl, ImVec2(p.x + S(8), p.y + S(6)), F().regular, fs::micro, col::text2, top ? "위에서" : "옆에서");
  w::text(dl, ImVec2(p.x + size.x - S(8) - w::text_size(F().regular, fs::micro, top ? "앞 ↑" : "위 ↑").x, p.y + S(6)),
          F().regular, fs::micro, col::text3, top ? "앞 ↑" : "위 ↑");
  if (core_) {
    const auto snap = core_->snapshot();
    auto pos = [&](double x, double y, double z) { return top ? to_screen(y, x) : to_screen(x, z); };
    const auto st = core_->ros().status();
    if (core_->ros().status_fresh() && st.arm_current)
      dl->AddCircleFilled(pos(st.arm_current->x, st.arm_current->y, st.arm_current->z), S(6), col::text2, 16);
    if (const auto ap = core_->ros().applied_arm_target()) dl->AddCircle(pos(ap->x, ap->y, ap->z), S(12), col::ok, 24, S(3));
    const auto& t = snap.target;
    const ImVec2 tp = pos(t.x, t.y, t.z);
    dl->AddCircleFilled(tp, S(8), snap.limit_active ? col::warn : col::accent, 20);
    if (!top) dl->AddLine(tp, ImVec2(tp.x + std::cos(static_cast<float>(t.pitch)) * S(28), tp.y - std::sin(static_cast<float>(t.pitch)) * S(28)), col::accent, S(3));
    if (active) {
      if (!snap.arm_enabled) {
        if (ImGui::IsItemActivated()) notify("팔 조종을 먼저 켜세요(LB 또는 팔 켜기)", Status::kWarn, 2.0);
      } else {
        double hh, v;
        const ImVec2 mp = ImGui::GetIO().MousePos;
        to_world(mp, &hh, &v);
        if (top) core_->arm_goto_partial(clampd(v, kWsXMin, kWsXMax), clampd(hh, kWsYMin, kWsYMax), std::nullopt, std::nullopt);
        else core_->arm_goto_partial(clampd(hh, kWsXMin, kWsXMax), std::nullopt, clampd(v, kWsZMin, kWsZMax), std::nullopt);
        dl->AddCircle(mp, S(24), col::accent, 24, S(2));
        dl->AddLine(ImVec2(mp.x - S(32), mp.y), ImVec2(mp.x + S(32), mp.y), col::alpha(col::accent, 0.6f), S(1));
        dl->AddLine(ImVec2(mp.x, mp.y - S(32)), ImVec2(mp.x, mp.y + S(32)), col::alpha(col::accent, 0.6f), S(1));
      }
    }
  }
  dl->PopClipRect();
}

void GuiApp::draw_tab_arm(ImVec2 size) {
  if (!core_) return;
  const auto snap = core_->snapshot();
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const float wdt = size.x;
  // title row
  {
    const ImVec2 p = ImGui::GetCursorScreenPos();
    w::text(dl, p, F().semibold, fs::title, col::text1, "팔");
    const float tx = p.x + w::text_size(F().semibold, fs::title, "팔").x + S(sp::s4);
    const Status s = snap.arm_enabled ? Status::kOk : Status::kIdle;
    w::status_symbol(dl, ImVec2(tx + S(6), p.y + S(20)), S(6), s);
    w::text(dl, ImVec2(tx + S(20), p.y + S(8)), F().regular, fs::label, snap.arm_enabled ? col::ok : col::text2,
            snap.arm_enabled ? "켜짐" : "꺼짐 — LB 또는 팔 켜기");
    ImGui::Dummy(ImVec2(wdt, S(34)));
  }
  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(S(sp::s3), S(sp::s2)));
  const float pw = (wdt - S(sp::s4)) * 0.5f;
  arm_plot("top", ImVec2(pw, S(190)), true);
  ImGui::SameLine(0, S(sp::s4));
  arm_plot("side", ImVec2(pw, S(190)), false);
  {
    const ImVec2 p = ImGui::GetCursorScreenPos();
    float x = p.x;
    auto legend = [&](ImU32 c, bool ring, const char* t) {
      if (ring) dl->AddCircle(ImVec2(x + S(7), p.y + S(11)), S(6), c, 16, S(2));
      else dl->AddCircleFilled(ImVec2(x + S(7), p.y + S(11)), S(6), c, 16);
      w::text(dl, ImVec2(x + S(18), p.y), F().regular, fs::micro, col::text2, t);
      x += S(18) + w::text_size(F().regular, fs::micro, t).x + S(sp::s4);
    };
    legend(col::accent, false, "내 목표");
    legend(col::ok, true, "로봇이 쓰는 목표");
    legend(col::text2, false, "로봇 현재");
    ImGui::Dummy(ImVec2(wdt, S(20)));
  }
  // numeric goal
  if (w::begin_card("num", nullptr, ImVec2(wdt, S(184)))) {
    const char* labels[] = {"x (m)", "y (m)", "z (m)", "pitch (°)"};
    const float steps[] = {0.01f, 0.01f, 0.01f, 5.0f};
    const float cell = (ImGui::GetContentRegionAvail().x - S(sp::s3)) * 0.5f;
    for (int i = 0; i < 4; ++i) {
      if (i % 2) ImGui::SameLine(0, S(sp::s3));
      ImGui::PushID(i);
      ImGui::BeginGroup();
      const ImVec2 cp = ImGui::GetCursorScreenPos();
      const float bh = S(44);
      if (w::button("m", "", ImVec2(bh, bh), w::Variant::kDefault, icon::kMinus)) num_[i] -= steps[i];
      ImGui::SameLine(0, S(sp::s1));
      const ImVec2 vp = ImGui::GetCursorScreenPos();
      const float vw = cell - 2 * bh - 2 * S(sp::s1);
      ImGui::Dummy(ImVec2(vw, bh));
      dl->AddRectFilled(vp, ImVec2(vp.x + vw, vp.y + bh), col::surface2, S(sp::radiusS));
      w::text(dl, ImVec2(vp.x + S(8), vp.y + S(2)), F().regular, fs::micro, col::text3, labels[i]);
      const std::string val = i == 3 ? strf("%+.0f", num_[i]) : strf("%.3f", num_[i]);
      const ImVec2 vs = w::text_size(F().mono, fs::label, val.c_str());
      w::text(dl, ImVec2(vp.x + vw - vs.x - S(8), vp.y + bh - vs.y - S(2)), F().mono, fs::label, col::text1, val.c_str());
      ImGui::SameLine(0, S(sp::s1));
      if (w::button("p", "", ImVec2(bh, bh), w::Variant::kDefault, icon::kPlus)) num_[i] += steps[i];
      (void)cp;
      ImGui::EndGroup();
      ImGui::PopID();
    }
    const float bw = (ImGui::GetContentRegionAvail().x - 2 * S(sp::s2)) / 3;
    if (w::button("goto", "이 목표로", ImVec2(bw, S(48)), w::Variant::kPrimary)) {
      core_->arm_goto(robot::ArmTarget{num_[0], num_[1], num_[2], rad(num_[3]), snap.target.gripper});
      notify("팔: 숫자 목표로 이동(속도 제한)", Status::kIdle, 2.0);
    }
    ImGui::SameLine(0, S(sp::s2));
    if (w::button("load", "지금 목표", ImVec2(bw, S(48)))) {
      num_[0] = static_cast<float>(snap.target.x);
      num_[1] = static_cast<float>(snap.target.y);
      num_[2] = static_cast<float>(snap.target.z);
      num_[3] = static_cast<float>(deg(snap.target.pitch));
    }
    ImGui::SameLine(0, S(sp::s2));
    if (w::button("home", "기본 자세", ImVec2(bw, S(48)), w::Variant::kDefault, icon::kHouse)) {
      std::string err;
      if (!core_->arm_home(&err)) notify("기본 자세 실패: " + err, Status::kDanger);
    }
  }
  w::end_card();
  // gripper
  num_grip_ = static_cast<float>(snap.target.gripper);
  if (w::slider("grip", &num_grip_, 0.0f, 1.0f, ImVec2(wdt, S(52)), "열림 0", "1 닫힘")) core_->set_gripper(num_grip_);
  // target table
  const auto st = core_->ros().status();
  const auto ap = core_->ros().applied_arm_target();
  ImGui::PushFont(F().mono, fs::micro);
  ImGui::PushStyleVar(ImGuiStyleVar_CellPadding, ImVec2(S(sp::s2), S(3)));
  if (ImGui::BeginTable("armtbl", 6, ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_RowBg)) {
    ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, S(150));
    for (const char* hname : {"x", "y", "z", "pitch", "grip"}) ImGui::TableSetupColumn(hname);
    ImGui::PushFont(F().regular, fs::micro);
    ImGui::TableHeadersRow();
    ImGui::PopFont();
    auto row = [&](const char* name, const std::optional<robot::Pose5>& p) {
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::PushFont(F().regular, fs::micro);
      ImGui::TextColored(col::v4(col::text2), "%s", name);
      ImGui::PopFont();
      if (!p) return;
      const double v[5] = {p->x, p->y, p->z, deg(p->pitch), p->gripper};
      for (int i = 0; i < 5; ++i) {
        ImGui::TableSetColumnIndex(i + 1);
        ImGui::Text(i == 3 ? "%+.0f" : "%.3f", v[i]);
      }
    };
    const auto& t = snap.target;
    row("내 목표", robot::Pose5{t.x, t.y, t.z, t.pitch, t.gripper});
    row("로봇이 쓰는 목표", ap);
    row(st.position_source == "measured" ? "로봇 현재(측정)" : "로봇 현재(명령)", core_->ros().status_fresh() ? st.arm_current : std::nullopt);
    ImGui::EndTable();
  }
  ImGui::PopStyleVar();
  ImGui::PopFont();
  // joints: one row of mini bars
  const auto js = core_->ros().joints();
  if (!js.position.empty()) {
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const size_t n = std::min<size_t>(6, std::min(js.position.size(), js.names.size()));
    const float cw = (wdt - S(sp::s2) * 5) / 6;
    for (size_t i = 0; i < n; ++i) {
      const float x = p.x + i * (cw + S(sp::s2));
      const float d = static_cast<float>(deg(js.position[i]));
      const float t = std::clamp((d + 120.0f) / 240.0f, 0.0f, 1.0f);
      dl->AddRectFilled(ImVec2(x, p.y + S(24)), ImVec2(x + cw, p.y + S(32)), col::border2, S(4));
      dl->AddRectFilled(ImVec2(x + cw * 0.5f, p.y + S(24)), ImVec2(x + cw * t, p.y + S(32)), col::text2, S(4));
      w::text(dl, ImVec2(x, p.y), F().mono, fs::micro, col::text2, strf("J%d %+.0f", static_cast<int>(i + 1), d).c_str());
    }
    ImGui::Dummy(ImVec2(wdt, S(36)));
  }
  ImGui::PopStyleVar();
}

void GuiApp::draw_tab_diag(ImVec2 size) {
  if (!core_) return;
  const auto snap = core_->snapshot();
  const float wdt = size.x - S(20);  // leave room for the scrollbar
  if (w::begin_card("dlink", "링크 세부", ImVec2(wdt, S(200)))) {
    ImGui::PushFont(F().mono, fs::micro);
    ImGui::Text("ACK  last %s  p50 %s  max %s ms", fmt_ms(snap.ack_ms_last).c_str(), fmt_ms(snap.ack_ms_p50).c_str(), fmt_ms(snap.ack_ms_max).c_str());
    ImGui::Text("패킷 보냄 %llu  ACK %llu  늦음 %llu  오류 %llu", static_cast<unsigned long long>(snap.packets_sent),
                static_cast<unsigned long long>(snap.acks), static_cast<unsigned long long>(snap.late_acks),
                static_cast<unsigned long long>(snap.send_errors));
    ImGui::Text("루프 %.1f Hz  밀림 %llu  화면 %.0f fps", snap.loop_hz, static_cast<unsigned long long>(snap.loop_overruns), ImGui::GetIO().Framerate);
    ImGui::Text("차체 무장 %s  자동 재무장 %d  차단: %s", snap.base.armed ? "예" : "아니오", snap.base.auto_rearms, snap.base.last_block_reason.c_str());
    ImGui::Text("팔   무장 %s  자동 재무장 %d  차단: %s", snap.arm.armed ? "예" : "아니오", snap.arm.auto_rearms, snap.arm.last_block_reason.c_str());
    ImGui::PopFont();
  }
  w::end_card();
  // packets
  if (w::begin_card("pk", "최근 패킷(이 앱의 통신만)", ImVec2(wdt, S(300)))) {
    bool keep_ros = comm_.keep_rosbridge();
    if (ImGui::Checkbox("rosbridge 메시지도 보기", &keep_ros)) comm_.set_keep_rosbridge(keep_ros);
    ImGui::PushStyleColor(ImGuiCol_ChildBg, col::v4(col::bg0));
    ImGui::BeginChild("pklog", ImVec2(0, 0), ImGuiChildFlags_Borders);
    ImGui::PushFont(F().mono, fs::micro);
    for (const auto& r : comm_.last(120)) {
      ImGui::TextColored(col::v4(col::text3), "%s %s %s", r.time.c_str() + 3, r.outgoing ? "→" : "←", net::channel_name(r.channel));
      ImGui::SameLine();
      ImGui::TextColored(col::v4(col::text2), "%s", privacy_ ? "(화면 저장 중 숨김)" : printable(r.data, 120).c_str());
    }
    if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 5) ImGui::SetScrollHereY(1.0f);
    ImGui::PopFont();
    ImGui::EndChild();
    ImGui::PopStyleColor();
  }
  w::end_card();
  const auto st = core_->ros().status();
  if (ImGui::CollapsingHeader("로봇 상태", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (!core_->ros().status_fresh()) {
      ImGui::TextColored(col::v4(col::warn), "미수신: %s", core_->ros().last_error().c_str());
    } else {
      ImGui::PushFont(F().regular, fs::micro);
      ImGui::TextWrapped("하드웨어 %s · 커널 %s · 위치 출처 %s · 팔 모드 %s", st.hardware.c_str(), st.kernel.c_str(), st.position_source.c_str(), st.arm_mode.c_str());
      ImGui::TextWrapped("UDP 허용 %s · IK 막힘 %s · 마지막 팔 목표 %s", st.udp_allowed ? "예" : "아니오", st.arm_ik_blocked ? "예" : "아니오", st.arm_last_target_result.c_str());
      if (!st.warnings.empty()) ImGui::TextColored(col::v4(col::warn), "경고: %s", st.warnings.c_str());
      for (const auto& wv : st.boot_warnings) ImGui::TextColored(col::v4(col::warn), "부팅 점검: %s", wv.c_str());
      const auto bs = core_->ros().bridge_state();
      if (bs.valid) ImGui::Text("수신기: 차체 %s · 팔 %s · 보정 %llu", bs.base_state.c_str(), bs.arm_state.c_str(), static_cast<unsigned long long>(bs.arm_adjust_total));
      ImGui::PopFont();
    }
  }
  if (ImGui::CollapsingHeader("관절 직접 조종")) {
    std::string why;
    const bool allowed = core_->joint_debug_allowed(&why);
    ImGui::TextColored(col::v4(allowed ? col::ok : col::text2), "%s", allowed ? "사용 가능" : why.c_str());
    ImGui::PushItemWidth(wdt - S(120));
    for (int i = 0; i < 6; ++i) ImGui::SliderFloat(robot::kJointNames[static_cast<size_t>(i)], &joints_deg_[i], -120.0f, 120.0f, "%.0f°");
    ImGui::PopItemWidth();
    if (w::button("jload", "현재값 불러오기", ImVec2(S(220), S(56)))) {
      const auto js = core_->ros().joints();
      for (size_t i = 0; i < js.position.size() && i < 6; ++i) joints_deg_[i] = static_cast<float>(deg(js.position[i]));
    }
    ImGui::SameLine();
    if (w::button("jsend", "관절 보내기", ImVec2(S(180), S(56)), w::Variant::kPrimary, nullptr, allowed)) {
      std::array<double, 6> r{};
      for (int i = 0; i < 6; ++i) r[static_cast<size_t>(i)] = rad(joints_deg_[i]);
      std::string err;
      if (core_->joint_debug(r, &err)) notify("관절 목표 보냄", Status::kIdle, 2.0);
      else notify("관절 보내기 실패: " + err, Status::kDanger);
    }
  }
  if (ImGui::CollapsingHeader("로봇 전체 정지·해제(rosbridge)")) {
    if (w::button("rstop", "로봇 전체 정지", ImVec2(S(220), S(64)), w::Variant::kDanger, icon::kOctagonX)) {
      std::string err;
      if (!core_->robot_estop(&err)) notify("전체 정지 요청 실패: " + err, Status::kDanger);
    }
    ImGui::SameLine();
    if (w::hold_ring_button("rreset", "길게: 전체 해제", ImVec2(S(220), S(64)), 0.8, 0.0f, true)) confirm_reset_ = true;
    if (confirm_reset_) {
      ImGui::TextColored(col::v4(col::warn), "로봇의 모든 비상정지(버튼·서비스 포함)를 풉니다. 주변이 안전합니까?");
      if (w::button("rok", "확인 — 해제", ImVec2(S(200), S(56)), w::Variant::kPrimary)) {
        std::string err;
        if (!core_->robot_reset(&err)) notify("전체 해제 실패: " + err, Status::kDanger);
        confirm_reset_ = false;
      }
      ImGui::SameLine();
      if (w::button("rcancel", "취소", ImVec2(S(120), S(56)))) confirm_reset_ = false;
    }
  }
  if (ImGui::CollapsingHeader("사건 기록", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushStyleColor(ImGuiCol_ChildBg, col::v4(col::bg0));
    ImGui::BeginChild("ev", ImVec2(0, S(220)), ImGuiChildFlags_Borders);
    ImGui::PushFont(F().regular, fs::micro);
    for (const auto& e : log_.last(60)) {
      const ImU32 c = e.level == Level::kError ? col::danger : e.level == Level::kWarn ? col::warn : e.level == Level::kOk ? col::ok : col::text2;
      ImGui::TextColored(col::v4(c), "%s %s", e.time.c_str() + 3, privacy_ ? mask_addresses(e.text).c_str() : e.text.c_str());
    }
    if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 5) ImGui::SetScrollHereY(1.0f);
    ImGui::PopFont();
    ImGui::EndChild();
    ImGui::PopStyleColor();
    if (w::button("logs", "기록 폴더 열기", ImVec2(S(220), S(56))) && plat_.open_folder) plat_.open_folder(settings_dir());
    ImGui::SameLine();
    if (w::button("cap", "화면 저장(F12)", ImVec2(S(220), S(56)), w::Variant::kDefault, icon::kCamera) && plat_.capture)
      plat_.capture(1, 0.0);
  }
}

void GuiApp::draw_tab_settings(ImVec2 size) {
  const float wdt = size.x - S(20);
  const float lw = S(190);
  if (w::begin_card("s_conn", "연결", ImVec2(wdt, S(360)))) {
    ImGui::PushItemWidth(-1);
    form_label("제어 포트", lw);
    ImGui::InputInt("##cp", &settings_.control_port, 0);
    form_label("검색 포트", lw);
    ImGui::InputInt("##dp", &settings_.discovery_port, 0);
    form_label("rosbridge(0=끔)", lw);
    ImGui::InputInt("##rp", &settings_.rosbridge_port, 0);
    form_label("카메라 포트", lw);
    ImGui::InputInt("##vp", &settings_.camera_port, 0);
    float ack = static_cast<float>(settings_.ack_timeout_s);
    form_label("ACK 허용 지연", lw);
    if (ImGui::SliderFloat("##ack", &ack, 0.15f, 1.0f, "%.2f s")) settings_.ack_timeout_s = ack;
    ImGui::PopItemWidth();
    settings_.control_port = std::clamp(settings_.control_port, 1, 65535);
    settings_.discovery_port = std::clamp(settings_.discovery_port, 1, 65535);
    settings_.rosbridge_port = std::clamp(settings_.rosbridge_port, 0, 65535);
    settings_.camera_port = std::clamp(settings_.camera_port, 1, 65535);
  }
  w::end_card();
  if (w::begin_card("s_input", "입력", ImVec2(wdt, settings_.tilt_drive ? S(340) : S(284)))) {
    ImGui::PushItemWidth(-1);
    form_label("스틱 데드존", lw);
    ImGui::SliderFloat("##dz", &settings_.deadzone, 0.05f, 0.35f, "%.2f");
    form_label("스틱 곡선", lw);
    ImGui::SliderFloat("##cv", &settings_.curve, 1.0f, 2.5f, "%.1f");
    ImGui::PopItemWidth();
    ImGui::Checkbox("LB 중에도 왼스틱으로 주행(고급)", &settings_.allow_drive_while_arm);
    ImGui::Checkbox("기울기 주행: RT를 누른 동안 Ally를 기울여 주행", &settings_.tilt_drive);
    if (settings_.tilt_drive) {
      ImGui::PushItemWidth(-1);
      form_label("최대 속도 기울기", lw);
      ImGui::SliderFloat("##tilt", &settings_.tilt_full_deg, 12.0f, 45.0f, "%.0f°");
      ImGui::PopItemWidth();
    }
  }
  w::end_card();
  if (w::begin_card("s_fb", "화면·진동", ImVec2(wdt, S(470)))) {
    ImGui::PushItemWidth(-1);
    ImGui::Checkbox("진동", &settings_.haptics);
    form_label("진동 세기", lw);
    ImGui::SliderFloat("##hs", &settings_.haptic_strength, 0.2f, 1.0f, "%.1f");
    form_label("글자 크기", lw);
    ImGui::SliderFloat("##fs", &settings_.font_scale, 1.0f, 1.3f, "x%.1f");
    ImGui::PopItemWidth();
    ImGui::Checkbox("영상 십자선", &settings_.show_crosshair);
    if (ImGui::Checkbox("전체 화면(F11)", &settings_.fullscreen) && plat_.set_fullscreen) plat_.set_fullscreen(settings_.fullscreen);
    form_label("화면 주사율", lw);
    const char* fps_items[] = {"30", "60", "120"};
    const int fps_vals[] = {30, 60, 120};
    int cur = settings_.fps_cap >= 120 ? 2 : settings_.fps_cap >= 60 ? 1 : 0;
    const int pick = w::segmented("fps", fps_items, nullptr, 3, cur, ImVec2((wdt - lw - S(40)) / 3, S(48)), false);
    if (pick >= 0) settings_.fps_cap = fps_vals[pick];
    if (w::button("hap_test", "진동 시험", ImVec2(S(180), S(56)))) haptics_.play(input::Haptic::kArmOn, mono_now());
  }
  w::end_card();
  if (w::button("save", "설정 저장", ImVec2(S(200), S(64)), w::Variant::kPrimary)) {
    save();
    notify("설정 저장됨", Status::kOk, 1.5);
  }
  ImGui::SameLine();
  if (w::button("quit", "앱 종료", ImVec2(S(180), S(64)), w::Variant::kDanger, icon::kPower) && plat_.quit) plat_.quit();
  ImGui::PushFont(F().regular, fs::micro);
  ImGui::TextColored(col::v4(col::text3), "%s %s · Dear ImGui %s · 키보드 F1–F5 탭, F11 전체 화면", kAppName, kVersion, IMGUI_VERSION);
  ImGui::TextColored(col::v4(col::text3), "글꼴: Pretendard, JetBrains Mono (OFL) · 아이콘: Lucide (ISC)");
  ImGui::PopFont();
}

// ───────────────────────── connect screen ─────────────────────────

void GuiApp::draw_connect_left(ImVec2 p0, ImVec2 size) {
  ImDrawList* dl = ImGui::GetWindowDrawList();
  dl->AddRectFilled(p0, ImVec2(p0.x + size.x, p0.y + size.y), col::surface1, S(sp::radiusL));
  dl->AddRect(p0, ImVec2(p0.x + size.x, p0.y + size.y), col::border1, S(sp::radiusL), 0, S(1));
  const float x = p0.x + S(sp::s6), y0 = p0.y + S(sp::s6);
  w::text(dl, ImVec2(x, y0), F().bold, fs::display, col::text1, "로봇 연결");
  w::text(dl, ImVec2(x, y0 + S(64)), F().regular, fs::body, col::text2, "ArmPi Pro와 같은 와이파이에 연결하세요.");
  w::text(dl, ImVec2(x, y0 + S(100)), F().regular, fs::label, col::text3, "처음에는 바퀴를 띄우고 속도 10–30%로 시작합니다.");
  // address field
  const float fy = y0 + S(150);
  ImGui::SetCursorScreenPos(ImVec2(x, fy));
  ImGui::PushFont(F().mono, fs::numL);
  ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(S(sp::s4), S(sp::s3)));
  ImGui::SetNextItemWidth(S(420));
  ImGui::InputTextWithHint("##ip", "로봇 IP 주소", ip_buf_, sizeof(ip_buf_),
                           ImGuiInputTextFlags_CharsDecimal | (privacy_ ? ImGuiInputTextFlags_Password : 0));
  ImGui::PopStyleVar();
  ImGui::PopFont();
  ImGui::SameLine(0, S(sp::s3));
  if (w::button("connect", "연결", ImVec2(S(160), S(64)), w::Variant::kPrimary, icon::kLink)) connect(ip_buf_);
  ImGui::SameLine(0, S(sp::s3));
  if (w::button("discover", discovering_ ? "찾는 중" : "로봇 찾기", ImVec2(S(200), S(64)), w::Variant::kDefault, icon::kSearch, !discovering_))
    start_discovery();
  if (core_) {
    ImGui::SameLine(0, S(sp::s3));
    if (w::button("back", "돌아가기", ImVec2(S(160), S(64)), w::Variant::kGhost)) show_connect_ = false;
  }
  // keypad (the Windows touch keyboard does not appear for ImGui text fields)
  const char* keys[] = {"7", "8", "9", "4", "5", "6", "1", "2", "3", ".", "0", "⌫"};
  const ImVec2 kb(S(112), S(80));
  const float ky = fy + S(96);
  for (int i = 0; i < 12; ++i) {
    ImGui::SetCursorScreenPos(ImVec2(x + (i % 3) * (kb.x + S(sp::s2)), ky + (i / 3) * (kb.y + S(sp::s2))));
    ImGui::PushID(i);
    const bool hit = i == 11 ? w::button("k", "", kb, w::Variant::kDefault, icon::kArrowLeft) : w::button("k", keys[i], kb);
    ImGui::PopID();
    if (hit) {
      const size_t n = std::strlen(ip_buf_);
      if (i == 11) {
        if (n) ip_buf_[n - 1] = '\0';
      } else if (n + 1 < sizeof(ip_buf_)) {
        ip_buf_[n] = keys[i][0];
        ip_buf_[n + 1] = '\0';
      }
    }
  }
  ImGui::SetCursorScreenPos(ImVec2(x, ky + 4 * (kb.y + S(sp::s2))));
  if (w::button("clr", "지우기", ImVec2(kb.x * 3 + S(sp::s2) * 2, S(64)), w::Variant::kGhost)) ip_buf_[0] = '\0';
  // controls cheat sheet on the right of the keypad
  const float hx = x + kb.x * 3 + S(sp::s2) * 2 + S(56);
  float hy = ky;
  w::text(dl, ImVec2(hx, hy), F().semibold, fs::label, col::text2, "게임패드");
  hy += S(40);
  struct H {
    w::PadKey k;
    const char* t;
  };
  const H hints[] = {{w::PadKey::kLStick, "주행 · 평행이동"}, {w::PadKey::kRStick, "회전"}, {w::PadKey::kLB, "누른 채 팔 조종"},
                     {w::PadKey::kRT, "그리퍼 닫기 (LT 열기)"}, {w::PadKey::kRB, "저속"}, {w::PadKey::kB, "비상정지"},
                     {w::PadKey::kView, "0.8초: 비상정지 해제"}, {w::PadKey::kY, "영상 크게 (X 켜기·끄기)"}, {w::PadKey::kMenu, "설정"}};
  for (const auto& h : hints) {
    if (hy + S(36) > p0.y + size.y - S(sp::s4)) break;
    w::hint(dl, ImVec2(hx, hy), h.k, h.t, S(30));
    hy += S(44);
  }
}

void GuiApp::draw_connect_right(ImVec2 p0, ImVec2 size) {
  if (!begin_region("connect_right", p0, size, true)) {
    end_region();
    return;
  }
  ImDrawList* dl = ImGui::GetWindowDrawList();
  ImGui::PushFont(F().semibold, fs::title);
  ImGui::TextUnformatted("찾은 로봇");
  ImGui::PopFont();
  std::vector<net::RobotInfo> found;
  std::string derr;
  {
    std::lock_guard<std::mutex> lk(discover_mu_);
    found = discovered_;
    derr = discover_error_;
  }
  if (found.empty()) {
    ImGui::PushStyleColor(ImGuiCol_Text, col::v4(col::text2));
    ImGui::TextWrapped("%s", discovering_ ? "찾는 중…" : (derr.empty() ? "아직 없음 — '로봇 찾기'를 누르세요." : derr.c_str()));
    ImGui::PopStyleColor();
  }
  const float cw = size.x - S(4);
  for (const auto& r : found) {
    // The name comes from the network: printable, short, and no "##" (ImGui ID separator).
    std::string name = printable(r.name, 48);
    for (size_t p = name.find("##"); p != std::string::npos; p = name.find("##")) name.replace(p, 2, "#");
    ImGui::PushID((r.address + "|" + r.device_id).c_str());
    const ImVec2 cp = ImGui::GetCursorScreenPos();
    const bool hit = ImGui::InvisibleButton("card", ImVec2(cw, S(112)));
    const bool act = ImGui::IsItemActive();
    ImGui::PopID();
    dl->AddRectFilled(cp, ImVec2(cp.x + cw, cp.y + S(112)), act ? col::surface3 : col::surface2, S(sp::radiusM));
    dl->AddRect(cp, ImVec2(cp.x + cw, cp.y + S(112)), col::border1, S(sp::radiusM), 0, S(1));
    w::text(dl, ImVec2(cp.x + S(sp::s4), cp.y + S(sp::s4)), F().regular, fs::title, col::text2, icon::kBot);
    w::text(dl, ImVec2(cp.x + S(60), cp.y + S(sp::s4)), F().semibold, fs::body, col::text1, privacy_ ? "ArmPi" : name.c_str());
    w::text(dl, ImVec2(cp.x + S(60), cp.y + S(60)), F().mono, fs::micro, col::text2,
            privacy_ ? "주소·ID 숨김(화면 저장 중)"
                     : strf("%s:%d · ID %.8s", r.address.c_str(), r.control_port, r.device_id.c_str()).c_str());
    const std::string rtt = strf("%.0f", r.rtt_ms);
    const ImVec2 rs = w::text_size(F().mono_medium, fs::numL, rtt.c_str());
    w::text(dl, ImVec2(cp.x + cw - rs.x - S(sp::s4), cp.y + S(sp::s4)), F().mono_medium, fs::numL, col::text1, rtt.c_str());
    w::text(dl, ImVec2(cp.x + cw - S(sp::s4) - S(28), cp.y + S(64)), F().regular, fs::micro, col::text3, "ms");
    if (hit) {
      settings_.control_port = r.control_port;
      std::snprintf(ip_buf_, sizeof(ip_buf_), "%s", r.address.c_str());
      connect(r.address);
    }
    ImGui::Dummy(ImVec2(0, S(sp::s1)));
  }
  if (!settings_.robot_host.empty() && demo_host_.empty()) {
    ImGui::Dummy(ImVec2(0, S(sp::s3)));
    ImGui::PushFont(F().semibold, fs::label);
    ImGui::TextColored(col::v4(col::text2), "최근 연결");
    ImGui::PopFont();
    if (w::button("recent", settings_.robot_host.c_str(), ImVec2(cw, S(80)), w::Variant::kDefault, icon::kClock)) connect(settings_.robot_host);
  }
  ImGui::Dummy(ImVec2(0, S(sp::s5)));
  const ImVec2 pp = ImGui::GetCursorScreenPos();
  w::status_item(dl, pp, S(32), last_pad_.connected ? "게임패드 연결됨" : "게임패드 없음",
                 last_pad_.connected ? Status::kOk : Status::kDanger, nullptr);
  ImGui::Dummy(ImVec2(cw, S(36)));
  ImGui::PushStyleColor(ImGuiCol_Text, col::v4(col::text3));
  ImGui::TextWrapped("내장 패드는 Armoury Crate가 '게임패드' 모드일 때 잡힙니다.");
  ImGui::PopStyleColor();
  end_region();
}

}  // namespace ally::ui
