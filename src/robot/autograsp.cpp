#include "robot/autograsp.hpp"

#include <algorithm>
#include <cmath>

namespace ally::robot {

namespace {

constexpr double kDeg = kPi / 180.0;
constexpr double kTickS = 0.05;          // sequencer runs at most at the control-loop rate
constexpr double kObserveSettleS = 0.3;  // frames taken this soon after arrival may still show motion
constexpr float kTrackRadiusPx = 40.0f;
constexpr float kVerifyRadiusPx = 25.0f;
constexpr double kNudgeStepM = 0.005;
constexpr double kNudgeMaxM = 0.03;

bool is_motion(GraspState s) {
  switch (s) {
    case GraspState::kObserve:
    case GraspState::kOpenGripper:
    case GraspState::kHover:
    case GraspState::kDescend:
    case GraspState::kClose:
    case GraspState::kLift:
    case GraspState::kVerify: return true;
    default: return false;
  }
}

// Rough time for the ramp to get from one target to another, with generous slack
// (precision mode halves the rates; servos lag the commanded target).
double ramp_timeout(const ArmTarget& a, const ArmTarget& b, double rate_scale) {
  const double d = std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
  const double r = std::max(0.2, rate_scale);
  const double t = std::max({d / (0.15 * r), std::fabs(a.pitch - b.pitch) / (0.6 * r), std::fabs(a.gripper - b.gripper) / (0.8 * r)});
  return 2.0 * t + 3.0;
}

ArmTarget pose_target(const Pose5& p, double gripper) { return ArmTarget{p.x, p.y, p.z, p.pitch, gripper}; }

float median(std::vector<float> v) {
  if (v.empty()) return 0.0f;
  std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(v.size() / 2), v.end());
  return v[v.size() / 2];
}

}  // namespace

const char* const kGraspStepLabels[kGraspSteps] = {"관측", "계획", "열기", "위로", "하강", "닫기", "들기", "확인"};

const char* to_string_ko(GraspState s) {
  switch (s) {
    case GraspState::kIdle: return "대기";
    case GraspState::kSelected: return "선택됨";
    case GraspState::kObserve: return "관측";
    case GraspState::kPlan: return "계획";
    case GraspState::kConfirm: return "하강 전 확인";
    case GraspState::kOpenGripper: return "집게 열기";
    case GraspState::kHover: return "물체 위로";
    case GraspState::kDescend: return "하강";
    case GraspState::kClose: return "집게 닫기";
    case GraspState::kLift: return "들어 올리기";
    case GraspState::kVerify: return "확인";
    case GraspState::kDone: return "완료";
    case GraspState::kPaused: return "일시정지";
    case GraspState::kFailed: return "실패";
    case GraspState::kAborted: return "중단";
  }
  return "?";
}

int grasp_step_index(GraspState s) {
  switch (s) {
    case GraspState::kObserve: return 0;
    case GraspState::kPlan:
    case GraspState::kConfirm: return 1;
    case GraspState::kOpenGripper: return 2;
    case GraspState::kHover: return 3;
    case GraspState::kDescend: return 4;
    case GraspState::kClose: return 5;
    case GraspState::kLift: return 6;
    case GraspState::kVerify: return 7;
    case GraspState::kDone: return kGraspSteps;
    default: return -1;
  }
}

// ---------------------------------------------------------------------------------------------- planner

std::optional<GraspPlan> GraspPlanner::plan_at(const vision::VisionCalib& calib, double x, double y,
                                               const vision::ObjectClass& cls, std::string* err) const {
  auto no = [&](const std::string& m) -> std::optional<GraspPlan> {
    if (err) *err = m;
    return std::nullopt;
  };
  if (!calib.z_table) return no("책상 높이를 정하지 않음 — 보정 필요");
  if (!std::isfinite(x) || !std::isfinite(y) || std::fabs(x) > 1.0 || std::fabs(y) > 1.0) return no("물체 위치 계산 실패");
  const double zt = *calib.z_table;
  // Never below the table (+ margin), whatever the class depth says.
  const double zg = std::max(zt + cls.grasp_depth_m, zt + opt_.floor_margin);
  const double close = std::clamp(cls.gripper_close, 0.1, 0.9);
  for (const double deg : {-90.0, -75.0, -60.0, -50.0}) {
    const double p = deg * kDeg;
    const ArmTarget grasp{x, y, zg, p, 0.0};
    const ArmTarget hover{x, y, zg + opt_.hover_dz, p, 0.0};
    const ArmTarget lift{x, y, zg + opt_.lift_dz, p, close};
    if (!robot_.reachable(grasp) || !robot_.reachable(hover) || !robot_.reachable(lift)) continue;
    GraspPlan g;
    g.observe = pose_target(calib.observe_pose, 0.0);
    g.hover = hover;
    g.grasp = grasp;
    g.lift = lift;
    g.gripper_close = close;
    g.obj_x = x;
    g.obj_y = y;
    g.pitch_deg = deg;
    g.note = strf("pitch %.0f°", deg);
    return g;
  }
  return no(strf("팔이 닿지 않음 (x %.1f cm, y %.1f cm) — 차체를 옮기세요", x * 100.0, y * 100.0));
}

std::optional<GraspPlan> GraspPlanner::plan(const vision::VisionCalib& calib, double u, double v,
                                            const vision::ObjectClass& cls, std::string* err) const {
  double x = 0, y = 0;
  if (!calib.pixel_to_table(u, v, cls.size_m, &x, &y)) {
    if (err) *err = "보정 변환 실패 — 다시 보정";
    return std::nullopt;
  }
  return plan_at(calib, x, y, cls, err);
}

std::vector<std::pair<float, float>> GraspPlanner::reach_band_pixels(const vision::VisionCalib& calib, int samples) const {
  std::vector<std::pair<float, float>> out;
  if (!calib.H.valid || !calib.z_table || samples < 2) return out;
  const double zg = *calib.z_table + 0.012;  // a typical block grasp height
  auto ok = [&](double x, double y) {
    for (const double deg : {-90.0, -75.0, -60.0, -50.0}) {
      const double p = deg * kDeg;
      if (robot_.reachable(ArmTarget{x, y, zg, p, 0.0}) && robot_.reachable(ArmTarget{x, y, zg + opt_.hover_dz, p, 0.0}))
        return true;
    }
    return false;
  };
  std::vector<std::pair<float, float>> outer, inner;
  for (int i = 0; i < samples; ++i) {
    const double th = (-75.0 + 150.0 * i / (samples - 1)) * kDeg;
    double rmin = -1, rmax = -1;
    for (double r = 0.04; r <= 0.345; r += 0.005) {
      if (!ok(r * std::cos(th), r * std::sin(th))) continue;
      if (rmin < 0) rmin = r;
      rmax = r;
    }
    if (rmin < 0) continue;
    double u = 0, v = 0;
    if (calib.table_to_pixel(rmax * std::cos(th), rmax * std::sin(th), &u, &v) && std::isfinite(u) && std::isfinite(v))
      outer.emplace_back(static_cast<float>(u), static_cast<float>(v));
    if (calib.table_to_pixel(rmin * std::cos(th), rmin * std::sin(th), &u, &v) && std::isfinite(u) && std::isfinite(v))
      inner.emplace_back(static_cast<float>(u), static_cast<float>(v));
  }
  out = outer;
  out.insert(out.end(), inner.rbegin(), inner.rend());
  return out;
}

// -------------------------------------------------------------------------------------------- sequencer

GraspSequencer::GraspSequencer(GraspRobot& robot, GraspVision vision, EventLog& log, GraspOptions opt)
    : robot_(robot), vision_(std::move(vision)), log_(log), opt_(opt), planner_(robot, opt) {}

void GraspSequencer::set_options(const GraspOptions& o) {
  opt_ = o;
  opt_.max_retries = std::clamp(opt_.max_retries, 0, 2);
  opt_.descend_rate_scale = std::clamp(opt_.descend_rate_scale, 0.2, 1.0);
  planner_.set_options(opt_);
}

bool GraspSequencer::active() const {
  return is_motion(st_.state) || st_.state == GraspState::kPlan || st_.state == GraspState::kConfirm ||
         st_.state == GraspState::kPaused;
}

void GraspSequencer::enter(GraspState s, double now, const std::string& msg) {
  st_.state = s;
  st_.state_since = now;
  if (!msg.empty()) {
    st_.message = msg;
  } else {
    switch (s) {
      case GraspState::kSelected: st_.message = "A를 누르고 있으면 집기를 시작합니다"; break;
      case GraspState::kConfirm: st_.message = "집기 점을 확인하고 A를 다시 누르세요"; break;
      default: st_.message = std::string(to_string_ko(s)) + " 중"; break;
    }
  }
}

void GraspSequencer::fail(double now, const std::string& why) {
  robot_.cancel_goal();
  st_.failed_state = st_.state == GraspState::kPaused ? st_.resume_state : st_.state;
  ++st_.fail_events;
  log_.warn("집기 실패: " + why);
  enter(GraspState::kFailed, now, why);
}

void GraspSequencer::pause(double now, const std::string& why) {
  robot_.cancel_goal();
  if (st_.state != GraspState::kPaused) st_.resume_state = st_.state;
  enter(GraspState::kPaused, now, why + " — A를 다시 누르면 이어서 합니다");
}

bool GraspSequencer::robot_ok(const CoreSnapshot& s, std::string* why) const {
  auto no = [&](const char* m) {
    if (why) *why = m;
    return false;
  };
  if (s.emergency || s.estop_pending) return no("비상정지");
  if (!s.connected) return no("로봇 연결 끊김");
  if (s.faulted) return no("로봇 fault");
  if (s.retired || s.session_released) return no("조종 세션 없음");
  if (s.arm.needs_release) return no("팔 채널을 다시 켜야 함");
  return true;
}

const vision::Detection* GraspSequencer::find_near(const vision::DetectionSet& ds, int cls, float u, float v,
                                                   float radius) const {
  const vision::Detection* best = nullptr;
  float bd = radius;
  for (const auto& d : ds.items) {
    if (d.class_index != cls) continue;
    const float dist = std::hypot(d.u - u, d.v - v);
    if (dist <= bd) {
      bd = dist;
      best = &d;
    }
  }
  return best;
}

bool GraspSequencer::select(int track_id, double now) {
  switch (st_.state) {
    case GraspState::kIdle:
    case GraspState::kSelected:
    case GraspState::kDone:
    case GraspState::kFailed:
    case GraspState::kAborted: break;
    default: return false;  // not while the arm is busy
  }
  const auto ds = vision_.latest ? vision_.latest() : nullptr;
  if (!ds) return false;
  for (const auto& d : ds->items) {
    if (d.track_id != track_id) continue;
    st_.selected_track = d.track_id;
    st_.class_index = d.class_index;
    st_.sel_u = d.u;
    st_.sel_v = d.v;
    st_.plan.reset();
    st_.attempt = 0;
    last_seq_ = ds->frame_seq;
    missing_ = 0;
    enter(GraspState::kSelected, now);
    return true;
  }
  return false;
}

void GraspSequencer::clear_selection() {
  if (active()) return;
  st_.selected_track = -1;
  st_.class_index = -1;
  st_.plan.reset();
  st_.state = GraspState::kIdle;
  st_.message.clear();
}

void GraspSequencer::cancel() {
  if (active()) robot_.cancel_goal();
  const uint64_t done = st_.done_events, failed = st_.fail_events;
  st_ = GraspStatus{};
  st_.done_events = done;
  st_.fail_events = failed;
  st_.message = "취소함";
  base_plan_.reset();
}

void GraspSequencer::nudge(double dx, double dy, double dz) {
  if (st_.state != GraspState::kConfirm || !base_plan_) return;
  const auto cls = vision_.classes ? vision_.classes() : std::vector<vision::ObjectClass>{};
  if (st_.class_index < 0 || st_.class_index >= static_cast<int>(cls.size())) return;
  const double nx = std::clamp(nudge_x_ + std::clamp(dx, -kNudgeStepM, kNudgeStepM), -kNudgeMaxM, kNudgeMaxM);
  const double ny = std::clamp(nudge_y_ + std::clamp(dy, -kNudgeStepM, kNudgeStepM), -kNudgeMaxM, kNudgeMaxM);
  const double nz = std::clamp(nudge_z_ + std::clamp(dz, -kNudgeStepM, kNudgeStepM), -kNudgeMaxM, kNudgeMaxM);
  vision::ObjectClass c = cls[static_cast<size_t>(st_.class_index)];
  c.grasp_depth_m += nz;  // plan_at still keeps it above the table floor
  std::string err;
  auto p = planner_.plan_at(calib_, base_plan_->obj_x + nx, base_plan_->obj_y + ny, c, &err);
  if (!p) {
    st_.message = "그 위치는 팔이 닿지 않습니다";
    return;
  }
  nudge_x_ = nx;
  nudge_y_ = ny;
  nudge_z_ = nz;
  st_.plan = p;
  st_.message = strf("집기 점 조정: x %+.0f mm, y %+.0f mm, z %+.0f mm — A를 다시 누르세요", nx * 1000, ny * 1000, nz * 1000);
}

void GraspSequencer::confirm(double now) {
  if (st_.state != GraspState::kConfirm || !st_.plan) return;
  log_.info(strf("집기: 물체 x %.1f cm, y %.1f cm, %s", st_.plan->obj_x * 100, st_.plan->obj_y * 100,
                 st_.plan->note.c_str()));
  goal_retried_ = false;
  enter(GraspState::kOpenGripper, now);
  send_step(now);
}

void GraspSequencer::send_step(double now) {
  const CoreSnapshot s = robot_.snapshot();
  const ArmTarget cur = s.target;
  ArmTarget goal = cur;
  double rate = 1.0;
  goal_id_ = 0;
  reached_at_ = -1;
  switch (st_.state) {
    case GraspState::kObserve:
      goal = pose_target(calib_.observe_pose, cur.gripper);
      samples_.clear();
      goal_id_ = robot_.arm_goto(goal, rate);
      break;
    case GraspState::kOpenGripper:
      goal.gripper = 0.0;
      goal_id_ = robot_.set_gripper(0.0);
      break;
    case GraspState::kHover:
      goal = st_.plan->hover;
      goal_id_ = robot_.arm_goto(goal, rate);
      break;
    case GraspState::kDescend:
      goal = st_.plan->grasp;
      if (goal_retried_) goal.z += 0.005;
      rate = opt_.descend_rate_scale;
      goal_id_ = robot_.arm_goto(goal, rate);
      break;
    case GraspState::kClose:
      goal.gripper = st_.plan->gripper_close;
      goal_id_ = robot_.set_gripper(goal.gripper);
      break;
    case GraspState::kLift:
      goal = st_.plan->lift;
      if (goal_retried_) goal.z = st_.plan->grasp.z + 0.05;
      rate = opt_.descend_rate_scale;
      goal_id_ = robot_.arm_goto(goal, rate);
      break;
    case GraspState::kVerify:
      goal = pose_target(calib_.observe_pose, st_.plan ? st_.plan->gripper_close : cur.gripper);
      verify_frames_ = verify_hits_ = 0;
      goal_id_ = robot_.arm_goto(goal, rate);
      break;
    default: return;
  }
  goal_sent_at_ = now;
  goal_timeout_s_ = ramp_timeout(cur, goal, rate);
  if (goal_id_ == 0) fail(now, "팔 명령이 거부됨 — 연결·세션·팔 상태를 확인하세요");
}

void GraspSequencer::track_selection(double now) {
  if (st_.selected_track < 0) return;
  const auto ds = vision_.latest ? vision_.latest() : nullptr;
  if (!ds || ds->frame_seq == last_seq_) return;
  last_seq_ = ds->frame_seq;
  const vision::Detection* hit = nullptr;
  for (const auto& d : ds->items)
    if (d.track_id == st_.selected_track) hit = &d;
  if (!hit) hit = find_near(*ds, st_.class_index, st_.sel_u, st_.sel_v, kTrackRadiusPx);
  if (hit) {
    st_.selected_track = hit->track_id;
    st_.sel_u = hit->u;
    st_.sel_v = hit->v;
    missing_ = 0;
    return;
  }
  if (++missing_ < 3) return;
  st_.selected_track = -1;
  st_.class_index = -1;
  if (st_.state == GraspState::kSelected) enter(GraspState::kIdle, now, "선택한 물체가 화면에서 사라졌습니다");
}

void GraspSequencer::step_goal(double now, const CoreSnapshot& s) {
  if (reached_at_ >= 0) return;  // waiting phase of observe / close / verify
  if (s.goal_id < goal_id_) {    // snapshot not caught up yet
    if (now - goal_sent_at_ > goal_timeout_s_) fail(now, std::string(to_string_ko(st_.state)) + " 시간 초과");
    return;
  }
  if (s.goal_id > goal_id_) {
    pause(now, "다른 팔 명령이 들어와 멈춤");
    return;
  }
  switch (s.goal_result) {
    case GoalResult::kNone:
      if (now - goal_sent_at_ > goal_timeout_s_) fail(now, std::string(to_string_ko(st_.state)) + " 시간 초과");
      return;
    case GoalResult::kCancelled: pause(now, "팔을 직접 조작해 멈춤"); return;
    case GoalResult::kStalled:
      if ((st_.state == GraspState::kDescend || st_.state == GraspState::kLift) && !goal_retried_) {
        goal_retried_ = true;
        log_.info(std::string("집기: ") + to_string_ko(st_.state) + " 목표에 닿지 않아 높이를 바꿔 한 번 더");
        send_step(now);
        return;
      }
      fail(now, std::string(to_string_ko(st_.state)) + " 자세에 팔이 닿지 않음");
      return;
    case GoalResult::kReached: break;
  }
  switch (st_.state) {
    case GraspState::kObserve:
    case GraspState::kVerify:
    case GraspState::kClose: reached_at_ = now; return;  // their own waiting phase follows
    case GraspState::kOpenGripper:
      goal_retried_ = false;
      enter(GraspState::kHover, now);
      send_step(now);
      return;
    case GraspState::kHover:
      goal_retried_ = false;
      enter(GraspState::kDescend, now);
      send_step(now);
      return;
    case GraspState::kDescend:
      enter(GraspState::kClose, now);
      send_step(now);
      return;
    case GraspState::kLift:
      enter(GraspState::kVerify, now);
      send_step(now);
      return;
    default: return;
  }
}

void GraspSequencer::step_observe(double now, const CoreSnapshot& s) {
  if (reached_at_ < 0) {
    step_goal(now, s);
    if (reached_at_ < 0) return;
  }
  const auto ds = vision_.latest ? vision_.latest() : nullptr;
  if (ds && ds->frame_seq != last_seq_ && ds->frame_mono >= reached_at_ + kObserveSettleS) {
    last_seq_ = ds->frame_seq;
    const vision::Detection* d = find_near(*ds, st_.class_index, st_.sel_u, st_.sel_v, kTrackRadiusPx);
    if (!d) {
      // The image can shift between the selection pose and the observe pose (camera on the arm):
      // accept the only object of that class if there is exactly one.
      int n = 0;
      for (const auto& it : ds->items)
        if (it.class_index == st_.class_index) {
          ++n;
          d = &it;
        }
      if (n != 1) d = nullptr;
    }
    if (d) samples_.emplace_back(d->u, d->v);
  }
  const bool enough = static_cast<int>(samples_.size()) >= std::max(1, opt_.observe_frames);
  const bool late = now - reached_at_ > opt_.observe_s + kObserveSettleS;
  if (!enough && !late) return;
  if (samples_.empty()) {
    fail(now, ds ? "관측 자세에서 물체를 찾지 못함" : "카메라 영상 없음");
    return;
  }
  std::vector<float> us, vs;
  for (const auto& p : samples_) {
    us.push_back(p.first);
    vs.push_back(p.second);
  }
  st_.orig_u = st_.sel_u = median(us);
  st_.orig_v = st_.sel_v = median(vs);
  enter(GraspState::kPlan, now);
  step_plan(now);
}

void GraspSequencer::step_plan(double now) {
  const auto cls = vision_.classes ? vision_.classes() : std::vector<vision::ObjectClass>{};
  if (st_.class_index < 0 || st_.class_index >= static_cast<int>(cls.size())) {
    fail(now, "물체 종류 정보가 바뀜 — 다시 선택하세요");
    return;
  }
  std::string err;
  const auto p = planner_.plan(calib_, st_.orig_u, st_.orig_v, cls[static_cast<size_t>(st_.class_index)], &err);
  if (!p) {
    fail(now, err);
    return;
  }
  st_.plan = p;
  base_plan_ = p;
  nudge_x_ = nudge_y_ = nudge_z_ = 0.0;
  if (opt_.confirm_before_descend) {
    enter(GraspState::kConfirm, now);
    return;
  }
  st_.state = GraspState::kConfirm;  // no confirmation wanted: go straight on
  confirm(now);
}

void GraspSequencer::step_verify(double now, const CoreSnapshot& s) {
  if (reached_at_ < 0) {
    step_goal(now, s);
    if (reached_at_ < 0) return;
  }
  const auto ds = vision_.latest ? vision_.latest() : nullptr;
  if (ds && ds->frame_seq != last_seq_ && ds->frame_mono >= reached_at_ + kObserveSettleS) {
    last_seq_ = ds->frame_seq;
    ++verify_frames_;
    if (find_near(*ds, st_.class_index, st_.orig_u, st_.orig_v, kVerifyRadiusPx)) ++verify_hits_;
  }
  const bool enough = verify_frames_ >= std::max(1, opt_.observe_frames);
  const bool late = now - reached_at_ > opt_.observe_s + kObserveSettleS;
  if (!enough && !late) return;
  if (verify_frames_ == 0) {
    fail(now, "들어 올렸지만 카메라 영상이 없어 확인하지 못함");
    return;
  }
  if (verify_hits_ * 2 <= verify_frames_) {
    ++st_.done_events;
    log_.info("집기 완료: 물체를 들어 올림");
    enter(GraspState::kDone, now, "완료 — 물체를 들고 있습니다. 내려놓을 때는 팔 탭에서 집게를 여세요");
    return;
  }
  if (st_.attempt < opt_.max_retries) {
    ++st_.attempt;
    log_.info(strf("집기: 물체가 그대로 있음 — 다시 시도 %d/%d", st_.attempt, opt_.max_retries));
    goal_retried_ = false;
    st_.sel_u = st_.orig_u;
    st_.sel_v = st_.orig_v;
    enter(GraspState::kObserve, now, strf("다시 시도 %d/%d", st_.attempt, opt_.max_retries));
    send_step(now);
    return;
  }
  fail(now, "집지 못함 — 물체가 제자리에 있습니다");
}

void GraspSequencer::tick(double now, bool deadman) {
  if (deadman && !prev_deadman_) pending_rise_ = true;
  if (!deadman) pending_release_ = true;
  prev_deadman_ = deadman;
  if (now - last_tick_ < kTickS) return;
  last_tick_ = now;
  const bool rise = pending_rise_;
  const bool released = pending_release_;
  pending_rise_ = pending_release_ = false;

  const GraspState state = st_.state;
  if (state == GraspState::kIdle) return;
  if (state == GraspState::kSelected || state == GraspState::kFailed || state == GraspState::kAborted ||
      state == GraspState::kDone) {
    track_selection(now);
    if (!rise || st_.selected_track < 0 || state == GraspState::kDone) return;
    // Start (or start over): needs a usable calibration and a healthy robot.
    const auto ds = vision_.latest ? vision_.latest() : nullptr;
    std::string why;
    if (!calib_.usable(ds ? ds->img_w : 0, ds ? ds->img_h : 0, &why)) {
      st_.message = "보정 필요: " + why;
      return;
    }
    if (!robot_ok(robot_.snapshot(), &why)) {
      st_.message = "시작할 수 없음: " + why;
      return;
    }
    st_.attempt = 0;
    st_.plan.reset();
    st_.failed_state = GraspState::kIdle;
    goal_retried_ = false;
    log_.info("집기: 시작");
    enter(GraspState::kObserve, now);
    send_step(now);
    return;
  }

  const CoreSnapshot s = robot_.snapshot();
  std::string why;
  if (!robot_ok(s, &why)) {
    robot_.cancel_goal();
    st_.failed_state = state == GraspState::kPaused ? st_.resume_state : state;
    ++st_.fail_events;
    log_.warn("집기 중단: " + why);
    enter(GraspState::kAborted, now, "중단: " + why);
    return;
  }

  if (state == GraspState::kPaused) {
    if (!rise) return;
    const GraspState back = st_.resume_state;
    enter(back, now);
    if (back == GraspState::kPlan) step_plan(now);
    else if (is_motion(back)) send_step(now);
    return;
  }
  if (state == GraspState::kConfirm) {
    if (rise) {
      confirm(now);
    } else if (now - st_.state_since > opt_.confirm_timeout_s) {
      pause(now, "확인을 오래 기다려 멈춤");
    }
    return;
  }
  // Motion steps (and plan) need the dead-man held the whole time.
  if (released || !deadman) {
    pause(now, "A를 놓아 멈춤");
    return;
  }
  switch (state) {
    case GraspState::kObserve: step_observe(now, s); break;
    case GraspState::kPlan: step_plan(now); break;
    case GraspState::kVerify: step_verify(now, s); break;
    case GraspState::kClose:
      if (reached_at_ < 0) {
        step_goal(now, s);
      } else if (now - reached_at_ >= opt_.close_settle_s) {
        goal_retried_ = false;
        enter(GraspState::kLift, now);
        send_step(now);
      }
      break;
    default: step_goal(now, s); break;
  }
}

}  // namespace ally::robot
