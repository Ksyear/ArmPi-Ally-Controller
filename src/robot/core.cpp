#include "robot/core.hpp"

#include <chrono>
#include <cmath>

#include "armpi_common/json.hpp"
#include "armpi_kinematics/reachability.hpp"
#include "net/udp_link.hpp"

namespace ally::robot {

namespace json = armpi_common::json;
namespace kin = armpi_kinematics;

namespace {

constexpr double kLoopPeriodS = armpi_protocol::kClientSendPeriodS;  // 50 ms, 20 Hz per channel
constexpr double kInputTimeoutS = 0.3;
constexpr double kEps = 1e-3;
constexpr double kAutoMaxF = 0.30, kAutoMaxT = 0.35;  // autograsp base approach, stick units
constexpr double kAutoMaxSpeed = 0.30;                // ... only while the speed limit is at most 30 %

// Link lengths from the robot config (Hiwonder-published ArmPi values). Joint limits are left open:
// the per-robot hardware profile is not available here, so the robot's own nearest rule stays authoritative.
kin::Params arm_params() {
  kin::Params p;
  p.base_height_m = 0.064605;
  p.upper_arm_m = 0.10048;
  p.forearm_m = 0.094714;
  p.hand_m = 0.16331;
  p.min_wrist_height_m = 0.0;
  p.min_elbow_bend_rad = 0.05;
  return p;
}

const kin::Workspace kWorkspace{0.05, 0.32, -0.25, 0.25, -0.08, 0.32};
constexpr double kMaxFollowJumpM = 0.05;      // a real nearest-adjustment stays near our own target
constexpr double kMaxFollowPitchRad = 0.6;

std::optional<double> num(const json::Value& o, const char* k) {
  const auto* v = o.find(k);
  if (v && v->is_number() && v->number_finite) return v->number;
  return std::nullopt;
}

double clamp1(double v) { return std::isfinite(v) ? clampd(v, -1.0, 1.0) : 0.0; }

// A pose reported over rosbridge is untrusted: finite, inside the workspace box (1 cm slack), sane pitch/gripper.
bool plausible_pose(double x, double y, double z, double pitch, double gripper) {
  constexpr double kSlack = 0.01;
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && std::isfinite(pitch) && std::isfinite(gripper) &&
         x >= kWorkspace.x_min - kSlack && x <= kWorkspace.x_max + kSlack && y >= kWorkspace.y_min - kSlack &&
         y <= kWorkspace.y_max + kSlack && z >= kWorkspace.z_min - kSlack && z <= kWorkspace.z_max + kSlack &&
         std::fabs(pitch) <= kPi && gripper >= -0.01 && gripper <= 1.01;
}

double distance3(const ArmTarget& a, double x, double y, double z) {
  return std::sqrt((a.x - x) * (a.x - x) + (a.y - y) * (a.y - y) + (a.z - z) * (a.z - z));
}

}  // namespace

RobotCore::RobotCore(CoreOptions opt, EventLog& log, net::CommLog& comm)
    : opt_(std::move(opt)), log_(log), comm_(comm), ros_(log, &comm), camera_(log), proj_(make_reach_projector()) {
  speed_ = clampd(opt_.speed, 0.1, 1.0);
}

std::shared_ptr<kin::TargetProjector> make_reach_projector() {
  return std::make_shared<kin::TargetProjector>(arm_params(), kWorkspace, kin::ReachOptions{});
}

RobotCore::~RobotCore() { stop(); }

std::string RobotCore::camera_url() const {
  return opt_.camera_url.empty() ? default_camera_url(opt_.host, opt_.camera_port) : opt_.camera_url;
}

bool RobotCore::start(std::string* err) {
  if (running_) return true;
  auto link = net::UdpLink::open(opt_.host, opt_.control_port, &comm_, err);
  if (!link) return false;
  armpi_protocol::ClientOptions co;
  co.ack_timeout_s = opt_.ack_timeout_s;
  co.home = opt_.home;
  co.log = [this](const std::string& m) { log_.info(m); };
  if (opt_.client_filter) {
    auto proj = proj_;
    co.arm_target_filter = [proj, this](const ArmTarget& want, const ArmTarget& cur) {
      const kin::Pose anchor{cur.x, cur.y, cur.z, cur.pitch};
      const bool anchor_ok = proj->reachable(anchor);
      const auto r = proj->project(kin::Pose{want.x, want.y, want.z, want.pitch}, anchor_ok ? &anchor : nullptr);
      if (!r.ok) return want;  // let the robot decide
      if (r.adjust != kin::kAdjustNone) filter_adjusted_ = true;
      return ArmTarget{r.pose.x, r.pose.y, r.pose.z, r.pose.pitch, want.gripper};
    };
  }
  client_ = std::make_shared<armpi_protocol::V3Client>(std::move(link), co);
  log_.info("조종 세션 시작: " + opt_.host + ":" + std::to_string(opt_.control_port));
  if (opt_.rosbridge_port > 0) ros_.start(opt_.host, opt_.rosbridge_port);
  stop_ = false;
  running_ = true;
  thread_ = std::thread([this] { loop(); });
  return true;
}

void RobotCore::stop() {
  if (!running_) return;
  stop_ = true;
  if (thread_.joinable()) thread_.join();
  running_ = false;
  if (client_ && !client_->retired()) {
    client_->disconnect();  // two neutrals + disconnect
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    client_->poll();
    log_.info("조종 세션 종료(disconnect)");
  }
  camera_.stop();
  ros_.stop();
}

void RobotCore::loop() {
  using clk = std::chrono::steady_clock;
  const auto period = std::chrono::duration_cast<clk::duration>(std::chrono::duration<double>(kLoopPeriodS));
  auto next = clk::now();
  double last = -1.0;
  while (!stop_) {
    const double now = mono_now();
    step(now);
    update_snapshot(now);
    if (last > 0 && now - last > 1e-3) {
      const double hz = 1.0 / (now - last);
      std::lock_guard<std::mutex> lk(mu_);
      loop_hz_ = loop_hz_ <= 0 ? hz : loop_hz_ * 0.9 + hz * 0.1;
    }
    last = now;
    next += period;
    if (next < clk::now()) {
      std::lock_guard<std::mutex> lk(mu_);
      ++overruns_;
      next = clk::now() + period;
    }
    while (!stop_ && clk::now() < next) {
      client_->poll();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
}

void RobotCore::bump_limit(double now) {
  std::lock_guard<std::mutex> lk(mu_);
  limit_until_ = now + 0.5;
  if (now - last_limit_cue_ > 0.4) {
    ++cues_.limit;
    last_limit_cue_ = now;
  }
}

void RobotCore::step(double now) {
  ControlInput in;
  bool fresh, arm_on, released;
  double speed;
  bool home_now = false;
  std::optional<ArmTarget> goal;
  std::optional<double> goal_grip;
  uint64_t gid = 0;
  double rate = 1.0;
  bool auto_on = false;
  double auto_f = 0.0, auto_t = 0.0;
  {
    std::lock_guard<std::mutex> lk(mu_);
    fresh = now - input_time_ < kInputTimeoutS;
    in = fresh ? input_ : ControlInput{};
    // The user's own drive input always wins over (and ends) the autograsp base approach.
    if (std::fabs(in.forward) > kEps || std::fabs(in.strafe) > kEps || std::fabs(in.turn) > kEps) auto_until_ = -1.0;
    auto_on = fresh && now < auto_until_ && speed_ <= kAutoMaxSpeed + 1e-9;
    auto_f = auto_f_;
    auto_t = auto_t_;
    arm_on = arm_enabled_;
    released = released_;
    speed = speed_;
    goal = goal_;
    goal_grip = goal_grip_;
    gid = goal_id_;
    rate = goal_rate_scale_;
    if (home_due_ > 0 && now >= home_due_) {
      home_now = true;
      home_due_ = -1.0;
    }
  }
  const double fine = in.precision ? opt_.precision_scale : 1.0;
  double f = clamp1(in.forward) * speed * fine;
  double s = clamp1(in.strafe) * speed * fine;
  double t = clamp1(in.turn) * speed * fine;
  if (auto_on) {
    f = clampd(auto_f, -kAutoMaxF, kAutoMaxF) * speed * fine;
    s = 0.0;
    t = clampd(auto_t, -kAutoMaxT, kAutoMaxT) * speed * fine;
  }
  const bool base_intent = fresh && (std::fabs(f) > kEps || std::fabs(s) > kEps || std::fabs(t) > kEps);
  const bool arm_intent = fresh && arm_on;
  bool any_arm = false;
  for (double a : in.arm) any_arm = any_arm || std::fabs(a) > kEps;

  armpi_protocol::ChannelStatus arm_ch;
  bool retired;
  bool goal_done = false, arm_activity = false, goal_user_cancel = false;
  double goal_dist = -1.0;  // remaining distance to the goal after this tick (m, pitch/gripper weighted)
  if (!released) {
    std::lock_guard<std::recursive_mutex> ck(client_->mutex());  // target move + tick as one unit
    const auto& r = opt_.rates;
    const double k = kLoopPeriodS * fine;
    if (arm_intent && client_->arm().armed && any_arm) {
      // Stick/button rates win over a pending absolute goal.
      goal_user_cancel = goal.has_value() || goal_grip.has_value();
      goal.reset();
      goal_grip.reset();
      goal_done = true;
      arm_activity = true;
      client_->move_arm_target(clamp1(in.arm[0]) * r.arm_m_s * k, clamp1(in.arm[1]) * r.arm_m_s * k,
                               clamp1(in.arm[2]) * r.arm_m_s * k, clamp1(in.arm[3]) * r.pitch_rad_s * k,
                               clamp1(in.arm[4]) * r.gripper_per_s * k);
    } else if (arm_intent && client_->arm().armed && (goal || goal_grip)) {
      // Ramp toward the absolute goal at the same limits as the sticks (no jumps).
      arm_activity = true;
      const ArmTarget cur = client_->arm_target();
      ArmTarget next = cur;
      const double kg = k * rate;  // slower ramps for e.g. the grasp descent
      if (goal) {
        const double dx = goal->x - cur.x, dy = goal->y - cur.y, dz = goal->z - cur.z;
        const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        const double maxd = r.arm_m_s * kg;
        const double f = dist > maxd ? maxd / dist : 1.0;
        next.x = cur.x + dx * f;
        next.y = cur.y + dy * f;
        next.z = cur.z + dz * f;
        next.pitch = cur.pitch + clampd(goal->pitch - cur.pitch, -r.pitch_rad_s * kg, r.pitch_rad_s * kg);
      }
      if (goal_grip) next.gripper = cur.gripper + clampd(*goal_grip - cur.gripper, -r.gripper_per_s * kg, r.gripper_per_s * kg);
      client_->set_arm_target(next);
      const ArmTarget after = client_->arm_target();
      const bool pos_ok = !goal || (std::fabs(after.x - goal->x) < 1e-3 && std::fabs(after.y - goal->y) < 1e-3 &&
                                    std::fabs(after.z - goal->z) < 1e-3 && std::fabs(after.pitch - goal->pitch) < 0.009);
      const bool grip_ok = !goal_grip || std::fabs(after.gripper - *goal_grip) < 1e-3;
      if (pos_ok && grip_ok) {
        goal_done = true;
      } else {
        goal_dist = 0.0;
        if (goal) {
          const double ex = after.x - goal->x, ey = after.y - goal->y, ez = after.z - goal->z;
          goal_dist += std::sqrt(ex * ex + ey * ey + ez * ez) + 0.05 * std::fabs(after.pitch - goal->pitch);
        }
        if (goal_grip) goal_dist += 0.01 * std::fabs(after.gripper - *goal_grip);
      }
    }
    try {
      client_->tick(base_intent, f, s, t, arm_intent);
    } catch (const std::exception& e) {
      client_->set_last_error(e.what());
    }
    arm_ch = client_->arm();
    retired = client_->retired();
  } else {
    arm_ch = client_->arm();
    retired = true;
  }
  if (filter_adjusted_.exchange(false)) bump_limit(now);

  // Follow the robot when it reports a nearest-adjustment: pull our target back to what it really uses,
  // so a held stick cannot run the target far outside the reachable space.
  // A genuine adjustment is always close to our own target; anything farther (or implausible) is ignored,
  // so forged rosbridge data cannot drag the arm somewhere else.
  if (opt_.follow_robot_adjust && arm_intent && ros_.connected()) {
    const auto bs = ros_.bridge_state();
    if (bs.valid && bs.arm_adjust_total > last_adjust_total_) {
      const bool first = last_adjust_total_ == 0;
      last_adjust_total_ = bs.arm_adjust_total;
      if (!first) {
        if (const auto applied = ros_.applied_arm_target()) {
          const ArmTarget cur = client_->arm_target();
          if (plausible_pose(applied->x, applied->y, applied->z, applied->pitch, applied->gripper) &&
              distance3(cur, applied->x, applied->y, applied->z) <= kMaxFollowJumpM &&
              std::fabs(applied->pitch - cur.pitch) <= kMaxFollowPitchRad) {
            client_->set_arm_target(applied->x, applied->y, applied->z, applied->pitch, cur.gripper);
            bump_limit(now);
          } else if (now - last_ignored_log_ > 2.0) {
            last_ignored_log_ = now;
            log_.warn("팔: 로봇이 알린 보정 목표가 내 목표와 너무 멀어 무시함");
          }
        }
      }
    }
  }

  bool idle_off = false, stalled_now = false;
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (arm_activity) last_arm_activity_ = now;
    // Only touch the goal this tick worked on: the UI may have started a new one meanwhile.
    if (goal_id_ == gid) {
      if (goal_done) {
        if (goal_.has_value() || goal_grip_.has_value()) goal_result_ = goal_user_cancel ? GoalResult::kCancelled : GoalResult::kReached;
        goal_.reset();
        goal_grip_.reset();
        goal_stall_since_ = -1.0;
        goal_remaining_ = goal_user_cancel ? goal_remaining_ : 0.0;
      } else if (goal_dist >= 0.0) {
        goal_remaining_ = goal_dist;
        // No progress for 0.5 s = the reach filter holds the target at the nearest reachable pose.
        if (goal_stall_since_ < 0 || goal_dist < goal_best_ - 5e-4) {
          goal_best_ = goal_dist;
          goal_stall_since_ = now;
        } else if (now - goal_stall_since_ > 0.5) {
          goal_.reset();
          goal_grip_.reset();
          goal_stall_since_ = -1.0;
          goal_result_ = GoalResult::kStalled;
          log_.info(strf("팔: 목표에 닿을 수 없어 가장 가까운 자세에서 멈춤(남은 거리 %.0f mm)", goal_dist * 1000));
          stalled_now = true;
        }
      }
    }
    if (arm_enabled_ && arm_latched_ && now - last_arm_activity_ > opt_.latched_arm_idle_s) {
      arm_enabled_ = false;
      arm_latched_ = false;
      idle_off = true;
      cancel_goal_locked();
    }
  }
  if (stalled_now) bump_limit(now);
  if (idle_off) log_.info(strf("팔 조종 토글: %.0f초 동안 입력이 없어 자동으로 끔", opt_.latched_arm_idle_s));
  if (arm_on && (arm_ch.needs_release || retired)) {
    {
      std::lock_guard<std::mutex> lk(mu_);
      arm_enabled_ = false;
      arm_latched_ = false;
      cancel_goal_locked();
    }
    log_.warn("팔 조종 꺼짐(" + (arm_ch.last_block_reason.empty() ? std::string("세션 없음") : arm_ch.last_block_reason) +
              ") — 다시 켜야 함");
  }
  if (home_now) {
    std::string err;
    if (!ros_.call_trigger("/armpi/arm/home", &err)) log_.warn("기본 자세 요청 실패: " + err);
  }
  {
    std::lock_guard<std::mutex> lk(mu_);
    snap_.applied_f = base_intent ? f : 0.0;
    snap_.applied_s = base_intent ? s : 0.0;
    snap_.applied_t = base_intent ? t : 0.0;
    snap_.input_fresh = fresh;
    snap_.precision = in.precision;
  }
}

void RobotCore::update_snapshot(double now) {
  std::string sj;
  armpi_protocol::ChannelStatus base, arm;
  ArmTarget target;
  bool retired;
  {
    std::lock_guard<std::recursive_mutex> ck(client_->mutex());
    sj = client_->snapshot_json();
    base = client_->base();
    arm = client_->arm();
    target = client_->arm_target();
    retired = client_->retired();
  }
  json::Value v;
  std::string err;
  json::parse(sj, v, err, json::lenient_topic_options());
  auto b = [&](const char* k) {
    const auto* f = v.find(k);
    return f && f->is_bool() && f->boolean;
  };
  auto s = [&](const char* k) {
    const auto* f = v.find(k);
    return f && f->is_string() ? f->string : std::string();
  };
  auto u = [&](const char* k) { return static_cast<uint64_t>(num(v, k).value_or(0)); };
  std::optional<ArmTarget> accepted;
  if (const auto* a = v.find("arm_last_accepted"); a && a->is_object()) {
    accepted = ArmTarget{num(*a, "x").value_or(0), num(*a, "y").value_or(0), num(*a, "z").value_or(0),
                         num(*a, "pitch").value_or(0), num(*a, "gripper").value_or(0)};
  }
  std::string reject_key;
  if (const auto* r = v.find("arm_last_reject"); r && r->is_object()) {
    const auto* rs = r->find("reason");
    const auto seq = num(*r, "sequence");
    reject_key = (rs && rs->is_string() ? rs->string : "") + "#" + std::to_string(static_cast<long long>(seq.value_or(-1)));
  }

  std::lock_guard<std::mutex> lk(mu_);
  CoreSnapshot& n = snap_;
  n.robot = s("robot");
  n.client_id = s("client_id");
  n.connected = b("connected");
  n.emergency = b("emergency");
  n.estop_pending = b("estop_pending");
  n.faulted = b("faulted");
  n.retired = retired;
  n.session_released = released_;
  n.base = base;
  n.arm = arm;
  n.target = target;
  n.last_accepted = accepted;
  n.ack_ms_last = num(v, "ack_ms_last");
  n.ack_ms_p50 = num(v, "ack_ms_p50");
  n.ack_ms_max = num(v, "ack_ms_max");
  n.packets_sent = u("packets_sent");
  n.acks = u("acks");
  n.late_acks = u("late_acks");
  n.send_errors = u("send_errors");
  n.last_error = s("last_error");
  n.arm_enabled = arm_enabled_;
  n.arm_latched = arm_enabled_ && arm_latched_;
  n.goal_active = goal_.has_value() || goal_grip_.has_value();
  n.auto_base_active = now < auto_until_;
  n.goal_result = goal_result_;
  n.goal_id = goal_id_;
  n.goal_remaining_m = goal_remaining_;
  n.speed = speed_;
  n.limit_active = now < limit_until_;
  n.loop_hz = loop_hz_;
  n.loop_overruns = overruns_;
  // Cues from state transitions
  if (n.emergency && !was_emergency_) ++cues_.estop;
  was_emergency_ = n.emergency;
  if (!n.connected && was_connected_ && !released_) ++cues_.link_lost;
  if (n.connected && !was_connected_) ++cues_.link_ok;
  was_connected_ = n.connected;
  if (!reject_key.empty() && reject_key != last_reject_key_) {
    ++cues_.rejected;
    last_reject_key_ = reject_key;
  }
  n.cues = cues_;
}

void RobotCore::set_input(const ControlInput& in) {
  std::lock_guard<std::mutex> lk(mu_);
  input_ = in;
  input_time_ = mono_now();
}

void RobotCore::clear_input() {
  std::lock_guard<std::mutex> lk(mu_);
  input_ = ControlInput{};
  input_time_ = -1e9;
  auto_until_ = -1.0;
}

bool RobotCore::set_auto_base(double forward, double turn, double ttl_s) {
  std::lock_guard<std::mutex> lk(mu_);
  const bool stop = !std::isfinite(forward) || !std::isfinite(turn) || !std::isfinite(ttl_s) || ttl_s <= 0 ||
                    (std::fabs(forward) < 1e-6 && std::fabs(turn) < 1e-6);
  if (stop || released_ || speed_ > kAutoMaxSpeed + 1e-9) {
    auto_until_ = -1.0;
    return stop;
  }
  auto_f_ = clampd(forward, -kAutoMaxF, kAutoMaxF);
  auto_t_ = clampd(turn, -kAutoMaxT, kAutoMaxT);
  auto_until_ = mono_now() + std::min(ttl_s, 0.5);
  return true;
}

void RobotCore::set_speed(double sp) {
  std::lock_guard<std::mutex> lk(mu_);
  speed_ = clampd(sp, 0.1, 1.0);
}

double RobotCore::speed() const {
  std::lock_guard<std::mutex> lk(mu_);
  return speed_;
}

bool RobotCore::arm_enabled() const {
  std::lock_guard<std::mutex> lk(mu_);
  return arm_enabled_;
}

void RobotCore::arm_sync() {
  if (!client_) return;
  if (!ros_.status_fresh(0.5)) {
    log_.info("팔: 로봇 현재값을 몰라 마지막 목표를 그대로 씀(rosbridge 상태 없음)");
    return;
  }
  const auto st = ros_.status();
  if (!st.arm_current) return;
  const auto& c = *st.arm_current;
  if (!plausible_pose(c.x, c.y, c.z, c.pitch, c.gripper)) {
    log_.warn("팔: 로봇이 보고한 현재 자세가 작업 공간 밖이라 무시함");
    return;
  }
  const ArmTarget before = client_->arm_target();
  const double jump = distance3(before, c.x, c.y, c.z);
  // Remote state is never marked "accepted": only robot ACKs do that.
  client_->set_arm_target(c.x, c.y, c.z, c.pitch, c.gripper, false);
  if (jump > kMaxFollowJumpM)
    log_.warn(strf("팔: 목표를 로봇 현재 자세로 %.0f mm 옮김 — 화면과 실제 자세가 같은지 확인", jump * 1000));
  else
    log_.info(strf("팔: 목표를 로봇 현재값으로 맞춤 x=%.3f y=%.3f z=%.3f", c.x, c.y, c.z));
}

void RobotCore::set_arm_enabled(bool on, bool latched) {
  bool was;
  {
    std::lock_guard<std::mutex> lk(mu_);
    was = arm_enabled_;
    if (on && released_) {
      log_.warn("세션을 놓은 상태라 팔을 켤 수 없음 — 먼저 '새 세션'");
      return;
    }
  }
  if (on && !was) arm_sync();
  std::lock_guard<std::mutex> lk(mu_);
  if (on && !was) {
    ++cues_.arm_on;
    last_arm_activity_ = mono_now();
  }
  if (!on) {
    cancel_goal_locked();
  }
  arm_enabled_ = on;
  arm_latched_ = on && latched;
}

// Caller holds mu_. Starts a fresh goal: new id, no result yet, stall tracking reset.
uint64_t RobotCore::new_goal_locked(double rate_scale) {
  ++goal_id_;
  goal_result_ = GoalResult::kNone;
  goal_rate_scale_ = clampd(rate_scale, 0.2, 1.0);
  goal_stall_since_ = -1.0;
  goal_remaining_ = -1.0;
  last_arm_activity_ = mono_now();
  return goal_id_;
}

uint64_t RobotCore::arm_goto(const ArmTarget& t, double rate_scale) {
  if (!client_) return 0;
  if (!arm_enabled()) set_arm_enabled(true, true);
  std::lock_guard<std::mutex> lk(mu_);
  if (!arm_enabled_) return 0;
  goal_ = ArmTarget{t.x, t.y, t.z, clampd(t.pitch, -kPi, kPi), 0.0};
  goal_grip_ = clampd(t.gripper, 0.0, 1.0);
  return new_goal_locked(rate_scale);
}

void RobotCore::arm_goto_partial(std::optional<double> x, std::optional<double> y, std::optional<double> z,
                                 std::optional<double> pitch) {
  if (!client_) return;
  if (!arm_enabled()) set_arm_enabled(true, true);
  const ArmTarget cur = client_->arm_target();
  std::lock_guard<std::mutex> lk(mu_);
  ArmTarget g = goal_.value_or(cur);
  if (x) g.x = *x;
  if (y) g.y = *y;
  if (z) g.z = *z;
  if (pitch) g.pitch = clampd(*pitch, -kPi, kPi);
  goal_ = g;
  new_goal_locked(1.0);
}

void RobotCore::arm_nudge(double dx, double dy, double dz, double dpitch) {
  if (!client_) return;
  if (!arm_enabled()) set_arm_enabled(true, true);
  const ArmTarget cur = client_->arm_target();
  std::lock_guard<std::mutex> lk(mu_);
  ArmTarget g = goal_.value_or(cur);
  g.x += dx;
  g.y += dy;
  g.z += dz;
  g.pitch = clampd(g.pitch + dpitch, -kPi, kPi);
  goal_ = g;
  new_goal_locked(1.0);
}

uint64_t RobotCore::set_gripper(double g) {
  if (!client_) return 0;
  if (!arm_enabled()) set_arm_enabled(true, true);
  std::lock_guard<std::mutex> lk(mu_);
  if (!arm_enabled_) return 0;
  goal_grip_ = clampd(g, 0.0, 1.0);
  return new_goal_locked(1.0);
}

void RobotCore::cancel_goal_locked() {
  if (goal_.has_value() || goal_grip_.has_value()) goal_result_ = GoalResult::kCancelled;
  goal_.reset();
  goal_grip_.reset();
  goal_stall_since_ = -1.0;
}

void RobotCore::cancel_goal() {
  std::lock_guard<std::mutex> lk(mu_);
  cancel_goal_locked();
}

bool RobotCore::reachable(const ArmTarget& t) const {
  if (!proj_) return false;
  return proj_->reachable(kin::Pose{t.x, t.y, t.z, t.pitch});
}

const char* to_string(GoalResult r) {
  switch (r) {
    case GoalResult::kNone: return "none";
    case GoalResult::kReached: return "reached";
    case GoalResult::kStalled: return "stalled";
    case GoalResult::kCancelled: return "cancelled";
  }
  return "?";
}

void RobotCore::estop() {
  if (client_) client_->emergency_stop();
  std::lock_guard<std::mutex> lk(mu_);
  arm_enabled_ = false;
  arm_latched_ = false;
  cancel_goal_locked();
  auto_until_ = -1.0;
  input_ = ControlInput{};
  input_time_ = -1e9;
  log_.error("비상정지!");
}

bool RobotCore::clear_estop() {
  if (!client_) return false;
  const bool ok = client_->clear_emergency();
  if (!ok) log_.warn("비상정지 해제 불가: 세션을 놓은 상태 — '새 세션' 뒤 다시");
  return ok;
}

bool RobotCore::robot_estop(std::string* err) {
  {
    std::lock_guard<std::mutex> lk(mu_);
    arm_enabled_ = false;
    cancel_goal_locked();
    auto_until_ = -1.0;
  }
  return ros_.call_trigger("/armpi/emergency_stop", err);
}

bool RobotCore::robot_reset(std::string* err) { return ros_.call_trigger("/armpi/reset_estop", err); }

bool RobotCore::arm_home(std::string* err) {
  if (!ros_.connected()) {
    if (err) *err = "rosbridge 미연결";
    return false;
  }
  std::lock_guard<std::mutex> lk(mu_);
  // The robot refuses /armpi/arm/home while the arm channel is active: send arm-neutral first, call shortly after.
  arm_enabled_ = false;
  cancel_goal_locked();
  home_due_ = mono_now() + 0.25;
  return true;
}

bool RobotCore::joint_debug_allowed(std::string* why) const {
  auto no = [&](const char* m) {
    if (why) *why = m;
    return false;
  };
  if (!ros_.connected()) return no("rosbridge 미연결");
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (!released_) return no("조종 세션을 먼저 놓아야 함(관절 직접 조종은 UDP 세션이 없을 때만)");
  }
  const auto st = ros_.status();
  if (!ros_.status_fresh()) return no("로봇 상태 없음");
  if (st.emergency_stop) return no("비상정지 중");
  if (st.fault) return no("fault 상태");
  if (!st.owner.empty()) return no("다른 조종기가 세션을 잡고 있음");
  if (why) why->clear();
  return true;
}

bool RobotCore::joint_debug(const std::array<double, 6>& rad, std::string* err) {
  if (!joint_debug_allowed(err)) return false;
  return ros_.publish_joint_debug(rad, err);
}

void RobotCore::release_session() {
  if (!client_) return;
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (released_) return;
    released_ = true;
    arm_enabled_ = false;
    cancel_goal_locked();
    auto_until_ = -1.0;
  }
  client_->disconnect();
  log_.info("조종 세션을 놓음(disconnect) — 다른 조종기·관절 직접 조종 가능");
}

void RobotCore::new_session() {
  if (!client_) return;
  client_->new_session();
  std::lock_guard<std::mutex> lk(mu_);
  released_ = false;
  arm_enabled_ = false;
  cancel_goal_locked();
  log_.info("새 조종 세션");
}

CoreSnapshot RobotCore::snapshot() const {
  std::lock_guard<std::mutex> lk(mu_);
  return snap_;
}

void RobotCore::start_camera() { camera_.start(camera_url()); }
void RobotCore::stop_camera() { camera_.stop(); }

}  // namespace ally::robot
