// Autograsp (docs/ARCHITECTURE.md, docs/SAFETY_KO.md rules 12-21): pick a detected object and lift it, arm only.
//
// GraspPlanner turns one detection into arm poses (observe / hover / grasp / lift).
// GraspSequencer walks the steps. It only moves while the dead-man (A or the on-screen hold button) is held:
// releasing it cancels the current arm goal and pauses; pressing again re-sends the current step's goal.
// It runs on the UI thread and only calls the public RobotCore API through GraspRobot, so a frozen UI also
// freezes the automation (the control loop ramps only while the UI keeps sending fresh input).
#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "common/util.hpp"
#include "robot/core.hpp"
#include "vision/calibration.hpp"
#include "vision/detector.hpp"

namespace ally::robot {

// What the sequencer needs from the robot. RobotCore via CoreGraspRobot; tests use a fake.
class GraspRobot {
 public:
  virtual ~GraspRobot() = default;
  virtual uint64_t arm_goto(const ArmTarget& t, double rate_scale) = 0;  // returns the goal id (0 = refused)
  virtual uint64_t set_gripper(double g) = 0;
  virtual void cancel_goal() = 0;
  virtual CoreSnapshot snapshot() const = 0;
  virtual bool reachable(const ArmTarget& t) const = 0;  // inside the box and IK-solvable as given
  // Base approach (stick units, short time to live). (0, 0) stops. false = refused (speed limit above 30 %).
  virtual bool set_auto_base(double forward, double turn, double ttl_s) = 0;
};

class CoreGraspRobot : public GraspRobot {
 public:
  explicit CoreGraspRobot(RobotCore& c) : c_(c) {}
  uint64_t arm_goto(const ArmTarget& t, double rate_scale) override { return c_.arm_goto(t, rate_scale); }
  uint64_t set_gripper(double g) override { return c_.set_gripper(g); }
  void cancel_goal() override { c_.cancel_goal(); }
  CoreSnapshot snapshot() const override { return c_.snapshot(); }
  bool reachable(const ArmTarget& t) const override { return c_.reachable(t); }
  bool set_auto_base(double forward, double turn, double ttl_s) override { return c_.set_auto_base(forward, turn, ttl_s); }

 private:
  RobotCore& c_;
};

enum class GraspState {
  kIdle, kSelected, kObserve, kPlan, kApproachBase, kConfirm, kOpenGripper, kHover, kDescend, kClose, kLift, kVerify,
  kDone, kPaused, kFailed, kAborted
};
const char* to_string_ko(GraspState s);
// Index in the 8-step progress bar (observe..verify), -1 for states outside it.
int grasp_step_index(GraspState s);
constexpr int kGraspSteps = 8;
extern const char* const kGraspStepLabels[kGraspSteps];

struct GraspPlan {
  ArmTarget observe{}, hover{}, grasp{}, lift{};
  double gripper_close{0.7};
  double obj_x{0}, obj_y{0};  // table point, robot frame (m)
  double pitch_deg{0};
  std::string note;
};

struct GraspOptions {
  bool confirm_before_descend{true};
  int max_retries{0};  // 0..2: full re-tries after a failed verify / stall
  double hover_dz{0.06}, lift_dz{0.08}, floor_margin{0.005};
  double descend_rate_scale{0.4};  // 0.4 x 0.15 m/s = 0.06 m/s
  double confirm_timeout_s{30.0};
  double observe_s{1.5};           // how long to collect frames at the observe pose
  int observe_frames{3};
  double close_settle_s{0.4};
  // Experimental, off by default:
  bool base_approach{false};    // object out of reach -> turn/drive the base towards it (speed limit <= 30 %)
  double approach_burst_s{2.0};  // longest continuous base motion before a stop-and-look
  double approach_total_s{20.0}; // total base motion per pick
  bool intrusion_check{false};  // pause when something large appears in the work area (camera fixed on the chassis)
};

// Where the sequencer gets detections. latest() may return nullptr (no camera / vision off).
struct GraspVision {
  std::function<std::shared_ptr<const vision::DetectionSet>()> latest;
  std::function<std::vector<vision::ObjectClass>()> classes;
};

class GraspPlanner {
 public:
  GraspPlanner(const GraspRobot& robot, GraspOptions opt) : robot_(robot), opt_(opt) {}
  void set_options(const GraspOptions& o) { opt_ = o; }
  // Table point + class -> poses. Tries pitch -90, -75, -60, -50 degrees; all three of hover, grasp and lift
  // must be reachable as given (no nearest-adjustment).
  std::optional<GraspPlan> plan_at(const vision::VisionCalib& calib, double x, double y, const vision::ObjectClass& cls,
                                   std::string* err) const;
  // Pixel (observe-pose image) -> plan.
  std::optional<GraspPlan> plan(const vision::VisionCalib& calib, double u, double v, const vision::ObjectClass& cls,
                                std::string* err) const;
  // Closed polygon (image pixels) around the table area the arm can pick from, for the camera overlay.
  // Empty when the calibration is not usable. Costly (thousands of IK checks): cache per calibration.
  std::vector<std::pair<float, float>> reach_band_pixels(const vision::VisionCalib& calib, int samples) const;

 private:
  const GraspRobot& robot_;
  GraspOptions opt_;
};

struct GraspStatus {
  GraspState state{GraspState::kIdle};
  GraspState resume_state{GraspState::kIdle};  // where kPaused goes back to
  GraspState failed_state{GraspState::kIdle};  // the step that failed / aborted (progress bar)
  int selected_track{-1};
  int class_index{-1};
  float sel_u{0}, sel_v{0};    // last known image position of the selection
  float orig_u{0}, orig_v{0};  // position confirmed at the observe pose (verify compares to this)
  std::optional<GraspPlan> plan;
  double state_since{0};
  std::string message;  // one line for the user
  int attempt{0};
  uint64_t done_events{0}, fail_events{0};  // the UI turns increments into toasts / rumble
  bool base_moving{false};        // base approach is commanding the chassis
  double approach_used_s{0};      // base motion time used in this pick
  bool intrusion_watch{false};    // work-area check armed for this pick
};

class GraspSequencer {
 public:
  GraspSequencer(GraspRobot& robot, GraspVision vision, EventLog& log, GraspOptions opt = {});

  void set_calib(const vision::VisionCalib& c) { calib_ = c; }
  const vision::VisionCalib& calib() const { return calib_; }
  void set_options(const GraspOptions& o);
  const GraspOptions& options() const { return opt_; }
  const GraspPlanner& planner() const { return planner_; }

  // Pick a detection from the latest set by track id. Allowed when idle / selected / finished.
  bool select(int track_id, double now);
  void clear_selection();
  // kConfirm only: shift the grasp point (m), each call clamped to +-5 mm, total to +-3 cm.
  void nudge(double dx, double dy, double dz);
  // kConfirm -> start opening the gripper (the dead-man must be held for motion to continue).
  void confirm(double now);
  void cancel();  // back to idle from anywhere, cancels the arm goal
  // Call every UI frame. Advances only while deadman is true; internally limited to 20 Hz.
  void tick(double now, bool deadman);
  const GraspStatus& status() const { return st_; }
  bool active() const;  // automation owns the arm (motion steps, confirm or paused)

 private:
  void enter(GraspState s, double now, const std::string& msg = {});
  void fail(double now, const std::string& why);
  void pause(double now, const std::string& why);
  bool robot_ok(const CoreSnapshot& s, std::string* why) const;
  void send_step(double now);  // (re)send the goal for the current motion step
  void track_selection(double now);
  void step_observe(double now, const CoreSnapshot& s);
  void step_plan(double now);
  void step_goal(double now, const CoreSnapshot& s);
  void step_verify(double now, const CoreSnapshot& s);
  void step_approach(double now, const CoreSnapshot& s);
  void stop_base();
  // Work-area intrusion check against the reference thumbnail taken at the observe pose (true = pause).
  bool intrusion(const vision::DetectionSet& ds);
  // Same-class detection nearest to (u, v) within radius; nullptr if none.
  const vision::Detection* find_near(const vision::DetectionSet& ds, int cls, float u, float v, float radius) const;

  GraspRobot& robot_;
  GraspVision vision_;
  EventLog& log_;
  GraspOptions opt_;
  GraspPlanner planner_;
  vision::VisionCalib calib_;
  GraspStatus st_;

  double last_tick_{-1e9};
  bool prev_deadman_{false}, pending_rise_{false}, pending_release_{false};
  uint64_t goal_id_{0};
  double goal_sent_at_{0}, goal_timeout_s_{0};
  bool goal_retried_{false};
  double reached_at_{-1};      // observe / verify: time the pose was reached; close: settle start
  uint64_t last_seq_{0};       // last detection frame seen while tracking / observing
  int missing_{0};
  std::vector<std::pair<float, float>> samples_;  // observe / verify detections
  int verify_frames_{0}, verify_hits_{0};
  double nudge_x_{0}, nudge_y_{0}, nudge_z_{0};
  float sel_area_{0};
  // base approach
  double burst_start_{-1}, settle_until_{-1}, lost_since_{-1}, last_frame_at_{-1}, last_cmd_at_{-1};
  // intrusion check
  std::vector<uint8_t> ref_gray_;
  int ref_gw_{0}, ref_gh_{0};
  uint64_t intrusion_seq_{0};
  std::optional<GraspPlan> base_plan_;  // plan before nudges
};

}  // namespace ally::robot
