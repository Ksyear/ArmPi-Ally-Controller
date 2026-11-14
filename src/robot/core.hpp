// RobotCore: one Protocol v3 session + rosbridge + camera, driven by a 20 Hz control loop.
// The UI (GUI or console) only calls the thread-safe methods below; it never touches sockets.
#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "armpi_protocol/client.hpp"
#include "common/util.hpp"
#include "net/comm_log.hpp"
#include "robot/camera.hpp"
#include "robot/rosbridge.hpp"

namespace ally::robot {

using armpi_protocol::ArmTarget;

struct Rates {
  double arm_m_s{0.15};       // tip target speed at full stick
  double pitch_rad_s{0.6};
  double gripper_per_s{0.8};
};

struct CoreOptions {
  std::string host;
  int control_port{5001};
  int rosbridge_port{9090};  // 0 = off
  int camera_port{8080};
  std::string camera_url;    // empty = default stream URL
  double ack_timeout_s{0.30};
  ArmTarget home{armpi_protocol::kDefaultHome};
  Rates rates;
  double speed{0.3};          // chassis scale 0.1..1.0 (1.0 = robot limit)
  double precision_scale{0.5};
  double latched_arm_idle_s{60.0};  // touch toggle turns itself off after this long without arm input
  bool client_filter{true};   // geometric reach filter on the arm target
  bool follow_robot_adjust{true};
};

// One frame of operator intent, merged by the UI from gamepad/touch/keyboard.
struct ControlInput {
  double forward{0}, strafe{0}, turn{0};  // -1..1 before speed scaling (+strafe = left, +turn = CCW)
  double arm[5]{0, 0, 0, 0, 0};           // x, y, z, pitch, gripper rates -1..1
  bool precision{false};                  // fine mode
};

// Monotonic event counters. The UI turns increments into rumble / sounds / flashes.
struct Cues {
  uint64_t estop{0}, limit{0}, link_lost{0}, link_ok{0}, arm_on{0}, rejected{0}, service_ok{0}, service_fail{0};
};

struct CoreSnapshot {
  std::string robot, client_id;
  bool connected{false}, emergency{false}, estop_pending{false}, faulted{false}, retired{false};
  bool session_released{false};
  armpi_protocol::ChannelStatus base, arm;
  ArmTarget target{};
  std::optional<ArmTarget> last_accepted;
  std::optional<double> ack_ms_last, ack_ms_p50, ack_ms_max;
  uint64_t packets_sent{0}, acks{0}, late_acks{0}, send_errors{0};
  std::string last_error;
  bool arm_enabled{false};
  bool input_fresh{false};
  bool precision{false};
  double speed{0.3};
  double applied_f{0}, applied_s{0}, applied_t{0};
  bool limit_active{false};  // target was clamped/adjusted in the last 0.5 s
  bool arm_latched{false};   // arm enabled by a toggle (not a held dead-man)
  bool goal_active{false};   // ramping toward an absolute target
  Cues cues;
  double loop_hz{0.0};
  uint64_t loop_overruns{0};
};

class RobotCore {
 public:
  RobotCore(CoreOptions opt, EventLog& log, net::CommLog& comm);
  ~RobotCore();
  RobotCore(const RobotCore&) = delete;
  RobotCore& operator=(const RobotCore&) = delete;

  bool start(std::string* err);
  void stop();  // neutral + disconnect, then join
  bool started() const { return running_; }
  const CoreOptions& options() const { return opt_; }

  void set_input(const ControlInput& in);  // every UI frame; older than 0.3 s -> neutral
  void clear_input();                       // window lost focus, pad unplugged, ...

  void set_speed(double s);
  double speed() const;
  // on: target snaps to the robot's current tip pose first. latched = touch toggle (auto-off after idle),
  // otherwise a held dead-man (LB / Shift) that the UI releases itself.
  void set_arm_enabled(bool on, bool latched = false);
  bool arm_enabled() const;
  // Absolute targets never jump: the loop ramps toward them at the arm rate limits.
  void arm_goto(const ArmTarget& t);
  void arm_goto_partial(std::optional<double> x, std::optional<double> y, std::optional<double> z,
                        std::optional<double> pitch);
  void arm_nudge(double dx, double dy, double dz, double dpitch);
  void set_gripper(double g);
  void cancel_goal();
  void arm_sync();

  void estop();        // UDP emergency_stop, retried until ACK (latches the whole robot)
  bool clear_estop();  // UDP clear_emergency (owner only)
  bool robot_estop(std::string* err);  // rosbridge /armpi/emergency_stop (second path)
  bool robot_reset(std::string* err);  // rosbridge /armpi/reset_estop
  bool arm_home(std::string* err);     // arm channel off, then /armpi/arm/home
  bool joint_debug_allowed(std::string* why) const;
  bool joint_debug(const std::array<double, 6>& rad, std::string* err);
  void release_session();  // disconnect: frees the robot for other controllers, allows joint debug
  void new_session();

  CoreSnapshot snapshot() const;
  RosBridge& ros() { return ros_; }
  CameraFeed& camera() { return camera_; }
  void start_camera();
  void stop_camera();
  std::string camera_url() const;

 private:
  void loop();
  void step(double now);
  void update_snapshot(double now);
  void bump_limit(double now);

  CoreOptions opt_;
  EventLog& log_;
  net::CommLog& comm_;
  RosBridge ros_;
  CameraFeed camera_;
  std::shared_ptr<armpi_protocol::V3Client> client_;

  std::thread thread_;
  std::atomic<bool> running_{false}, stop_{false};

  mutable std::mutex mu_;  // guards everything below
  ControlInput input_;
  double input_time_{-1e9};
  bool arm_enabled_{false};
  bool arm_latched_{false};
  double last_arm_activity_{0.0};
  std::optional<ArmTarget> goal_;   // x,y,z,pitch ramp goal
  std::optional<double> goal_grip_;
  double goal_stall_since_{-1.0};
  double goal_best_{0.0};
  bool released_{false};
  double speed_{0.4};
  double home_due_{-1.0};
  double last_limit_cue_{-1e9};
  double limit_until_{-1e9};
  std::atomic<bool> filter_adjusted_{false};
  uint64_t last_adjust_total_{0};
  double last_ignored_log_{-1e9};
  bool was_connected_{false}, was_emergency_{false};
  std::string last_reject_key_;
  CoreSnapshot snap_;
  Cues cues_;
  double loop_hz_{0.0};
  uint64_t overruns_{0};
};

}  // namespace ally::robot
