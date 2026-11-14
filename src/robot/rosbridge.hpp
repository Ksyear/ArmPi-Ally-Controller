// rosbridge (WebSocket 9090) client: robot status/battery/joints, robot-wide services, joint_debug.
// Runs on its own thread and reconnects every 2 s. UDP control keeps working without it.
#pragma once

#include <array>
#include <atomic>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "common/util.hpp"
#include "net/comm_log.hpp"
#include "net/websocket.hpp"

namespace ally::robot {

struct Pose5 {
  double x{0}, y{0}, z{0}, pitch{0}, gripper{0};
};

// Parsed /armpi/status (keys from armpi_control status_json)
struct RobotStatus {
  bool valid{false};
  double rx_mono{0.0};
  std::string hardware;  // "real" / "mock"
  bool emergency_stop{false};
  std::vector<std::string> estop_sources;
  bool fault{false};
  std::string fault_reason;
  bool udp_allowed{true};
  std::optional<double> battery_v;
  bool battery_low{false};
  std::string position_source;
  bool arm_initialized{false};
  std::string arm_mode;
  std::optional<Pose5> arm_target, arm_current;
  bool arm_moving{false};
  bool arm_ik_blocked{false};
  std::string arm_last_target_result;
  std::string owner, owner_addr, base_state, arm_state;
  double base_vx{0}, base_vy{0}, base_wz{0};
  std::string warnings;
  std::string kernel;
  std::vector<std::string> boot_warnings;
};

struct JointSnapshot {
  std::vector<std::string> names;
  std::vector<double> position;  // rad
  double rx_mono{0.0};
};

struct BatterySnapshot {
  std::optional<double> voltage, percentage;
  double rx_mono{0.0};
};

// /armpi_vr_bridge/state: who owns the robot, per-channel state, nearest-adjust counters
struct BridgeState {
  bool valid{false};
  double rx_mono{0.0};
  std::string owner, owner_addr, base_state, arm_state;
  bool emergency_stop{false};
  std::optional<Pose5> arm_target;
  uint64_t arm_adjust_total{0};
};

struct ServiceResult {
  std::string service;
  bool ok{false};
  std::string message;
  double mono{0.0};
};

inline const std::array<const char*, 6> kJointNames = {"joint1", "joint2", "joint3", "joint4", "joint5", "r_joint"};

// Replace bare NaN / Infinity / -Infinity tokens (outside strings) with null.
// rosbridge (Python json) emits them for unset BatteryState fields; our strict parser would drop the message.
std::string sanitize_nonfinite(const std::string& text);

class RosBridge {
 public:
  RosBridge(EventLog& log, net::CommLog* comm);
  ~RosBridge();
  RosBridge(const RosBridge&) = delete;
  RosBridge& operator=(const RosBridge&) = delete;

  void start(const std::string& host, int port);
  void stop();
  bool running() const { return running_; }
  bool connected() const { return connected_; }
  std::string endpoint() const;

  RobotStatus status() const;
  bool status_fresh(double max_age_s = 2.0) const;
  JointSnapshot joints() const;
  BatterySnapshot battery() const;
  BridgeState bridge_state() const;
  std::optional<Pose5> applied_arm_target() const;  // /armpi/arm/target (what the robot actually uses)

  // std_srvs/Trigger, asynchronous. Result goes to last_result() and the event log.
  bool call_trigger(const std::string& service, std::string* err);
  std::optional<ServiceResult> last_result() const;
  bool publish_joint_debug(const std::array<double, 6>& rad, std::string* err);

  uint64_t messages() const { return messages_; }
  uint64_t reconnects() const { return connects_; }
  std::string last_error() const;

  // Exposed for unit tests
  void handle_message(const std::string& text);

 private:
  void loop();
  bool send_json(const std::string& text, std::string* err);

  EventLog& log_;
  net::CommLog* comm_;
  net::WebSocketClient ws_;
  std::thread thread_;
  std::atomic<bool> running_{false}, stop_{false}, connected_{false};
  std::atomic<uint64_t> messages_{0}, connects_{0}, ids_{0};

  mutable std::mutex mu_;
  std::string host_;
  int port_{9090};
  RobotStatus status_;
  JointSnapshot joints_;
  BatterySnapshot battery_;
  BridgeState bridge_;
  std::optional<Pose5> applied_target_;
  double applied_target_rx_{0.0};
  std::optional<ServiceResult> last_result_;
  std::map<std::string, std::string> pending_;  // call id -> service
  std::string last_error_;
};

}  // namespace ally::robot
