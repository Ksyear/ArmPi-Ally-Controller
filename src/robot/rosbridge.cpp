#include "robot/rosbridge.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>

#include "armpi_common/json.hpp"

namespace ally::robot {

namespace json = armpi_common::json;

namespace {

// A counter from the network: finite, positive, and below 1e15 before it becomes an integer (converting a huge or
// non-finite double to an integer is undefined behaviour in C++).
uint64_t to_count(double v) { return std::isfinite(v) && v > 0 ? static_cast<uint64_t>(std::min(v, 1e15)) : 0; }

const json::Value* field(const json::Value* v, const char* k) { return v ? v->find(k) : nullptr; }

std::string get_str(const json::Value* v, const char* k, const std::string& def = "") {
  const auto* f = field(v, k);
  return f && f->is_string() ? f->string : def;
}

bool get_bool(const json::Value* v, const char* k, bool def = false) {
  const auto* f = field(v, k);
  return f && f->is_bool() ? f->boolean : def;
}

std::optional<double> get_num(const json::Value* v, const char* k) {
  const auto* f = field(v, k);
  if (f && f->is_number() && f->number_finite) return f->number;
  return std::nullopt;
}

std::optional<Pose5> get_pose(const json::Value* v, const char* k) {
  const auto* p = k ? field(v, k) : v;
  if (!p || !p->is_object()) return std::nullopt;
  const auto x = get_num(p, "x"), y = get_num(p, "y"), z = get_num(p, "z"), pitch = get_num(p, "pitch");
  if (!x || !y || !z || !pitch) return std::nullopt;
  return Pose5{*x, *y, *z, *pitch, get_num(p, "gripper").value_or(0.0)};
}

std::vector<std::string> get_str_list(const json::Value* v, const char* k) {
  std::vector<std::string> out;
  const auto* f = field(v, k);
  if (f && f->is_array())
    for (const auto& e : f->array)
      if (e.is_string()) out.push_back(e.string);
  return out;
}

constexpr size_t kMaxJsonBytes = 1024 * 1024;  // status/joint messages are a few KB

bool parse_lenient(const std::string& text, json::Value& out, std::string& err) {
  if (text.size() > kMaxJsonBytes) {
    err = "메시지가 너무 큼";
    return false;
  }
  return json::parse(sanitize_nonfinite(text), out, err, json::lenient_topic_options(kMaxJsonBytes, 32));
}

}  // namespace

std::string sanitize_nonfinite(const std::string& t) {
  std::string out;
  out.reserve(t.size());
  bool in_str = false;
  for (size_t i = 0; i < t.size(); ++i) {
    const char c = t[i];
    if (in_str) {
      out.push_back(c);
      if (c == '\\' && i + 1 < t.size()) {
        out.push_back(t[++i]);
      } else if (c == '"') {
        in_str = false;
      }
      continue;
    }
    if (c == '"') {
      in_str = true;
      out.push_back(c);
      continue;
    }
    auto token = [&](const char* w) {
      const size_t n = std::char_traits<char>::length(w);
      return t.compare(i, n, w) == 0;
    };
    if (token("-Infinity")) {
      out += "null";
      i += 8;
    } else if (token("Infinity")) {
      out += "null";
      i += 7;
    } else if (token("NaN")) {
      out += "null";
      i += 2;
    } else {
      out.push_back(c);
    }
  }
  return out;
}

RosBridge::RosBridge(EventLog& log, net::CommLog* comm) : log_(log), comm_(comm) {}

RosBridge::~RosBridge() { stop(); }

std::string RosBridge::endpoint() const {
  std::lock_guard<std::mutex> lk(mu_);
  return host_ + ":" + std::to_string(port_);
}

void RosBridge::start(const std::string& host, int port) {
  stop();
  {
    std::lock_guard<std::mutex> lk(mu_);
    host_ = host;
    port_ = port;
    status_ = RobotStatus{};
    joints_ = JointSnapshot{};
    battery_ = BatterySnapshot{};
    bridge_ = BridgeState{};
    applied_target_.reset();
    pending_.clear();
  }
  stop_ = false;
  running_ = true;
  thread_ = std::thread([this] { loop(); });
}

void RosBridge::stop() {
  stop_ = true;
  if (thread_.joinable()) thread_.join();
  running_ = false;
  connected_ = false;
}

bool RosBridge::send_json(const std::string& text, std::string* err) {
  if (!ws_.send_text(text, err)) return false;
  if (comm_) comm_->record(true, net::Channel::kRosbridge, endpoint(), text);
  return true;
}

void RosBridge::loop() {
  std::string host;
  int port;
  {
    std::lock_guard<std::mutex> lk(mu_);
    host = host_;
    port = port_;
  }
  bool logged_fail = false;
  int backoff_ds = 20;  // 2 s, doubles up to 10 s while the server keeps failing
  while (!stop_) {
    std::string err;
    if (!ws_.connect(host, port, "/", 2.0, &err)) {
      {
        std::lock_guard<std::mutex> lk(mu_);
        last_error_ = err;
      }
      if (!logged_fail) log_.warn("rosbridge 연결 실패(" + host + ":" + std::to_string(port) + "): " + err + " — 다시 시도");
      logged_fail = true;
      for (int i = 0; i < backoff_ds && !stop_; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
      backoff_ds = std::min(100, backoff_ds * 2);
      continue;
    }
    logged_fail = false;
    backoff_ds = 20;
    ++connects_;
    connected_ = true;
    log_.ok("rosbridge 연결: " + host + ":" + std::to_string(port));
    // Topic types were checked against the real robot (pkg/msg/Type form).
    const char* subs[][3] = {
        {"/armpi/status", "std_msgs/msg/String", "0"},
        {"/armpi/battery", "sensor_msgs/msg/BatteryState", "0"},
        {"/joint_states", "sensor_msgs/msg/JointState", "100"},
        {"/armpi/arm/target", "armpi_interfaces/msg/ArmTarget", "100"},
        {"/armpi_vr_bridge/state", "std_msgs/msg/String", "0"},
    };
    bool ok = true;
    for (const auto& s : subs) {
      std::string req = std::string("{\"op\":\"subscribe\",\"topic\":\"") + s[0] + "\",\"type\":\"" + s[1] + "\"";
      if (std::string(s[2]) != "0") req += std::string(",\"throttle_rate\":") + s[2] + ",\"queue_length\":1";
      req += "}";
      ok = ok && send_json(req, &err);
    }
    ok = ok && send_json("{\"op\":\"advertise\",\"topic\":\"/armpi/arm/joint_debug\",\"type\":\"sensor_msgs/msg/JointState\"}", &err);
    while (ok && !stop_) {
      std::string msg;
      const int r = ws_.receive(&msg, 0.2, &err);
      if (r == 0) continue;
      if (r < 0) break;
      ++messages_;
      if (comm_) comm_->record(false, net::Channel::kRosbridge, host + ":" + std::to_string(port), msg);
      handle_message(msg);
    }
    connected_ = false;
    ws_.close();
    if (!stop_) {
      {
        std::lock_guard<std::mutex> lk(mu_);
        last_error_ = err.empty() ? "연결 끊김" : err;
      }
      log_.warn("rosbridge 끊김: " + (err.empty() ? std::string("연결 끊김") : err));
      for (int i = 0; i < 20 && !stop_; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  ws_.close();
}

void RosBridge::handle_message(const std::string& text) {
  json::Value v;
  std::string err;
  if (!parse_lenient(text, v, err) || !v.is_object()) return;
  const std::string op = get_str(&v, "op");
  const double now = mono_now();
  if (op == "publish") {
    const std::string topic = get_str(&v, "topic");
    const json::Value* m = v.find("msg");
    if (!m || !m->is_object()) return;
    if (topic == "/armpi/status" || topic == "/armpi_vr_bridge/state") {
      json::Value s;
      if (!parse_lenient(get_str(m, "data"), s, err) || !s.is_object()) return;
      if (topic == "/armpi/status") {
        RobotStatus st;
        st.valid = true;
        st.rx_mono = now;
        st.hardware = get_str(&s, "hardware");
        st.emergency_stop = get_bool(&s, "emergency_stop");
        st.estop_sources = get_str_list(&s, "estop_sources");
        st.fault = get_bool(&s, "fault");
        st.fault_reason = get_str(&s, "fault_reason");
        st.udp_allowed = get_bool(&s, "udp_allowed", true);
        st.battery_v = get_num(&s, "battery_v");
        st.battery_low = get_bool(&s, "battery_low");
        st.position_source = get_str(&s, "position_source");
        st.arm_initialized = get_bool(&s, "arm_initialized");
        st.arm_mode = get_str(&s, "arm_mode");
        st.arm_target = get_pose(&s, "arm_target");
        st.arm_current = get_pose(&s, "arm_current");
        st.arm_moving = get_bool(&s, "arm_moving");
        st.arm_ik_blocked = get_bool(&s, "arm_ik_blocked");
        st.arm_last_target_result = get_str(&s, "arm_last_target_result");
        st.owner = get_str(&s, "owner");
        st.owner_addr = get_str(&s, "owner_addr");
        st.base_state = get_str(&s, "base_state");
        st.arm_state = get_str(&s, "arm_state");
        if (const auto* b = s.find("base_cmd")) {
          st.base_vx = get_num(b, "vx").value_or(0);
          st.base_vy = get_num(b, "vy").value_or(0);
          st.base_wz = get_num(b, "wz").value_or(0);
        }
        st.warnings = get_str(&s, "warnings");
        st.kernel = get_str(&s, "kernel");
        st.boot_warnings = get_str_list(&s, "boot_warnings");
        std::lock_guard<std::mutex> lk(mu_);
        status_ = std::move(st);
      } else {
        BridgeState b;
        b.valid = true;
        b.rx_mono = now;
        b.owner = get_str(&s, "owner");
        b.owner_addr = get_str(&s, "owner_addr");
        b.base_state = get_str(&s, "base_state");
        b.arm_state = get_str(&s, "arm_state");
        b.emergency_stop = get_bool(&s, "emergency_stop");
        b.arm_target = get_pose(&s, "arm_target");
        if (const auto* p = s.find("packets"); p && p->is_object()) {
          if (const auto* a = p->find("arm_adjust"); a && a->is_object()) {
            for (const auto& kv : a->object)
              if (kv.second.is_number()) b.arm_adjust_total += to_count(kv.second.number);
          } else if (a && a->is_number()) {
            b.arm_adjust_total = to_count(a->number);
          }
        }
        std::lock_guard<std::mutex> lk(mu_);
        bridge_ = std::move(b);
      }
    } else if (topic == "/armpi/battery") {
      BatterySnapshot b;
      b.voltage = get_num(m, "voltage");
      b.percentage = get_num(m, "percentage");
      b.rx_mono = now;
      std::lock_guard<std::mutex> lk(mu_);
      battery_ = b;
    } else if (topic == "/joint_states") {
      JointSnapshot j;
      j.names = get_str_list(m, "name");
      if (const auto* p = m->find("position"); p && p->is_array())
        for (const auto& e : p->array) j.position.push_back(e.is_number() ? e.number : 0.0);
      j.rx_mono = now;
      std::lock_guard<std::mutex> lk(mu_);
      joints_ = std::move(j);
    } else if (topic == "/armpi/arm/target") {
      auto p = get_pose(m, nullptr);
      std::lock_guard<std::mutex> lk(mu_);
      if (p && get_bool(m, "active", true)) {
        applied_target_ = p;
        applied_target_rx_ = now;
      }
    }
  } else if (op == "service_response") {
    ServiceResult r;
    r.mono = now;
    const std::string id = get_str(&v, "id");
    r.service = get_str(&v, "service");
    const json::Value* vals = v.find("values");
    if (vals && vals->is_object()) {
      r.ok = get_bool(&v, "result", true) && get_bool(vals, "success", true);
      r.message = get_str(vals, "message");
    } else {
      r.ok = false;
      r.message = vals && vals->is_string() ? vals->string : "응답 형식 오류";
    }
    {
      std::lock_guard<std::mutex> lk(mu_);
      if (r.service.empty() && pending_.count(id)) r.service = pending_[id];
      pending_.erase(id);
      last_result_ = r;
    }
    if (r.ok) log_.ok("서비스 " + r.service + " 성공 " + r.message);
    else log_.warn("서비스 " + r.service + " 실패: " + r.message);
  }
}

RobotStatus RosBridge::status() const { std::lock_guard<std::mutex> lk(mu_); return status_; }

bool RosBridge::status_fresh(double max_age_s) const {
  std::lock_guard<std::mutex> lk(mu_);
  return status_.valid && mono_now() - status_.rx_mono <= max_age_s;
}

JointSnapshot RosBridge::joints() const { std::lock_guard<std::mutex> lk(mu_); return joints_; }
BatterySnapshot RosBridge::battery() const { std::lock_guard<std::mutex> lk(mu_); return battery_; }
BridgeState RosBridge::bridge_state() const { std::lock_guard<std::mutex> lk(mu_); return bridge_; }

std::optional<Pose5> RosBridge::applied_arm_target() const {
  std::lock_guard<std::mutex> lk(mu_);
  if (applied_target_ && mono_now() - applied_target_rx_ < 1.0) return applied_target_;
  return std::nullopt;
}

std::optional<ServiceResult> RosBridge::last_result() const { std::lock_guard<std::mutex> lk(mu_); return last_result_; }

std::string RosBridge::last_error() const { std::lock_guard<std::mutex> lk(mu_); return last_error_; }

bool RosBridge::call_trigger(const std::string& service, std::string* err) {
  if (!connected_) {
    if (err) *err = "rosbridge 미연결";
    return false;
  }
  const std::string id = "ally-" + std::to_string(++ids_);
  {
    std::lock_guard<std::mutex> lk(mu_);
    pending_[id] = service;
  }
  const std::string req = "{\"op\":\"call_service\",\"id\":" + json::quote(id) + ",\"service\":" + json::quote(service) +
                          ",\"type\":\"std_srvs/srv/Trigger\",\"args\":{}}";
  if (!send_json(req, err)) return false;
  log_.info("서비스 요청 " + service);
  return true;
}

bool RosBridge::publish_joint_debug(const std::array<double, 6>& rad, std::string* err) {
  if (!connected_) {
    if (err) *err = "rosbridge 미연결";
    return false;
  }
  std::string names, pos;
  for (size_t i = 0; i < rad.size(); ++i) {
    if (i) {
      names += ",";
      pos += ",";
    }
    names += json::quote(kJointNames[i]);
    pos += json::fixed(rad[i], 5);
  }
  const std::string req = "{\"op\":\"publish\",\"topic\":\"/armpi/arm/joint_debug\",\"msg\":{\"name\":[" + names +
                          "],\"position\":[" + pos + "],\"velocity\":[],\"effort\":[]}}";
  return send_json(req, err);
}

}  // namespace ally::robot
