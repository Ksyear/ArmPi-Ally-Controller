// Unit tests (no framework dependency): run `ctest` or the binary directly.
#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

#include "common/util.hpp"
#include "input/mapper.hpp"
#include "input/pad.hpp"
#include "input/tilt.hpp"
#include "net/crypto.hpp"
#include "net/discovery.hpp"
#include "net/http.hpp"
#include "net/websocket.hpp"
#include "robot/camera.hpp"
#include "robot/rosbridge.hpp"
#include <random>

using namespace ally;

static int g_fail = 0, g_pass = 0;
#define CHECK(cond)                                                       \
  do {                                                                    \
    if (cond) {                                                           \
      ++g_pass;                                                           \
    } else {                                                              \
      ++g_fail;                                                           \
      std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);       \
    }                                                                     \
  } while (0)
#define CHECK_NEAR(a, b) CHECK(std::fabs((a) - (b)) < 1e-6)

static void test_crypto() {
  // RFC 6455 section 1.3 example
  CHECK(net::websocket_accept("dGhlIHNhbXBsZSBub25jZQ==") == "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=");
  CHECK(net::base64_encode("f") == "Zg==");
  CHECK(net::base64_encode("fo") == "Zm8=");
  CHECK(net::base64_encode("foo") == "Zm9v");
}

static void test_websocket_frames() {
  const unsigned char mask[4] = {1, 2, 3, 4};
  for (size_t n : {0u, 5u, 125u, 126u, 300u, 70000u}) {
    std::string payload(n, 'x');
    for (size_t i = 0; i < n; ++i) payload[i] = static_cast<char>('a' + i % 26);
    std::string buf = net::WebSocketClient::build_frame(0x1, payload, mask);
    int op = 0;
    bool fin = false;
    std::string out;
    CHECK(net::WebSocketClient::take_frame(buf, &op, &fin, &out) == 1);
    CHECK(op == 1 && fin && out == payload && buf.empty());
  }
  std::string partial = net::WebSocketClient::build_frame(0x1, "hello", mask);
  partial.pop_back();
  int op;
  bool fin;
  std::string out;
  CHECK(net::WebSocketClient::take_frame(partial, &op, &fin, &out) == 0);
}

static void test_discovery_and_http() {
  const std::string nonce = "0123456789abcdef0123456789abcdef";
  auto r = net::parse_discovery_reply("ARMPI_HERE/1|" + nonce + "|58ad7561bb722863312f66ee26e535e7|arm%20pi|5001|3", nonce);
  CHECK(r && r->name == "arm pi" && r->control_port == 5001 && r->protocol == 3);
  CHECK(!net::parse_discovery_reply("ARMPI_HERE/1|ffff|58ad7561bb722863312f66ee26e535e7|x|5001|3", nonce));
  CHECK(!net::parse_discovery_reply("ARMPI_HERE/1|" + nonce + "|short|x|5001|3", nonce));
  auto u = net::parse_url("http://10.0.0.5:8080/stream?topic=/a&b=1");
  CHECK(u && u->host == "10.0.0.5" && u->port == 8080 && u->path == "/stream?topic=/a&b=1");
  CHECK(!net::parse_url("https://x/"));
  net::HttpResponse h;
  CHECK(net::parse_response_head("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nX-A:  b \r\n\r\n", &h));
  CHECK(h.status == 101 && h.headers["upgrade"] == "websocket" && h.headers["x-a"] == "b");
}

static void test_rosbridge_parse() {
  CHECK(robot::sanitize_nonfinite(R"({"a":NaN,"b":-Infinity,"c":"NaN","d":Infinity})") ==
        R"({"a":null,"b":null,"c":"NaN","d":null})");
  EventLog log;
  robot::RosBridge rb(log, nullptr);
  const std::string status =
      R"({"source":"armpi_control","hardware":"real","emergency_stop":true,"estop_sources":["udp"],"fault":false,)"
      R"("battery_v":7.812,"battery_low":false,"arm_target":{"x":0.1,"y":0,"z":0.07,"pitch":-1,"gripper":0.2},)"
      R"("arm_current":{"x":0.11,"y":0.01,"z":0.08,"pitch":-0.9,"gripper":0.25},"arm_moving":true,"owner":"abc",)"
      R"("base_cmd":{"vx":0.1,"vy":0,"wz":-0.2},"kernel":"5.15","boot_warnings":["w1"]})";
  std::string esc;
  for (char c : status) {
    if (c == '"') esc += "\\\"";
    else esc += c;
  }
  rb.handle_message(R"({"op":"publish","topic":"/armpi/status","msg":{"data":")" + esc + R"("}})");
  const auto st = rb.status();
  CHECK(st.valid && st.hardware == "real" && st.emergency_stop && st.estop_sources.size() == 1);
  CHECK(st.battery_v && std::fabs(*st.battery_v - 7.812) < 1e-9);
  CHECK(st.arm_current && std::fabs(st.arm_current->gripper - 0.25) < 1e-9 && st.arm_moving);
  CHECK(st.owner == "abc" && std::fabs(st.base_wz + 0.2) < 1e-9 && st.boot_warnings.size() == 1);
  rb.handle_message(R"({"op":"publish","topic":"/armpi/battery","msg":{"voltage":7.5,"percentage":NaN,"temperature":NaN}})");
  CHECK(rb.battery().voltage && !rb.battery().percentage);
  rb.handle_message(R"({"op":"publish","topic":"/joint_states","msg":{"name":["joint1","joint2"],"position":[0.1,-0.2]}})");
  CHECK(rb.joints().position.size() == 2);
  rb.handle_message(R"({"op":"service_response","service":"/armpi/arm/home","values":{"success":false,"message":"busy"},"result":true,"id":"x"})");
  CHECK(rb.last_result() && !rb.last_result()->ok && rb.last_result()->message == "busy");
}

static void test_mapper() {
  input::Mapper m;
  input::PadState pad;
  pad.connected = true;
  input::KeyState k;
  input::TouchControls t;
  double now = 0.0;
  auto step = [&]() { now += 0.016; return m.map(pad, k, t, now, true); };
  step();  // centered once
  pad.ly = 1.0f;
  auto r = step();
  CHECK_NEAR(r.input.forward, 1.0);
  CHECK(!r.arm_hold);
  pad.ly = 0.5f;
  r = step();
  CHECK(std::fabs(r.input.forward - std::pow(0.5, 1.6)) < 1e-6);  // response curve
  pad.ly = 0.0f;
  pad.lx = -1.0f;  // stick left = +strafe (REP-103)
  r = step();
  CHECK_NEAR(r.input.strafe, 1.0);
  pad.lx = 0.0f;
  pad.rx = -1.0f;  // stick left = turn left (+)
  r = step();
  CHECK_NEAR(r.input.turn, 1.0);
  pad.rx = 0.0f;
  // LB: sticks move the arm, chassis neutral
  pad.buttons = input::kLB;
  pad.ly = 1.0f;
  pad.rt = 1.0f;
  r = step();
  CHECK(r.arm_hold);
  CHECK_NEAR(r.input.forward, 0.0);
  CHECK_NEAR(r.input.arm[0], 1.0);
  CHECK_NEAR(r.input.arm[4], 1.0);
  pad.ly = 0.0f;
  pad.rt = 0.0f;
  pad.buttons = 0;
  step();
  // B = estop on the press edge only
  pad.buttons = input::kB;
  CHECK(step().ev.estop);
  CHECK(!step().ev.estop);
  pad.buttons = 0;
  step();
  // View held 0.8 s clears once
  pad.buttons = input::kBack;
  bool fired = false;
  int fires = 0;
  for (int i = 0; i < 70; ++i) {
    r = step();
    if (r.ev.clear_estop) {
      fired = true;
      ++fires;
    }
  }
  CHECK(fired && fires == 1);
  pad.buttons = 0;
  step();
  // focus lost: neutral, but estop still works
  pad.ly = 1.0f;
  now += 0.016;
  r = m.map(pad, k, t, now, false);
  CHECK_NEAR(r.input.forward, 0.0);
  pad.buttons = input::kB;
  now += 0.016;
  CHECK(m.map(pad, k, t, now, false).ev.estop);
  pad.buttons = 0;
  // after focus returns, a held stick is ignored until centered
  r = step();
  CHECK_NEAR(r.input.forward, 0.0);
  pad.ly = 0.0f;
  step();
  pad.ly = 1.0f;
  CHECK_NEAR(step().input.forward, 1.0);
  pad.ly = 0.0f;
  // keyboard + pad sum is clamped
  k.w = true;
  pad.ly = 1.0f;
  CHECK_NEAR(step().input.forward, 1.0);
  k.w = false;
  pad.ly = 0.0f;
  k.shift = true;
  k.i = true;
  r = step();
  CHECK(r.arm_hold);
  CHECK_NEAR(r.input.arm[0], 1.0);
}

static void test_pad_and_haptics() {
  float x = 0.1f, y = 0.0f;
  input::apply_radial_deadzone(&x, &y, 0.15f);
  CHECK(x == 0.0f && y == 0.0f);
  x = 1.0f;
  y = 0.0f;
  input::apply_radial_deadzone(&x, &y, 0.15f);
  CHECK(std::fabs(x - 1.0f) < 1e-6f);
  input::HapticPlayer hp;
  hp.play(input::Haptic::kLimit, 10.0);
  float lo, hi;
  hp.sample(10.01, &lo, &hi);
  CHECK(lo > 0.0f);
  hp.sample(10.2, &lo, &hi);
  CHECK(lo == 0.0f && hi == 0.0f);
  hp.play(input::Haptic::kLimit, 11.0);
  hp.play(input::Haptic::kEstop, 11.0);  // e-stop replaces the queue
  hp.sample(11.01, &lo, &hi);
  CHECK(lo == 1.0f && hi == 1.0f);
}

static void test_security_limits() {
  // PNG header that declares a 20000 x 20000 image (pixel bomb) must be rejected before decoding.
  std::string png = "\x89PNG\r\n\x1a\n";
  png += std::string("\x00\x00\x00\x0dIHDR", 8);
  png += std::string("\x00\x00\x4e\x20\x00\x00\x4e\x20\x08\x06\x00\x00\x00", 13);
  png += std::string("\x00\x00\x00\x00", 4);  // CRC (not checked)
  int w = 0, h = 0;
  std::vector<uint8_t> rgba;
  std::string err;
  CHECK(!robot::decode_image(png, &w, &h, &rgba, &err) && rgba.empty());
  // WebSocket: a frame header announcing more than the limit is refused without buffering it.
  std::string big;
  big.push_back(static_cast<char>(0x81));
  big.push_back(static_cast<char>(127));
  for (int i = 7; i >= 0; --i) big.push_back(static_cast<char>(((net::kMaxMessageBytes + 1) >> (8 * i)) & 0xFF));
  int op;
  bool fin;
  std::string out;
  CHECK(net::WebSocketClient::take_frame(big, &op, &fin, &out) == -1);
  // Log/notification text is cut on a UTF-8 boundary.
  CHECK(utf8_truncate("한글abc", 4) == "한…");
  CHECK(utf8_truncate("abc", 10) == "abc");
  // The session id / nonces depend on std::random_device: it must not be deterministic.
  std::random_device rd;
  bool differ = false;
  const unsigned first = rd();
  for (int i = 0; i < 8 && !differ; ++i) differ = rd() != first;
  CHECK(differ);
}

static void test_util() {
  CHECK(display_width("abc") == 3);
  CHECK(display_width("한글") == 4);
  CHECK(display_width("\x1b[31mab\x1b[0m") == 2);
  CHECK(parse_double("1.5") && !parse_double("x"));
}

static void test_mask_addresses() {
  CHECK(mask_addresses("조종 세션 시작: 192.168.1.20:5001") == "조종 세션 시작: \xE2\x80\xA2\xE2\x80\xA2\xE2\x80\xA2");
  CHECK(mask_addresses("rosbridge 연결: 10.0.0.7:9090 ok") == "rosbridge 연결: \xE2\x80\xA2\xE2\x80\xA2\xE2\x80\xA2 ok");
  CHECK(mask_addresses("x 0.105 y 0.000 v1.2.3") == "x 0.105 y 0.000 v1.2.3");  // not four groups
  CHECK(mask_addresses("1.2.3.4.5") == "1.2.3.4.5" || mask_addresses("1.2.3.4.5").find("1.2.3.4") == std::string::npos);
  CHECK(mask_addresses("") .empty());
}

static void test_tilt() {
  using input::Gravity;
  // upright, facing the user: pitch 0; screen up on a table: pitch 90; right side down: roll +
  CHECK(std::fabs(input::tilt_pitch_deg(Gravity{0, -1, 0})) < 1e-9);
  CHECK(std::fabs(input::tilt_pitch_deg(Gravity{0, 0, -1}) - 90.0) < 1e-9);
  CHECK(std::fabs(input::tilt_roll_deg(Gravity{0, -1, 0})) < 1e-9);
  CHECK(input::tilt_roll_deg(Gravity{0.5, -0.86, 0}) > 25.0);
  // deadband, proportional band, saturation, signs
  auto d = input::tilt_to_drive(40, 10, 38, 8, 5, 25);  // 2 degrees each: inside the deadband
  CHECK(d.forward == 0.0 && d.turn == 0.0);
  d = input::tilt_to_drive(53, 0, 38, 0, 5, 25);  // 15 degrees forward: half way
  CHECK(std::fabs(d.forward - 0.5) < 1e-9 && d.turn == 0.0);
  d = input::tilt_to_drive(38, 40, 38, 0, 5, 25);  // right side far down: full clockwise (negative) turn
  CHECK(d.turn == -1.0 && d.forward == 0.0);
  d = input::tilt_to_drive(20, -20, 38, 0, 5, 25);  // tipped back + left side down
  CHECK(d.forward < 0 && d.turn > 0);
  d = input::tilt_to_drive(-179, 0, 179, 0, 5, 25);  // reference near +-180: wraps instead of jumping
  CHECK(std::fabs(d.forward) < 1e-9);
  d = input::tilt_to_drive(std::nan(""), 0, 0, 0, 5, 25);
  CHECK(d.forward == 0.0);
  d = input::tilt_to_drive(40, 40, 0, 0, 5, std::nan(""));  // a broken setting never means full speed
  CHECK(d.forward == 0.0 && d.turn == 0.0);
  // Held at 45 degrees, then turned 45 degrees like a steering wheel (about the screen normal): turn only.
  const double c = std::cos(0.785398), s45 = std::sin(0.785398), a = 0.785398;
  const Gravity held{0, -c, -s45}, steered{c * std::sin(a), -c * std::cos(a), -s45};
  d = input::tilt_to_drive(input::tilt_pitch_deg(steered), input::tilt_roll_deg(steered), input::tilt_pitch_deg(held),
                           input::tilt_roll_deg(held), 5, 25);
  CHECK(std::fabs(d.forward) < 1e-9 && d.turn < -0.5);
}

int main() {
  const std::vector<std::pair<const char*, std::function<void()>>> tests = {
      {"crypto", test_crypto},           {"websocket_frames", test_websocket_frames},
      {"discovery_http", test_discovery_and_http}, {"rosbridge_parse", test_rosbridge_parse},
      {"mapper", test_mapper},           {"pad_haptics", test_pad_and_haptics},
      {"util", test_util},               {"security_limits", test_security_limits},
      {"tilt", test_tilt},               {"mask_addresses", test_mask_addresses},
  };
  for (const auto& [name, fn] : tests) {
    const int before = g_fail;
    fn();
    std::printf("%s %s\n", g_fail == before ? "[ok]  " : "[FAIL]", name);
  }
  std::printf("checks: %d passed, %d failed\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
