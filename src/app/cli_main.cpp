// armpi_ally_cli: console companion of the GUI.
//   discover            find robots on the local network (UDP 5002)
//   selftest --robot IP run the whole non-GUI stack against a robot or the mock robot, print PASS/FAIL
//   pads                show XInput gamepad state (Windows), with --rumble to test vibration
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include "common/util.hpp"
#include "input/pad.hpp"
#include "net/discovery.hpp"
#include "net/sock.hpp"
#include "robot/core.hpp"
#include "version.hpp"

#ifdef _WIN32
#include <windows.h>
#endif

using namespace ally;

namespace {

void sleep_s(double s) { std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(s * 1000))); }

bool wait_until(double timeout_s, const std::function<bool()>& cond) {
  const double end = mono_now() + timeout_s;
  while (mono_now() < end) {
    if (cond()) return true;
    sleep_s(0.02);
  }
  return cond();
}

struct Args {
  std::string cmd;
  std::string robot;
  std::vector<std::string> targets;
  double timeout{2.0};
  bool camera{true};
  bool rosbridge{true};
  bool drive{true};
  int rumble_ms{0};
  int seconds{5};
  int port{5001}, discovery_port{5002}, rosbridge_port{9090}, camera_port{8080};
};

void usage() {
  std::printf(
      "ArmPi Ally CLI %s\n"
      "사용법:\n"
      "  armpi_ally_cli discover [--target IP]... [--timeout 초] [--discovery-port N]\n"
      "  armpi_ally_cli selftest --robot IP [--no-camera] [--no-rosbridge] [--no-drive]\n"
      "                          [--port N] [--rosbridge-port N] [--camera-port N]\n"
      "  armpi_ally_cli pads [--seconds N] [--rumble 밀리초]\n"
      "  armpi_ally_cli version\n",
      kVersion);
}

int cmd_discover(const Args& a, EventLog& log) {
  std::string err;
  std::printf("[*] 로봇 찾는 중(%.1f초, 대상 %s)\n", a.timeout, a.targets.empty() ? "모든 브로드캐스트" : "지정");
  const auto found = net::discover(a.timeout, a.targets, nullptr, &err, a.discovery_port);
  if (found.empty()) {
    std::printf("[-] 응답한 로봇 없음%s%s\n", err.empty() ? "" : ": ", err.c_str());
    return 1;
  }
  for (const auto& r : found)
    std::printf("[+] %-15s 포트 %d  이름 %s  id %s  %.0f ms\n", r.address.c_str(), r.control_port, r.name.c_str(),
                r.device_id.c_str(), r.rtt_ms);
  (void)log;
  return 0;
}

struct Check {
  int pass{0}, fail{0}, skip{0};
  void ok(const std::string& name, bool cond, const std::string& detail = "") {
    if (cond) ++pass;
    else ++fail;
    std::printf("%s %s%s%s\n", cond ? "[PASS]" : "[FAIL]", name.c_str(), detail.empty() ? "" : " — ", detail.c_str());
    std::fflush(stdout);
  }
  void skipped(const std::string& name, const std::string& why) {
    ++skip;
    std::printf("[SKIP] %s — %s\n", name.c_str(), why.c_str());
  }
};

int cmd_selftest(const Args& a, EventLog& log) {
  if (a.robot.empty()) {
    std::printf("[-] --robot IP 가 필요합니다\n");
    return 2;
  }
  net::CommLog comm;
  robot::CoreOptions opt;
  opt.host = a.robot;
  opt.control_port = a.port;
  opt.rosbridge_port = a.rosbridge ? a.rosbridge_port : 0;
  opt.camera_port = a.camera_port;
  robot::RobotCore core(opt, log, comm);
  Check c;
  std::string err;
  const double t0 = mono_now();
  c.ok("UDP 세션 열기", core.start(&err), err);
  if (!core.started()) return 1;

  // keep a neutral input flowing like the GUI does every frame
  auto hold = [&](robot::ControlInput in, double secs) {
    const double end = mono_now() + secs;
    while (mono_now() < end) {
      core.set_input(in);
      sleep_s(0.02);
    }
  };
  c.ok("로봇 ACK 수신", wait_until(3.0, [&] { core.set_input({}); return core.snapshot().connected; }),
       strf("%.0f ms", (mono_now() - t0) * 1000));
  const auto s0 = core.snapshot();
  c.ok("차체 채널 중립 확인", wait_until(2.0, [&] { core.set_input({}); return core.snapshot().base.neutral_confirmed; }),
       "state=" + core.snapshot().base.server_state);

  if (a.rosbridge) {
    c.ok("rosbridge 연결", wait_until(5.0, [&] { return core.ros().connected(); }), core.ros().last_error());
    c.ok("로봇 상태(/armpi/status) 수신", wait_until(4.0, [&] { return core.ros().status_fresh(); }));
    const auto st = core.ros().status();
    if (st.battery_v) std::printf("       배터리 %.2f V, 하드웨어 %s, 비상정지 %s\n", *st.battery_v, st.hardware.c_str(),
                                  st.emergency_stop ? "걸림" : "아님");
  } else {
    c.skipped("rosbridge", "--no-rosbridge");
  }

  if (a.drive) {
    robot::ControlInput fwd;
    fwd.forward = 0.5;
    bool active = false;
    const double end = mono_now() + 1.0;
    while (mono_now() < end && !active) {
      core.set_input(fwd);
      active = core.snapshot().base.server_state == "active";
      sleep_s(0.02);
    }
    c.ok("차체 앞으로(0.5×속도) 활성", active, "applied_f=" + strf("%.2f", core.snapshot().applied_f));
    hold({}, 0.3);
    c.ok("손 떼면 차체 중립", core.snapshot().applied_f == 0.0);
  } else {
    c.skipped("차체 주행", "--no-drive");
  }

  core.set_arm_enabled(true);
  const auto before = core.snapshot().target;
  robot::ControlInput jog;
  jog.arm[0] = 1.0;  // +x
  bool arm_active = false;
  const double jend = mono_now() + 1.5;
  while (mono_now() < jend) {
    core.set_input(jog);
    arm_active = arm_active || core.snapshot().arm.server_state == "active";
    sleep_s(0.02);
  }
  const auto after = core.snapshot().target;
  c.ok("팔 채널 활성", arm_active, "state=" + core.snapshot().arm.server_state);
  c.ok("팔 목표 +x 이동", after.x > before.x + 0.01, strf("x %.3f → %.3f m", before.x, after.x));
  // Release the stick first: a held arm input (intentionally) cancels absolute goals.
  hold({}, 0.15);
  // The mock robot keeps its pose between runs, so aim the gripper away from wherever it is now.
  const double grip_goal = core.snapshot().target.gripper < 0.5 ? 0.9 : 0.1;
  core.set_gripper(grip_goal);
  const double g0 = mono_now();
  hold({}, 0.15);  // snapshot updates at 20 Hz
  const bool grip_ok = wait_until(2.5, [&] { core.set_input({}); return std::fabs(core.snapshot().target.gripper - grip_goal) < 1e-3; });
  const double grip_s = mono_now() - g0;
  const auto gs = core.snapshot();
  c.ok(strf("그리퍼 목표 %.1f(램프 0.8/s)", grip_goal), grip_ok && grip_s > 0.5,
       strf("%.2f s, 현재 %.2f, 팔 %s·%s armed=%d goal=%d fresh=%d", grip_s, gs.target.gripper,
            gs.arm_enabled ? "켬" : "끔", gs.arm.server_state.c_str(), gs.arm.armed, gs.goal_active, gs.input_fresh));
  core.arm_goto(robot::ArmTarget{0.15, 0.05, 0.10, -0.5, grip_goal});
  hold({}, 0.15);
  double max_step = 0.0;
  auto prev = core.snapshot().target;
  const bool goal_ok = wait_until(3.0, [&] {
    core.set_input({});
    const auto t = core.snapshot().target;
    max_step = std::max(max_step, std::sqrt((t.x - prev.x) * (t.x - prev.x) + (t.y - prev.y) * (t.y - prev.y) +
                                            (t.z - prev.z) * (t.z - prev.z)));
    prev = t;
    return !core.snapshot().goal_active;
  });
  const auto tg = core.snapshot().target;
  c.ok("절대 목표로 천천히 이동(점프 없음)", goal_ok && std::fabs(tg.x - 0.15) < 2e-3 && max_step < 0.03,
       strf("x=%.3f y=%.3f z=%.3f, 한 번에 최대 %.1f mm", tg.x, tg.y, tg.z, max_step * 1000));
  core.arm_goto(robot::ArmTarget{0.60, 0.0, 0.10, 0.0, grip_goal});
  hold({}, 0.15);
  wait_until(4.0, [&] { core.set_input({}); return !core.snapshot().goal_active; });
  const auto clamped = core.snapshot().target;
  c.ok("닿지 않는 목표(x=0.60)는 작업 공간 안에서 멈춤", clamped.x <= 0.3201 && !core.snapshot().goal_active,
       strf("x=%.3f", clamped.x));

  core.estop();
  c.ok("비상정지 latch", wait_until(1.5, [&] { core.set_input({}); return core.snapshot().emergency; }));
  c.ok("비상정지 뒤 팔 조종 꺼짐", !core.snapshot().arm_enabled);
  core.clear_estop();
  c.ok("비상정지 해제", wait_until(1.5, [&] { core.set_input({}); return !core.snapshot().emergency; }));

  if (a.rosbridge && core.ros().connected()) {
    std::string e;
    const auto before_res = core.ros().last_result();
    c.ok("기본 자세 요청", core.arm_home(&e), e);
    const bool got = wait_until(2.5, [&] {
      core.set_input({});
      const auto r = core.ros().last_result();
      return r && (!before_res || r->mono != before_res->mono) && r->service == "/armpi/arm/home";
    });
    const auto r = core.ros().last_result();
    c.ok("기본 자세 응답", got && r && r->ok, r ? r->message : "응답 없음");
  }

  if (a.camera) {
    core.start_camera();
    std::shared_ptr<const robot::Frame> f;
    const bool got = wait_until(5.0, [&] { core.set_input({}); return (f = core.camera().latest(0)) != nullptr; });
    c.ok("카메라 프레임 해독", got, got ? strf("%dx%d", f->width, f->height) : core.camera().status());
    core.stop_camera();
  } else {
    c.skipped("카메라", "--no-camera");
  }

  if (a.rosbridge && core.ros().connected()) {
    core.release_session();
    std::string why;
    const bool allowed = wait_until(3.0, [&] { return core.joint_debug_allowed(&why); });
    c.ok("세션 놓은 뒤 관절 직접 조종 허용", allowed, why);
    if (allowed) {
      std::string e;
      c.ok("관절 직접 조종 발행", core.joint_debug({0, 0.2, -0.3, -0.5, 0, 0.3}, &e), e);
    }
    core.new_session();
    c.ok("새 세션 뒤 다시 ACK", wait_until(2.0, [&] { core.set_input({}); return core.snapshot().connected; }));
  }

  const auto sn = core.snapshot();
  std::printf("       ACK 지연 마지막 %.1f / 중앙 %.1f / 최대 %.1f ms, 보낸 패킷 %llu, ACK %llu, 루프 %.1f Hz\n",
              sn.ack_ms_last.value_or(-1), sn.ack_ms_p50.value_or(-1), sn.ack_ms_max.value_or(-1),
              static_cast<unsigned long long>(sn.packets_sent), static_cast<unsigned long long>(sn.acks), sn.loop_hz);
  (void)s0;
  core.stop();
  std::printf("결과: 통과 %d, 실패 %d, 건너뜀 %d (%.1f s)\n", c.pass, c.fail, c.skip, mono_now() - t0);
  return c.fail == 0 ? 0 : 1;
}

int cmd_pads(const Args& a) {
  input::Gamepad pad;
  std::printf("XInput: %s\n", pad.backend().c_str());
  if (!pad.available()) return 1;
  bool any = false;
  for (int i = 0; i < 4; ++i) {
    const auto p = pad.poll_index(i);
    std::printf("  슬롯 %d: %s\n", i, p.connected ? "연결됨" : "-");
    any = any || p.connected;
  }
  if (!any) {
    std::printf("[-] 연결된 게임패드 없음(Armoury Crate 에서 '게임패드' 모드인지 확인)\n");
    return 1;
  }
  const double end = mono_now() + a.seconds;
  if (a.rumble_ms > 0) {
    pad.poll();
    pad.set_rumble(0.6f, 0.6f);
    sleep_s(a.rumble_ms / 1000.0);
    pad.set_rumble(0, 0);
    std::printf("진동 %d ms 보냄\n", a.rumble_ms);
  }
  uint32_t last_packet = 0;
  while (mono_now() < end) {
    const auto p = pad.poll();
    if (p.connected && p.packet != last_packet) {
      last_packet = p.packet;
      std::printf("LS(%+.2f,%+.2f) RS(%+.2f,%+.2f) LT %.2f RT %.2f 버튼 0x%04x\n", p.lx, p.ly, p.rx, p.ry, p.lt, p.rt,
                  p.buttons);
    }
    sleep_s(0.05);
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
#ifdef _WIN32
  ::SetDefaultDllDirectories(LOAD_LIBRARY_SEARCH_SYSTEM32);
  ::SetConsoleOutputCP(CP_UTF8);
#endif
  std::string err;
  if (!net::init_sockets(&err)) {
    std::printf("[-] %s\n", err.c_str());
    return 1;
  }
  Args a;
  for (int i = 1; i < argc; ++i) {
    const std::string s = argv[i];
    auto next = [&]() -> std::string { return i + 1 < argc ? argv[++i] : ""; };
    if (a.cmd.empty() && s[0] != '-') a.cmd = s;
    else if (s == "--robot") a.robot = next();
    else if (s == "--target") a.targets.push_back(next());
    else if (s == "--timeout") a.timeout = parse_double(next()).value_or(2.0);
    else if (s == "--no-camera") a.camera = false;
    else if (s == "--no-rosbridge") a.rosbridge = false;
    else if (s == "--no-drive") a.drive = false;
    else if (s == "--rumble") a.rumble_ms = static_cast<int>(parse_int(next()).value_or(300));
    else if (s == "--port") a.port = static_cast<int>(parse_int(next()).value_or(5001));
    else if (s == "--discovery-port") a.discovery_port = static_cast<int>(parse_int(next()).value_or(5002));
    else if (s == "--rosbridge-port") a.rosbridge_port = static_cast<int>(parse_int(next()).value_or(9090));
    else if (s == "--camera-port") a.camera_port = static_cast<int>(parse_int(next()).value_or(8080));
    else if (s == "--seconds") a.seconds = static_cast<int>(parse_int(next()).value_or(5));
    else {
      usage();
      return 2;
    }
  }
  EventLog log;
  log.set_echo(a.cmd == "selftest");
  if (a.cmd == "discover") return cmd_discover(a, log);
  if (a.cmd == "selftest") return cmd_selftest(a, log);
  if (a.cmd == "pads") return cmd_pads(a);
  if (a.cmd == "version") {
    std::printf("%s\n", kVersion);
    return 0;
  }
  usage();
  return a.cmd.empty() ? 0 : 2;
}
