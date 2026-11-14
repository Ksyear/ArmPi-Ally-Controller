// Vision + autograsp unit tests (no framework, no robot, no camera): synthetic images and a fake robot.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "common/util.hpp"
#include "robot/autograsp.hpp"
#include "robot/core.hpp"
#include "vision/calibration.hpp"
#include "vision/detector.hpp"
#include "vision/homography.hpp"
#include "vision/image_ops.hpp"

using namespace ally;

static int g_fail = 0, g_pass = 0;
#define CHECK(cond)                                                 \
  do {                                                              \
    if (cond) {                                                     \
      ++g_pass;                                                     \
    } else {                                                        \
      ++g_fail;                                                     \
      std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
    }                                                               \
  } while (0)

namespace {

// ------------------------------------------------------------------------------------- synthetic images

struct Rgba {
  int w, h;
  std::vector<uint8_t> px;
  Rgba(int w_, int h_, uint8_t g = 128) : w(w_), h(h_), px(static_cast<size_t>(w_) * h_ * 4, g) {
    for (size_t i = 3; i < px.size(); i += 4) px[i] = 255;
  }
  void set(int x, int y, uint8_t r, uint8_t g, uint8_t b) {
    if (x < 0 || y < 0 || x >= w || y >= h) return;
    uint8_t* p = &px[(static_cast<size_t>(y) * w + x) * 4];
    p[0] = r;
    p[1] = g;
    p[2] = b;
  }
  void square(float cx, float cy, int side, uint8_t r, uint8_t g, uint8_t b) {
    const int x0 = static_cast<int>(std::lround(cx - side / 2.0f)), y0 = static_cast<int>(std::lround(cy - side / 2.0f));
    for (int y = y0; y < y0 + side; ++y)
      for (int x = x0; x < x0 + side; ++x) set(x, y, r, g, b);
  }
  void disc(float cx, float cy, float rad, uint8_t r, uint8_t g, uint8_t b) {
    for (int y = static_cast<int>(cy - rad) - 1; y <= static_cast<int>(cy + rad) + 1; ++y)
      for (int x = static_cast<int>(cx - rad) - 1; x <= static_cast<int>(cx + rad) + 1; ++x)
        if ((x + 0.5f - cx) * (x + 0.5f - cx) + (y + 0.5f - cy) * (y + 0.5f - cy) <= rad * rad) set(x, y, r, g, b);
  }
};

int class_named(const std::vector<vision::ObjectClass>& cls, const std::string& name) {
  for (size_t i = 0; i < cls.size(); ++i)
    if (cls[i].name == name) return static_cast<int>(i);
  return -1;
}

// The synthetic camera used by the tests and tools/fake_cam_server.py: u = 320 - 1500 y, v = 555 - 1500 x
// with a little perspective so the fit is not trivially affine.
const double kTrueH[9] = {0.0, -1500.0, 320.0, -1500.0, 0.0, 555.0, 0.0, 0.0, 1.0};

vision::VisionCalib synthetic_calib() {
  vision::VisionCalib c;
  c.image_w = 640;
  c.image_h = 480;
  c.z_table = -0.05;
  vision::Homography H;
  for (int i = 0; i < 9; ++i) H.h[i] = kTrueH[i];
  H.valid = true;
  for (const auto& xy : std::vector<std::pair<double, double>>{{0.12, 0}, {0.20, 0}, {0.16, 0.08}, {0.16, -0.08}, {0.24, 0}, {0.14, 0.05}}) {
    vision::PointPair p;
    p.x = xy.first;
    p.y = xy.second;
    vision::apply(H, p.x, p.y, &p.u, &p.v);
    c.points.push_back(p);
  }
  std::string err;
  c.refit(&err);
  return c;
}

// ------------------------------------------------------------------------------------------ test cases

void test_hsv() {
  uint16_t h;
  uint8_t s, v;
  vision::rgb_to_hsv(255, 0, 0, &h, &s, &v);
  CHECK(h == 0 && s == 255 && v == 255);
  vision::rgb_to_hsv(0, 255, 0, &h, &s, &v);
  CHECK(h == 120);
  vision::rgb_to_hsv(0, 0, 255, &h, &s, &v);
  CHECK(h == 240);
  vision::rgb_to_hsv(255, 255, 0, &h, &s, &v);
  CHECK(h == 60);
  vision::rgb_to_hsv(0, 255, 255, &h, &s, &v);
  CHECK(h == 180);
  vision::rgb_to_hsv(255, 0, 255, &h, &s, &v);
  CHECK(h == 300);
  vision::rgb_to_hsv(128, 128, 128, &h, &s, &v);
  CHECK(s == 0 && v == 128);
  // 2x2 average pooling: half red, half black -> dark red
  Rgba img(2, 2, 0);
  img.set(0, 0, 255, 0, 0);
  img.set(1, 0, 255, 0, 0);
  const auto hsv = vision::rgba_to_hsv_downscaled(img.px.data(), 2, 2, 2);
  CHECK(hsv.w == 1 && hsv.h == 1);
  CHECK(hsv.hue[0] == 0 && hsv.s[0] == 255 && std::abs(static_cast<int>(hsv.v[0]) - 128) <= 1);
  // Wrapping hue range (red)
  vision::HsvRange red{345, 15, 0.4f, 1.0f, 0.2f, 1.0f};
  CHECK(vision::in_range(red, 0, 255, 255) && vision::in_range(red, 350, 255, 255) && !vision::in_range(red, 30, 255, 255));
  CHECK(!vision::in_range(red, 0, 50, 255));  // too grey
}

void test_blobs() {
  Rgba img(640, 480);
  img.square(200, 200, 46, 220, 30, 30);  // red square
  img.square(450, 120, 30, 220, 30, 30);  // smaller red square
  img.disc(320, 360, 30, 30, 60, 220);    // blue disc
  img.square(40, 40, 4, 220, 30, 30);     // speck: filtered by min area
  const auto hsv = vision::rgba_to_hsv_downscaled(img.px.data(), img.w, img.h, 2);
  vision::Mask red = vision::threshold(hsv, vision::HsvRange{345, 15, 0.45f, 1.0f, 0.25f, 1.0f});
  vision::erode3(red);
  vision::dilate3(red);
  const auto rb = vision::connected_components(red, 15, 12);
  CHECK(rb.size() == 2);
  if (rb.size() == 2) {
    CHECK(std::fabs((rb[0].cx + 0.5f) * 2 - 200) <= 1.5f && std::fabs((rb[0].cy + 0.5f) * 2 - 200) <= 1.5f);
    // Half-covered edge pixels pool to a paler red that still passes: up to one extra row/column per side.
    CHECK(std::fabs(rb[0].area * 4.0f - 46 * 46) / (46 * 46) < 0.12f);
    CHECK(rb[0].area > rb[1].area);
    CHECK(rb[0].circularity < 0.85f && rb[0].oriented_fill > 0.85f);
  }
  vision::Mask blue = vision::threshold(hsv, vision::HsvRange{200, 250, 0.45f, 1.0f, 0.25f, 1.0f});
  vision::erode3(blue);
  vision::dilate3(blue);
  const auto bb = vision::connected_components(blue, 15, 12);
  CHECK(bb.size() == 1);
  if (bb.size() == 1) {
    CHECK(bb[0].circularity >= 0.85f);
    CHECK(std::fabs((bb[0].cx + 0.5f) * 2 - 320) <= 1.5f && std::fabs((bb[0].cy + 0.5f) * 2 - 360) <= 1.5f);
  }
  // max_blobs cap
  CHECK(vision::connected_components(red, 15, 1).size() == 1);
}

void test_detector() {
  vision::Detector det;
  const auto cls = det.classes();
  const int red_cube = class_named(cls, "빨간 블록"), blue_ball = class_named(cls, "파란 공");
  CHECK(red_cube >= 0 && blue_ball >= 0);
  Rgba a(640, 480);
  a.square(320, 285, 45, 220, 30, 30);
  a.disc(440, 330, 30, 30, 60, 220);
  const auto da = det.detect_rgba(a.px.data(), a.w, a.h, nullptr);
  CHECK(da.items.size() == 2);
  int cube_track = -1, ball_track = -1;
  for (const auto& d : da.items) {
    if (d.class_index == red_cube) {
      cube_track = d.track_id;
      CHECK(std::fabs(d.u - 320) <= 2 && std::fabs(d.v - 285) <= 2);
    }
    if (d.class_index == blue_ball) {
      ball_track = d.track_id;
      CHECK(std::fabs(d.u - 440) <= 2 && std::fabs(d.v - 330) <= 2);
    }
  }
  CHECK(cube_track > 0 && ball_track > 0 && cube_track != ball_track);
  if (da.items.size() == 2) CHECK(da.items[0].score >= da.items[1].score);
  // Objects move 3 px: same track ids
  Rgba b(640, 480);
  b.square(323, 287, 45, 220, 30, 30);
  b.disc(437, 331, 30, 30, 60, 220);
  const auto db = det.detect_rgba(b.px.data(), b.w, b.h, &da);
  bool kept_cube = false, kept_ball = false;
  for (const auto& d : db.items) {
    kept_cube = kept_cube || (d.class_index == red_cube && d.track_id == cube_track);
    kept_ball = kept_ball || (d.class_index == blue_ball && d.track_id == ball_track);
  }
  CHECK(kept_cube && kept_ball);
  // Class dictionary JSON round trip and validation
  std::vector<vision::ObjectClass> back;
  std::string err;
  CHECK(vision::classes_from_json(vision::classes_to_json(cls), &back, &err));
  CHECK(back.size() == cls.size() && back[0].name == cls[0].name && back[1].shape == vision::Shape::kBall);
  CHECK(!vision::classes_from_json("{\"classes\": []}", &back, &err));
  CHECK(!vision::classes_from_json("{\"classes\": [{\"name\": \"x\", \"shape\": \"cube\", \"h\": [0, 400], \"s\": [0, 1], \"v\": [0, 1]}]}",
                                   &back, &err));
}

void test_homography() {
  vision::Homography T;
  const double ht[9] = {12.0, -1450.0, 330.0, -1500.0, 40.0, 560.0, 0.3, 0.1, 1.0};
  for (int i = 0; i < 9; ++i) T.h[i] = ht[i];
  T.valid = true;
  std::vector<vision::PointPair> pts;
  for (double x : {0.10, 0.16, 0.22, 0.26})
    for (double y : {-0.08, 0.06}) {
      vision::PointPair p;
      p.x = x;
      p.y = y;
      vision::apply(T, x, y, &p.u, &p.v);
      pts.push_back(p);
    }
  std::string err;
  const auto H = vision::fit_homography(pts, &err);
  CHECK(H.valid);
  double worst = 0;
  for (const auto& p : pts) {
    double u, v;
    vision::apply(H, p.x, p.y, &u, &v);
    worst = std::max(worst, std::hypot(u - p.u, v - p.v));
  }
  CHECK(worst < 1e-6);
  // invert + apply round trip
  const auto Hi = vision::invert(H);
  CHECK(Hi.valid);
  double x, y, u, v;
  vision::apply(H, 0.18, -0.03, &u, &v);
  vision::apply(Hi, u, v, &x, &y);
  CHECK(std::fabs(x - 0.18) < 1e-9 && std::fabs(y + 0.03) < 1e-9);
  // 1 px Gaussian noise -> reprojection RMS stays small
  std::mt19937 rng(7);
  std::normal_distribution<double> n(0.0, 1.0);
  auto noisy = pts;
  for (auto& p : noisy) {
    p.u += n(rng);
    p.v += n(rng);
  }
  const auto Hn = vision::fit_homography(noisy, &err);
  CHECK(Hn.valid && vision::residual(Hn, noisy).rms_px <= 1.5);
  // Degenerate: collinear points
  std::vector<vision::PointPair> line;
  for (int i = 0; i < 4; ++i) line.push_back({100.0 + i * 10, 200.0, 0.1 + i * 0.01, 0.0});
  CHECK(!vision::fit_homography(line, &err).valid);
  CHECK(!vision::fit_homography(std::vector<vision::PointPair>(pts.begin(), pts.begin() + 3), &err).valid);
}

void test_calib() {
  const auto c = synthetic_calib();
  CHECK(c.H.valid && c.res.rms_px < 1e-6);
  std::string why;
  CHECK(c.usable(640, 480, &why));
  CHECK(!c.usable(320, 240, &why));
  vision::VisionCalib noz = c;
  noz.z_table.reset();
  CHECK(!noz.usable(640, 480, &why));
  double x, y, u, v;
  CHECK(c.table_to_pixel(0.18, 0.0, &u, &v) && std::fabs(u - 320) < 1e-6 && std::fabs(v - 285) < 1e-6);
  CHECK(c.pixel_to_table(440, 330, 0, &x, &y) && std::fabs(x - 0.15) < 1e-6 && std::fabs(y + 0.08) < 1e-6);
  // JSON round trip
  vision::VisionCalib back;
  std::string err;
  CHECK(vision::calib_from_json(vision::calib_to_json(c), &back, &err));
  CHECK(back.H.valid && back.z_table && std::fabs(*back.z_table + 0.05) < 1e-12 && back.points.size() == c.points.size());
  CHECK(back.image_w == 640 && back.image_h == 480);
  // Rejections: non-finite, absurd values, broken JSON
  CHECK(!vision::calib_from_json("{\"image_w\": 640, \"image_h\": 480, \"z_table\": 1e999}", &back, &err));
  CHECK(!vision::calib_from_json("{\"image_w\": 99999, \"image_h\": 480}", &back, &err));
  CHECK(!vision::calib_from_json("{\"image_w\": 640, \"image_h\": 480, \"z_table\": 5}", &back, &err));
  CHECK(!vision::calib_from_json("{\"image_w\": 640, \"image_h\": 480, \"z_table\": -0.2}", &back, &err));
  CHECK(!vision::calib_from_json("{\"image_w\": 640, \"image_h\": 480, \"observe_pose\": {\"x\": 0.45, \"y\": 0, \"z\": 0.1, \"pitch\": -1}}",
                                 &back, &err));
  CHECK(!vision::calib_from_json("{\"image_w\": 640, ", &back, &err));
}

// ----------------------------------------------------------------------------------------- fake robot

struct FakeRobot : robot::GraspRobot {
  robot::CoreSnapshot s;
  std::shared_ptr<armpi_kinematics::TargetProjector> proj = robot::make_reach_projector();
  bool auto_reach{true};  // goals finish immediately
  int stall_next{0};      // that many next arm_goto calls end stalled
  int cancels{0};
  std::vector<robot::ArmTarget> gotos;
  std::vector<double> rates;
  // Base: pose in the world (starts at the origin), integrated like the mock robot (0.25 m/s, 1.2 rad/s at full stick).
  double now{0}, px{0}, py{0}, yaw{0};
  double af{0}, at{0}, a_until{-1};
  int auto_cmds{0};

  FakeRobot() {
    s.connected = true;
    s.speed = 0.3;
    s.target = robot::ArmTarget{0.1047, 0.0, 0.0753, -1.0263, 0.0};
  }
  uint64_t start(const robot::ArmTarget& t) {
    ++s.goal_id;
    s.goal_active = true;
    s.goal_result = robot::GoalResult::kNone;
    if (stall_next > 0) {
      --stall_next;
      s.goal_active = false;
      s.goal_result = robot::GoalResult::kStalled;
    } else if (auto_reach) {
      s.target = t;
      s.goal_active = false;
      s.goal_result = robot::GoalResult::kReached;
    }
    return s.goal_id;
  }
  uint64_t arm_goto(const robot::ArmTarget& t, double rate) override {
    gotos.push_back(t);
    rates.push_back(rate);
    return start(t);
  }
  uint64_t set_gripper(double g) override {
    robot::ArmTarget t = s.target;
    t.gripper = g;
    return start(t);
  }
  void cancel_goal() override {
    ++cancels;
    if (s.goal_active) s.goal_result = robot::GoalResult::kCancelled;
    s.goal_active = false;
  }
  robot::CoreSnapshot snapshot() const override {
    robot::CoreSnapshot c = s;
    c.auto_base_active = base_moving();
    return c;
  }
  bool reachable(const robot::ArmTarget& t) const override {
    return proj->reachable(armpi_kinematics::Pose{t.x, t.y, t.z, t.pitch});
  }
  bool set_auto_base(double f, double t, double ttl) override {
    if ((std::fabs(f) < 1e-9 && std::fabs(t) < 1e-9) || ttl <= 0) {
      a_until = -1;
      return true;
    }
    if (s.speed > 0.3 + 1e-9) {
      a_until = -1;
      return false;
    }
    af = std::clamp(f, -0.3, 0.3);
    at = std::clamp(t, -0.35, 0.35);
    a_until = now + std::min(ttl, 0.5);
    ++auto_cmds;
    return true;
  }
  bool base_moving() const { return now < a_until; }
  void advance(double dt) {
    if (!base_moving()) return;
    const double vx = af * s.speed * 0.25, wz = at * s.speed * 1.2;
    px += vx * std::cos(yaw) * dt;
    py += vx * std::sin(yaw) * dt;
    yaw += wz * dt;
  }
};

struct FakeScene {
  struct Obj {
    std::string cls;
    double wx, wy;  // world position (= robot frame at the start)
    int track;
    bool shown{true};
  };
  std::shared_ptr<vision::DetectionSet> cur;
  std::vector<vision::ObjectClass> cls = vision::default_classes();
  std::vector<Obj> objs{{"빨간 블록", 0.18, 0.00, 7}};
  bool present{true};  // shortcut for objs[0].shown
  // brightness thumbnail (80 x 60 = 1/8 of 640 x 480): flat table + optional bright patch in image pixels
  bool with_gray{false};
  uint8_t base_gray{128};
  int patch[4]{0, 0, 0, 0};
  uint64_t seq{0};
  void frame(double now, const FakeRobot& bot) {
    auto ds = std::make_shared<vision::DetectionSet>();
    ds->frame_seq = ++seq;
    ds->frame_mono = now;
    ds->img_w = 640;
    ds->img_h = 480;
    if (!objs.empty()) objs[0].shown = present;
    for (const auto& o : objs) {
      if (!o.shown) continue;
      // world -> robot frame -> synthetic camera (u = 320 - 1500 y, v = 555 - 1500 x)
      const double dx = o.wx - bot.px, dy = o.wy - bot.py;
      const double xr = std::cos(bot.yaw) * dx + std::sin(bot.yaw) * dy;
      const double yr = -std::sin(bot.yaw) * dx + std::cos(bot.yaw) * dy;
      vision::Detection d;
      d.class_index = class_named(cls, o.cls);
      d.u = static_cast<float>(320.0 - 1500.0 * yr);
      d.v = static_cast<float>(555.0 - 1500.0 * xr);
      if (d.u < 0 || d.v < 0 || d.u >= 640 || d.v >= 480) continue;
      d.area_px = 45 * 45;
      d.score = 1;
      d.track_id = o.track;
      ds->items.push_back(d);
    }
    if (with_gray) {
      ds->gw = 80;
      ds->gh = 60;
      ds->gray.assign(80 * 60, base_gray);
      for (int y = 0; y < 60; ++y)
        for (int x = 0; x < 80; ++x) {
          const int u = x * 8 + 4, v = y * 8 + 4;
          if (u >= patch[0] && u < patch[2] && v >= patch[1] && v < patch[3]) ds->gray[static_cast<size_t>(y * 80 + x)] = 235;
        }
    }
    cur = ds;
  }
  robot::GraspVision source() {
    return robot::GraspVision{[this] { return std::shared_ptr<const vision::DetectionSet>(cur); }, [this] { return cls; }};
  }
};

struct Rig {
  FakeRobot bot;
  FakeScene scene;
  EventLog log;
  robot::GraspSequencer seq;
  double now{100.0};
  explicit Rig(robot::GraspOptions o = {}) : seq(bot, scene.source(), log, o) {
    seq.set_calib(synthetic_calib());
    bot.now = now;
    scene.frame(now, bot);
  }
  // Advance ~one control tick: base motion, a new camera frame, one sequencer tick.
  void step(bool deadman) {
    bot.advance(0.06);
    now += 0.06;
    bot.now = now;
    scene.frame(now, bot);
    seq.tick(now, deadman);
  }
  bool run_until(robot::GraspState want, bool deadman, int max_steps = 200) {
    for (int i = 0; i < max_steps; ++i) {
      if (seq.status().state == want) return true;
      step(deadman);
    }
    return seq.status().state == want;
  }
  void press() {  // release then press: a fresh dead-man edge
    step(false);
    step(true);
  }
  void start(int track = 7) {
    CHECK(seq.select(track, now));
    press();
  }
};

void test_planner() {
  FakeRobot bot;
  robot::GraspPlanner pl(bot, robot::GraspOptions{});
  const auto calib = synthetic_calib();
  const auto cls = vision::default_classes();
  const auto& cube = cls[static_cast<size_t>(class_named(cls, "빨간 블록"))];
  std::string err;
  const auto p = pl.plan(calib, 320, 285, cube, &err);
  CHECK(p.has_value());
  if (p) {
    CHECK(std::fabs(p->obj_x - 0.18) < 1e-6 && std::fabs(p->obj_y) < 1e-6);
    CHECK(std::fabs(p->grasp.z - (-0.05 + cube.grasp_depth_m)) < 1e-9);
    CHECK(std::fabs(p->hover.z - p->grasp.z - 0.06) < 1e-9 && std::fabs(p->lift.z - p->grasp.z - 0.08) < 1e-9);
    CHECK(bot.reachable(p->grasp) && bot.reachable(p->hover) && bot.reachable(p->lift));
    CHECK(p->grasp.gripper == 0.0 && std::fabs(p->lift.gripper - cube.gripper_close) < 1e-9);
    std::printf("  plan at (0.18, 0): pitch %.0f deg\n", p->pitch_deg);
  }
  // The floor: a negative depth never goes below table + margin
  vision::ObjectClass flat = cube;
  flat.grasp_depth_m = -0.02;
  const auto pf = pl.plan_at(calib, 0.18, 0.0, flat, &err);
  CHECK(pf && pf->grasp.z >= -0.05 + 0.005 - 1e-12);
  // Out of reach
  CHECK(!pl.plan_at(calib, 0.40, 0.0, cube, &err) && !err.empty());
  // No table height -> refused
  auto noz = calib;
  noz.z_table.reset();
  CHECK(!pl.plan_at(noz, 0.18, 0.0, cube, &err));
  // Reach band: a closed polygon around the reachable table area; the planned point is inside it.
  const auto band = pl.reach_band_pixels(calib, 24);
  CHECK(band.size() >= 10);
  double u, v;
  calib.table_to_pixel(0.18, 0.0, &u, &v);
  bool inside = false;
  for (size_t i = 0, j = band.size() - 1; i < band.size(); j = i++) {
    const auto& a = band[i];
    const auto& b = band[j];
    if ((a.second > v) != (b.second > v) && u < (b.first - a.first) * (v - a.second) / (b.second - a.second) + a.first)
      inside = !inside;
  }
  CHECK(inside);
  calib.table_to_pixel(0.40, 0.0, &u, &v);
  bool inside_far = false;
  for (size_t i = 0, j = band.size() - 1; i < band.size(); j = i++) {
    const auto& a = band[i];
    const auto& b = band[j];
    if ((a.second > v) != (b.second > v) && u < (b.first - a.first) * (v - a.second) / (b.second - a.second) + a.first)
      inside_far = !inside_far;
  }
  CHECK(!inside_far);
}

void test_sequencer_full() {
  Rig r;
  CHECK(r.seq.status().state == robot::GraspState::kIdle);
  r.start();
  CHECK(r.seq.status().state == robot::GraspState::kObserve);
  CHECK(r.run_until(robot::GraspState::kConfirm, true));
  const auto st = r.seq.status();
  CHECK(st.plan.has_value() && std::fabs(st.orig_u - 320) < 1e-3 && std::fabs(st.orig_v - 285) < 1e-3);
  // Still holding A at kConfirm: nothing moves until a fresh press
  const size_t gotos = r.bot.gotos.size();
  for (int i = 0; i < 5; ++i) r.step(true);
  CHECK(r.seq.status().state == robot::GraspState::kConfirm && r.bot.gotos.size() == gotos);
  // Nudge the grasp point by +5 mm in x
  const double gx = st.plan->grasp.x;
  r.seq.nudge(0.02, 0, 0);  // clamped to one 5 mm step
  CHECK(std::fabs(r.seq.status().plan->grasp.x - (gx + 0.005)) < 1e-9);
  r.press();
  CHECK(r.seq.status().state != robot::GraspState::kConfirm);
  // Object gone after the lift -> verify succeeds
  r.scene.present = false;
  CHECK(r.run_until(robot::GraspState::kDone, true));
  CHECK(r.seq.status().done_events == 1 && r.seq.status().fail_events == 0);
  // Order of arm goals: observe, hover, grasp (slow), lift (slow), observe again
  CHECK(r.bot.gotos.size() == 5);
  if (r.bot.gotos.size() == 5) {
    const auto& g = r.bot.gotos;
    CHECK(std::fabs(g[1].z - g[2].z - 0.06) < 1e-9);
    CHECK(std::fabs(g[3].z - g[2].z - 0.08) < 1e-9);
    CHECK(std::fabs(g[2].x - (gx + 0.005)) < 1e-9);
    CHECK(r.bot.rates[2] < 0.5 && r.bot.rates[3] < 0.5 && r.bot.rates[1] == 1.0);
    CHECK(std::fabs(g[4].x - 0.1047) < 1e-9 && g[4].gripper > 0.5);
  }
  CHECK(r.bot.s.target.gripper > 0.5);  // still holding
}

void test_sequencer_safety() {
  {  // Releasing A pauses and cancels; pressing again re-sends the same step
    Rig r;
    r.bot.auto_reach = false;
    r.start();
    CHECK(r.seq.status().state == robot::GraspState::kObserve);
    const size_t n = r.bot.gotos.size();
    r.step(false);
    CHECK(r.seq.status().state == robot::GraspState::kPaused && r.bot.cancels >= 1);
    CHECK(r.seq.status().resume_state == robot::GraspState::kObserve);
    r.step(true);
    CHECK(r.seq.status().state == robot::GraspState::kObserve && r.bot.gotos.size() == n + 1);
  }
  {  // Emergency stop aborts
    Rig r;
    r.bot.auto_reach = false;
    r.start();
    r.step(true);
    r.bot.s.emergency = true;
    r.step(true);
    CHECK(r.seq.status().state == robot::GraspState::kAborted && r.seq.status().fail_events == 1);
  }
  {  // Link loss aborts, even while paused
    Rig r;
    r.bot.auto_reach = false;
    r.start();
    r.step(false);
    CHECK(r.seq.status().state == robot::GraspState::kPaused);
    r.bot.s.connected = false;
    r.step(false);
    CHECK(r.seq.status().state == robot::GraspState::kAborted);
  }
  {  // The user moves the arm (core cancels the goal) -> pause
    Rig r;
    r.bot.auto_reach = false;
    r.start();
    r.step(true);
    r.bot.s.goal_result = robot::GoalResult::kCancelled;
    r.step(true);
    CHECK(r.seq.status().state == robot::GraspState::kPaused);
  }
  {  // Another goal (e.g. an arm-tab button) -> pause
    Rig r;
    r.bot.auto_reach = false;
    r.start();
    r.step(true);
    ++r.bot.s.goal_id;
    r.step(true);
    CHECK(r.seq.status().state == robot::GraspState::kPaused);
  }
  {  // Confirm waits at most 30 s
    Rig r;
    r.start();
    CHECK(r.run_until(robot::GraspState::kConfirm, true));
    CHECK(r.run_until(robot::GraspState::kPaused, false, 600));
    CHECK(r.seq.status().resume_state == robot::GraspState::kConfirm);
  }
  {  // Descend stalls twice -> one retry (5 mm higher), then fail
    robot::GraspOptions o;
    o.confirm_before_descend = false;
    Rig r2(o);
    r2.start();
    CHECK(r2.run_until(robot::GraspState::kHover, true));
    // The hover goal is already reached (sent on entry); the next two arm goals are descents: make them stall.
    r2.bot.stall_next = 2;
    CHECK(r2.run_until(robot::GraspState::kFailed, true));
    CHECK(r2.seq.status().failed_state == robot::GraspState::kDescend);
    const auto& g = r2.bot.gotos;
    CHECK(g.size() >= 2 && std::fabs(g[g.size() - 1].z - g[g.size() - 2].z - 0.005) < 1e-9);
  }
  {  // Object still there after the lift -> failed (no retries by default)
    robot::GraspOptions o;
    o.confirm_before_descend = false;
    Rig r(o);
    r.start();
    CHECK(r.run_until(robot::GraspState::kFailed, true));
    CHECK(r.seq.status().failed_state == robot::GraspState::kVerify);
  }
  {  // ... and with one retry it goes round once more
    robot::GraspOptions o;
    o.confirm_before_descend = false;
    o.max_retries = 1;
    Rig r(o);
    r.start();
    CHECK(r.run_until(robot::GraspState::kFailed, true, 400));
    CHECK(r.seq.status().attempt == 1 && r.bot.gotos.size() == 10);
  }
  {  // No calibration -> refuses to start
    Rig r;
    r.seq.set_calib(vision::VisionCalib{});
    r.start();
    CHECK(r.seq.status().state == robot::GraspState::kSelected);
    CHECK(r.bot.gotos.empty());
  }
  {  // Selection disappears for 3 frames -> back to idle
    Rig r;
    CHECK(r.seq.select(7, r.now));
    r.scene.present = false;
    for (int i = 0; i < 4; ++i) r.step(false);
    CHECK(r.seq.status().state == robot::GraspState::kIdle && r.seq.status().selected_track < 0);
  }
  {  // Cancel from anywhere
    Rig r;
    r.bot.auto_reach = false;
    r.start();
    r.seq.cancel();
    CHECK(r.seq.status().state == robot::GraspState::kIdle && r.bot.cancels >= 1);
  }
}

void test_parallax() {
  auto c = synthetic_calib();
  const double tpx = -0.8, tpy = 0.3;  // true parallax: table shift per metre of height
  double u0, v0, u1, v1;
  c.table_to_pixel(0.18, 0.05, &u0, &v0);                          // table mark
  c.table_to_pixel(0.18 + 0.03 * tpx, 0.05 + 0.03 * tpy, &u1, &v1);  // top of a 3 cm block standing on it
  std::string err;
  CHECK(c.measure_parallax(u0, v0, u1, v1, 0.03, &err));
  CHECK(std::fabs(c.parallax_x - tpx) < 1e-6 && std::fabs(c.parallax_y - tpy) < 1e-6);
  // A 3 cm cube at (0.20, -0.02): its blob centre is about 1.5 cm up -> corrected back onto its base.
  double bu, bv, x, y;
  c.table_to_pixel(0.20 + 0.015 * tpx, -0.02 + 0.015 * tpy, &bu, &bv);
  CHECK(c.pixel_to_table(bu, bv, 0.03, &x, &y) && std::fabs(x - 0.20) < 1e-6 && std::fabs(y + 0.02) < 1e-6);
  CHECK(c.pixel_to_table(bu, bv, 0.0, &x, &y) && std::fabs(x - 0.20) > 0.01);  // no correction without a height
  // Bad inputs
  CHECK(!c.measure_parallax(u0, v0, u1, v1, 0.0, &err));
  double fu, fv;
  c.table_to_pixel(0.18 + 0.03 * 9.0, 0.05, &fu, &fv);
  CHECK(!c.measure_parallax(u0, v0, fu, fv, 0.03, &err));
  // JSON keeps it; out-of-range values are refused
  vision::VisionCalib back;
  CHECK(vision::calib_from_json(vision::calib_to_json(c), &back, &err) && std::fabs(back.parallax_x - tpx) < 1e-9);
  CHECK(!vision::calib_from_json("{\"image_w\": 640, \"image_h\": 480, \"parallax\": [9, 9]}", &back, &err));
}

void test_approach() {
  robot::GraspOptions o;
  o.base_approach = true;
  {  // Far object: turn towards it, drive in, stop, observe again, plan
    Rig r(o);
    r.scene.objs.push_back({"초록 블록", 0.36, 0.08, 9});
    r.step(false);
    r.start(9);
    bool approached = false;
    for (int i = 0; i < 800 && r.seq.status().state != robot::GraspState::kConfirm; ++i) {
      r.step(true);
      approached = approached || r.seq.status().state == robot::GraspState::kApproachBase;
      if (r.seq.status().state == robot::GraspState::kFailed) break;
    }
    const auto& st = r.seq.status();
    CHECK(approached);
    CHECK(st.state == robot::GraspState::kConfirm);
    CHECK(st.plan.has_value() && !r.bot.base_moving() && r.bot.auto_cmds > 0);
    CHECK(st.approach_used_s > 0.5 && st.approach_used_s < o.approach_total_s);
    CHECK(std::fabs(r.bot.yaw) > 0.05 && r.bot.px > 0.01);  // it turned left and drove forward
    std::printf("  approach: %.1f s of base motion, base moved %.1f cm, turned %.0f deg, %s\n", st.approach_used_s,
                std::hypot(r.bot.px, r.bot.py) * 100, r.bot.yaw * 180 / 3.14159265, st.message.c_str());
  }
  {  // Letting go of A while the base moves stops it at once
    Rig r(o);
    r.scene.objs.push_back({"초록 블록", 0.36, 0.08, 9});
    r.step(false);
    r.start(9);
    CHECK(r.run_until(robot::GraspState::kApproachBase, true));
    for (int i = 0; i < 5; ++i) r.step(true);
    CHECK(r.bot.base_moving());
    r.step(false);
    CHECK(r.seq.status().state == robot::GraspState::kPaused && !r.bot.base_moving());
    CHECK(r.seq.status().resume_state == robot::GraspState::kApproachBase);
  }
  {  // Speed limit above 30 %: no base approach
    Rig r(o);
    r.bot.s.speed = 0.6;
    r.scene.objs.push_back({"초록 블록", 0.36, 0.08, 9});
    r.step(false);
    r.start(9);
    CHECK(r.run_until(robot::GraspState::kFailed, true));
    CHECK(r.bot.auto_cmds == 0);
  }
  {  // Option off (default): out of reach simply fails
    Rig r;
    r.scene.objs.push_back({"초록 블록", 0.36, 0.08, 9});
    r.step(false);
    r.start(9);
    CHECK(r.run_until(robot::GraspState::kFailed, true));
    CHECK(r.bot.auto_cmds == 0 && r.seq.status().failed_state == robot::GraspState::kPlan);
  }
  {  // Object disappears during the approach -> stop, then fail
    Rig r(o);
    r.scene.objs.push_back({"초록 블록", 0.36, 0.08, 9});
    r.step(false);
    r.start(9);
    CHECK(r.run_until(robot::GraspState::kApproachBase, true));
    r.scene.objs[1].shown = false;
    r.step(true);
    CHECK(!r.bot.base_moving());
    CHECK(r.run_until(robot::GraspState::kFailed, true, 60));
  }
}

void test_intrusion() {
  robot::GraspOptions o;
  o.confirm_before_descend = false;
  o.intrusion_check = true;
  auto to_hover = [](Rig& r) {
    r.scene.with_gray = true;
    r.start();
    CHECK(r.run_until(robot::GraspState::kOpenGripper, true));
    r.bot.auto_reach = false;  // park in the hover move
    CHECK(r.run_until(robot::GraspState::kHover, true));
    CHECK(r.seq.status().intrusion_watch);
  };
  {  // Something big away from the arm's corridor -> pause
    Rig r(o);
    to_hover(r);
    r.scene.patch[0] = 40, r.scene.patch[1] = 40, r.scene.patch[2] = 170, r.scene.patch[3] = 170;
    r.step(true);
    CHECK(r.seq.status().state == robot::GraspState::kPaused);
    CHECK(r.seq.status().message.find("작업 영역") != std::string::npos);
  }
  {  // The same size inside the arm corridor, including where the raised hand shows up past the object -> keeps going
    Rig r(o);
    to_hover(r);
    r.scene.patch[0] = 270, r.scene.patch[1] = 120, r.scene.patch[2] = 370, r.scene.patch[3] = 250;
    for (int i = 0; i < 3; ++i) r.step(true);
    CHECK(r.seq.status().state == robot::GraspState::kHover);
  }
  {  // Whole image changes (camera on the arm) -> the check switches itself off, no false pause
    Rig r(o);
    to_hover(r);
    r.scene.base_gray = 250;
    for (int i = 0; i < 3; ++i) r.step(true);
    CHECK(r.seq.status().state == robot::GraspState::kHover && !r.seq.status().intrusion_watch);
  }
  {  // Option off: no watch
    Rig r;
    r.scene.with_gray = true;
    r.start();
    CHECK(r.run_until(robot::GraspState::kConfirm, true));
    CHECK(!r.seq.status().intrusion_watch);
  }
}

void test_color_sample() {
  Rgba img(64, 64);
  img.square(32, 32, 30, 40, 200, 90);  // green patch
  const auto hsv = vision::rgba_to_hsv_downscaled(img.px.data(), img.w, img.h, 2);
  const vision::HsvRange r = vision::sample_range(hsv, 16, 16, 4);
  uint16_t h;
  uint8_t sat, val;
  vision::rgb_to_hsv(40, 200, 90, &h, &sat, &val);
  CHECK(vision::in_range(r, h, sat, val));
  vision::rgb_to_hsv(220, 30, 30, &h, &sat, &val);
  CHECK(!vision::in_range(r, h, sat, val));
  vision::rgb_to_hsv(128, 128, 128, &h, &sat, &val);
  CHECK(!vision::in_range(r, h, sat, val));
}

}  // namespace

int main() {
  const std::vector<std::pair<const char*, std::function<void()>>> tests = {
      {"hsv", test_hsv},
      {"blobs", test_blobs},
      {"detector", test_detector},
      {"homography", test_homography},
      {"calib", test_calib},
      {"planner", test_planner},
      {"sequencer_full", test_sequencer_full},
      {"sequencer_safety", test_sequencer_safety},
      {"parallax", test_parallax},
      {"approach", test_approach},
      {"intrusion", test_intrusion},
      {"color_sample", test_color_sample},
  };
  for (const auto& [name, fn] : tests) {
    const int before = g_fail;
    fn();
    std::printf("%s %s\n", g_fail == before ? "[ok]  " : "[FAIL]", name);
  }
  std::printf("checks: %d passed, %d failed\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
