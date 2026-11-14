#include "vision/calibration.hpp"

#include <cmath>
#include <ctime>
#include <sstream>

#include "armpi_common/json.hpp"
#include "common/settings.hpp"

namespace ally::vision {

namespace json = armpi_common::json;

bool VisionCalib::usable(int w, int h, std::string* why) const {
  auto no = [&](const char* m) {
    if (why) *why = m;
    return false;
  };
  if (!H.valid) return no("보정 안 됨(대응점 4개 이상 필요)");
  if (!z_table) return no("책상 높이를 정하지 않음");
  if (w > 0 && h > 0 && (w != image_w || h != image_h)) return no("카메라 해상도가 보정 때와 다름 — 다시 보정");
  return true;
}

bool VisionCalib::pixel_to_table(double u, double v, double object_h, double* x, double* y) const {
  const Homography inv = invert(H);
  if (!H.valid || !inv.valid || !apply(inv, u, v, x, y)) return false;
  // Optional height correction: a blob of height h seen at tilt theta sits h/2*cot(theta) "behind" its base.
  // Applied along the direction from the image centre's table point (approximate camera axis) to the object.
  if (cam_tilt_rad > 0.05 && object_h > 0) {
    double cx = 0, cy = 0;
    if (apply(inv, image_w * 0.5, image_h * 0.5, &cx, &cy)) {
      const double dx = *x - cx, dy = *y - cy, d = std::hypot(dx, dy);
      const double shift = object_h * 0.5 / std::tan(cam_tilt_rad);
      if (d > 1e-6) {
        *x -= dx / d * shift;
        *y -= dy / d * shift;
      }
    }
  }
  return std::isfinite(*x) && std::isfinite(*y);
}

bool VisionCalib::table_to_pixel(double x, double y, double* u, double* v) const {
  return H.valid && apply(H, x, y, u, v);
}

bool VisionCalib::refit(std::string* err) {
  H = fit_homography(points, err);
  res = H.valid ? residual(H, points) : Residual{};
  return H.valid;
}

std::string calib_path() { return settings_dir() + "/vision_calib.json"; }

std::string calib_to_json(const VisionCalib& c) {
  std::ostringstream o;
  auto n = [](double v) { return json::number(v); };
  o << "{\n  \"version\": 1,\n  \"image_w\": " << c.image_w << ",\n  \"image_h\": " << c.image_h << ",\n"
    << "  \"observe_pose\": {\"x\": " << n(c.observe_pose.x) << ", \"y\": " << n(c.observe_pose.y) << ", \"z\": " << n(c.observe_pose.z)
    << ", \"pitch\": " << n(c.observe_pose.pitch) << ", \"gripper\": " << n(c.observe_pose.gripper) << "},\n"
    << "  \"z_table\": " << (c.z_table ? n(*c.z_table) : std::string("null")) << ",\n"
    << "  \"cam_tilt_rad\": " << n(c.cam_tilt_rad) << ",\n  \"points\": [";
  for (size_t i = 0; i < c.points.size(); ++i) {
    const auto& p = c.points[i];
    o << (i ? ", " : "") << "{\"u\": " << n(p.u) << ", \"v\": " << n(p.v) << ", \"x\": " << n(p.x) << ", \"y\": " << n(p.y) << "}";
  }
  o << "],\n  \"H\": [";
  for (int i = 0; i < 9; ++i) o << (i ? ", " : "") << n(c.H.h[i]);
  o << "],\n  \"residual_px\": " << n(c.res.rms_px) << ",\n  \"residual_mm\": " << n(c.res.rms_m * 1000.0) << ",\n"
    << "  \"saved_at\": " << json::quote(c.saved_at) << "\n}\n";
  return o.str();
}

namespace {
bool num(const json::Value* v, double lo, double hi, double* out) {
  if (!v || !v->is_number() || !v->number_finite || v->number < lo || v->number > hi) return false;
  *out = v->number;
  return true;
}
}  // namespace

bool calib_from_json(const std::string& text, VisionCalib* c, std::string* err) {
  json::Value v;
  std::string e;
  if (!json::parse(text, v, e, json::lenient_topic_options(256 * 1024)) || !v.is_object()) {
    if (err) *err = "보정 파일 JSON 오류: " + e;
    return false;
  }
  VisionCalib r;
  double d = 0;
  auto fail = [&](const char* m) {
    if (err) *err = std::string("보정 파일 값 오류: ") + m;
    return false;
  };
  if (!num(v.find("image_w"), 16, 8192, &d)) return fail("image_w");
  r.image_w = static_cast<int>(d);
  if (!num(v.find("image_h"), 16, 8192, &d)) return fail("image_h");
  r.image_h = static_cast<int>(d);
  if (const auto* op = v.find("observe_pose"); op && op->is_object()) {
    double x, y, z, p, g = 0;
    // Inside the arm workspace box (x 0.05..0.32, y +-0.25, z -0.08..0.32): a file cannot send the arm elsewhere.
    if (!num(op->find("x"), 0.05, 0.32, &x) || !num(op->find("y"), -0.25, 0.25, &y) || !num(op->find("z"), -0.08, 0.32, &z) ||
        !num(op->find("pitch"), -3.2, 3.2, &p))
      return fail("observe_pose");
    num(op->find("gripper"), 0, 1, &g);
    r.observe_pose = robot::Pose5{x, y, z, p, g};
  }
  if (const auto* zt = v.find("z_table"); zt && !zt->is_null()) {
    if (!num(zt, -0.08, 0.20, &d)) return fail("z_table");  // the grasp floor follows it: keep it in the arm's z range
    r.z_table = d;
  }
  if (num(v.find("cam_tilt_rad"), 0, 1.6, &d)) r.cam_tilt_rad = d;
  if (const auto* pts = v.find("points"); pts && pts->is_array()) {
    if (pts->array.size() > 64) return fail("points 개수");
    for (const auto& p : pts->array) {
      PointPair pp;
      if (!num(p.find("u"), -1e5, 1e5, &pp.u) || !num(p.find("v"), -1e5, 1e5, &pp.v) || !num(p.find("x"), -1, 1, &pp.x) ||
          !num(p.find("y"), -1, 1, &pp.y))
        return fail("points");
      r.points.push_back(pp);
    }
  }
  if (const auto* hs = v.find("H"); hs && hs->is_array() && hs->array.size() == 9) {
    for (int i = 0; i < 9; ++i)
      if (!num(&hs->array[static_cast<size_t>(i)], -1e9, 1e9, &r.H.h[i])) return fail("H");
    r.H.valid = invert(r.H).valid;
  }
  if (r.points.size() >= 4) {
    std::string fe;
    r.refit(&fe);  // recompute from the stored points: the file's H is only a fallback
  } else if (r.H.valid) {
    r.res = residual(r.H, r.points);
  }
  if (const auto* sa = v.find("saved_at"); sa && sa->is_string() && sa->string.size() < 64) r.saved_at = sa->string;
  *c = std::move(r);
  return true;
}

bool load_calib(const std::string& path, VisionCalib* c, std::string* err) {
  std::string text;
  if (!read_text_file(path, &text, err)) {
    if (err) *err = "보정 파일 없음";
    return false;
  }
  return calib_from_json(text, c, err);
}

bool save_calib(const std::string& path, const VisionCalib& c, std::string* err) {
  VisionCalib out = c;
  const std::time_t t = std::time(nullptr);
  std::tm tm{};
#ifdef _WIN32
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm);
#endif
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &tm);
  out.saved_at = buf;
  return write_text_file_atomic(path, calib_to_json(out), err);
}

}  // namespace ally::vision
