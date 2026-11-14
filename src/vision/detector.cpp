#include "vision/detector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>

#include "armpi_common/json.hpp"

namespace ally::vision {

namespace json = armpi_common::json;

const char* shape_name(Shape s) {
  switch (s) {
    case Shape::kCube: return "cube";
    case Shape::kBall: return "ball";
    case Shape::kAny: return "any";
  }
  return "any";
}

Detector::Detector(DetectorParams p) : p_(p), classes_(default_classes()) {}

void Detector::set_classes(std::vector<ObjectClass> c) {
  std::lock_guard<std::mutex> lk(mu_);
  classes_ = std::move(c);
}

std::vector<ObjectClass> Detector::classes() const {
  std::lock_guard<std::mutex> lk(mu_);
  return classes_;
}

float shape_score(const Blob& b, Shape s, const DetectorParams& p) {
  const bool round = b.radial_cv <= p.ball_cv_max;
  switch (s) {
    case Shape::kBall:
      return round ? std::min(1.0f, 0.6f + 0.4f * (1.0f - b.radial_cv / (p.ball_cv_max + 1e-6f))) : 0.0f;
    case Shape::kCube:
      if (round || b.oriented_fill < p.cube_ofill_min) return 0.0f;
      return std::min(1.0f, 0.6f + 0.4f * (b.oriented_fill - p.cube_ofill_min) / (1.0f - p.cube_ofill_min + 1e-3f));
    case Shape::kAny: return 0.5f;
  }
  return 0.0f;
}

DetectionSet Detector::detect(const robot::Frame& f, const DetectionSet* prev) const {
  DetectionSet ds = detect_rgba(f.rgba.data(), f.width, f.height, prev);
  ds.frame_seq = f.seq;
  ds.frame_mono = f.mono;
  return ds;
}

DetectionSet Detector::detect_rgba(const uint8_t* rgba, int w, int h, const DetectionSet* prev) const {
  const auto t0 = std::chrono::steady_clock::now();
  DetectionSet ds;
  ds.img_w = w;
  ds.img_h = h;
  if (!rgba || w <= 0 || h <= 0) return ds;
  const std::vector<ObjectClass> cls = classes();
  const int sc = std::max(1, p_.scale);
  const HsvImage hsv = rgba_to_hsv_downscaled(rgba, w, h, sc);
  // brightness thumbnail at 1/8 of the original size
  {
    const int k = std::max(1, 8 / sc);
    ds.gw = hsv.w / k;
    ds.gh = hsv.h / k;
    ds.gray.assign(static_cast<size_t>(ds.gw) * static_cast<size_t>(ds.gh), 0);
    for (int y = 0; y < ds.gh; ++y)
      for (int x = 0; x < ds.gw; ++x) {
        unsigned sum = 0;
        for (int dy = 0; dy < k; ++dy)
          for (int dx = 0; dx < k; ++dx) sum += hsv.v[static_cast<size_t>(y * k + dy) * hsv.w + static_cast<size_t>(x * k + dx)];
        ds.gray[static_cast<size_t>(y) * ds.gw + x] = static_cast<uint8_t>(sum / static_cast<unsigned>(k * k));
      }
  }
  const int min_area = std::max(4, p_.min_area_px / (sc * sc));
  std::vector<Detection> found;
  for (size_t ci = 0; ci < cls.size(); ++ci) {
    Mask m = threshold(hsv, cls[ci].color);
    erode3(m);
    dilate3(m);
    for (const Blob& b : connected_components(m, min_area, p_.max_blobs)) {
      const float score = shape_score(b, cls[ci].shape, p_);
      if (score <= 0) continue;
      Detection d;
      d.class_index = static_cast<int>(ci);
      d.u = (b.cx + 0.5f) * sc;
      d.v = (b.cy + 0.5f) * sc;
      d.bbox[0] = static_cast<float>(b.x0 * sc);
      d.bbox[1] = static_cast<float>(b.y0 * sc);
      d.bbox[2] = static_cast<float>((b.x1 + 1) * sc);
      d.bbox[3] = static_cast<float>((b.y1 + 1) * sc);
      d.area_px = static_cast<float>(b.area * sc * sc);
      d.score = score;
      found.push_back(d);
    }
  }
  // One blob can match two classes with overlapping colours (e.g. a cube and a ball of the same colour):
  // keep the better score when centres nearly coincide.
  std::sort(found.begin(), found.end(), [](const Detection& a, const Detection& b) { return a.score > b.score; });
  for (const Detection& d : found) {
    bool dup = false;
    for (const Detection& k : ds.items) dup = dup || std::hypot(k.u - d.u, k.v - d.v) < 10.0f;
    if (!dup) ds.items.push_back(d);
  }
  // Track ids: nearest previous detection of the same class within match_radius_px.
  for (Detection& d : ds.items) {
    float best = p_.match_radius_px;
    if (prev) {
      for (const Detection& p : prev->items) {
        if (p.class_index != d.class_index) continue;
        const float dist = std::hypot(p.u - d.u, p.v - d.v);
        if (dist < best) {
          best = dist;
          d.track_id = p.track_id;
        }
      }
    }
    if (d.track_id < 0) d.track_id = next_track_++;
  }
  ds.compute_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  return ds;
}

std::vector<ObjectClass> default_classes() {
  struct C {
    const char* color;
    float h_lo, h_hi;
  };
  // Saturation/value floors are generous: lighting varies; users can re-sample a colour on screen.
  const C colors[] = {{"빨간", 345, 15}, {"파란", 200, 250}, {"초록", 90, 160}, {"노란", 40, 70}};
  std::vector<ObjectClass> out;
  for (const auto& c : colors) {
    ObjectClass cube;
    cube.name = std::string(c.color) + " 블록";
    cube.shape = Shape::kCube;
    cube.size_m = 0.03;
    cube.color = HsvRange{c.h_lo, c.h_hi, 0.45f, 1.0f, 0.25f, 1.0f};
    cube.grasp_depth_m = 0.012;
    cube.gripper_close = 0.7;
    out.push_back(cube);
    ObjectClass ball = cube;
    ball.name = std::string(c.color) + " 공";
    ball.shape = Shape::kBall;
    ball.size_m = 0.04;
    ball.grasp_depth_m = 0.015;
    ball.gripper_close = 0.6;
    out.push_back(ball);
  }
  return out;
}

namespace {
bool num_in(const json::Value* v, double lo, double hi, double* out) {
  if (!v || !v->is_number() || !v->number_finite || v->number < lo || v->number > hi) return false;
  *out = v->number;
  return true;
}
}  // namespace

bool classes_from_json(const std::string& text, std::vector<ObjectClass>* out, std::string* err) {
  json::Value v;
  std::string e;
  if (!json::parse(text, v, e, json::lenient_topic_options(256 * 1024)) || !v.is_object()) {
    if (err) *err = "색 사전 JSON 오류: " + e;
    return false;
  }
  const json::Value* arr = v.find("classes");
  if (!arr || !arr->is_array() || arr->array.empty() || arr->array.size() > 32) {
    if (err) *err = "classes 배열 없음(1–32개)";
    return false;
  }
  std::vector<ObjectClass> res;
  for (const auto& c : arr->array) {
    ObjectClass oc;
    const json::Value* name = c.find("name");
    const json::Value* shape = c.find("shape");
    if (!name || !name->is_string() || name->string.empty() || name->string.size() > 64 || !shape || !shape->is_string()) {
      if (err) *err = "클래스 이름·모양 오류";
      return false;
    }
    oc.name = name->string;
    oc.shape = shape->string == "ball" ? Shape::kBall : shape->string == "cube" ? Shape::kCube : Shape::kAny;
    double d = 0;
    if (num_in(c.find("size_m"), 0.01, 0.10, &d)) oc.size_m = d;
    if (num_in(c.find("grasp_depth_m"), 0.0, 0.06, &d)) oc.grasp_depth_m = d;
    if (num_in(c.find("gripper_close"), 0.1, 0.9, &d)) oc.gripper_close = d;
    const json::Value* hr = c.find("h");
    const json::Value* sr = c.find("s");
    const json::Value* vr = c.find("v");
    auto pair = [&](const json::Value* a, double lo, double hi, float* p0, float* p1) {
      if (!a || !a->is_array() || a->array.size() != 2) return false;
      double x = 0, y = 0;
      if (!num_in(&a->array[0], lo, hi, &x) || !num_in(&a->array[1], lo, hi, &y)) return false;
      *p0 = static_cast<float>(x);
      *p1 = static_cast<float>(y);
      return true;
    };
    if (!pair(hr, 0, 360, &oc.color.h_lo, &oc.color.h_hi) || !pair(sr, 0, 1, &oc.color.s_lo, &oc.color.s_hi) ||
        !pair(vr, 0, 1, &oc.color.v_lo, &oc.color.v_hi)) {
      if (err) *err = "색 범위 오류: " + oc.name;
      return false;
    }
    res.push_back(oc);
  }
  *out = std::move(res);
  return true;
}

std::string classes_to_json(const std::vector<ObjectClass>& c) {
  std::ostringstream o;
  o << "{\n  \"version\": 1,\n  \"classes\": [\n";
  for (size_t i = 0; i < c.size(); ++i) {
    const auto& k = c[i];
    o << "    {\"name\": " << json::quote(k.name) << ", \"shape\": \"" << shape_name(k.shape) << "\", \"size_m\": "
      << json::number(k.size_m) << ", \"grasp_depth_m\": " << json::number(k.grasp_depth_m)
      << ", \"gripper_close\": " << json::number(k.gripper_close) << ", \"h\": [" << json::number(k.color.h_lo) << ", "
      << json::number(k.color.h_hi) << "], \"s\": [" << json::number(k.color.s_lo) << ", " << json::number(k.color.s_hi)
      << "], \"v\": [" << json::number(k.color.v_lo) << ", " << json::number(k.color.v_hi) << "]}"
      << (i + 1 < c.size() ? ",\n" : "\n");
  }
  o << "  ]\n}\n";
  return o.str();
}

uint32_t class_display_rgb(const ObjectClass& c) {
  float h = c.color.h_lo <= c.color.h_hi ? (c.color.h_lo + c.color.h_hi) * 0.5f
                                          : std::fmod((c.color.h_lo + c.color.h_hi + 360.0f) * 0.5f, 360.0f);
  const float s = 0.75f, v = 0.95f;
  const float cc = v * s, x = cc * (1 - std::fabs(std::fmod(h / 60.0f, 2.0f) - 1)), m = v - cc;
  float r = 0, g = 0, b = 0;
  if (h < 60) { r = cc; g = x; }
  else if (h < 120) { r = x; g = cc; }
  else if (h < 180) { g = cc; b = x; }
  else if (h < 240) { g = x; b = cc; }
  else if (h < 300) { r = x; b = cc; }
  else { r = cc; b = x; }
  auto to8 = [&](float f) { return static_cast<uint32_t>(std::lround((f + m) * 255.0f)) & 0xFF; };
  return (to8(r) << 16) | (to8(g) << 8) | to8(b);
}

}  // namespace ally::vision
