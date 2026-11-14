// Colour + shape object detector for known, single-colour objects on a flat table (docs/ARCHITECTURE.md).
#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "robot/camera.hpp"
#include "vision/image_ops.hpp"

namespace ally::vision {

enum class Shape { kCube, kBall, kAny };
const char* shape_name(Shape s);

struct ObjectClass {
  std::string name;             // e.g. "빨간 블록"
  Shape shape{Shape::kCube};
  double size_m{0.03};          // cube edge or ball diameter
  HsvRange color;
  double grasp_depth_m{0.012};  // grasp height above the table
  double gripper_close{0.7};    // closing target 0..1 (never fully closed: servo heat)
};

struct Detection {
  int class_index{-1};
  float u{0}, v{0};     // centre, original image pixels
  float bbox[4]{};      // u0, v0, u1, v1
  float area_px{0};     // original pixels
  float score{0};       // shape match 0..1
  int track_id{-1};     // kept across frames by nearest match (stable selection)
};

struct DetectionSet {
  uint64_t frame_seq{0};
  double frame_mono{0};
  int img_w{0}, img_h{0};
  std::vector<Detection> items;  // best score first
  double compute_ms{0};
  // Brightness thumbnail (1/8 of the image size, 0..255) for the work-area intrusion check.
  int gw{0}, gh{0};
  std::vector<uint8_t> gray;
};

struct DetectorParams {
  int scale{2};          // processing downscale (640x480 -> 320x240)
  int min_area_px{60};   // in original pixels
  int max_blobs{12};
  // Ball = boundary distances to the centroid nearly constant (radial_cv). Measured on the detector pipeline:
  // discs 0.012-0.041 (radius >= 12 px), squares >= 0.088 at any rotation (side >= 30 px). 4*pi*A/P^2 is not used:
  // the boundary-pixel perimeter estimate makes a 45-degree square look rounder than a disc.
  float ball_cv_max{0.06f};
  float cube_ofill_min{0.82f};
  float match_radius_px{40};
};

class Detector {
 public:
  explicit Detector(DetectorParams p = {});
  void set_classes(std::vector<ObjectClass> c);
  std::vector<ObjectClass> classes() const;
  DetectorParams params() const { return p_; }
  // One frame. prev keeps track ids stable (may be null).
  DetectionSet detect(const robot::Frame& f, const DetectionSet* prev) const;
  DetectionSet detect_rgba(const uint8_t* rgba, int w, int h, const DetectionSet* prev) const;

 private:
  DetectorParams p_;
  mutable std::mutex mu_;
  std::vector<ObjectClass> classes_;
  mutable std::atomic<int> next_track_{1};
};

// Score how well a blob matches a shape (0 = no match).
float shape_score(const Blob& b, Shape s, const DetectorParams& p);

std::vector<ObjectClass> default_classes();  // red/blue/green/yellow 3 cm blocks + 4 cm balls
bool classes_from_json(const std::string& text, std::vector<ObjectClass>* out, std::string* err);
std::string classes_to_json(const std::vector<ObjectClass>& c);
// Display colour (RGB) for a class: centre of its hue range at high saturation.
uint32_t class_display_rgb(const ObjectClass& c);

}  // namespace ally::vision
