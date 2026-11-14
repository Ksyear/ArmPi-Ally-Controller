// Dependency-free image operations for object detection (docs/ARCHITECTURE.md, autograsp):
// RGBA -> downscaled HSV, colour threshold, 3x3 morphology, 8-connected components with shape measures.
#pragma once

#include <cstdint>
#include <vector>

namespace ally::vision {

struct HsvImage {
  int w{0}, h{0};
  std::vector<uint16_t> hue;  // 0..359
  std::vector<uint8_t> s, v;  // 0..255
};

struct Mask {
  int w{0}, h{0};
  std::vector<uint8_t> px;  // 0 / 1
};

// h_lo > h_hi means the range wraps through 0 (e.g. red: 345..15). s, v are 0..1.
struct HsvRange {
  float h_lo{0}, h_hi{360};
  float s_lo{0}, s_hi{1};
  float v_lo{0}, v_hi{1};
};

// Average-pools scale x scale blocks (scale 1, 2 or 4) and converts to HSV.
HsvImage rgba_to_hsv_downscaled(const uint8_t* rgba, int w, int h, int scale);
void rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b, uint16_t* hue, uint8_t* s, uint8_t* v);
bool in_range(const HsvRange& r, uint16_t hue, uint8_t s, uint8_t v);
Mask threshold(const HsvImage& img, const HsvRange& r);
void erode3(Mask& m);
void dilate3(Mask& m);

struct Blob {
  int area{0};                     // pixels (in the image the mask came from)
  float cx{0}, cy{0};              // centroid
  int x0{0}, y0{0}, x1{0}, y1{0};  // bounding box, inclusive
  float perimeter{0};              // boundary pixels x pi/(2*sqrt 2) (~Euclidean length)
  float circularity{0};            // 4*pi*A/P^2: ~1 disc, ~0.64 square
  float fill{0};                   // area / bbox area
  float oriented_fill{0};          // area / (12*sqrt(l1*l2)): ~1 for any rectangle, ~1.05 disc
  float aspect{1};                 // bbox width / height
};

// 8-connected components; blobs smaller than min_area are dropped; largest first, at most max_blobs.
std::vector<Blob> connected_components(const Mask& m, int min_area, int max_blobs);

// Colour range around a tapped point: circular mean hue +-15 deg, median S/V with margins.
HsvRange sample_range(const HsvImage& img, int cx, int cy, int radius);

}  // namespace ally::vision
