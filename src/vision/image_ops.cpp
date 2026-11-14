#include "vision/image_ops.hpp"

#include <algorithm>
#include <cmath>

namespace ally::vision {

void rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b, uint16_t* hue, uint8_t* s, uint8_t* v) {
  const int mx = std::max({r, g, b}), mn = std::min({r, g, b});
  const int d = mx - mn;
  *v = static_cast<uint8_t>(mx);
  *s = static_cast<uint8_t>(mx == 0 ? 0 : (d * 255 + mx / 2) / mx);
  if (d == 0) {
    *hue = 0;
    return;
  }
  float hh;
  if (mx == r) hh = 60.0f * std::fmod(static_cast<float>(g - b) / d, 6.0f);
  else if (mx == g) hh = 60.0f * (static_cast<float>(b - r) / d + 2.0f);
  else hh = 60.0f * (static_cast<float>(r - g) / d + 4.0f);
  if (hh < 0) hh += 360.0f;
  *hue = static_cast<uint16_t>(std::lround(hh) % 360);
}

HsvImage rgba_to_hsv_downscaled(const uint8_t* rgba, int w, int h, int scale) {
  HsvImage out;
  if (!rgba || w <= 0 || h <= 0) return out;
  scale = scale >= 4 ? 4 : (scale >= 2 ? 2 : 1);
  out.w = w / scale;
  out.h = h / scale;
  const size_t n = static_cast<size_t>(out.w) * out.h;
  out.hue.resize(n);
  out.s.resize(n);
  out.v.resize(n);
  const int area = scale * scale;
  for (int y = 0; y < out.h; ++y) {
    for (int x = 0; x < out.w; ++x) {
      int r = 0, g = 0, b = 0;
      for (int dy = 0; dy < scale; ++dy) {
        const uint8_t* p = rgba + (static_cast<size_t>(y * scale + dy) * w + static_cast<size_t>(x * scale)) * 4;
        for (int dx = 0; dx < scale; ++dx, p += 4) {
          r += p[0];
          g += p[1];
          b += p[2];
        }
      }
      const size_t i = static_cast<size_t>(y) * out.w + x;
      rgb_to_hsv(static_cast<uint8_t>(r / area), static_cast<uint8_t>(g / area), static_cast<uint8_t>(b / area), &out.hue[i],
                 &out.s[i], &out.v[i]);
    }
  }
  return out;
}

bool in_range(const HsvRange& r, uint16_t hue, uint8_t s, uint8_t v) {
  const float sf = s / 255.0f, vf = v / 255.0f;
  if (sf < r.s_lo || sf > r.s_hi || vf < r.v_lo || vf > r.v_hi) return false;
  const float hf = static_cast<float>(hue);
  if (r.h_lo <= r.h_hi) return hf >= r.h_lo && hf <= r.h_hi;
  return hf >= r.h_lo || hf <= r.h_hi;  // wraps through 0
}

Mask threshold(const HsvImage& img, const HsvRange& r) {
  Mask m;
  m.w = img.w;
  m.h = img.h;
  m.px.resize(img.hue.size());
  for (size_t i = 0; i < img.hue.size(); ++i) m.px[i] = in_range(r, img.hue[i], img.s[i], img.v[i]) ? 1 : 0;
  return m;
}

namespace {
void morph(Mask& m, bool erode) {
  if (m.w < 3 || m.h < 3) return;
  std::vector<uint8_t> out(m.px.size(), 0);
  for (int y = 0; y < m.h; ++y) {
    for (int x = 0; x < m.w; ++x) {
      bool all = true, any = false;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const int xx = x + dx, yy = y + dy;
          const bool on = xx >= 0 && yy >= 0 && xx < m.w && yy < m.h && m.px[static_cast<size_t>(yy) * m.w + xx];
          all = all && on;
          any = any || on;
        }
      }
      out[static_cast<size_t>(y) * m.w + x] = erode ? all : any;
    }
  }
  m.px.swap(out);
}

int find_root(std::vector<int>& parent, int a) {
  while (parent[static_cast<size_t>(a)] != a) {
    parent[static_cast<size_t>(a)] = parent[static_cast<size_t>(parent[static_cast<size_t>(a)])];
    a = parent[static_cast<size_t>(a)];
  }
  return a;
}
}  // namespace

void erode3(Mask& m) { morph(m, true); }
void dilate3(Mask& m) { morph(m, false); }

std::vector<Blob> connected_components(const Mask& m, int min_area, int max_blobs) {
  std::vector<Blob> blobs;
  if (m.w <= 0 || m.h <= 0) return blobs;
  const size_t n = static_cast<size_t>(m.w) * m.h;
  std::vector<int> label(n, -1);
  std::vector<int> parent;
  // first pass: provisional labels with union-find on the 4 already-visited 8-neighbours
  for (int y = 0; y < m.h; ++y) {
    for (int x = 0; x < m.w; ++x) {
      const size_t i = static_cast<size_t>(y) * m.w + x;
      if (!m.px[i]) continue;
      int best = -1;
      const int nb[4][2] = {{-1, -1}, {0, -1}, {1, -1}, {-1, 0}};
      for (const auto& d : nb) {
        const int xx = x + d[0], yy = y + d[1];
        if (xx < 0 || yy < 0 || xx >= m.w) continue;
        const int l = label[static_cast<size_t>(yy) * m.w + xx];
        if (l < 0) continue;
        if (best < 0) best = l;
        else {
          const int ra = find_root(parent, best), rb = find_root(parent, l);
          if (ra != rb) parent[static_cast<size_t>(std::max(ra, rb))] = std::min(ra, rb);
        }
      }
      if (best < 0) {
        best = static_cast<int>(parent.size());
        parent.push_back(best);
      }
      label[i] = best;
    }
  }
  // second pass: accumulate statistics per root
  struct Acc {
    long long area{0};
    double sx{0}, sy{0}, sxx{0}, syy{0}, sxy{0};
    int x0{1 << 30}, y0{1 << 30}, x1{-1}, y1{-1};
    int boundary{0};
  };
  std::vector<int> root_index(parent.size(), -1);
  std::vector<Acc> acc;
  for (int y = 0; y < m.h; ++y) {
    for (int x = 0; x < m.w; ++x) {
      const size_t i = static_cast<size_t>(y) * m.w + x;
      if (label[i] < 0) continue;
      const int r = find_root(parent, label[i]);
      if (root_index[static_cast<size_t>(r)] < 0) {
        root_index[static_cast<size_t>(r)] = static_cast<int>(acc.size());
        acc.emplace_back();
      }
      Acc& a = acc[static_cast<size_t>(root_index[static_cast<size_t>(r)])];
      ++a.area;
      a.sx += x;
      a.sy += y;
      a.sxx += static_cast<double>(x) * x;
      a.syy += static_cast<double>(y) * y;
      a.sxy += static_cast<double>(x) * y;
      a.x0 = std::min(a.x0, x);
      a.y0 = std::min(a.y0, y);
      a.x1 = std::max(a.x1, x);
      a.y1 = std::max(a.y1, y);
      // boundary pixel: any 4-neighbour outside the mask or outside the image
      const bool edge = x == 0 || y == 0 || x == m.w - 1 || y == m.h - 1 || !m.px[i - 1] || !m.px[i + 1] ||
                        !m.px[i - static_cast<size_t>(m.w)] || !m.px[i + static_cast<size_t>(m.w)];
      if (edge) ++a.boundary;
    }
  }
  constexpr double kPi = 3.14159265358979323846;
  for (const Acc& a : acc) {
    if (a.area < min_area) continue;
    Blob b;
    b.area = static_cast<int>(a.area);
    b.cx = static_cast<float>(a.sx / a.area);
    b.cy = static_cast<float>(a.sy / a.area);
    b.x0 = a.x0;
    b.y0 = a.y0;
    b.x1 = a.x1;
    b.y1 = a.y1;
    b.perimeter = static_cast<float>(a.boundary * (kPi / (2.0 * std::sqrt(2.0))));
    b.circularity = b.perimeter > 0 ? static_cast<float>(4.0 * kPi * a.area / (b.perimeter * b.perimeter)) : 0.0f;
    const double bw = a.x1 - a.x0 + 1, bh = a.y1 - a.y0 + 1;
    b.fill = static_cast<float>(a.area / (bw * bh));
    b.aspect = static_cast<float>(bw / bh);
    // second central moments -> eigenvalues -> equivalent oriented rectangle
    const double mxx = a.sxx / a.area - b.cx * static_cast<double>(b.cx);
    const double myy = a.syy / a.area - b.cy * static_cast<double>(b.cy);
    const double mxy = a.sxy / a.area - b.cx * static_cast<double>(b.cy);
    const double tr = mxx + myy, det = mxx * myy - mxy * mxy;
    const double disc = std::sqrt(std::max(0.0, tr * tr / 4 - det));
    const double l1 = tr / 2 + disc + 1.0 / 12, l2 = std::max(1e-9, tr / 2 - disc + 1.0 / 12);  // +1/12: pixel area
    b.oriented_fill = static_cast<float>(a.area / (12.0 * std::sqrt(l1 * l2)));
    blobs.push_back(b);
  }
  std::sort(blobs.begin(), blobs.end(), [](const Blob& p, const Blob& q) { return p.area > q.area; });
  if (static_cast<int>(blobs.size()) > max_blobs) blobs.resize(static_cast<size_t>(std::max(0, max_blobs)));
  return blobs;
}

HsvRange sample_range(const HsvImage& img, int cx, int cy, int radius) {
  HsvRange r;
  double vx = 0, vy = 0;
  std::vector<uint8_t> ss, vs;
  for (int y = cy - radius; y <= cy + radius; ++y) {
    for (int x = cx - radius; x <= cx + radius; ++x) {
      if (x < 0 || y < 0 || x >= img.w || y >= img.h) continue;
      if ((x - cx) * (x - cx) + (y - cy) * (y - cy) > radius * radius) continue;
      const size_t i = static_cast<size_t>(y) * img.w + x;
      const double a = img.hue[i] * 3.14159265358979323846 / 180.0;
      const double wgt = img.s[i] / 255.0;  // grey pixels say little about hue
      vx += std::cos(a) * wgt;
      vy += std::sin(a) * wgt;
      ss.push_back(img.s[i]);
      vs.push_back(img.v[i]);
    }
  }
  if (ss.empty()) return r;
  auto median = [](std::vector<uint8_t>& v) {
    std::nth_element(v.begin(), v.begin() + static_cast<long>(v.size() / 2), v.end());
    return v[v.size() / 2] / 255.0f;
  };
  double hue = std::atan2(vy, vx) * 180.0 / 3.14159265358979323846;
  if (hue < 0) hue += 360.0;
  const float s = median(ss), v = median(vs);
  r.h_lo = static_cast<float>(std::fmod(hue - 15.0 + 360.0, 360.0));
  r.h_hi = static_cast<float>(std::fmod(hue + 15.0, 360.0));
  r.s_lo = std::max(0.25f, s - 0.25f);
  r.s_hi = 1.0f;
  r.v_lo = std::max(0.15f, v - 0.35f);
  r.v_hi = 1.0f;
  return r;
}

}  // namespace ally::vision
