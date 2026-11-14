#include "vision/homography.hpp"

#include <algorithm>
#include <cmath>

namespace ally::vision {

namespace {

// Similarity transform moving the centroid to 0 and the mean distance to sqrt(2).
struct Norm {
  double s{1}, tx{0}, ty{0};
};

Norm normaliser(const std::vector<double>& xs, const std::vector<double>& ys) {
  Norm n;
  double mx = 0, my = 0;
  for (size_t i = 0; i < xs.size(); ++i) {
    mx += xs[i];
    my += ys[i];
  }
  mx /= xs.size();
  my /= ys.size();
  double d = 0;
  for (size_t i = 0; i < xs.size(); ++i) d += std::hypot(xs[i] - mx, ys[i] - my);
  d /= xs.size();
  n.s = d > 1e-12 ? std::sqrt(2.0) / d : 1.0;
  n.tx = -mx * n.s;
  n.ty = -my * n.s;
  return n;
}

void mul3(const double a[9], const double b[9], double out[9]) {
  double r[9];
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) r[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
  std::copy(r, r + 9, out);
}

// Solves A x = b for an n x n system in place. Returns false when (near) singular.
bool solve(std::vector<double>& A, std::vector<double>& b, int n) {
  for (int c = 0; c < n; ++c) {
    int piv = c;
    for (int r = c + 1; r < n; ++r)
      if (std::fabs(A[static_cast<size_t>(r * n + c)]) > std::fabs(A[static_cast<size_t>(piv * n + c)])) piv = r;
    if (std::fabs(A[static_cast<size_t>(piv * n + c)]) < 1e-10) return false;
    if (piv != c) {
      for (int k = 0; k < n; ++k) std::swap(A[static_cast<size_t>(c * n + k)], A[static_cast<size_t>(piv * n + k)]);
      std::swap(b[static_cast<size_t>(c)], b[static_cast<size_t>(piv)]);
    }
    for (int r = c + 1; r < n; ++r) {
      const double f = A[static_cast<size_t>(r * n + c)] / A[static_cast<size_t>(c * n + c)];
      for (int k = c; k < n; ++k) A[static_cast<size_t>(r * n + k)] -= f * A[static_cast<size_t>(c * n + k)];
      b[static_cast<size_t>(r)] -= f * b[static_cast<size_t>(c)];
    }
  }
  for (int r = n - 1; r >= 0; --r) {
    double s = b[static_cast<size_t>(r)];
    for (int k = r + 1; k < n; ++k) s -= A[static_cast<size_t>(r * n + k)] * b[static_cast<size_t>(k)];
    b[static_cast<size_t>(r)] = s / A[static_cast<size_t>(r * n + r)];
  }
  return true;
}

}  // namespace

Homography fit_homography(const std::vector<PointPair>& pts, std::string* err) {
  Homography H;
  if (pts.size() < 4) {
    if (err) *err = "대응점이 4개 이상 필요";
    return H;
  }
  std::vector<double> xs, ys, us, vs;
  for (const auto& p : pts) {
    if (!std::isfinite(p.u) || !std::isfinite(p.v) || !std::isfinite(p.x) || !std::isfinite(p.y)) {
      if (err) *err = "유한하지 않은 대응점";
      return H;
    }
    xs.push_back(p.x);
    ys.push_back(p.y);
    us.push_back(p.u);
    vs.push_back(p.v);
  }
  const Norm nx = normaliser(xs, ys), nu = normaliser(us, vs);
  // Normal equations for h0..h7 (h8 = 1) in normalised coordinates.
  std::vector<double> AtA(64, 0.0), Atb(8, 0.0);
  for (size_t i = 0; i < pts.size(); ++i) {
    const double x = xs[i] * nx.s + nx.tx, y = ys[i] * nx.s + nx.ty;
    const double u = us[i] * nu.s + nu.tx, v = vs[i] * nu.s + nu.ty;
    const double r1[8] = {x, y, 1, 0, 0, 0, -x * u, -y * u};
    const double r2[8] = {0, 0, 0, x, y, 1, -x * v, -y * v};
    for (int a = 0; a < 8; ++a) {
      for (int b = 0; b < 8; ++b) AtA[static_cast<size_t>(a * 8 + b)] += r1[a] * r1[b] + r2[a] * r2[b];
      Atb[static_cast<size_t>(a)] += r1[a] * u + r2[a] * v;
    }
  }
  if (!solve(AtA, Atb, 8)) {
    if (err) *err = "대응점이 한 줄에 있거나 겹침(퇴화)";
    return H;
  }
  const double Hn[9] = {Atb[0], Atb[1], Atb[2], Atb[3], Atb[4], Atb[5], Atb[6], Atb[7], 1.0};
  // H = Tu^-1 * Hn * Tx
  const double Tx[9] = {nx.s, 0, nx.tx, 0, nx.s, nx.ty, 0, 0, 1};
  const double TuInv[9] = {1 / nu.s, 0, -nu.tx / nu.s, 0, 1 / nu.s, -nu.ty / nu.s, 0, 0, 1};
  double tmp[9];
  mul3(Hn, Tx, tmp);
  mul3(TuInv, tmp, H.h);
  if (std::fabs(H.h[8]) > 1e-12) {
    const double d = H.h[8];
    for (double& e : H.h) e /= d;
  }
  // A valid plane homography must be invertible and finite.
  bool finite = true;
  for (double e : H.h) finite = finite && std::isfinite(e);
  const Homography inv = invert(H);
  H.valid = finite && inv.valid;
  if (!H.valid && err) *err = "호모그래피가 퇴화함";
  return H;
}

bool apply(const Homography& H, double x, double y, double* u, double* v) {
  const double w = H.h[6] * x + H.h[7] * y + H.h[8];
  if (std::fabs(w) < 1e-12) return false;
  *u = (H.h[0] * x + H.h[1] * y + H.h[2]) / w;
  *v = (H.h[3] * x + H.h[4] * y + H.h[5]) / w;
  return std::isfinite(*u) && std::isfinite(*v);
}

Homography invert(const Homography& H) {
  const double* a = H.h;
  Homography r;
  const double c0 = a[4] * a[8] - a[5] * a[7], c1 = a[5] * a[6] - a[3] * a[8], c2 = a[3] * a[7] - a[4] * a[6];
  const double det = a[0] * c0 + a[1] * c1 + a[2] * c2;
  if (!std::isfinite(det) || std::fabs(det) < 1e-15) return r;
  r.h[0] = c0 / det;
  r.h[1] = (a[2] * a[7] - a[1] * a[8]) / det;
  r.h[2] = (a[1] * a[5] - a[2] * a[4]) / det;
  r.h[3] = c1 / det;
  r.h[4] = (a[0] * a[8] - a[2] * a[6]) / det;
  r.h[5] = (a[2] * a[3] - a[0] * a[5]) / det;
  r.h[6] = c2 / det;
  r.h[7] = (a[1] * a[6] - a[0] * a[7]) / det;
  r.h[8] = (a[0] * a[4] - a[1] * a[3]) / det;
  if (std::fabs(r.h[8]) > 1e-15) {
    const double d = r.h[8];
    for (double& e : r.h) e /= d;
  }
  r.valid = true;
  return r;
}

Residual residual(const Homography& H, const std::vector<PointPair>& pts) {
  Residual res;
  if (!H.valid || pts.empty()) return res;
  const Homography inv = invert(H);
  double spx = 0, sm = 0;
  for (const auto& p : pts) {
    double u = 0, v = 0, x = 0, y = 0;
    if (apply(H, p.x, p.y, &u, &v)) {
      const double e = std::hypot(u - p.u, v - p.v);
      spx += e * e;
      res.max_px = std::max(res.max_px, e);
    }
    if (inv.valid && apply(inv, p.u, p.v, &x, &y)) {
      const double e = std::hypot(x - p.x, y - p.y);
      sm += e * e;
      res.max_m = std::max(res.max_m, e);
    }
  }
  res.rms_px = std::sqrt(spx / pts.size());
  res.rms_m = std::sqrt(sm / pts.size());
  return res;
}

}  // namespace ally::vision
