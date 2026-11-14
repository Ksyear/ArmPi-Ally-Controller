// Table-plane homography (x, y in metres on the table) <-> (u, v) image pixels.
// Normalised DLT solved with 8x8 normal equations (Gaussian elimination, partial pivoting); no external maths library.
#pragma once

#include <string>
#include <vector>

namespace ally::vision {

struct Homography {
  double h[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};
  bool valid{false};
};

struct PointPair {
  double u{0}, v{0};  // pixel
  double x{0}, y{0};  // robot table coordinates, metres
};

// Fits H with (u, v, 1) ~ H (x, y, 1). Needs at least 4 non-degenerate pairs.
Homography fit_homography(const std::vector<PointPair>& pts, std::string* err);
bool apply(const Homography& H, double x, double y, double* u, double* v);
Homography invert(const Homography& H);

struct Residual {
  double rms_px{0}, max_px{0};  // forward reprojection error
  double rms_m{0}, max_m{0};    // inverse mapping error on the table
};
Residual residual(const Homography& H, const std::vector<PointPair>& pts);

}  // namespace ally::vision
