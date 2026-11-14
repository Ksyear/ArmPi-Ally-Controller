// Camera-to-table calibration (docs/ARCHITECTURE.md): images are only used from one "observe pose",
// where a table-plane homography maps pixels to robot (x, y) on the table at height z_table.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "robot/rosbridge.hpp"
#include "vision/homography.hpp"

namespace ally::vision {

struct VisionCalib {
  int version{1};
  int image_w{0}, image_h{0};
  robot::Pose5 observe_pose{0.1047, 0.0, 0.0753, -1.0263, 0.0};
  std::optional<double> z_table;  // no table height = grasping is blocked
  // Parallax: a point h metres above the table shows up on the table plane at (x, y) + h * (parallax_x, parallax_y).
  // Measured in the wizard (table mark, then a block of known height on it). 0 = no object-height correction.
  double parallax_x{0.0}, parallax_y{0.0};
  std::vector<PointPair> points;
  Homography H;                   // (x, y) -> (u, v)
  Residual res;
  std::string saved_at;

  // H valid, table height known and image size matches.
  bool usable(int w, int h, std::string* why) const;
  // Pixel -> table point. object_h: object height; its blob centre is taken to sit at h/2 and is moved back by the
  // parallax (0 skips the correction).
  bool pixel_to_table(double u, double v, double object_h, double* x, double* y) const;
  // Parallax from one table mark tapped at (u0, v0) and the top of an object of height h standing on it at (u1, v1).
  bool measure_parallax(double u0, double v0, double u1, double v1, double h, std::string* err);
  bool table_to_pixel(double x, double y, double* u, double* v) const;
  // Refit H from points (n >= 4) and update residuals.
  bool refit(std::string* err);
};

std::string calib_path();  // settings_dir()/vision_calib.json
bool load_calib(const std::string& path, VisionCalib* c, std::string* err);
bool save_calib(const std::string& path, const VisionCalib& c, std::string* err);
std::string calib_to_json(const VisionCalib& c);
bool calib_from_json(const std::string& text, VisionCalib* c, std::string* err);

}  // namespace ally::vision
