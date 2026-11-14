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
  double cam_tilt_rad{0.0};       // 0 = no object-height correction
  std::vector<PointPair> points;
  Homography H;                   // (x, y) -> (u, v)
  Residual res;
  std::string saved_at;

  // H valid, table height known and image size matches.
  bool usable(int w, int h, std::string* why) const;
  // Pixel -> table point (object_h for the optional height correction; 0 skips it).
  bool pixel_to_table(double u, double v, double object_h, double* x, double* y) const;
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
