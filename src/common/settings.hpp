// User settings, stored as JSON in %LOCALAPPDATA%\ArmPi_Ally_Controller\settings.json (Windows)
// or ~/.config/armpi_ally/settings.json. Only addresses and preferences — never passwords or Wi-Fi data.
#pragma once

#include <string>

namespace ally {

struct Settings {
  std::string robot_host;  // last robot address (empty = ask)
  int control_port{5001};
  int discovery_port{5002};
  int rosbridge_port{9090};
  int camera_port{8080};
  std::string camera_url;  // empty = default stream URL with qos_profile=sensor_data
  double ack_timeout_s{0.30};
  double speed{0.30};
  float deadzone{0.15f};
  float curve{1.6f};
  bool haptics{true};
  float haptic_strength{1.0f};
  bool fullscreen{true};
  int fps_cap{120};
  bool camera_on{true};
  bool allow_drive_while_arm{false};
  bool show_crosshair{true};
};

std::string settings_dir();   // created if missing (UTF-8 path)
std::string settings_path();  // settings_dir()/settings.json
bool load_settings(const std::string& path, Settings* s, std::string* err);
bool save_settings(const std::string& path, const Settings& s, std::string* err);
std::string settings_to_json(const Settings& s);
bool settings_from_json(const std::string& text, Settings* s, std::string* err);
// Delete all but the newest `keep` ally_*.log files in dir (UTF-8 path).
void prune_logs(const std::string& dir, size_t keep);

}  // namespace ally
