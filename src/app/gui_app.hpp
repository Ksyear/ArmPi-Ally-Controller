// GuiApp: the whole touch/gamepad UI (Dear ImGui), independent of the Win32/D3D11 layer.
// The platform layer calls frame() between ImGui::NewFrame() and ImGui::Render().
#pragma once

#include <atomic>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "common/settings.hpp"
#include "common/util.hpp"
#include "imgui.h"
#include "input/mapper.hpp"
#include "input/pad.hpp"
#include "net/comm_log.hpp"
#include "net/discovery.hpp"
#include "robot/core.hpp"

namespace ally::ui {

struct Platform {
  // Upload an RGBA frame into the camera texture (re-created when the size changes).
  std::function<ImTextureID(int w, int h, const uint8_t* rgba)> upload_camera;
  std::function<void(bool on)> set_fullscreen;
  std::function<void(bool on)> keep_awake;
  std::function<void()> quit;
  std::function<int()> ally_battery_percent;  // -1 = unknown
  std::function<bool()> ally_on_ac;
  std::function<void(const std::string& utf8_path)> open_folder;
};

struct Fonts {
  ImFont* regular{nullptr};
  ImFont* bold{nullptr};
};

class GuiApp {
 public:
  GuiApp(Platform p, Settings s, Fonts fonts);
  ~GuiApp();
  GuiApp(const GuiApp&) = delete;
  GuiApp& operator=(const GuiApp&) = delete;

  void set_focused(bool focused);
  // Test/demo run: connect to host with ports base+1 / base+4090 / base+3080 on the first frame.
  // These ports and this host are used for this run only and are never written to settings.json.
  void set_demo_connect(const std::string& host, int port_base, const std::string& tab);
  void frame(double now);
  void shutdown();  // neutral + disconnect, rumble off, save settings
  const Settings& settings() const { return settings_; }
  EventLog& log() { return log_; }

 private:
  enum class Tab { kDrive, kArm, kDiag, kSettings };
  struct Note {
    std::string text;
    ImU32 color;
    double until;
  };

  void connect(const std::string& host);
  void disconnect();
  void start_discovery();
  void handle_input(double now);
  void handle_cues(double now);
  void notify(const std::string& text, ImU32 color, double secs = 3.0);
  void save();

  // drawing
  void draw_top_bar(double now);
  void draw_camera(ImVec2 size, double now);
  void draw_side_column(ImVec2 size, double now);
  void draw_bottom_bar(ImVec2 size);
  void draw_tab_drive();
  void draw_tab_arm();
  void draw_tab_diag();
  void draw_tab_settings();
  void draw_connect_panel(ImVec2 size);
  void arm_plot(const char* id, ImVec2 size, bool top_view);
  bool hold_button(const char* id, const char* label, ImVec2 size, double hold_s, ImU32 color, float* progress);
  bool virtual_stick(const char* id, float radius, float* x, float* y);
  void chip(const char* text, ImU32 bg, ImU32 fg = IM_COL32(255, 255, 255, 255));
  float S(float px) const { return px * scale_; }

  Platform plat_;
  Settings settings_;
  Fonts fonts_;
  EventLog log_;
  net::CommLog comm_;
  std::unique_ptr<robot::RobotCore> core_;
  input::Gamepad pad_;
  input::HapticPlayer haptics_;
  input::Mapper mapper_;

  bool focused_{true};
  float scale_{1.0f};
  float chip_limit_x_{0.0f};  // top bar: chips past this x are skipped (tabs live there)
  Tab tab_{Tab::kDrive};
  bool camera_big_{false};
  bool show_connect_{true};
  std::deque<Note> notes_;
  robot::Cues last_cues_;
  bool cues_init_{false};
  double last_service_seen_{0.0};

  // touch controls (written by widgets, read next frame by the mapper)
  input::TouchControls touch_;
  bool touch_arm_toggle_{false};
  bool prev_arm_hold_{false};
  float clear_progress_pad_{0.0f};
  float clear_progress_touch_{0.0f};
  input::PadState last_pad_;
  bool pad_was_connected_{false};

  // camera texture
  ImTextureID cam_tex_{};
  int cam_w_{0}, cam_h_{0};
  uint64_t cam_seq_{0};
  double cam_last_frame_{0.0};

  // connect panel
  char ip_buf_[64]{};
  std::thread discover_thread_;
  std::atomic<bool> discovering_{false};
  std::mutex discover_mu_;
  std::vector<net::RobotInfo> discovered_;
  std::string discover_error_;

  // arm tab inputs
  float num_x_{0.15f}, num_y_{0.0f}, num_z_{0.08f}, num_pitch_deg_{-60.0f}, num_grip_{0.0f};
  // joint debug sliders (deg)
  float joints_deg_[6]{0, 0, 0, 0, 0, 0};
  // robot-wide reset confirmation
  bool confirm_reset_{false};

  std::string demo_host_;
  int demo_base_{0};
  bool demo_pending_{false};
  int ally_batt_{-1};
  bool ally_ac_{false};
  double next_power_poll_{0.0};
  bool keep_awake_on_{false};
};

}  // namespace ally::ui
