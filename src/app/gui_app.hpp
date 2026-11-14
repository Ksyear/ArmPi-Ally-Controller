// GuiApp: the whole touch/gamepad UI (Dear ImGui), independent of the Win32/D3D11 layer.
// The platform layer calls frame() between ImGui::NewFrame() and ImGui::Render().
// Look and layout follow docs/DESIGN_PLAN_v001.md; tokens and widgets live in ui_theme / ui_widgets.
#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "app/ui_theme.hpp"
#include "app/ui_widgets.hpp"
#include "common/settings.hpp"
#include "common/util.hpp"
#include "imgui.h"
#include "input/mapper.hpp"
#include "input/pad.hpp"
#include "input/tilt.hpp"
#include "net/comm_log.hpp"
#include "net/discovery.hpp"
#include "robot/autograsp.hpp"
#include "robot/core.hpp"
#include "vision/calibration.hpp"
#include "vision/vision_worker.hpp"

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
  std::function<std::vector<TouchPoint>()> touches;  // extra touch contacts (not the one driving the mouse)
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
  // Screenshot-only display states: "estop" (latch an e-stop on the mock robot), "cambig" (full camera).
  void set_demo_state(const std::string& state) { demo_state_ = state; }
  // Test/demo run: use this camera calibration for the run only (never saved).
  void set_demo_calib(const std::string& path);
  void frame(double now);
  void shutdown();  // neutral + disconnect, rumble off, save settings
  const Settings& settings() const { return settings_; }
  // Centres of the drive stick and the e-stop in window pixels (multi-touch self check).
  ImVec2 touch_target(int i) const { return touch_targets_[i & 1]; }
  EventLog& log() { return log_; }

 private:
  enum class Tab { kDrive, kArm, kGrasp, kDiag, kSettings };

  void connect(const std::string& host);
  void disconnect();
  void start_discovery();
  void handle_input(double now);
  void handle_cues(double now);
  void notify(const std::string& text, Status kind, double secs = 3.0);
  void save();
  void trigger_estop(double now);

  // drawing (screen rectangles in pixels)
  void draw_top_bar(ImVec2 p0, ImVec2 size);
  void draw_camera(ImVec2 p0, ImVec2 size);
  void draw_safety_column(ImVec2 p0, ImVec2 size);
  void draw_bottom_bar(ImVec2 p0, ImVec2 size);
  void draw_tab_panel(ImVec2 p0, ImVec2 size);
  void draw_tab_drive(ImVec2 size);
  void draw_tab_arm(ImVec2 size);
  void draw_tab_diag(ImVec2 size);
  void draw_tab_settings(ImVec2 size);
  // autograsp (grasp_tab.cpp)
  void draw_tab_grasp(ImVec2 size);
  void draw_grasp_candidates(float wdt);
  void draw_grasp_calibration(float wdt);
  void draw_grasp_colors(float wdt);
  void apply_color(const std::string& group);
  void draw_grasp_overlay(ImDrawList* dl, ImVec2 p0, ImVec2 p1);  // camera HUD, image rectangle p0..p1
  void grasp_frame(double now);                // dead-man + sequencer tick + cues, every frame
  void grasp_pad(uint16_t pressed);            // D-pad edges in the grasp tab
  void grasp_pick(float u, float v, double now);  // camera tap, image pixels
  void set_calib(const vision::VisionCalib& c, bool save_file);
  void start_grasp();
  void stop_grasp();
  void draw_connect_left(ImVec2 p0, ImVec2 size);
  void draw_connect_right(ImVec2 p0, ImVec2 size);
  void arm_plot(const char* id, ImVec2 size, bool top_view);

  Platform plat_;
  Settings settings_;
  EventLog log_;
  net::CommLog comm_;
  std::unique_ptr<robot::RobotCore> core_;
  input::Gamepad pad_;
  input::HapticPlayer haptics_;
  input::Mapper mapper_;

  bool focused_{true};
  Tab tab_{Tab::kDrive};
  bool camera_big_{false};
  bool show_connect_{true};
  std::vector<w::Toast> toasts_;
  robot::Cues last_cues_;
  bool cues_init_{false};
  double last_service_seen_{0.0};
  double estop_flash_until_{0.0};

  // touch controls (written by widgets, read next frame by the mapper)
  input::TouchControls touch_;
  bool touch_arm_toggle_{false};
  bool prev_arm_hold_{false};
  float clear_progress_pad_{0.0f};
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
  float num_[4]{0.15f, 0.0f, 0.08f, -60.0f};  // x, y, z (m), pitch (deg)
  float num_grip_{0.0f};
  float joints_deg_[6]{0, 0, 0, 0, 0, 0};
  bool confirm_reset_{false};

  // autograsp: detector thread, camera calibration, sequencer (exists while connected)
  vision::VisionWorker vision_;
  vision::VisionCalib calib_;
  std::unique_ptr<robot::CoreGraspRobot> grasp_robot_;
  std::unique_ptr<robot::GraspSequencer> grasp_;
  std::vector<std::pair<float, float>> reach_band_;
  bool reach_band_dirty_{true};
  bool grasp_hold_touch_{false};  // on-screen hold button, read next frame
  uint64_t grasp_done_seen_{0}, grasp_fail_seen_{0};
  uint16_t prev_buttons_{0};
  bool demo_calib_{false};
  std::string grasp_hint_;
  bool calib_mode_{false};
  int calib_step_{0};
  vision::VisionCalib calib_edit_;
  bool calib_tap_set_{false};
  float calib_tap_u_{0}, calib_tap_v_{0};
  int parallax_stage_{0};  // 0 = tap the table mark, 1 = tap the top of the block on it, 2 = measured
  float par_u0_{0}, par_v0_{0};
  float parallax_h_{0.03f};
  // colour re-sampling
  bool color_pick_mode_{false};
  std::optional<vision::HsvRange> color_sample_;
  uint32_t color_sample_rgb_{0};
  bool user_active_{false};
  double quit_armed_until_{0.0};  // top-bar quit button: second tap before this time quits
  // tilt driving (RT held): sensor, reference pose captured on the press, current output for the HUD
  input::TiltSensor tilt_;
  bool tilt_tried_{false}, tilt_on_{false}, tilt_armed_{false}, tilt_warned_{false}, tilt_ref_set_{false};
  double tilt_ref_pitch_{0}, tilt_ref_roll_{0}, tilt_press_t_{0};
  bool grasp_armed_{false};  // the grasp dead-man must be seen released after any interruption
  input::TiltDrive tilt_out_;
  ImVec2 touch_targets_[2]{};  // the user is driving / moving the arm this frame (pauses autograsp)

  std::string demo_host_;
  int demo_base_{0};
  bool demo_pending_{false};
  std::string demo_state_;
  bool demo_state_done_{false};
  int ally_batt_{-1};
  bool ally_ac_{false};
  double next_power_poll_{0.0};
  bool keep_awake_on_{false};
};

}  // namespace ally::ui
