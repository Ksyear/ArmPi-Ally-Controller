#include "common/settings.hpp"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <vector>
#include <fstream>
#include <sstream>

#include "armpi_common/json.hpp"
#include "common/util.hpp"

#ifdef _WIN32
#include <windows.h>
#endif

namespace ally {

namespace json = armpi_common::json;
namespace fs = std::filesystem;

namespace {

fs::path to_path(const std::string& utf8) {
#ifdef _WIN32
  const int n = ::MultiByteToWideChar(CP_UTF8, 0, utf8.c_str(), -1, nullptr, 0);
  std::wstring w(static_cast<size_t>(n > 0 ? n - 1 : 0), L'\0');
  if (n > 1) ::MultiByteToWideChar(CP_UTF8, 0, utf8.c_str(), -1, w.data(), n);
  return fs::path(w);
#else
  return fs::path(utf8);
#endif
}

std::string from_path(const fs::path& p) {
#ifdef _WIN32
  const std::wstring w = p.wstring();
  const int n = ::WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, nullptr, 0, nullptr, nullptr);
  std::string s(static_cast<size_t>(n > 0 ? n - 1 : 0), '\0');
  if (n > 1) ::WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, s.data(), n, nullptr, nullptr);
  return s;
#else
  return p.string();
#endif
}

}  // namespace

std::string settings_dir() {
  fs::path dir;
#ifdef _WIN32
  if (const wchar_t* la = ::_wgetenv(L"LOCALAPPDATA")) dir = fs::path(la) / L"ArmPi_Ally_Controller";
  else dir = fs::path(L".") / L"settings";
#else
  if (const char* xdg = std::getenv("XDG_CONFIG_HOME")) dir = fs::path(xdg) / "armpi_ally";
  else if (const char* home = std::getenv("HOME")) dir = fs::path(home) / ".config" / "armpi_ally";
  else dir = fs::path(".") / "settings";
#endif
  std::error_code ec;
  fs::create_directories(dir, ec);
  return from_path(dir);
}

std::string settings_path() { return from_path(to_path(settings_dir()) / "settings.json"); }

std::string settings_to_json(const Settings& s) {
  std::ostringstream o;
  o << "{\n"
    << "  \"robot_host\": " << json::quote(s.robot_host) << ",\n"
    << "  \"control_port\": " << s.control_port << ",\n"
    << "  \"discovery_port\": " << s.discovery_port << ",\n"
    << "  \"rosbridge_port\": " << s.rosbridge_port << ",\n"
    << "  \"camera_port\": " << s.camera_port << ",\n"
    << "  \"camera_url\": " << json::quote(s.camera_url) << ",\n"
    << "  \"ack_timeout_s\": " << json::number(s.ack_timeout_s) << ",\n"
    << "  \"speed\": " << json::number(s.speed) << ",\n"
    << "  \"deadzone\": " << json::number(s.deadzone) << ",\n"
    << "  \"curve\": " << json::number(s.curve) << ",\n"
    << "  \"haptics\": " << json::boolean(s.haptics) << ",\n"
    << "  \"haptic_strength\": " << json::number(s.haptic_strength) << ",\n"
    << "  \"fullscreen\": " << json::boolean(s.fullscreen) << ",\n"
    << "  \"fps_cap\": " << s.fps_cap << ",\n"
    << "  \"camera_on\": " << json::boolean(s.camera_on) << ",\n"
    << "  \"allow_drive_while_arm\": " << json::boolean(s.allow_drive_while_arm) << ",\n"
    << "  \"show_crosshair\": " << json::boolean(s.show_crosshair) << ",\n"
    << "  \"font_scale\": " << json::number(s.font_scale) << ",\n"
    << "  \"autograsp_confirm\": " << json::boolean(s.autograsp_confirm) << ",\n"
    << "  \"autograsp_retry\": " << s.autograsp_retry << ",\n"
    << "  \"autograsp_base\": " << json::boolean(s.autograsp_base) << ",\n"
    << "  \"autograsp_intrusion\": " << json::boolean(s.autograsp_intrusion) << ",\n"
    << "  \"tilt_drive\": " << json::boolean(s.tilt_drive) << ",\n"
    << "  \"tilt_full_deg\": " << json::number(s.tilt_full_deg) << "\n"
    << "}\n";
  return o.str();
}

bool settings_from_json(const std::string& text, Settings* s, std::string* err) {
  json::Value v;
  std::string e;
  if (!json::parse(text, v, e, json::lenient_topic_options()) || !v.is_object()) {
    if (err) *err = "설정 JSON 오류: " + e;
    return false;
  }
  auto str = [&](const char* k, std::string* out) {
    if (const auto* f = v.find(k); f && f->is_string()) *out = f->string;
  };
  auto i32 = [&](const char* k, int* out, int lo, int hi) {
    if (const auto* f = v.find(k); f && f->is_number() && f->number >= lo && f->number <= hi) *out = static_cast<int>(f->number);
  };
  auto dbl = [&](const char* k, double* out, double lo, double hi) {
    if (const auto* f = v.find(k); f && f->is_number() && f->number >= lo && f->number <= hi) *out = f->number;
  };
  auto flt = [&](const char* k, float* out, double lo, double hi) {
    if (const auto* f = v.find(k); f && f->is_number() && f->number >= lo && f->number <= hi) *out = static_cast<float>(f->number);
  };
  auto bln = [&](const char* k, bool* out) {
    if (const auto* f = v.find(k); f && f->is_bool()) *out = f->boolean;
  };
  str("robot_host", &s->robot_host);
  for (char c : s->robot_host)
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '.' || c == '-')) {
      s->robot_host.clear();  // only plain hostnames / IPv4 literals
      break;
    }
  i32("control_port", &s->control_port, 1, 65535);
  i32("discovery_port", &s->discovery_port, 1, 65535);
  i32("rosbridge_port", &s->rosbridge_port, 0, 65535);
  i32("camera_port", &s->camera_port, 1, 65535);
  str("camera_url", &s->camera_url);
  if (s->camera_url.find_first_of(" \t\r\n") != std::string::npos) s->camera_url.clear();
  dbl("ack_timeout_s", &s->ack_timeout_s, 0.1, 2.0);
  dbl("speed", &s->speed, 0.1, 1.0);
  flt("deadzone", &s->deadzone, 0.0, 0.5);
  flt("curve", &s->curve, 1.0, 3.0);
  bln("haptics", &s->haptics);
  flt("haptic_strength", &s->haptic_strength, 0.0, 1.0);
  bln("fullscreen", &s->fullscreen);
  i32("fps_cap", &s->fps_cap, 30, 240);
  bln("camera_on", &s->camera_on);
  bln("allow_drive_while_arm", &s->allow_drive_while_arm);
  bln("show_crosshair", &s->show_crosshair);
  flt("font_scale", &s->font_scale, 1.0, 1.3);
  bln("autograsp_confirm", &s->autograsp_confirm);
  i32("autograsp_retry", &s->autograsp_retry, 0, 2);
  bln("autograsp_base", &s->autograsp_base);
  bln("autograsp_intrusion", &s->autograsp_intrusion);
  bln("tilt_drive", &s->tilt_drive);
  flt("tilt_full_deg", &s->tilt_full_deg, 12.0, 45.0);
  return true;
}

void prune_logs(const std::string& dir_utf8, size_t keep) {
  std::error_code ec;
  std::vector<fs::path> logs;
  for (const auto& e : fs::directory_iterator(to_path(dir_utf8), ec)) {
    const fs::path& p = e.path();
    const auto name = p.filename().native();
    const auto prefix = fs::path("ally_").native();
    if (e.is_regular_file(ec) && p.extension() == ".log" && name.compare(0, prefix.size(), prefix) == 0) logs.push_back(p);
  }
  if (logs.size() <= keep) return;
  std::sort(logs.begin(), logs.end());  // names carry a sortable timestamp: ally_YYYYmmdd_HHMMSS.log
  for (size_t i = 0; i + keep < logs.size(); ++i) fs::remove(logs[i], ec);
}

bool read_text_file(const std::string& path, std::string* out, std::string* err) {
  std::ifstream f(to_path(path), std::ios::binary);
  if (!f) {
    if (err) *err = "파일 없음";
    return false;
  }
  std::ostringstream ss;
  ss << f.rdbuf();
  *out = ss.str();
  return true;
}

bool write_text_file_atomic(const std::string& path, const std::string& text, std::string* err) {
  const fs::path p = to_path(path);
  fs::path tmp = p;
  tmp += ".tmp";
  {
    std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
    if (!f) {
      if (err) *err = "저장 실패";
      return false;
    }
    f << text;
  }
  std::error_code ec;
  fs::rename(tmp, p, ec);
  if (ec) {
    if (err) *err = "저장 실패: " + ec.message();
    return false;
  }
  return true;
}

bool load_settings(const std::string& path, Settings* s, std::string* err) {
  std::string text;
  if (!read_text_file(path, &text, err)) {
    if (err) *err = "설정 파일 없음(기본값 사용)";
    return false;
  }
  return settings_from_json(text, s, err);
}

bool save_settings(const std::string& path, const Settings& s, std::string* err) {
  return write_text_file_atomic(path, settings_to_json(s), err);
}

}  // namespace ally
