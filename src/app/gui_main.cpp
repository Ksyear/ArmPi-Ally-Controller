// Windows entry point: Win32 window + Direct3D 11 + Dear ImGui, hosting ally::ui::GuiApp.
// Platform duties live here: focus tracking, full screen, keep-awake, battery, camera texture, single instance.
#include <d3d11.h>
#include <dxgi.h>
#include <shellapi.h>
#include <windows.h>

#include <timeapi.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "app/gui_app.hpp"
#include "app/ui_theme.hpp"
#include "common/settings.hpp"
#include "imgui.h"
#include "imgui_impl_dx11.h"
#include "imgui_impl_win32.h"
#include "net/sock.hpp"
#include "version.hpp"

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

namespace {

ID3D11Device* g_device = nullptr;
ID3D11DeviceContext* g_ctx = nullptr;
IDXGISwapChain* g_swap = nullptr;
ID3D11RenderTargetView* g_rtv = nullptr;
UINT g_resize_w = 0, g_resize_h = 0;
bool g_occluded = false;
bool g_focused = true;
bool g_quit = false;
bool g_fullscreen = true;
RECT g_windowed_rect{100, 100, 1700, 1000};
HWND g_hwnd = nullptr;
const char* g_driver = "none";
bool g_smoke = false;  // --smoke-test: run a fixed number of frames, log every start-up step, never block on dialogs
std::FILE* g_boot = nullptr;

// camera texture
ID3D11Texture2D* g_cam_tex = nullptr;
ID3D11ShaderResourceView* g_cam_srv = nullptr;
int g_cam_w = 0, g_cam_h = 0;

std::wstring to_wide(const std::string& s);

// Start-up log (settings folder/boot.log): lets a headless check see how far start-up got.
void boot_log(const std::string& msg) {
  if (!g_boot) return;
  std::fprintf(g_boot, "%s %s\n", ally::wall_clock_ms().c_str(), msg.c_str());
  std::fflush(g_boot);
}

void fatal(const std::wstring& msg, const std::string& log_msg) {
  boot_log("FATAL " + log_msg);
  if (!g_smoke) MessageBoxW(nullptr, msg.c_str(), L"ArmPi Ally Controller", MB_ICONERROR);
}

std::wstring to_wide(const std::string& s) {
  const int n = ::MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, nullptr, 0);
  std::wstring w(static_cast<size_t>(n > 0 ? n - 1 : 0), L'\0');
  if (n > 1) ::MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, w.data(), n);
  return w;
}

std::string to_utf8(const std::wstring& w) {
  const int n = ::WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, nullptr, 0, nullptr, nullptr);
  std::string s(static_cast<size_t>(n > 0 ? n - 1 : 0), '\0');
  if (n > 1) ::WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, s.data(), n, nullptr, nullptr);
  return s;
}

void create_render_target() {
  ID3D11Texture2D* back = nullptr;
  g_swap->GetBuffer(0, IID_PPV_ARGS(&back));
  if (back) {
    g_device->CreateRenderTargetView(back, nullptr, &g_rtv);
    back->Release();
  }
}

void cleanup_render_target() {
  if (g_rtv) {
    g_rtv->Release();
    g_rtv = nullptr;
  }
}

bool create_device(HWND hwnd) {
  DXGI_SWAP_CHAIN_DESC sd{};
  sd.BufferCount = 2;
  sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
  sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
  sd.OutputWindow = hwnd;
  sd.SampleDesc.Count = 1;
  sd.Windowed = TRUE;
  sd.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;  // flip model: lower latency at 120 Hz
  const D3D_FEATURE_LEVEL levels[] = {D3D_FEATURE_LEVEL_11_0, D3D_FEATURE_LEVEL_10_0};
  D3D_FEATURE_LEVEL got;
  HRESULT hr = D3D11CreateDeviceAndSwapChain(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, 0, levels, 2, D3D11_SDK_VERSION,
                                             &sd, &g_swap, &g_device, &got, &g_ctx);
  g_driver = "hardware/flip";
  if (FAILED(hr)) {  // older driver: classic blit model
    sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;
    hr = D3D11CreateDeviceAndSwapChain(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, 0, levels, 2, D3D11_SDK_VERSION, &sd,
                                       &g_swap, &g_device, &got, &g_ctx);
    g_driver = "hardware/blit";
  }
  if (FAILED(hr)) {
    hr = D3D11CreateDeviceAndSwapChain(nullptr, D3D_DRIVER_TYPE_WARP, nullptr, 0, levels, 2, D3D11_SDK_VERSION, &sd,
                                       &g_swap, &g_device, &got, &g_ctx);
    g_driver = "warp";
  }
  if (FAILED(hr)) return false;
  create_render_target();
  return true;
}

void release_camera_texture() {
  if (g_cam_srv) g_cam_srv->Release();
  if (g_cam_tex) g_cam_tex->Release();
  g_cam_srv = nullptr;
  g_cam_tex = nullptr;
  g_cam_w = g_cam_h = 0;
}

// Save the current back buffer as a 32-bit BMP (used by --screenshot for docs and the smoke test).
// step 2 = half size (every other pixel), used for capture bursts.
bool save_backbuffer_bmp(const std::wstring& path, uint32_t step = 1) {
  ID3D11Texture2D* back = nullptr;
  if (FAILED(g_swap->GetBuffer(0, IID_PPV_ARGS(&back))) || !back) return false;
  D3D11_TEXTURE2D_DESC d{};
  back->GetDesc(&d);
  d.Usage = D3D11_USAGE_STAGING;
  d.BindFlags = 0;
  d.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
  d.MiscFlags = 0;
  ID3D11Texture2D* staging = nullptr;
  bool ok = false;
  if (SUCCEEDED(g_device->CreateTexture2D(&d, nullptr, &staging))) {
    g_ctx->CopyResource(staging, back);
    D3D11_MAPPED_SUBRESOURCE m{};
    if (SUCCEEDED(g_ctx->Map(staging, 0, D3D11_MAP_READ, 0, &m))) {
      step = std::max<uint32_t>(1, step);
      const uint32_t w = d.Width / step, h = d.Height / step, row = w * 4;
      BITMAPFILEHEADER fh{};
      BITMAPINFOHEADER ih{};
      fh.bfType = 0x4D42;
      fh.bfOffBits = sizeof(fh) + sizeof(ih);
      fh.bfSize = fh.bfOffBits + row * h;
      ih.biSize = sizeof(ih);
      ih.biWidth = static_cast<LONG>(w);
      ih.biHeight = -static_cast<LONG>(h);  // top-down
      ih.biPlanes = 1;
      ih.biBitCount = 32;
      ih.biCompression = BI_RGB;
      if (std::FILE* f = _wfopen(path.c_str(), L"wb")) {
        std::fwrite(&fh, sizeof(fh), 1, f);
        std::fwrite(&ih, sizeof(ih), 1, f);
        std::vector<uint8_t> line(row);
        for (uint32_t y = 0; y < h; ++y) {
          const uint8_t* src = static_cast<const uint8_t*>(m.pData) + static_cast<size_t>(y) * step * m.RowPitch;
          for (uint32_t x = 0; x < w; ++x) {  // RGBA -> BGRA
            const uint8_t* px = src + static_cast<size_t>(x) * step * 4;
            line[x * 4 + 0] = px[2];
            line[x * 4 + 1] = px[1];
            line[x * 4 + 2] = px[0];
            line[x * 4 + 3] = 255;
          }
          std::fwrite(line.data(), 1, row, f);
        }
        ok = std::fclose(f) == 0;
      }
      g_ctx->Unmap(staging, 0);
    }
    staging->Release();
  }
  back->Release();
  return ok;
}

// Screen captures of the running app (tests and documentation): F12 / the diag tab button / a request file that a
// remote session can drop into the settings folder ("capture.request": empty = one shot, "burst N S" = N half-size
// frames every S seconds). Files go to <settings>/captures; only the newest 200 are kept.
struct Captures {
  bool fresh{false};  // just requested: draw one frame with addresses hidden before saving
  int left{0};
  double interval{0.2}, next{0}, next_poll{0};
  uint32_t step{1};
  int seq{0};
  std::string stamp;
};
Captures g_cap;

void capture_request(int count, double interval) {
  g_cap.left = std::clamp(count, 1, 100);
  g_cap.interval = std::clamp(interval, 0.1, 2.0);
  g_cap.step = g_cap.left > 1 ? 2 : 1;
  g_cap.next = 0;
  g_cap.seq = 0;
  g_cap.stamp = ally::wall_stamp();
  g_cap.fresh = true;
}

void capture_poll_request(double now) {
  if (now < g_cap.next_poll) return;
  g_cap.next_poll = now + 0.5;
  const std::string req = ally::settings_dir() + "/capture.request";
  std::string text, err;
  if (!ally::read_text_file(req, &text, &err)) return;
  DeleteFileW(to_wide(req).c_str());
  int n = 1;
  double s = 0.2;
  if (text.size() < 64) std::sscanf(text.c_str(), "burst %d %lf", &n, &s);
  capture_request(n, s);
}

void capture_prune(const std::wstring& dir, size_t keep) {
  std::vector<std::wstring> files;
  WIN32_FIND_DATAW fd;
  HANDLE h = FindFirstFileW((dir + L"\\cap_*.bmp").c_str(), &fd);
  if (h == INVALID_HANDLE_VALUE) return;
  do files.push_back(fd.cFileName);
  while (FindNextFileW(h, &fd));
  FindClose(h);
  if (files.size() <= keep) return;
  std::sort(files.begin(), files.end());  // names start with the time stamp
  for (size_t i = 0; i + keep < files.size(); ++i) DeleteFileW((dir + L"\\" + files[i]).c_str());
}

// Called after the frame is drawn, before Present.
void capture_step(double now, ally::ui::GuiApp& app) {
  ally::EventLog& log = app.log();
  capture_poll_request(now);
  if (g_cap.left <= 0) return;
  if (g_cap.fresh) {  // this frame was drawn before the request: hide addresses, save from the next frame on
    g_cap.fresh = false;
    app.set_privacy(true);
    return;
  }
  if (now < g_cap.next) return;
  const std::wstring dir = to_wide(ally::settings_dir() + "/captures");
  CreateDirectoryW(dir.c_str(), nullptr);
  const std::wstring path = dir + L"\\cap_" + to_wide(g_cap.stamp) + to_wide(ally::strf("_%03d.bmp", g_cap.seq));
  if (!save_backbuffer_bmp(path, g_cap.step)) {
    log.warn("화면 저장 실패");
    g_cap.left = 0;
    app.set_privacy(false);
    return;
  }
  ++g_cap.seq;
  --g_cap.left;
  g_cap.next = now + g_cap.interval;
  if (g_cap.left == 0) {
    app.set_privacy(false);
    log.info(g_cap.seq > 1 ? ally::strf("화면 %d장 연속 저장(captures)", g_cap.seq) : std::string("화면 저장(captures)"));
    capture_prune(dir, 200);
  }
}

void cleanup_device() {
  release_camera_texture();
  cleanup_render_target();
  if (g_swap) g_swap->Release();
  if (g_ctx) g_ctx->Release();
  if (g_device) g_device->Release();
  g_swap = nullptr;
  g_ctx = nullptr;
  g_device = nullptr;
}

ImTextureID upload_camera(int w, int h, const uint8_t* rgba) {
  if (w <= 0 || h <= 0 || !g_device) return ImTextureID{};
  if (w != g_cam_w || h != g_cam_h || !g_cam_tex) {
    release_camera_texture();
    D3D11_TEXTURE2D_DESC td{};
    td.Width = static_cast<UINT>(w);
    td.Height = static_cast<UINT>(h);
    td.MipLevels = 1;
    td.ArraySize = 1;
    td.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    td.SampleDesc.Count = 1;
    td.Usage = D3D11_USAGE_DYNAMIC;
    td.BindFlags = D3D11_BIND_SHADER_RESOURCE;
    td.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
    if (FAILED(g_device->CreateTexture2D(&td, nullptr, &g_cam_tex))) return ImTextureID{};
    D3D11_SHADER_RESOURCE_VIEW_DESC sv{};
    sv.Format = td.Format;
    sv.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
    sv.Texture2D.MipLevels = 1;
    if (FAILED(g_device->CreateShaderResourceView(g_cam_tex, &sv, &g_cam_srv))) {
      release_camera_texture();
      return ImTextureID{};
    }
    g_cam_w = w;
    g_cam_h = h;
  }
  D3D11_MAPPED_SUBRESOURCE m{};
  if (SUCCEEDED(g_ctx->Map(g_cam_tex, 0, D3D11_MAP_WRITE_DISCARD, 0, &m))) {
    const size_t row = static_cast<size_t>(w) * 4;
    for (int y = 0; y < h; ++y)
      std::memcpy(static_cast<uint8_t*>(m.pData) + static_cast<size_t>(y) * m.RowPitch, rgba + static_cast<size_t>(y) * row, row);
    g_ctx->Unmap(g_cam_tex, 0);
  }
  return reinterpret_cast<ImTextureID>(g_cam_srv);
}

// Borderless full screen on the current monitor, or back to the remembered window rectangle.
void apply_fullscreen(bool on) {
  if (!g_hwnd) return;
  if (on && !g_fullscreen) GetWindowRect(g_hwnd, &g_windowed_rect);
  g_fullscreen = on;
  if (on) {
    MONITORINFO mi{};
    mi.cbSize = sizeof(mi);
    GetMonitorInfoW(MonitorFromWindow(g_hwnd, MONITOR_DEFAULTTOPRIMARY), &mi);
    SetWindowLongPtrW(g_hwnd, GWL_STYLE, WS_POPUP | WS_VISIBLE);
    SetWindowPos(g_hwnd, HWND_TOP, mi.rcMonitor.left, mi.rcMonitor.top, mi.rcMonitor.right - mi.rcMonitor.left,
                 mi.rcMonitor.bottom - mi.rcMonitor.top, SWP_FRAMECHANGED | SWP_SHOWWINDOW);
  } else {
    SetWindowLongPtrW(g_hwnd, GWL_STYLE, WS_OVERLAPPEDWINDOW | WS_VISIBLE);
    SetWindowPos(g_hwnd, HWND_NOTOPMOST, g_windowed_rect.left, g_windowed_rect.top,
                 g_windowed_rect.right - g_windowed_rect.left, g_windowed_rect.bottom - g_windowed_rect.top,
                 SWP_FRAMECHANGED | SWP_SHOWWINDOW);
  }
}

void keep_awake(bool on) {
  // While connected the display and system must not sleep (sleep = focus loss = robot stops mid-task).
  SetThreadExecutionState(on ? (ES_CONTINUOUS | ES_DISPLAY_REQUIRED | ES_SYSTEM_REQUIRED) : ES_CONTINUOUS);
}

int battery_percent() {
  SYSTEM_POWER_STATUS ps{};
  if (!GetSystemPowerStatus(&ps) || ps.BatteryLifePercent == 255) return -1;
  return ps.BatteryLifePercent;
}

bool on_ac() {
  SYSTEM_POWER_STATUS ps{};
  return GetSystemPowerStatus(&ps) && ps.ACLineStatus == 1;
}

void open_folder(const std::string& path) {
  ShellExecuteW(nullptr, L"open", to_wide(path).c_str(), nullptr, nullptr, SW_SHOWNORMAL);
}

// Touch contacts other than the primary one (Windows turns the primary contact into mouse messages for ImGui).
struct Contact {
  POINT pos{}, down{};
  bool fresh{true};
  bool up{false};  // lifted, but kept until the UI has seen it once (a quick tap must still reach the e-stop)
};
std::map<UINT32, Contact> g_touches;

void on_pointer(HWND hwnd, UINT msg, WPARAM wp) {
  const UINT32 id = GET_POINTERID_WPARAM(wp);
  POINTER_INPUT_TYPE type = PT_POINTER;
  if (!GetPointerType(id, &type) || type != PT_TOUCH) return;
  if (msg == WM_POINTERUP || msg == WM_POINTERCAPTURECHANGED) {
    auto it = g_touches.find(id);
    if (it != g_touches.end()) {
      if (it->second.fresh) it->second.up = true;  // not reported yet: report it once, then drop it
      else g_touches.erase(it);
    }
    return;
  }
  POINTER_INFO pi{};
  if (!GetPointerInfo(id, &pi)) return;
  if ((pi.pointerFlags & POINTER_FLAG_PRIMARY) || !(pi.pointerFlags & POINTER_FLAG_INCONTACT)) {
    g_touches.erase(id);
    return;
  }
  POINT pt = pi.ptPixelLocation;
  ScreenToClient(hwnd, &pt);
  auto it = g_touches.find(id);
  if (it == g_touches.end()) {
    if (g_touches.size() < 10) g_touches[id] = Contact{pt, pt, true};
  } else {
    it->second.pos = pt;
  }
}

std::vector<ally::ui::TouchPoint> take_touches() {
  std::vector<ally::ui::TouchPoint> out;
  for (auto it = g_touches.begin(); it != g_touches.end();) {
    Contact& c = it->second;
    ally::ui::TouchPoint t;
    t.id = it->first;
    t.pos = ImVec2(static_cast<float>(c.pos.x), static_cast<float>(c.pos.y));
    t.down_pos = ImVec2(static_cast<float>(c.down.x), static_cast<float>(c.down.y));
    t.just_down = c.fresh;
    c.fresh = false;
    out.push_back(t);
    if (c.up) it = g_touches.erase(it);
    else ++it;
  }
  return out;
}

// --demo-touch (with --smoke-test and a demo robot only): inject two fingers, one holding the drive stick and a
// second tapping the e-stop, to check on the device that a second finger reaches the e-stop.
struct DemoTouch {
  bool on{false}, started{false}, b_down{false};
  POINT a{}, b{};
};
DemoTouch g_dt;

POINTER_TOUCH_INFO touch_contact(UINT32 id, POINT p, POINTER_FLAGS flags) {
  POINTER_TOUCH_INFO t{};
  t.pointerInfo.pointerType = PT_TOUCH;
  t.pointerInfo.pointerId = id;
  t.pointerInfo.ptPixelLocation = p;
  t.pointerInfo.pointerFlags = flags;
  t.touchFlags = TOUCH_FLAG_NONE;
  t.touchMask = TOUCH_MASK_CONTACTAREA;
  t.rcContact = RECT{p.x - 4, p.y - 4, p.x + 4, p.y + 4};
  return t;
}

void demo_touch_step(int frame, const ally::ui::GuiApp& app) {
  constexpr int kStart = 100, kSecond = 130;
  constexpr POINTER_FLAGS kDown = POINTER_FLAG_DOWN | POINTER_FLAG_INRANGE | POINTER_FLAG_INCONTACT;
  constexpr POINTER_FLAGS kMove = POINTER_FLAG_UPDATE | POINTER_FLAG_INRANGE | POINTER_FLAG_INCONTACT;
  if (!g_dt.on || frame < kStart) return;
  if (frame == kStart) {
    const ImVec2 s = app.touch_target(0), e = app.touch_target(1);
    g_dt.a = POINT{static_cast<LONG>(s.x), static_cast<LONG>(s.y)};  // holds the stick at its centre: no drive command
    g_dt.b = POINT{static_cast<LONG>(e.x), static_cast<LONG>(e.y)};
    ClientToScreen(g_hwnd, &g_dt.a);
    ClientToScreen(g_hwnd, &g_dt.b);
    // Only ever touch our own window.
    const bool mine = GetAncestor(WindowFromPoint(g_dt.a), GA_ROOT) == g_hwnd && GetAncestor(WindowFromPoint(g_dt.b), GA_ROOT) == g_hwnd;
    if (!mine || !InitializeTouchInjection(2, TOUCH_FEEDBACK_NONE)) {
      boot_log(mine ? "demo-touch: touch injection unavailable" : "demo-touch: skipped (window not on top)");
      g_dt.on = false;
      return;
    }
    POINTER_TOUCH_INFO c[1] = {touch_contact(0, g_dt.a, kDown)};
    g_dt.started = InjectTouchInput(1, c) != FALSE;
    boot_log(g_dt.started ? "demo-touch: finger 1 on the drive stick" : "demo-touch: inject failed");
    return;
  }
  if (!g_dt.started) return;
  POINTER_TOUCH_INFO c[2] = {touch_contact(0, g_dt.a, kMove), touch_contact(1, g_dt.b, frame == kSecond ? kDown : kMove)};
  const UINT32 n = frame >= kSecond ? 2 : 1;
  if (frame == kSecond) boot_log("demo-touch: finger 2 on the e-stop");
  g_dt.b_down = n == 2;
  InjectTouchInput(n, c);
}

void demo_touch_release() {
  if (!g_dt.started) return;
  POINTER_TOUCH_INFO c[2] = {touch_contact(0, g_dt.a, POINTER_FLAG_UP), touch_contact(1, g_dt.b, POINTER_FLAG_UP)};
  InjectTouchInput(g_dt.b_down ? 2 : 1, c);
  g_dt.started = false;
}

LRESULT WINAPI wnd_proc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
  if (ImGui_ImplWin32_WndProcHandler(hwnd, msg, wp, lp)) return true;
  switch (msg) {
    case WM_POINTERDOWN:
    case WM_POINTERUPDATE:
    case WM_POINTERUP:
    case WM_POINTERCAPTURECHANGED:
      on_pointer(hwnd, msg, wp);
      break;  // DefWindowProc still turns the primary contact into mouse input
    case WM_SIZE:
      if (wp == SIZE_MINIMIZED) {
        g_focused = false;
        return 0;
      }
      g_resize_w = LOWORD(lp);
      g_resize_h = HIWORD(lp);
      return 0;
    case WM_ACTIVATE:
      g_focused = LOWORD(wp) != WA_INACTIVE && !HIWORD(wp);
      return 0;
    case WM_KILLFOCUS:
      g_focused = false;
      break;
    case WM_SETFOCUS:
      g_focused = true;
      break;
    case WM_SYSCOMMAND:
      if ((wp & 0xfff0) == SC_KEYMENU) return 0;  // Alt would open the window menu and steal input
      if ((wp & 0xfff0) == SC_SCREENSAVE || (wp & 0xfff0) == SC_MONITORPOWER) return 0;
      break;
    case WM_CLOSE:
      g_quit = true;
      return 0;
    case WM_DESTROY:
      PostQuitMessage(0);
      return 0;
    default:
      break;
  }
  return DefWindowProcW(hwnd, msg, wp, lp);
}

std::string font_path(const wchar_t* file) {
  wchar_t dir[MAX_PATH] = {0};
  GetWindowsDirectoryW(dir, MAX_PATH);
  return to_utf8(std::wstring(dir) + L"\\Fonts\\" + file);
}

bool file_exists(const std::string& utf8);

struct Blob {
  void* data{nullptr};
  int size{0};
};

Blob resource(int id) {
  HRSRC r = FindResourceW(nullptr, MAKEINTRESOURCEW(id), MAKEINTRESOURCEW(10) /* RT_RCDATA */);
  if (!r) return {};
  HGLOBAL g = LoadResource(nullptr, r);
  if (!g) return {};
  return Blob{LockResource(g), static_cast<int>(SizeofResource(nullptr, r))};
}

// Resource ids (res/fonts.rc.in)
constexpr int kResPretendardRegular = 201, kResPretendardSemiBold = 202, kResPretendardBold = 203;
constexpr int kResMonoRegular = 211, kResMonoMedium = 212, kResLucide = 221;

// Pretendard also has glyphs in the Private Use Area where the Lucide icons live; exclude them so icons win.
const ImWchar kPrivateUseArea[] = {0xE000, 0xF8FF, 0};

ImFont* add_font(ImGuiIO& io, const Blob& b, const char* name, bool merge, bool exclude_pua = false) {
  if (!b.data) return nullptr;
  ImFontConfig cfg;
  cfg.FontDataOwnedByAtlas = false;  // resource memory lives as long as the process
  cfg.MergeMode = merge;
  if (exclude_pua) cfg.GlyphExcludeRanges = kPrivateUseArea;
  std::snprintf(cfg.Name, sizeof(cfg.Name), "%s", name);
  return io.Fonts->AddFontFromMemoryTTF(b.data, b.size, 22.0f, &cfg);
}

ally::ui::Fonts load_fonts(ImGuiIO& io) {
  ally::ui::Fonts f;
  const Blob reg = resource(kResPretendardRegular), semi = resource(kResPretendardSemiBold), bold = resource(kResPretendardBold);
  const Blob mono = resource(kResMonoRegular), mono_med = resource(kResMonoMedium), icons = resource(kResLucide);
  if (reg.data && semi.data && bold.data && mono.data && mono_med.data) {
    auto text_font = [&](const Blob& b, const char* name) {
      ImFont* font = add_font(io, b, name, false, true);
      add_font(io, icons, "Lucide", true);  // icons usable inline in every text font
      return font;
    };
    auto num_font = [&](const Blob& b, const char* name) {
      ImFont* font = add_font(io, b, name, false);
      add_font(io, reg, "Pretendard (Hangul fallback)", true);  // mono has no Hangul
      return font;
    };
    f.regular = text_font(reg, "Pretendard Regular");
    f.semibold = text_font(semi, "Pretendard SemiBold");
    f.bold = text_font(bold, "Pretendard Bold");
    f.mono = num_font(mono, "JetBrains Mono");
    f.mono_medium = num_font(mono_med, "JetBrains Mono Medium");
    f.embedded = f.regular && f.semibold && f.bold && f.mono && f.mono_medium;
  }
  if (!f.embedded) {
    const std::string malgun = font_path(L"malgun.ttf"), malgun_bd = font_path(L"malgunbd.ttf");
    ImFont* r = file_exists(malgun) ? io.Fonts->AddFontFromFileTTF(malgun.c_str(), 22.0f) : io.Fonts->AddFontDefault();
    ImFont* b = file_exists(malgun_bd) ? io.Fonts->AddFontFromFileTTF(malgun_bd.c_str(), 22.0f) : r;
    f.regular = f.mono = f.mono_medium = r;
    f.semibold = f.bold = b ? b : r;
  }
  io.FontDefault = f.regular;
  return f;
}

std::string strf_scale(float scale) { return ally::strf(", ui scale %.2f", static_cast<double>(scale)); }

bool file_exists(const std::string& utf8) {
  return GetFileAttributesW(to_wide(utf8).c_str()) != INVALID_FILE_ATTRIBUTES;
}

}  // namespace

int WINAPI WinMain(HINSTANCE inst, HINSTANCE, LPSTR, int) {
  // Run-time LoadLibrary calls (ours and the ImGui backend's) search System32 only, never the exe folder.
  SetDefaultDllDirectories(LOAD_LIBRARY_SEARCH_SYSTEM32);
  // Options (all optional, for tests and documentation screenshots):
  //   --smoke-test [--frames N]  draw N frames (default 180) and exit, never block on dialogs
  //   --screenshot FILE.bmp      save the last frame (implies --smoke-test)
  //   --demo-connect HOST --demo-ports BASE [--tab drive|arm|grasp|diag|settings]  connect for this run only
  //   --demo-state estop|cambig|grasp  screenshot-only display states (e-stop latched on the mock robot / full camera /
  //                              first detected object selected; nothing moves)
  //   --demo-calib FILE          camera calibration for this run only (never saved)
  //   --demo-touch               with --smoke-test and --demo-connect: inject a 2-finger touch (stick + e-stop)
  int argc = 0;
  LPWSTR* argv = CommandLineToArgvW(GetCommandLineW(), &argc);
  int smoke_frames = 180, demo_base = 0;
  std::wstring shot;
  std::string demo_host, demo_tab, demo_state, demo_calib;
  for (int i = 1; argv && i < argc; ++i) {
    const std::wstring a = argv[i];
    auto next = [&]() { return i + 1 < argc ? std::wstring(argv[++i]) : std::wstring(); };
    if (a == L"--smoke-test") g_smoke = true;
    else if (a == L"--frames") smoke_frames = std::max(1, _wtoi(next().c_str()));
    else if (a == L"--screenshot") { shot = next(); g_smoke = true; }
    else if (a == L"--demo-connect") demo_host = to_utf8(next());
    else if (a == L"--demo-ports") demo_base = _wtoi(next().c_str());
    else if (a == L"--tab") demo_tab = to_utf8(next());
    else if (a == L"--demo-state") demo_state = to_utf8(next());
    else if (a == L"--demo-calib") demo_calib = to_utf8(next());
    else if (a == L"--demo-touch") g_dt.on = true;
  }
  if (argv) LocalFree(argv);
  {
    const std::wstring boot = to_wide(ally::settings_dir() + "/boot.log");
    g_boot = _wfopen(boot.c_str(), L"w");
    boot_log(std::string(ally::kAppName) + " " + ally::kVersion + (g_smoke ? " (smoke test)" : ""));
  }
  // One controller per device: two copies would fight over the robot session and the gamepad.
  HANDLE single = CreateMutexW(nullptr, TRUE, L"Local\\ArmPiAllyController.SingleInstance");
  if (single && GetLastError() == ERROR_ALREADY_EXISTS) {
    boot_log("already running");
    if (!g_smoke) MessageBoxW(nullptr, L"ArmPi Ally Controller가 이미 실행 중입니다.", L"ArmPi Ally Controller", MB_ICONINFORMATION);
    return 0;
  }
  ImGui_ImplWin32_EnableDpiAwareness();
  std::string err;
  if (!ally::net::init_sockets(&err)) {
    fatal(to_wide(err), err);
    return 1;
  }
  ally::Settings settings;
  ally::load_settings(ally::settings_path(), &settings, &err);
  g_fullscreen = settings.fullscreen;

  WNDCLASSEXW wc{sizeof(wc), CS_CLASSDC, wnd_proc, 0, 0, inst, LoadIconW(inst, MAKEINTRESOURCEW(1)), LoadCursorW(nullptr, IDC_ARROW),
                 nullptr, nullptr, L"ArmPiAllyWindow", LoadIconW(inst, MAKEINTRESOURCEW(1))};
  RegisterClassExW(&wc);
  g_hwnd = CreateWindowExW(0, wc.lpszClassName, L"ArmPi Ally Controller", WS_OVERLAPPEDWINDOW, g_windowed_rect.left,
                           g_windowed_rect.top, g_windowed_rect.right - g_windowed_rect.left,
                           g_windowed_rect.bottom - g_windowed_rect.top, nullptr, nullptr, inst, nullptr);
  if (!g_hwnd || !create_device(g_hwnd)) {
    cleanup_device();
    fatal(L"Direct3D 11 장치를 만들지 못했습니다.", g_hwnd ? "Direct3D 11 device" : "CreateWindow");
    return 1;
  }
  boot_log(std::string("window + D3D11 ok, driver ") + g_driver);
  const bool want_full = g_fullscreen;
  g_fullscreen = false;
  ShowWindow(g_hwnd, SW_SHOWDEFAULT);
  apply_fullscreen(want_full);
  UpdateWindow(g_hwnd);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.IniFilename = nullptr;  // layout is code-defined; no imgui.ini next to the exe
  io.ConfigFlags &= ~(ImGuiConfigFlags_NavEnableKeyboard | ImGuiConfigFlags_NavEnableGamepad);  // keys/pad drive the robot

  RECT rc;
  GetClientRect(g_hwnd, &rc);
  const float scale = std::max(0.6f, static_cast<float>(rc.bottom - rc.top) / 1080.0f);
  ally::ui::apply_theme(ImGui::GetStyle(), scale);
  ImGuiStyle& style = ImGui::GetStyle();
  style.FontSizeBase = 22.0f;
  style.FontScaleMain = scale;

  // Fonts are embedded as RCDATA: Pretendard (text) + JetBrains Mono (numbers), both OFL, with the Lucide icon
  // subset (ISC) merged into the text fonts. If the resources are missing, fall back to the system Malgun Gothic.
  const ally::ui::Fonts fonts = load_fonts(io);
  boot_log(std::string("fonts: ") + (fonts.embedded ? "embedded Pretendard + JetBrains Mono + Lucide" : "system fallback") +
           strf_scale(scale));

  ImGui_ImplWin32_Init(g_hwnd);
  ImGui_ImplDX11_Init(g_device, g_ctx);

  ally::ui::Platform plat;
  plat.upload_camera = upload_camera;
  plat.set_fullscreen = apply_fullscreen;
  plat.keep_awake = keep_awake;
  plat.quit = [] { g_quit = true; };
  plat.ally_battery_percent = battery_percent;
  plat.ally_on_ac = on_ac;
  plat.open_folder = open_folder;
  plat.touches = take_touches;
  plat.capture = [](int n, double s) { capture_request(n, s); };

  {
    ally::ui::GuiApp app(plat, settings, fonts);
    if (!demo_host.empty() || !demo_tab.empty()) app.set_demo_connect(demo_host, demo_base, demo_tab);
    if (!demo_state.empty()) app.set_demo_state(demo_state);
    if (!demo_calib.empty()) app.set_demo_calib(demo_calib);
    boot_log("ui ready");
    int frames = 0;
    const double smoke_start = ally::mono_now();
    LARGE_INTEGER freq, t0, t;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&t0);
    timeBeginPeriod(1);
    while (!g_quit) {
      MSG msg;
      while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) {
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
        if (msg.message == WM_QUIT) g_quit = true;
      }
      if (g_quit) break;
      const bool focused = g_focused && GetForegroundWindow() == g_hwnd;
      app.set_focused(focused);
      if (g_resize_w && g_resize_h) {
        cleanup_render_target();
        g_swap->ResizeBuffers(0, g_resize_w, g_resize_h, DXGI_FORMAT_UNKNOWN, 0);
        g_resize_w = g_resize_h = 0;
        create_render_target();
      }
      QueryPerformanceCounter(&t);
      const double frame_start = static_cast<double>(t.QuadPart) / static_cast<double>(freq.QuadPart);
      if (g_occluded && g_swap->Present(0, DXGI_PRESENT_TEST) == DXGI_STATUS_OCCLUDED) {
        // Hidden: the UI still has to run the input heartbeat at a low rate so the core keeps (neutral) state.
        ImGui_ImplDX11_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();
        app.frame(ally::mono_now());
        ImGui::EndFrame();
        Sleep(33);
        continue;
      }
      g_occluded = false;
      ImGui_ImplDX11_NewFrame();
      ImGui_ImplWin32_NewFrame();
      ImGui::NewFrame();
      app.frame(ally::mono_now());
      ImGui::Render();
      const float clear[4] = {0x13 / 255.0f, 0x16 / 255.0f, 0x1B / 255.0f, 1.0f};  // bg1 token
      g_ctx->OMSetRenderTargets(1, &g_rtv, nullptr);
      g_ctx->ClearRenderTargetView(g_rtv, clear);
      ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
      if (g_smoke && !demo_host.empty()) demo_touch_step(frames, app);
      capture_step(ally::mono_now(), app);
      if (g_smoke && frames + 1 >= smoke_frames && !shot.empty())
        boot_log(save_backbuffer_bmp(shot) ? "screenshot saved" : "screenshot FAILED");
      const HRESULT hr = g_swap->Present(1, 0);  // vsync (120 Hz on the Ally X)
      g_occluded = hr == DXGI_STATUS_OCCLUDED;
      if (g_smoke && ++frames >= smoke_frames) {
        boot_log(ally::strf("smoke: %d frames in %.2f s", frames, ally::mono_now() - smoke_start));
        g_quit = true;
      }
      // frame cap below the refresh rate (battery), 30 fps when not focused
      const int cap = focused ? app.settings().fps_cap : 30;
      QueryPerformanceCounter(&t);
      const double elapsed = static_cast<double>(t.QuadPart) / static_cast<double>(freq.QuadPart) - frame_start;
      const double want = 1.0 / std::max(30, cap);
      if (cap < 110 && elapsed < want) Sleep(static_cast<DWORD>((want - elapsed) * 1000.0));
    }
    timeEndPeriod(1);
    demo_touch_release();
    app.shutdown();  // neutral + disconnect + rumble off + save, before the window goes away
    boot_log("shutdown ok");
  }
  keep_awake(false);
  ImGui_ImplDX11_Shutdown();
  ImGui_ImplWin32_Shutdown();
  ImGui::DestroyContext();
  cleanup_device();
  DestroyWindow(g_hwnd);
  UnregisterClassW(wc.lpszClassName, wc.hInstance);
  if (single) {
    ReleaseMutex(single);
    CloseHandle(single);
  }
  if (g_boot) std::fclose(g_boot);
  return 0;
}
