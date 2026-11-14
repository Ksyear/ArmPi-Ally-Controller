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
#include <string>
#include <vector>

#include "app/gui_app.hpp"
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
bool save_backbuffer_bmp(const std::wstring& path) {
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
      const uint32_t w = d.Width, h = d.Height, row = w * 4;
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
          const uint8_t* src = static_cast<const uint8_t*>(m.pData) + static_cast<size_t>(y) * m.RowPitch;
          for (uint32_t x = 0; x < w; ++x) {  // RGBA -> BGRA
            line[x * 4 + 0] = src[x * 4 + 2];
            line[x * 4 + 1] = src[x * 4 + 1];
            line[x * 4 + 2] = src[x * 4 + 0];
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

LRESULT WINAPI wnd_proc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
  if (ImGui_ImplWin32_WndProcHandler(hwnd, msg, wp, lp)) return true;
  switch (msg) {
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
  //   --demo-connect HOST --demo-ports BASE [--tab drive|arm|diag|settings]  connect for this run only
  int argc = 0;
  LPWSTR* argv = CommandLineToArgvW(GetCommandLineW(), &argc);
  int smoke_frames = 180, demo_base = 0;
  std::wstring shot;
  std::string demo_host, demo_tab;
  for (int i = 1; argv && i < argc; ++i) {
    const std::wstring a = argv[i];
    auto next = [&]() { return i + 1 < argc ? std::wstring(argv[++i]) : std::wstring(); };
    if (a == L"--smoke-test") g_smoke = true;
    else if (a == L"--frames") smoke_frames = std::max(1, _wtoi(next().c_str()));
    else if (a == L"--screenshot") { shot = next(); g_smoke = true; }
    else if (a == L"--demo-connect") demo_host = to_utf8(next());
    else if (a == L"--demo-ports") demo_base = _wtoi(next().c_str());
    else if (a == L"--tab") demo_tab = to_utf8(next());
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
  ImGui::StyleColorsDark();
  ImGuiStyle& style = ImGui::GetStyle();
  style.FrameRounding = 10.0f;
  style.GrabRounding = 10.0f;
  style.WindowRounding = 0.0f;
  style.ChildRounding = 8.0f;
  style.FramePadding = ImVec2(12, 8);
  style.ItemSpacing = ImVec2(10, 10);
  style.ScrollbarSize = 22.0f;
  style.GrabMinSize = 28.0f;
  style.Colors[ImGuiCol_Button] = ImVec4(0.30f, 0.33f, 0.38f, 1.0f);
  style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.38f, 0.42f, 0.48f, 1.0f);
  style.Colors[ImGuiCol_ButtonActive] = ImVec4(1.0f, 0.55f, 0.16f, 1.0f);
  style.Colors[ImGuiCol_SliderGrab] = ImVec4(1.0f, 0.55f, 0.16f, 1.0f);
  style.Colors[ImGuiCol_SliderGrabActive] = ImVec4(1.0f, 0.65f, 0.30f, 1.0f);
  style.Colors[ImGuiCol_CheckMark] = ImVec4(1.0f, 0.55f, 0.16f, 1.0f);
  style.Colors[ImGuiCol_Header] = ImVec4(0.25f, 0.28f, 0.33f, 1.0f);
  style.ScaleAllSizes(scale);
  style.FontSizeBase = 22.0f;
  style.FontScaleMain = scale;

  // Fonts are loaded from Windows at run time (Malgun Gothic covers Hangul); nothing is bundled.
  ally::ui::Fonts fonts;
  const std::string regular = font_path(L"malgun.ttf"), bold = font_path(L"malgunbd.ttf");
  fonts.regular = file_exists(regular) ? io.Fonts->AddFontFromFileTTF(regular.c_str(), 22.0f) : io.Fonts->AddFontDefault();
  fonts.bold = file_exists(bold) ? io.Fonts->AddFontFromFileTTF(bold.c_str(), 22.0f) : fonts.regular;
  if (!fonts.regular) fonts.regular = io.Fonts->AddFontDefault();
  if (!fonts.bold) fonts.bold = fonts.regular;
  io.FontDefault = fonts.regular;
  boot_log(std::string("fonts: ") + (file_exists(regular) ? "malgun.ttf" : "default") + ", " +
           (file_exists(bold) ? "malgunbd.ttf" : "regular") + strf_scale(scale));

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

  {
    ally::ui::GuiApp app(plat, settings, fonts);
    if (!demo_host.empty()) app.set_demo_connect(demo_host, demo_base, demo_tab);
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
      const float clear[4] = {0.09f, 0.10f, 0.12f, 1.0f};
      g_ctx->OMSetRenderTargets(1, &g_rtv, nullptr);
      g_ctx->ClearRenderTargetView(g_rtv, clear);
      ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
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
