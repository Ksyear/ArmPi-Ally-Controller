#include "input/tilt.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <thread>

#include "common/util.hpp"

#ifdef _WIN32
#include <windows.h>
// clang-format off
#include <objbase.h>
#include <propidl.h>
#include <propsys.h>  // PROPERTYKEY (WIN32_LEAN_AND_MEAN leaves it out)
#include <initguid.h>  // defines the sensor GUIDs / property keys used below in this file
#include <propkeydef.h>
#include <sensors.h>
#include <sensorsapi.h>
// clang-format on
#endif

namespace ally::input {

namespace {
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
}

double tilt_pitch_deg(const Gravity& g) { return std::atan2(-g.z, std::sqrt(g.x * g.x + g.y * g.y)) * kRadToDeg; }

double tilt_roll_deg(const Gravity& g) { return std::atan2(g.x, std::sqrt(g.y * g.y + g.z * g.z)) * kRadToDeg; }

TiltDrive tilt_to_drive(double pitch_deg, double roll_deg, double ref_pitch_deg, double ref_roll_deg, double dead_deg,
                        double full_deg) {
  auto axis = [&](double d) {
    if (!std::isfinite(d)) return 0.0;
    // wrap to -180..180 so the reference crossing +-180 does not jump
    while (d > 180.0) d -= 360.0;
    while (d < -180.0) d += 360.0;
    const double a = std::fabs(d);
    if (!(full_deg > dead_deg) || !(dead_deg >= 0.0) || a <= dead_deg) return 0.0;  // also catches NaN settings
    return std::copysign(std::min(1.0, (a - dead_deg) / (full_deg - dead_deg)), d);
  };
  TiltDrive t;
  t.forward = axis(pitch_deg - ref_pitch_deg);
  t.turn = -axis(roll_deg - ref_roll_deg);  // right side down = clockwise = negative turn
  return t;
}

#ifdef _WIN32

// The Sensor API call can block for a long time, so it runs on its own (MTA) thread; the UI only reads the cache.
struct TiltSensor::Impl {
  std::thread th;
  std::atomic<bool> stop{false};
  std::mutex mu;
  std::condition_variable cv;
  int state{0};  // 0 = starting, 1 = running, -1 = failed
  std::string err;
  bool have{false};
  Gravity last;
  double last_ok{-1e9};  // mono time of the last successful answer (new report or "unchanged")
  double last_call_ms{0};
  bool exited{false};

  void run() {
    const HRESULT ci = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    ISensorManager* mgr = nullptr;
    ISensor* sensor = nullptr;
    std::string e;
    HRESULT hr = CoCreateInstance(CLSID_SensorManager, nullptr, CLSCTX_INPROC_SERVER, IID_ISensorManager,
                                  reinterpret_cast<void**>(&mgr));
    if (SUCCEEDED(hr) && mgr) {
      ISensorCollection* coll = nullptr;
      hr = mgr->GetSensorsByType(SENSOR_TYPE_ACCELEROMETER_3D, &coll);
      ULONG n = 0;
      if (SUCCEEDED(hr) && coll) coll->GetCount(&n);
      if (n > 0) coll->GetAt(0, &sensor);
      if (coll) coll->Release();
      if (!sensor) e = "가속도계 없음";
    } else {
      e = "센서 관리자 없음";
    }
    {
      std::lock_guard<std::mutex> lk(mu);
      state = sensor ? 1 : -1;
      err = e;
    }
    cv.notify_all();
    while (sensor && !stop) {
      const double t0 = mono_now();
      ISensorDataReport* rep = nullptr;
      hr = sensor->GetData(&rep);
      double v[3] = {0, 0, 0};
      bool ok = SUCCEEDED(hr);
      bool fresh = false;
      if (ok && rep) {
        const PROPERTYKEY keys[3] = {SENSOR_DATA_TYPE_ACCELERATION_X_G, SENSOR_DATA_TYPE_ACCELERATION_Y_G,
                                     SENSOR_DATA_TYPE_ACCELERATION_Z_G};
        for (int i = 0; i < 3; ++i) {
          PROPVARIANT pv;
          PropVariantInit(&pv);
          if (SUCCEEDED(rep->GetSensorValue(keys[i], &pv)) && pv.vt == VT_R8) v[i] = pv.dblVal;
          else ok = false;
          PropVariantClear(&pv);
        }
        rep->Release();
        const double m = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        ok = ok && std::isfinite(m) && m > 0.3 && m < 3.0;  // about 1 g at rest; reject garbage
        fresh = ok;
      }
      {
        std::lock_guard<std::mutex> lk(mu);
        last_call_ms = (mono_now() - t0) * 1000.0;
        if (fresh) {
          last = Gravity{v[0], v[1], v[2]};
          have = true;
        }
        // S_OK without a report = nothing changed (the accelerometer reports on change only)
        if (ok && have) last_ok = mono_now();
        if (!SUCCEEDED(hr)) {
          char b[48];
          std::snprintf(b, sizeof(b), "GetData 0x%08lx", static_cast<unsigned long>(hr));
          err = b;
        }
      }
      for (int i = 0; i < 5 && !stop; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (sensor) sensor->Release();
    if (mgr) mgr->Release();
    if (SUCCEEDED(ci)) CoUninitialize();
    {
      std::lock_guard<std::mutex> lk(mu);
      exited = true;
    }
    cv.notify_all();
  }
};

TiltSensor::TiltSensor() : impl_(std::make_shared<Impl>()) {}

TiltSensor::~TiltSensor() {
  if (!impl_->th.joinable()) return;
  impl_->stop = true;
  // A sensor call can block for seconds while the device lies still: wait briefly, then let the thread finish on its
  // own (it holds its own reference to the shared state).
  std::unique_lock<std::mutex> lk(impl_->mu);
  const bool done = impl_->cv.wait_for(lk, std::chrono::milliseconds(1000), [&] { return impl_->exited; });
  lk.unlock();
  if (done) impl_->th.join();
  else impl_->th.detach();
}

void TiltSensor::start() {
  if (impl_->th.joinable()) return;
  std::shared_ptr<Impl> self = impl_;
  impl_->th = std::thread([self] { self->run(); });
}

int TiltSensor::state() const {
  std::lock_guard<std::mutex> lk(impl_->mu);
  return impl_->state;
}

bool TiltSensor::open(std::string* err) {
  start();
  std::unique_lock<std::mutex> lk(impl_->mu);
  impl_->cv.wait_for(lk, std::chrono::milliseconds(1500), [&] { return impl_->state != 0; });
  if (impl_->state == 1) return true;
  if (err) *err = impl_->state == 0 ? "센서 응답 없음" : impl_->err;
  return false;
}

bool TiltSensor::available() const {
  std::lock_guard<std::mutex> lk(impl_->mu);
  return impl_->state == 1;
}

bool TiltSensor::read(Gravity* g, double* stamp) {
  std::lock_guard<std::mutex> lk(impl_->mu);
  // Stale for more than a second (sensor stuck or gone): no tilt input.
  if (impl_->state != 1 || !impl_->have || mono_now() - impl_->last_ok > 1.0) return false;
  *g = impl_->last;
  if (stamp) *stamp = impl_->last_ok;
  return true;
}

std::string TiltSensor::last_error() const {
  std::lock_guard<std::mutex> lk(impl_->mu);
  return impl_->err + (impl_->err.empty() ? "" : " · ") + strf("마지막 읽기 %.0f ms", impl_->last_call_ms);
}

#else

struct TiltSensor::Impl {};
TiltSensor::TiltSensor() : impl_(std::make_shared<Impl>()) {}
TiltSensor::~TiltSensor() = default;
void TiltSensor::start() {}
int TiltSensor::state() const { return -1; }
bool TiltSensor::open(std::string* err) {
  if (err) *err = "이 플랫폼에는 기울기 센서 없음";
  return false;
}
bool TiltSensor::available() const { return false; }
bool TiltSensor::read(Gravity*, double*) { return false; }
std::string TiltSensor::last_error() const { return {}; }

#endif

}  // namespace ally::input
