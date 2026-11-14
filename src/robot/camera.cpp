#include "robot/camera.hpp"

#include <chrono>

#include "net/http.hpp"

namespace ally::robot {

std::string default_camera_url(const std::string& host, int port) {
  return "http://" + host + ":" + std::to_string(port) +
         "/stream?topic=/usb_cam/image_raw&type=ros_compressed&qos_profile=sensor_data";
}

CameraFeed::~CameraFeed() { stop(); }

void CameraFeed::start(const std::string& url) {
  stop();
  {
    std::lock_guard<std::mutex> lk(mu_);
    url_ = url;
    status_ = "연결 중";
    fps_ = 0.0;
  }
  stop_ = false;
  running_ = true;
  thread_ = std::thread([this] { loop(); });
}

void CameraFeed::stop() {
  stop_ = true;
  if (thread_.joinable()) thread_.join();
  running_ = false;
  streaming_ = false;
  std::lock_guard<std::mutex> lk(mu_);
  status_ = "꺼짐";
}

std::shared_ptr<const Frame> CameraFeed::latest(uint64_t have_seq) const {
  std::lock_guard<std::mutex> lk(mu_);
  if (!frame_ || frame_->seq <= have_seq) return nullptr;
  return frame_;
}

double CameraFeed::fps() const {
  std::lock_guard<std::mutex> lk(mu_);
  return mono_now() - last_frame_mono_ > 2.0 ? 0.0 : fps_;
}

std::string CameraFeed::status() const {
  std::lock_guard<std::mutex> lk(mu_);
  return status_;
}

std::string CameraFeed::url() const {
  std::lock_guard<std::mutex> lk(mu_);
  return url_;
}

std::string CameraFeed::latest_encoded() const {
  std::lock_guard<std::mutex> lk(mu_);
  return encoded_;
}

void CameraFeed::loop() {
  std::string url;
  {
    std::lock_guard<std::mutex> lk(mu_);
    url = url_;
  }
  const auto u = net::parse_url(url);
  if (!u) {
    std::lock_guard<std::mutex> lk(mu_);
    status_ = "영상 주소 오류";
    log_.error("영상 주소 오류: " + url);
    return;
  }
  bool logged = false;
  while (!stop_) {
    std::string decode_err;
    const std::string why = net::read_mjpeg(*u, stop_, 3.0, [&](const std::string&, std::string&& data) {
      auto f = std::make_shared<Frame>();
      if (!decode_image(data, &f->width, &f->height, &f->rgba, &decode_err)) return !stop_;
      const double now = mono_now();
      std::lock_guard<std::mutex> lk(mu_);
      if (last_frame_mono_ > 0) {
        const double dt = now - last_frame_mono_;
        if (dt > 0) fps_ = fps_ <= 0 ? 1.0 / dt : fps_ * 0.9 + 0.1 / dt;
      }
      last_frame_mono_ = now;
      f->seq = ++frames_;
      f->mono = now;
      frame_ = std::move(f);
      encoded_ = std::move(data);
      if (!streaming_) {
        streaming_ = true;
        status_ = "수신 중";
        logged = false;
        log_.ok("영상 수신 시작");
      }
      return !stop_;
    });
    streaming_ = false;
    if (stop_) break;
    {
      std::lock_guard<std::mutex> lk(mu_);
      status_ = why.empty() ? "끊김" : why;
    }
    if (!logged) log_.warn("영상: " + (why.empty() ? std::string("끊김") : why) + " — 2초 뒤 다시 시도");
    logged = true;
    for (int i = 0; i < 20 && !stop_; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

}  // namespace ally::robot
