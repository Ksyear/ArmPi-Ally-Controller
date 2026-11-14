// Robot camera: MJPEG over HTTP (web_video_server) -> decoded RGBA frames on a background thread.
// Always request qos_profile=sensor_data: the default (reliable) QoS once stalled ROS traffic inside the robot.
#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "common/util.hpp"

namespace ally::robot {

constexpr size_t kMaxFrameBytes = 8u * 1024 * 1024;      // one MJPEG frame (real frames are a few hundred KB)
constexpr long long kMaxFramePixels = 3840LL * 2160;     // decoded size limit

struct Frame {
  int width{0}, height{0};
  std::vector<uint8_t> rgba;
  uint64_t seq{0};
  double mono{0.0};
};

std::string default_camera_url(const std::string& host, int port = 8080);

// JPEG/PNG -> RGBA8 (stb_image). Returns false with a message on failure.
bool decode_image(const std::string& bytes, int* w, int* h, std::vector<uint8_t>* rgba, std::string* err);

class CameraFeed {
 public:
  explicit CameraFeed(EventLog& log) : log_(log) {}
  ~CameraFeed();
  CameraFeed(const CameraFeed&) = delete;
  CameraFeed& operator=(const CameraFeed&) = delete;

  void start(const std::string& url);
  void stop();
  bool running() const { return running_; }
  bool streaming() const { return streaming_; }

  // Newest frame if its seq is greater than have_seq, else nullptr. Shared, never copied.
  std::shared_ptr<const Frame> latest(uint64_t have_seq) const;
  double fps() const;
  uint64_t frames() const { return frames_; }
  std::string status() const;
  std::string url() const;
  // Raw bytes of the newest encoded frame (for saving a snapshot)
  std::string latest_encoded() const;

 private:
  void loop();

  EventLog& log_;
  std::thread thread_;
  std::atomic<bool> running_{false}, stop_{false}, streaming_{false};
  std::atomic<uint64_t> frames_{0};
  mutable std::mutex mu_;
  std::string url_;
  std::shared_ptr<const Frame> frame_;
  std::string encoded_;
  double fps_{0.0};
  double last_frame_mono_{0.0};
  std::string status_{"꺼짐"};
};

}  // namespace ally::robot
