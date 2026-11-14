// Detection thread: the UI hands it the newest camera frame (shared, no copy); it runs the detector at
// up to max_hz and publishes the latest DetectionSet. Never blocks the UI or the control loop.
#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include "vision/detector.hpp"

namespace ally::vision {

class VisionWorker {
 public:
  VisionWorker();
  ~VisionWorker();
  VisionWorker(const VisionWorker&) = delete;
  VisionWorker& operator=(const VisionWorker&) = delete;

  void start();
  void stop();
  void set_enabled(bool on);  // off: frames are ignored (saves CPU outside the grasp tab)
  bool enabled() const { return enabled_; }
  void set_max_hz(double hz);
  void submit(std::shared_ptr<const robot::Frame> frame);  // newest frame wins
  Detector& detector() { return detector_; }
  std::shared_ptr<const DetectionSet> latest() const;
  double avg_ms() const;
  void clear();

 private:
  void loop();

  Detector detector_;
  std::thread thread_;
  std::atomic<bool> stop_{false}, running_{false}, enabled_{false};
  mutable std::mutex mu_;
  std::condition_variable cv_;
  std::shared_ptr<const robot::Frame> pending_;
  std::shared_ptr<const DetectionSet> latest_;
  double min_period_s_{0.1};
  double avg_ms_{0.0};
};

}  // namespace ally::vision
