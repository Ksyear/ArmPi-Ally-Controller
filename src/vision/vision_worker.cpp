#include "vision/vision_worker.hpp"

#include <chrono>

#include "common/util.hpp"

namespace ally::vision {

VisionWorker::VisionWorker() = default;

VisionWorker::~VisionWorker() { stop(); }

void VisionWorker::start() {
  if (running_) return;
  stop_ = false;
  running_ = true;
  thread_ = std::thread([this] { loop(); });
}

void VisionWorker::stop() {
  {
    std::lock_guard<std::mutex> lk(mu_);
    stop_ = true;
  }
  cv_.notify_all();
  if (thread_.joinable()) thread_.join();
  running_ = false;
}

void VisionWorker::set_enabled(bool on) {
  enabled_ = on;
  if (!on) {
    std::lock_guard<std::mutex> lk(mu_);
    pending_.reset();
  }
}

void VisionWorker::set_max_hz(double hz) {
  std::lock_guard<std::mutex> lk(mu_);
  min_period_s_ = hz > 0 ? 1.0 / hz : 0.0;
}

void VisionWorker::submit(std::shared_ptr<const robot::Frame> frame) {
  if (!enabled_ || !frame) return;
  {
    std::lock_guard<std::mutex> lk(mu_);
    pending_ = std::move(frame);
  }
  cv_.notify_one();
}

std::shared_ptr<const DetectionSet> VisionWorker::latest() const {
  std::lock_guard<std::mutex> lk(mu_);
  return latest_;
}

double VisionWorker::avg_ms() const {
  std::lock_guard<std::mutex> lk(mu_);
  return avg_ms_;
}

void VisionWorker::clear() {
  std::lock_guard<std::mutex> lk(mu_);
  latest_.reset();
  pending_.reset();
}

void VisionWorker::loop() {
  double last_run = 0.0;
  while (true) {
    std::shared_ptr<const robot::Frame> f;
    double period;
    {
      std::unique_lock<std::mutex> lk(mu_);
      cv_.wait_for(lk, std::chrono::milliseconds(100), [&] { return stop_.load() || pending_ != nullptr; });
      if (stop_) break;
      if (!pending_) continue;
      period = min_period_s_;
      if (mono_now() - last_run < period) {
        lk.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        continue;
      }
      f = std::move(pending_);
      pending_.reset();
    }
    last_run = mono_now();
    std::shared_ptr<const DetectionSet> prev = latest();
    auto ds = std::make_shared<DetectionSet>(detector_.detect(*f, prev.get()));
    std::lock_guard<std::mutex> lk(mu_);
    avg_ms_ = avg_ms_ <= 0 ? ds->compute_ms : avg_ms_ * 0.9 + ds->compute_ms * 0.1;
    latest_ = std::move(ds);
  }
}

}  // namespace ally::vision
