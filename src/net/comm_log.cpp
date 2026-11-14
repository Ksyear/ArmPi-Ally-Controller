#include "net/comm_log.hpp"

#include "common/util.hpp"

namespace ally::net {

const char* channel_name(Channel c) {
  switch (c) {
    case Channel::kControl: return "UDP 5001";
    case Channel::kDiscovery: return "UDP 5002";
    case Channel::kRosbridge: return "WS 9090";
  }
  return "?";
}

void CommLog::record(bool outgoing, Channel ch, const std::string& peer, const std::string& data) {
  std::lock_guard<std::mutex> lk(mu_);
  if (outgoing) ++tx_;
  else ++rx_;
  if (ch == Channel::kRosbridge && !keep_ros_) return;
  // Keep only the head of each message: the panel shows ~160 characters and messages can be large.
  constexpr size_t kKeep = 4096;
  ring_.push_back(CommRecord{mono_now(), wall_clock_ms(), outgoing, ch, peer, data.size() > kKeep ? data.substr(0, kKeep) : data});
  while (ring_.size() > cap_) ring_.pop_front();
}

std::vector<CommRecord> CommLog::last(size_t n) const {
  std::lock_guard<std::mutex> lk(mu_);
  const size_t k = std::min(n, ring_.size());
  return std::vector<CommRecord>(ring_.end() - static_cast<long>(k), ring_.end());
}

uint64_t CommLog::tx() const { std::lock_guard<std::mutex> lk(mu_); return tx_; }
uint64_t CommLog::rx() const { std::lock_guard<std::mutex> lk(mu_); return rx_; }

void CommLog::set_keep_rosbridge(bool on) {
  std::lock_guard<std::mutex> lk(mu_);
  keep_ros_ = on;
}

bool CommLog::keep_rosbridge() const {
  std::lock_guard<std::mutex> lk(mu_);
  return keep_ros_;
}

}  // namespace ally::net
