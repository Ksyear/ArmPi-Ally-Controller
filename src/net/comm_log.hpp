// Recent messages this app sent/received (debug panel). Only our own traffic.
#pragma once

#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

namespace ally::net {

enum class Channel { kControl, kDiscovery, kRosbridge };
const char* channel_name(Channel c);

struct CommRecord {
  double mono{0.0};
  std::string time;  // HH:MM:SS.mmm
  bool outgoing{false};
  Channel channel{Channel::kControl};
  std::string peer;  // ip:port
  std::string data;
};

class CommLog {
 public:
  explicit CommLog(size_t capacity = 2000) : cap_(capacity) {}
  void record(bool outgoing, Channel ch, const std::string& peer, const std::string& data);
  std::vector<CommRecord> last(size_t n) const;
  uint64_t tx() const;
  uint64_t rx() const;
  // rosbridge messages arrive at tens per second and would push UDP out of the ring
  void set_keep_rosbridge(bool on);
  bool keep_rosbridge() const;

 private:
  mutable std::mutex mu_;
  std::deque<CommRecord> ring_;
  size_t cap_;
  uint64_t tx_{0}, rx_{0};
  bool keep_ros_{false};
};

}  // namespace ally::net
