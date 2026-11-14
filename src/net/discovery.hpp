// Robot discovery (Protocol v3, UDP 5002): ARMPI_DISCOVER/1|<nonce> -> ARMPI_HERE/1|<nonce>|<id>|<name>|<port>|3
// With no explicit targets, the request goes to each local interface's broadcast address + 255.255.255.255.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "net/comm_log.hpp"

namespace ally::net {

constexpr int kDiscoveryPort = 5002;
constexpr int kControlPort = 5001;

struct RobotInfo {
  std::string address;
  std::string device_id;
  std::string name;
  int control_port{kControlPort};
  int protocol{3};
  double rtt_ms{0.0};
};

std::optional<RobotInfo> parse_discovery_reply(const std::string& packet, const std::string& nonce);
std::string url_decode(const std::string& s);

std::vector<RobotInfo> discover(double timeout_s, const std::vector<std::string>& targets, CommLog* log,
                                std::string* err, int port = kDiscoveryPort);

}  // namespace ally::net
