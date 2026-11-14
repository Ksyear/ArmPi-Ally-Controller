#include "net/discovery.hpp"

#include <cstdlib>
#include <map>
#include <random>

#include "common/util.hpp"
#include "net/sock.hpp"

namespace ally::net {

namespace {

std::string random_hex(size_t n_bytes) {
  std::random_device rd;
  static const char* kHex = "0123456789abcdef";
  std::string s;
  for (size_t i = 0; i < n_bytes; ++i) {
    const unsigned v = rd() & 0xFF;
    s.push_back(kHex[v >> 4]);
    s.push_back(kHex[v & 0xF]);
  }
  return s;
}

bool is_hex32(const std::string& s) {
  if (s.size() != 32) return false;
  for (char c : s) {
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'))) return false;
  }
  return true;
}

std::vector<std::string> split_bar(const std::string& s) {
  std::vector<std::string> out;
  size_t a = 0;
  while (true) {
    const size_t b = s.find('|', a);
    out.push_back(s.substr(a, b == std::string::npos ? std::string::npos : b - a));
    if (b == std::string::npos) break;
    a = b + 1;
  }
  return out;
}

}  // namespace

std::string url_decode(const std::string& s) {
  std::string o;
  for (size_t i = 0; i < s.size(); ++i) {
    if (s[i] == '%' && i + 2 < s.size()) {
      const std::string h = s.substr(i + 1, 2);
      char* end = nullptr;
      const long v = std::strtol(h.c_str(), &end, 16);
      if (end && *end == '\0') {
        o.push_back(static_cast<char>(v));
        i += 2;
        continue;
      }
    }
    o.push_back(s[i] == '+' ? ' ' : s[i]);
  }
  return o;
}

std::optional<RobotInfo> parse_discovery_reply(const std::string& packet, const std::string& nonce) {
  const auto f = split_bar(packet);
  if (f.size() != 6 || f[0] != "ARMPI_HERE/1" || f[1] != nonce || !is_hex32(f[2])) return std::nullopt;
  const auto port = parse_int(f[4]);
  const auto proto = parse_int(f[5]);
  if (!port || *port < 1 || *port > 65535 || !proto) return std::nullopt;
  RobotInfo r;
  r.device_id = to_lower(f[2]);
  r.name = url_decode(f[3]);
  r.control_port = static_cast<int>(*port);
  r.protocol = static_cast<int>(*proto);
  return r;
}

std::vector<RobotInfo> discover(double timeout_s, const std::vector<std::string>& targets_in, CommLog* log,
                                std::string* err, int port) {
  std::vector<RobotInfo> found;
  socket_t s = udp_socket(true, err);
  if (s == kInvalidSocket) return found;
  const std::vector<std::string> targets = targets_in.empty() ? broadcast_targets() : targets_in;
  const std::string nonce = random_hex(16);
  const std::string request = "ARMPI_DISCOVER/1|" + nonce;

  std::map<std::string, bool> seen;  // id|addr
  const double start = mono_now();
  const double end = start + timeout_s;
  double next_probe = 0.0;
  std::string last_send_err;
  while (mono_now() < end) {
    if (mono_now() >= next_probe) {
      next_probe = mono_now() + 0.5;
      for (const auto& t : targets) {
        std::string e;
        auto dst = resolve_ipv4(t, port, &e);
        if (!dst) {
          last_send_err = e;
          continue;
        }
        if (udp_sendto(s, request.data(), request.size(), *dst) < 0) {
          last_send_err = t + ": " + socket_error_text(last_socket_error());
          continue;
        }
        if (log) log->record(true, Channel::kDiscovery, addr_to_string(*dst), request);
      }
    }
    if (!wait_readable(s, 0.05)) continue;
    char buf[1024];
    sockaddr_in src{};
    const long n = udp_recvfrom(s, buf, sizeof(buf), &src);
    if (n <= 0) continue;
    const std::string pkt(buf, static_cast<size_t>(n));
    if (log) log->record(false, Channel::kDiscovery, addr_to_string(src), pkt);
    if (ntohs(src.sin_port) != port) continue;  // spec: replies come from port 5002
    auto info = parse_discovery_reply(pkt, nonce);
    if (!info) continue;
    info->address = ip_to_string(src.sin_addr.s_addr);
    info->rtt_ms = (mono_now() - start) * 1000.0;
    const std::string key = info->device_id + "|" + info->address;
    if (seen.count(key)) continue;
    seen[key] = true;
    found.push_back(*info);
  }
  close_socket(s);
  if (found.empty() && err && !last_send_err.empty()) *err = "송신 오류: " + last_send_err;
  return found;
}

}  // namespace ally::net
