#include "net/udp_link.hpp"

namespace ally::net {

std::unique_ptr<UdpLink> UdpLink::open(const std::string& host, int port, CommLog* log, std::string* err) {
  if (port < 1 || port > 65535) {
    if (err) *err = "포트 범위 오류";
    return nullptr;
  }
  auto peer = resolve_ipv4(host, port, err);
  if (!peer) return nullptr;
  socket_t s = udp_socket(false, err);
  if (s == kInvalidSocket) return nullptr;
  // Connected UDP: the OS only hands us datagrams carrying the robot's address/port (spec section 1).
  // This is a filter, not authentication: Protocol v3 has none, and source addresses can be spoofed on the LAN.
  if (!udp_connect(s, *peer, err)) {
    close_socket(s);
    return nullptr;
  }
  return std::unique_ptr<UdpLink>(new UdpLink(s, addr_to_string(*peer), addr_to_string(local_address(s)), log));
}

UdpLink::~UdpLink() { close_socket(s_); }

bool UdpLink::send(const std::string& data, std::string* error) {
  const long n = udp_send(s_, data.data(), data.size());
  if (n == static_cast<long>(data.size())) {
    if (log_) log_->record(true, Channel::kControl, peer_, data);
    return true;
  }
  if (error) *error = n < 0 ? socket_error_text(last_socket_error()) : "일부만 송신";
  return false;
}

std::optional<std::string> UdpLink::receive(std::string* error) {
  char buf[4096];
  const long n = udp_recv(s_, buf, sizeof(buf));
  if (n >= 0) {
    std::string d(buf, static_cast<size_t>(n));
    if (log_) log_->record(false, Channel::kControl, peer_, d);
    return d;
  }
  if (n == -1 && error) *error = socket_error_text(last_socket_error());
  return std::nullopt;
}

}  // namespace ally::net
