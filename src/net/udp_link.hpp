// Protocol v3 UDP link (implements armpi_protocol::DatagramLink) on the portable socket layer.
// Every datagram goes into CommLog for the debug panel.
#pragma once

#include <memory>
#include <optional>
#include <string>

#include "armpi_protocol/datagram_link.hpp"
#include "net/comm_log.hpp"
#include "net/sock.hpp"

namespace ally::net {

class UdpLink : public armpi_protocol::DatagramLink {
 public:
  static std::unique_ptr<UdpLink> open(const std::string& host, int port, CommLog* log, std::string* err);
  ~UdpLink() override;
  UdpLink(const UdpLink&) = delete;
  UdpLink& operator=(const UdpLink&) = delete;

  bool send(const std::string& data, std::string* error) override;
  std::optional<std::string> receive(std::string* error) override;
  std::string peer() const override { return peer_; }
  std::string local() const { return local_; }

 private:
  UdpLink(socket_t s, std::string peer, std::string local, CommLog* log)
      : s_(s), peer_(std::move(peer)), local_(std::move(local)), log_(log) {}
  socket_t s_;
  std::string peer_;
  std::string local_;
  CommLog* log_;
};

}  // namespace ally::net
