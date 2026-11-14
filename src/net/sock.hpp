// Platform socket layer: Winsock2 on Windows, BSD sockets elsewhere.
// Everything else in src/net uses only this header for sockets, so the network code
// builds and is unit-tested on macOS/Linux and ships on Windows (ROG Ally).
#pragma once

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#endif

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace ally::net {

#ifdef _WIN32
using socket_t = SOCKET;
inline constexpr socket_t kInvalidSocket = INVALID_SOCKET;
#else
using socket_t = int;
inline constexpr socket_t kInvalidSocket = -1;
#endif

// WSAStartup on Windows (idempotent, thread safe). No-op elsewhere.
bool init_sockets(std::string* err = nullptr);

int last_socket_error();
std::string socket_error_text(int code);  // UTF-8
bool would_block(int code);

std::optional<sockaddr_in> resolve_ipv4(const std::string& host, int port, std::string* err);
std::string ip_to_string(uint32_t ip_net_order);
std::string addr_to_string(const sockaddr_in& a);
bool is_ipv4_literal(const std::string& s);

// Directed broadcast addresses of every up, non-loopback IPv4 interface + 255.255.255.255.
std::vector<std::string> broadcast_targets();

void close_socket(socket_t& s);
bool set_nonblocking(socket_t s, bool on);
// true: readable (or error/hangup pending) within timeout
bool wait_readable(socket_t s, double timeout_s);
bool wait_writable(socket_t s, double timeout_s);

// TCP helpers (blocking socket, explicit timeouts)
socket_t tcp_connect(const std::string& host, int port, double timeout_s, std::string* err);
bool send_all(socket_t s, const void* data, size_t len, double timeout_s, std::string* err);
// > 0 bytes, 0 closed by peer, -1 error, -2 timeout
long recv_some(socket_t s, void* buf, size_t len, double timeout_s);

// UDP helpers
socket_t udp_socket(bool broadcast, std::string* err);
bool udp_connect(socket_t s, const sockaddr_in& peer, std::string* err);
long udp_send(socket_t s, const void* data, size_t len);  // connected socket
long udp_recv(socket_t s, void* buf, size_t len);         // non-blocking connected socket, -2 = nothing
long udp_sendto(socket_t s, const void* data, size_t len, const sockaddr_in& to);
long udp_recvfrom(socket_t s, void* buf, size_t len, sockaddr_in* from);  // -2 = nothing
sockaddr_in local_address(socket_t s);

}  // namespace ally::net
