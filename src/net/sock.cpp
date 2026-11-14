#include "net/sock.hpp"

#include <cstring>
#include <mutex>
#include <set>

#include "common/util.hpp"

#ifdef _WIN32
#include <iphlpapi.h>
#else
#include <fcntl.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <unistd.h>

#include <cerrno>
#endif

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

namespace ally::net {

bool init_sockets(std::string* err) {
#ifdef _WIN32
  static std::once_flag once;
  static int rc = 0;
  std::call_once(once, [] {
    WSADATA d;
    rc = ::WSAStartup(MAKEWORD(2, 2), &d);
  });
  if (rc != 0) {
    if (err) *err = "WSAStartup 실패: " + std::to_string(rc);
    return false;
  }
#else
  (void)err;
#endif
  return true;
}

int last_socket_error() {
#ifdef _WIN32
  return ::WSAGetLastError();
#else
  return errno;
#endif
}

bool would_block(int code) {
#ifdef _WIN32
  return code == WSAEWOULDBLOCK || code == WSAEINPROGRESS || code == WSAEINTR;
#else
  return code == EAGAIN || code == EWOULDBLOCK || code == EINPROGRESS || code == EINTR;
#endif
}

std::string socket_error_text(int code) {
#ifdef _WIN32
  wchar_t* buf = nullptr;
  const DWORD n = ::FormatMessageW(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
                                       FORMAT_MESSAGE_IGNORE_INSERTS,
                                   nullptr, static_cast<DWORD>(code), 0, reinterpret_cast<LPWSTR>(&buf), 0, nullptr);
  std::string out;
  if (n && buf) {
    const int len = ::WideCharToMultiByte(CP_UTF8, 0, buf, static_cast<int>(n), nullptr, 0, nullptr, nullptr);
    out.resize(static_cast<size_t>(len));
    ::WideCharToMultiByte(CP_UTF8, 0, buf, static_cast<int>(n), out.data(), len, nullptr, nullptr);
    ::LocalFree(buf);
  }
  out = trim(out);
  return out.empty() ? "WSA 오류 " + std::to_string(code) : out + " (" + std::to_string(code) + ")";
#else
  return std::strerror(code);
#endif
}

std::optional<sockaddr_in> resolve_ipv4(const std::string& host, int port, std::string* err) {
  init_sockets();
  if (port < 0 || port > 65535) {
    if (err) *err = "포트 범위 오류";
    return std::nullopt;
  }
  addrinfo hints{};
  hints.ai_family = AF_INET;
  addrinfo* res = nullptr;
  const int rc = ::getaddrinfo(host.c_str(), nullptr, &hints, &res);
  if (rc != 0 || !res) {
    if (err) *err = "주소를 찾지 못함: " + host;
    return std::nullopt;
  }
  sockaddr_in a{};
  std::memcpy(&a, res->ai_addr, sizeof(a));
  ::freeaddrinfo(res);
  a.sin_port = htons(static_cast<uint16_t>(port));
  return a;
}

std::string ip_to_string(uint32_t ip_net_order) {
  in_addr ia{};
  ia.s_addr = ip_net_order;
  char buf[INET_ADDRSTRLEN] = {0};
  ::inet_ntop(AF_INET, &ia, buf, sizeof(buf));
  return buf;
}

std::string addr_to_string(const sockaddr_in& a) {
  return ip_to_string(a.sin_addr.s_addr) + ":" + std::to_string(ntohs(a.sin_port));
}

bool is_ipv4_literal(const std::string& s) {
  in_addr ia{};
  return ::inet_pton(AF_INET, s.c_str(), &ia) == 1;
}

std::vector<std::string> broadcast_targets() {
  init_sockets();
  std::set<std::string> seen;
  std::vector<std::string> out;
  auto add = [&](const std::string& b) {
    if (!b.empty() && seen.insert(b).second) out.push_back(b);
  };
#ifdef _WIN32
  ULONG size = 16 * 1024;
  std::vector<unsigned char> buf(size);
  ULONG rc = ERROR_BUFFER_OVERFLOW;
  for (int tries = 0; tries < 3 && rc == ERROR_BUFFER_OVERFLOW; ++tries) {
    buf.resize(size);
    rc = ::GetAdaptersAddresses(AF_INET, GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST | GAA_FLAG_SKIP_DNS_SERVER,
                                nullptr, reinterpret_cast<IP_ADAPTER_ADDRESSES*>(buf.data()), &size);
  }
  if (rc == NO_ERROR) {
    for (auto* a = reinterpret_cast<IP_ADAPTER_ADDRESSES*>(buf.data()); a; a = a->Next) {
      if (a->OperStatus != IfOperStatusUp || a->IfType == IF_TYPE_SOFTWARE_LOOPBACK) continue;
      for (auto* u = a->FirstUnicastAddress; u; u = u->Next) {
        if (!u->Address.lpSockaddr || u->Address.lpSockaddr->sa_family != AF_INET) continue;
        const uint32_t ip = ntohl(reinterpret_cast<sockaddr_in*>(u->Address.lpSockaddr)->sin_addr.s_addr);
        const unsigned plen = u->OnLinkPrefixLength;
        if (plen == 0 || plen >= 31) continue;  // /32 (VPN) or /31: no broadcast
        if ((ip >> 16) == 0xA9FE) continue;     // 169.254.x.x link-local
        const uint32_t mask = plen >= 32 ? 0xFFFFFFFFu : ~((1u << (32 - plen)) - 1u);
        add(ip_to_string(htonl(ip | ~mask)));
      }
    }
  }
#else
  ifaddrs* ifa = nullptr;
  if (::getifaddrs(&ifa) == 0) {
    for (ifaddrs* p = ifa; p; p = p->ifa_next) {
      if (!p->ifa_addr || p->ifa_addr->sa_family != AF_INET) continue;
      if (!(p->ifa_flags & IFF_UP) || (p->ifa_flags & IFF_LOOPBACK) || !(p->ifa_flags & IFF_BROADCAST)) continue;
      if (!p->ifa_broadaddr) continue;
      add(ip_to_string(reinterpret_cast<sockaddr_in*>(p->ifa_broadaddr)->sin_addr.s_addr));
    }
    ::freeifaddrs(ifa);
  }
#endif
  add("255.255.255.255");
  return out;
}

void close_socket(socket_t& s) {
  if (s == kInvalidSocket) return;
#ifdef _WIN32
  ::closesocket(s);
#else
  ::close(s);
#endif
  s = kInvalidSocket;
}

bool set_nonblocking(socket_t s, bool on) {
#ifdef _WIN32
  u_long mode = on ? 1 : 0;
  return ::ioctlsocket(s, FIONBIO, &mode) == 0;
#else
  const int flags = ::fcntl(s, F_GETFL);
  return ::fcntl(s, F_SETFL, on ? (flags | O_NONBLOCK) : (flags & ~O_NONBLOCK)) == 0;
#endif
}

static bool wait_for(socket_t s, short events, double timeout_s) {
  const int ms = timeout_s <= 0 ? 0 : static_cast<int>(timeout_s * 1000.0 + 0.5);
#ifdef _WIN32
  WSAPOLLFD p{};
  p.fd = s;
  p.events = events;
  return ::WSAPoll(&p, 1, ms) > 0;
#else
  pollfd p{s, events, 0};
  int rc;
  do {
    rc = ::poll(&p, 1, ms);
  } while (rc < 0 && errno == EINTR);
  return rc > 0;
#endif
}

bool wait_readable(socket_t s, double timeout_s) { return wait_for(s, POLLIN, timeout_s); }
bool wait_writable(socket_t s, double timeout_s) { return wait_for(s, POLLOUT, timeout_s); }

socket_t tcp_connect(const std::string& host, int port, double timeout_s, std::string* err) {
  if (!init_sockets(err)) return kInvalidSocket;
  auto a = resolve_ipv4(host, port, err);
  if (!a) return kInvalidSocket;
  socket_t s = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (s == kInvalidSocket) {
    if (err) *err = "socket: " + socket_error_text(last_socket_error());
    return kInvalidSocket;
  }
#ifndef _WIN32
  ::fcntl(s, F_SETFD, FD_CLOEXEC);
#endif
#ifdef SO_NOSIGPIPE
  int one = 1;
  ::setsockopt(s, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one));
#endif
  int nodelay = 1;
  ::setsockopt(s, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&nodelay), sizeof(nodelay));
  set_nonblocking(s, true);
  if (::connect(s, reinterpret_cast<sockaddr*>(&*a), sizeof(*a)) != 0) {
    const int e = last_socket_error();
    if (!would_block(e)) {
      if (err) *err = "연결 실패: " + socket_error_text(e);
      close_socket(s);
      return kInvalidSocket;
    }
    if (!wait_writable(s, timeout_s)) {
      if (err) *err = "연결 시간 초과(" + host + ":" + std::to_string(port) + ")";
      close_socket(s);
      return kInvalidSocket;
    }
    int so_err = 0;
    socklen_t len = sizeof(so_err);
    ::getsockopt(s, SOL_SOCKET, SO_ERROR, reinterpret_cast<char*>(&so_err), &len);
    if (so_err != 0) {
      if (err) *err = "연결 실패: " + socket_error_text(so_err);
      close_socket(s);
      return kInvalidSocket;
    }
  }
  set_nonblocking(s, false);
  return s;
}

bool send_all(socket_t s, const void* data, size_t len, double timeout_s, std::string* err) {
  const char* p = static_cast<const char*>(data);
  const double end = mono_now() + timeout_s;
  while (len > 0) {
    if (!wait_writable(s, end - mono_now())) {
      if (err) *err = "송신 시간 초과";
      return false;
    }
    const auto n = ::send(s, p, static_cast<int>(len), MSG_NOSIGNAL);
    if (n < 0) {
      const int e = last_socket_error();
      if (would_block(e)) continue;
      if (err) *err = "송신 실패: " + socket_error_text(e);
      return false;
    }
    p += n;
    len -= static_cast<size_t>(n);
  }
  return true;
}

long recv_some(socket_t s, void* buf, size_t len, double timeout_s) {
  if (!wait_readable(s, timeout_s)) return -2;
  const auto n = ::recv(s, static_cast<char*>(buf), static_cast<int>(len), 0);
  return n < 0 ? -1 : static_cast<long>(n);
}

socket_t udp_socket(bool broadcast, std::string* err) {
  if (!init_sockets(err)) return kInvalidSocket;
  socket_t s = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (s == kInvalidSocket) {
    if (err) *err = "socket: " + socket_error_text(last_socket_error());
    return kInvalidSocket;
  }
#ifndef _WIN32
  ::fcntl(s, F_SETFD, FD_CLOEXEC);
#else
  // Windows reports ICMP port-unreachable as WSAECONNRESET on the next recv; ignore it on UDP.
  BOOL no = FALSE;
  DWORD ret = 0;
  ::WSAIoctl(s, _WSAIOW(IOC_VENDOR, 12) /* SIO_UDP_CONNRESET */, &no, sizeof(no), nullptr, 0, &ret, nullptr, nullptr);
#endif
  if (broadcast) {
    int one = 1;
    ::setsockopt(s, SOL_SOCKET, SO_BROADCAST, reinterpret_cast<const char*>(&one), sizeof(one));
  }
  set_nonblocking(s, true);
  return s;
}

bool udp_connect(socket_t s, const sockaddr_in& peer, std::string* err) {
  if (::connect(s, reinterpret_cast<const sockaddr*>(&peer), sizeof(peer)) != 0) {
    if (err) *err = "connect: " + socket_error_text(last_socket_error());
    return false;
  }
  return true;
}

long udp_send(socket_t s, const void* data, size_t len) {
  const auto n = ::send(s, static_cast<const char*>(data), static_cast<int>(len), 0);
  return n < 0 ? -1 : static_cast<long>(n);
}

long udp_recv(socket_t s, void* buf, size_t len) {
  const auto n = ::recv(s, static_cast<char*>(buf), static_cast<int>(len), 0);
  if (n >= 0) return static_cast<long>(n);
  return would_block(last_socket_error()) ? -2 : -1;
}

long udp_sendto(socket_t s, const void* data, size_t len, const sockaddr_in& to) {
  const auto n = ::sendto(s, static_cast<const char*>(data), static_cast<int>(len), 0,
                          reinterpret_cast<const sockaddr*>(&to), sizeof(to));
  return n < 0 ? -1 : static_cast<long>(n);
}

long udp_recvfrom(socket_t s, void* buf, size_t len, sockaddr_in* from) {
  socklen_t fl = sizeof(*from);
  const auto n = ::recvfrom(s, static_cast<char*>(buf), static_cast<int>(len), 0, reinterpret_cast<sockaddr*>(from), &fl);
  if (n >= 0) return static_cast<long>(n);
  return would_block(last_socket_error()) ? -2 : -1;
}

sockaddr_in local_address(socket_t s) {
  sockaddr_in a{};
  socklen_t len = sizeof(a);
  ::getsockname(s, reinterpret_cast<sockaddr*>(&a), &len);
  return a;
}

}  // namespace ally::net
