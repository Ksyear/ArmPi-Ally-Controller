#include "net/websocket.hpp"

#include <random>

#include "common/util.hpp"
#include "net/crypto.hpp"
#include "net/http.hpp"
#include "net/sock.hpp"

namespace ally::net {

WebSocketClient::~WebSocketClient() { close(); }

bool WebSocketClient::is_open() const {
  std::lock_guard<std::mutex> lk(send_mu_);
  return fd_ != kInvalidSocket;
}

// 수신 버퍼(rbuf_ 등)는 읽는 스레드만 만진다. close() 는 소켓만 닫는다.
void WebSocketClient::close() {
  std::lock_guard<std::mutex> lk(send_mu_);
  close_socket(fd_);
}

bool WebSocketClient::connect(const std::string& host, int port, const std::string& path, double timeout_s,
                              std::string* err) {
  close();
  rbuf_.clear();
  frag_.clear();
  in_frag_ = false;
  socket_t fd = tcp_connect(host, port, timeout_s, err);
  if (fd == kInvalidSocket) return false;
  std::random_device rd;
  std::string raw;
  for (int i = 0; i < 16; ++i) raw.push_back(static_cast<char>(rd() & 0xFF));
  const std::string key = base64_encode(raw);
  const std::string req = "GET " + path + " HTTP/1.1\r\nHost: " + host + ":" + std::to_string(port) +
                          "\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: " + key +
                          "\r\nSec-WebSocket-Version: 13\r\nUser-Agent: armpi-ally/0.1\r\n\r\n";
  if (!send_all(fd, req.data(), req.size(), timeout_s, err)) {
    close_socket(fd);
    return false;
  }
  std::string buf;
  char tmp[4096];
  const double end = mono_now() + timeout_s;
  while (buf.find("\r\n\r\n") == std::string::npos) {
    const long n = recv_some(fd, tmp, sizeof(tmp), end - mono_now());
    if (n <= 0 || buf.size() > 65536) {
      if (err) *err = n == -2 ? "핸드셰이크 시간 초과" : "핸드셰이크 중 연결 닫힘";
      close_socket(fd);
      return false;
    }
    buf.append(tmp, static_cast<size_t>(n));
  }
  const size_t he = buf.find("\r\n\r\n");
  HttpResponse r;
  if (!parse_response_head(buf.substr(0, he + 4), &r) || r.status != 101) {
    if (err) *err = "WebSocket 업그레이드 거부(HTTP " + std::to_string(r.status) + ")";
    close_socket(fd);
    return false;
  }
  if (r.headers["sec-websocket-accept"] != websocket_accept(key)) {
    if (err) *err = "Sec-WebSocket-Accept 불일치";
    close_socket(fd);
    return false;
  }
  std::lock_guard<std::mutex> lk(send_mu_);
  fd_ = fd;
  rbuf_ = buf.substr(he + 4);
  return true;
}

std::string WebSocketClient::build_frame(int opcode, const std::string& payload, const unsigned char mask[4]) {
  std::string f;
  f.push_back(static_cast<char>(0x80 | (opcode & 0x0F)));
  const size_t n = payload.size();
  if (n < 126) {
    f.push_back(static_cast<char>(0x80 | n));
  } else if (n < 65536) {
    f.push_back(static_cast<char>(0x80 | 126));
    f.push_back(static_cast<char>((n >> 8) & 0xFF));
    f.push_back(static_cast<char>(n & 0xFF));
  } else {
    f.push_back(static_cast<char>(0x80 | 127));
    for (int i = 7; i >= 0; --i) f.push_back(static_cast<char>((static_cast<uint64_t>(n) >> (8 * i)) & 0xFF));
  }
  f.append(reinterpret_cast<const char*>(mask), 4);
  for (size_t i = 0; i < n; ++i) f.push_back(static_cast<char>(payload[i] ^ mask[i % 4]));
  return f;
}

int WebSocketClient::take_frame(std::string& buf, int* opcode, bool* fin, std::string* payload) {
  if (buf.size() < 2) return 0;
  const uint8_t b0 = static_cast<uint8_t>(buf[0]);
  const uint8_t b1 = static_cast<uint8_t>(buf[1]);
  *fin = (b0 & 0x80) != 0;
  *opcode = b0 & 0x0F;
  const bool masked = (b1 & 0x80) != 0;
  uint64_t len = b1 & 0x7F;
  size_t pos = 2;
  if (len == 126) {
    if (buf.size() < 4) return 0;
    len = (static_cast<uint64_t>(static_cast<uint8_t>(buf[2])) << 8) | static_cast<uint8_t>(buf[3]);
    pos = 4;
  } else if (len == 127) {
    if (buf.size() < 10) return 0;
    len = 0;
    for (int i = 0; i < 8; ++i) len = (len << 8) | static_cast<uint8_t>(buf[2 + static_cast<size_t>(i)]);
    pos = 10;
  }
  if (len > kMaxMessageBytes) return -1;
  unsigned char mask[4] = {0, 0, 0, 0};
  if (masked) {
    if (buf.size() < pos + 4) return 0;
    for (int i = 0; i < 4; ++i) mask[i] = static_cast<unsigned char>(buf[pos + static_cast<size_t>(i)]);
    pos += 4;
  }
  if (buf.size() < pos + len) return 0;
  payload->assign(buf, pos, static_cast<size_t>(len));
  if (masked)
    for (size_t i = 0; i < payload->size(); ++i) (*payload)[i] = static_cast<char>((*payload)[i] ^ mask[i % 4]);
  buf.erase(0, pos + static_cast<size_t>(len));
  return 1;
}

bool WebSocketClient::send_frame(int opcode, const std::string& payload, std::string* err) {
  static thread_local std::mt19937 rng{std::random_device{}()};
  unsigned char mask[4];
  for (auto& m : mask) m = static_cast<unsigned char>(rng() & 0xFF);
  const std::string f = build_frame(opcode, payload, mask);
  std::lock_guard<std::mutex> lk(send_mu_);
  if (fd_ == kInvalidSocket) {
    if (err) *err = "연결 안 됨";
    return false;
  }
  return send_all(fd_, f.data(), f.size(), 2.0, err);
}

bool WebSocketClient::send_text(const std::string& text, std::string* err) { return send_frame(0x1, text, err); }

int WebSocketClient::receive(std::string* msg, double timeout_s, std::string* err) {
  const double end = mono_now() + timeout_s;
  char tmp[16384];
  while (true) {
    int op = 0;
    bool fin = false;
    std::string payload;
    const int r = take_frame(rbuf_, &op, &fin, &payload);
    if (r < 0) {
      if (err) *err = "WebSocket 프레임 형식 오류";
      return -1;
    }
    if (r == 1) {
      if (op == 0x8) {
        if (err) *err = "상대가 연결을 닫음";
        send_frame(0x8, "", nullptr);
        return -1;
      }
      if (op == 0x9) {
        send_frame(0xA, payload, nullptr);
        continue;
      }
      if (op == 0xA) continue;
      if (op == 0x1 || op == 0x2) {
        if (fin) {
          *msg = std::move(payload);
          return 1;
        }
        in_frag_ = true;
        frag_ = std::move(payload);
        continue;
      }
      if (op == 0x0 && in_frag_) {
        if (frag_.size() + payload.size() > kMaxMessageBytes) {  // check before joining: never hold twice the cap
          if (err) *err = "WebSocket 메시지가 너무 큼";
          in_frag_ = false;
          frag_.clear();
          return -1;
        }
        frag_ += payload;
        if (fin) {
          in_frag_ = false;
          *msg = std::move(frag_);
          frag_.clear();
          return 1;
        }
        continue;
      }
      continue;
    }
    socket_t fd;
    {
      std::lock_guard<std::mutex> lk(send_mu_);
      fd = fd_;
    }
    if (fd == kInvalidSocket) return -1;
    const double left = end - mono_now();
    if (left <= 0) return 0;
    const long n = recv_some(fd, tmp, sizeof(tmp), left);
    if (n == -2) return 0;
    if (n <= 0) {
      if (err) *err = n == 0 ? "상대가 연결을 닫음" : "수신 오류";
      return -1;
    }
    rbuf_.append(tmp, static_cast<size_t>(n));
  }
}

}  // namespace ally::net
