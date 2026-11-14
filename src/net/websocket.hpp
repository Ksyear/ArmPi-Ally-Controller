// 최소 WebSocket 클라이언트(RFC 6455, ws:// 텍스트 메시지). rosbridge 9090 에 쓴다.
#pragma once

#include <cstdint>
#include <mutex>
#include <string>

#include "net/sock.hpp"

namespace ally::net {

// Largest frame or reassembled message we accept (rosbridge messages are a few KB).
constexpr uint64_t kMaxMessageBytes = 8ull * 1024 * 1024;

// Threading: one reader thread calls connect()/receive()/close(); send_text() may be called from any thread.
class WebSocketClient {
 public:
  ~WebSocketClient();
  bool connect(const std::string& host, int port, const std::string& path, double timeout_s, std::string* err);
  // 여러 스레드에서 불러도 된다
  bool send_text(const std::string& text, std::string* err);
  // 1 = 텍스트 메시지 받음, 0 = 시간 초과, -1 = 닫힘·오류
  int receive(std::string* msg, double timeout_s, std::string* err);
  void close();
  bool is_open() const;

  // 시험용: 마스킹한 클라이언트 프레임, 서버 프레임 해석
  static std::string build_frame(int opcode, const std::string& payload, const unsigned char mask[4]);
  // buf 앞에서 프레임 하나를 꺼낸다. 아직 덜 왔으면 0, 성공 1, 형식 오류 -1
  static int take_frame(std::string& buf, int* opcode, bool* fin, std::string* payload);

 private:
  bool send_frame(int opcode, const std::string& payload, std::string* err);
  mutable std::mutex send_mu_;
  socket_t fd_{kInvalidSocket};
  std::string rbuf_;
  std::string frag_;
  bool in_frag_{false};
};

}  // namespace ally::net
