// 아주 작은 HTTP/1.1 클라이언트: 주소 해석, GET(배너 확인), MJPEG(multipart/x-mixed-replace) 프레임 읽기
#pragma once

#include <atomic>
#include <functional>
#include <map>
#include <optional>
#include <string>

namespace ally::net {

struct Url {
  std::string host;
  int port{80};
  std::string path{"/"};  // 질의 문자열 포함
};
std::optional<Url> parse_url(const std::string& url);

struct HttpResponse {
  int status{0};
  std::string reason;
  std::map<std::string, std::string> headers;  // 이름은 소문자
  std::string body;
};

// 헤더 덩어리("HTTP/1.1 200 OK\r\n...\r\n\r\n" 앞부분) 해석
bool parse_response_head(const std::string& head, HttpResponse* out);

// 짧은 GET. 본문은 max_body 까지만 읽는다.
std::optional<HttpResponse> http_get(const std::string& host, int port, const std::string& path, double timeout_s,
                                     size_t max_body, std::string* err);

using FrameFn = std::function<bool(const std::string& content_type, std::string&& data)>;

// MJPEG 스트림 읽기. on_frame 이 false 를 돌려주거나 stop 이 켜지면 끝난다.
// 반환: 끝난 이유(정상 중지면 빈 문자열)
std::string read_mjpeg(const Url& url, const std::atomic<bool>& stop, double timeout_s, const FrameFn& on_frame);

}  // namespace ally::net
