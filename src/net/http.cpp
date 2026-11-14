#include "net/http.hpp"

#include "common/util.hpp"
#include "net/sock.hpp"

namespace ally::net {

std::optional<Url> parse_url(const std::string& url) {
  std::string rest = url;
  if (starts_with(rest, "http://")) rest = rest.substr(7);
  else if (rest.find("://") != std::string::npos) return std::nullopt;  // https 등은 지원 안 함
  Url u;
  const size_t slash = rest.find('/');
  std::string hostport = slash == std::string::npos ? rest : rest.substr(0, slash);
  u.path = slash == std::string::npos ? "/" : rest.substr(slash);
  const size_t colon = hostport.rfind(':');
  if (colon != std::string::npos) {
    const auto p = parse_int(hostport.substr(colon + 1));
    if (!p || *p < 1 || *p > 65535) return std::nullopt;
    u.port = static_cast<int>(*p);
    hostport = hostport.substr(0, colon);
  }
  if (hostport.empty()) return std::nullopt;
  u.host = hostport;
  return u;
}

bool parse_response_head(const std::string& head, HttpResponse* out) {
  size_t eol = head.find("\r\n");
  const std::string status_line = head.substr(0, eol);
  const auto parts = split_ws(status_line);
  if (parts.size() < 2 || !starts_with(parts[0], "HTTP/")) return false;
  const auto code = parse_int(parts[1]);
  if (!code) return false;
  out->status = static_cast<int>(*code);
  const size_t rs = status_line.find(parts[1]) + parts[1].size();
  out->reason = trim(status_line.substr(rs));
  size_t pos = eol == std::string::npos ? head.size() : eol + 2;
  while (pos < head.size()) {
    size_t e = head.find("\r\n", pos);
    if (e == std::string::npos) e = head.size();
    const std::string line = head.substr(pos, e - pos);
    pos = e + 2;
    if (line.empty()) break;
    const size_t c = line.find(':');
    if (c == std::string::npos) continue;
    out->headers[to_lower(trim(line.substr(0, c)))] = trim(line.substr(c + 1));
  }
  return true;
}

namespace {

std::string request(const std::string& host, int port, const std::string& path) {
  return "GET " + path + " HTTP/1.1\r\nHost: " + host + ":" + std::to_string(port) +
         "\r\nUser-Agent: armpi-ally/0.1\r\nAccept: */*\r\nConnection: close\r\n\r\n";
}

// buf 에 head 끝(\r\n\r\n)이 들어올 때까지 읽는다
bool read_head(socket_t fd, std::string& buf, double end, std::string* err) {
  char tmp[4096];
  while (buf.find("\r\n\r\n") == std::string::npos) {
    if (buf.size() > 65536) {
      if (err) *err = "헤더가 너무 큼";
      return false;
    }
    const long n = recv_some(fd, tmp, sizeof(tmp), end - mono_now());
    if (n == -2) {
      if (err) *err = "응답 시간 초과";
      return false;
    }
    if (n <= 0) {
      if (err) *err = "응답 전에 연결이 닫힘";
      return false;
    }
    buf.append(tmp, static_cast<size_t>(n));
  }
  return true;
}

}  // namespace

std::optional<HttpResponse> http_get(const std::string& host, int port, const std::string& path, double timeout_s,
                                     size_t max_body, std::string* err) {
  socket_t fd = tcp_connect(host, port, timeout_s, err);
  if (fd == kInvalidSocket) return std::nullopt;
  const double end = mono_now() + timeout_s;
  const std::string req = request(host, port, path);
  if (!send_all(fd, req.data(), req.size(), timeout_s, err)) {
    close_socket(fd);
    return std::nullopt;
  }
  std::string buf;
  if (!read_head(fd, buf, end, err)) {
    close_socket(fd);
    return std::nullopt;
  }
  const size_t he = buf.find("\r\n\r\n");
  HttpResponse r;
  if (!parse_response_head(buf.substr(0, he + 4), &r)) {
    if (err) *err = "HTTP 응답 형식 오류";
    close_socket(fd);
    return std::nullopt;
  }
  r.body = buf.substr(he + 4);
  size_t want = max_body;
  if (auto it = r.headers.find("content-length"); it != r.headers.end()) {
    if (auto cl = parse_int(it->second); cl && *cl >= 0) want = std::min(want, static_cast<size_t>(*cl));
  }
  char tmp[4096];
  while (r.body.size() < want && mono_now() < end) {
    const long n = recv_some(fd, tmp, sizeof(tmp), end - mono_now());
    if (n <= 0) break;
    r.body.append(tmp, static_cast<size_t>(n));
  }
  if (r.body.size() > max_body) r.body.resize(max_body);
  close_socket(fd);
  return r;
}

std::string read_mjpeg(const Url& url, const std::atomic<bool>& stop, double timeout_s, const FrameFn& on_frame) {
  std::string err;
  socket_t fd = tcp_connect(url.host, url.port, timeout_s, &err);
  if (fd == kInvalidSocket) return err;
  const std::string req = request(url.host, url.port, url.path);
  if (!send_all(fd, req.data(), req.size(), timeout_s, &err)) {
    close_socket(fd);
    return err;
  }
  std::string buf;
  if (!read_head(fd, buf, mono_now() + timeout_s, &err)) {
    close_socket(fd);
    return err;
  }
  const size_t he = buf.find("\r\n\r\n");
  HttpResponse r;
  if (!parse_response_head(buf.substr(0, he + 4), &r) || r.status != 200) {
    close_socket(fd);
    return "카메라 HTTP " + std::to_string(r.status) + " " + r.reason;
  }
  const std::string ctype = r.headers["content-type"];
  const size_t bpos = to_lower(ctype).find("boundary=");
  if (bpos == std::string::npos) {
    close_socket(fd);
    return "multipart 경계 없음(Content-Type: " + ctype + ")";
  }
  std::string boundary = trim(ctype.substr(bpos + 9));
  if (const size_t semi = boundary.find(';'); semi != std::string::npos) boundary = boundary.substr(0, semi);
  if (boundary.size() >= 2 && boundary.front() == '"') boundary = boundary.substr(1, boundary.size() - 2);
  if (starts_with(boundary, "--")) boundary = boundary.substr(2);
  if (boundary.empty() || boundary.size() > 70) {  // RFC 2046: 1..70 characters
    close_socket(fd);
    return "multipart 경계 형식 오류";
  }
  const std::string marker = "--" + boundary;
  constexpr size_t kMaxPartHeader = 64 * 1024;
  constexpr size_t kMaxPart = 16u * 1024 * 1024;
  constexpr double kIdleTimeoutS = 5.0;
  buf.erase(0, he + 4);

  char tmp[16384];
  double last_data = mono_now();
  bool idle = false;
  auto fill = [&](size_t want) -> bool {
    while (buf.size() < want) {
      if (stop) return false;
      const long n = recv_some(fd, tmp, sizeof(tmp), 0.25);
      if (n == -2) {
        if (mono_now() - last_data > kIdleTimeoutS) {  // connected but silent camera
          idle = true;
          return false;
        }
        continue;
      }
      if (n <= 0) return false;
      last_data = mono_now();
      buf.append(tmp, static_cast<size_t>(n));
    }
    return true;
  };
  auto lost = [&]() -> std::string { return stop ? "" : (idle ? "영상 유휴 시간 초과" : "카메라 연결 끊김"); };
  std::string why;
  while (!stop) {
    // 다음 경계 찾기
    size_t m = buf.find(marker);
    while (m == std::string::npos) {
      if (buf.size() > 8 * 1024 * 1024) buf.erase(0, buf.size() - marker.size());
      const size_t before = buf.size();
      if (!fill(before + 1)) {
        why = lost();
        close_socket(fd);
        return why;
      }
      m = buf.find(marker);
    }
    buf.erase(0, m + marker.size());
    // 부분 헤더
    size_t ph = buf.find("\r\n\r\n");
    while (ph == std::string::npos) {
      if (buf.size() > kMaxPartHeader) {
        close_socket(fd);
        return "영상 파트 헤더가 너무 큼";
      }
      if (!fill(buf.size() + 1)) {
        close_socket(fd);
        return lost();
      }
      ph = buf.find("\r\n\r\n");
    }
    HttpResponse part;
    part.status = 200;
    // 부분 헤더는 상태 줄이 없으므로 가짜 상태 줄을 붙여 같은 해석기를 쓴다
    parse_response_head("HTTP/1.1 200 OK\r\n" + buf.substr(0, ph + 4), &part);
    buf.erase(0, ph + 4);
    std::string data;
    if (auto it = part.headers.find("content-length"); it != part.headers.end()) {
      const auto cl = parse_int(it->second);
      if (!cl || *cl < 0 || static_cast<size_t>(*cl) > kMaxPart) {
        close_socket(fd);
        return "프레임 길이 오류";
      }
      if (!fill(static_cast<size_t>(*cl))) {
        close_socket(fd);
        return lost();
      }
      data = buf.substr(0, static_cast<size_t>(*cl));
      buf.erase(0, static_cast<size_t>(*cl));
    } else {
      size_t nm = buf.find("\r\n" + marker);
      while (nm == std::string::npos) {
        if (buf.size() > kMaxPart) {
          close_socket(fd);
          return "프레임 길이 오류";
        }
        if (!fill(buf.size() + 1)) {
          close_socket(fd);
          return lost();
        }
        nm = buf.find("\r\n" + marker);
      }
      data = buf.substr(0, nm);
      buf.erase(0, nm);
    }
    if (!on_frame(part.headers["content-type"], std::move(data))) break;
  }
  close_socket(fd);
  return "";
}

}  // namespace ally::net
