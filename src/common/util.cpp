#include "common/util.hpp"

#include <cerrno>
#include <chrono>
#include <cstdarg>
#include <cstdlib>
#include <cstring>
#include <ctime>

namespace ally {

double mono_now() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

static std::tm local_tm(std::time_t t) {
  std::tm tm{};
#ifdef _WIN32
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm);
#endif
  return tm;
}

std::string wall_clock_ms() {
  const auto now = std::chrono::system_clock::now();
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;
  const std::tm tm = local_tm(std::chrono::system_clock::to_time_t(now));
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%02d:%02d:%02d.%03d", tm.tm_hour, tm.tm_min, tm.tm_sec, static_cast<int>(ms));
  return buf;
}

std::string wall_stamp() {
  const std::tm tm = local_tm(std::time(nullptr));
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
  return buf;
}

std::string strf(const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  va_list ap2;
  va_copy(ap2, ap);
  const int n = std::vsnprintf(nullptr, 0, fmt, ap);
  va_end(ap);
  std::string out;
  if (n > 0) {
    out.resize(static_cast<size_t>(n) + 1);
    std::vsnprintf(out.data(), out.size(), fmt, ap2);
    out.resize(static_cast<size_t>(n));
  }
  va_end(ap2);
  return out;
}

std::string trim(std::string_view s) {
  size_t a = 0, b = s.size();
  while (a < b && (s[a] == ' ' || s[a] == '\t' || s[a] == '\r' || s[a] == '\n')) ++a;
  while (b > a && (s[b - 1] == ' ' || s[b - 1] == '\t' || s[b - 1] == '\r' || s[b - 1] == '\n')) --b;
  return std::string(s.substr(a, b - a));
}

std::string to_lower(std::string_view s) {
  std::string o(s);
  for (auto& c : o) {
    if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
  }
  return o;
}

std::vector<std::string> split_ws(std::string_view s) {
  std::vector<std::string> out;
  size_t i = 0;
  while (i < s.size()) {
    while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) ++i;
    if (i >= s.size()) break;
    size_t j = i;
    while (j < s.size() && s[j] != ' ' && s[j] != '\t') ++j;
    out.emplace_back(s.substr(i, j - i));
    i = j;
  }
  return out;
}

bool starts_with(std::string_view s, std::string_view p) { return s.size() >= p.size() && s.substr(0, p.size()) == p; }

std::optional<double> parse_double(std::string_view s) {
  const std::string t(trim(s));
  if (t.empty()) return std::nullopt;
  char* end = nullptr;
  errno = 0;
  const double v = std::strtod(t.c_str(), &end);
  if (errno != 0 || end == t.c_str() || *end != '\0' || v != v) return std::nullopt;
  return v;
}

std::optional<long long> parse_int(std::string_view s) {
  const std::string t(trim(s));
  if (t.empty()) return std::nullopt;
  char* end = nullptr;
  errno = 0;
  const long long v = std::strtoll(t.c_str(), &end, 10);
  if (errno != 0 || end == t.c_str() || *end != '\0') return std::nullopt;
  return v;
}

// UTF-8 한 글자를 읽는다. 잘못된 바이트는 1바이트 1글자로 본다.
static uint32_t next_cp(std::string_view s, size_t& i) {
  const unsigned char c = static_cast<unsigned char>(s[i]);
  int len = 1;
  uint32_t cp = c;
  if (c >= 0xF0 && c < 0xF8) { len = 4; cp = c & 0x07; }
  else if (c >= 0xE0) { len = 3; cp = c & 0x0F; }
  else if (c >= 0xC0) { len = 2; cp = c & 0x1F; }
  if (len > 1) {
    if (i + static_cast<size_t>(len) > s.size()) { ++i; return c; }
    for (int k = 1; k < len; ++k) {
      const unsigned char cc = static_cast<unsigned char>(s[i + static_cast<size_t>(k)]);
      if ((cc & 0xC0) != 0x80) { ++i; return c; }
      cp = (cp << 6) | (cc & 0x3F);
    }
  }
  i += static_cast<size_t>(len);
  return cp;
}

static int cp_width(uint32_t cp) {
  if (cp == 0) return 0;
  if (cp < 32 || (cp >= 0x7F && cp < 0xA0)) return 0;
  if (cp >= 0x0300 && cp <= 0x036F) return 0;  // 결합 부호
  if (cp == 0x200B || cp == 0x200D || (cp >= 0xFE00 && cp <= 0xFE0F)) return 0;
  if ((cp >= 0x1100 && cp <= 0x115F) || (cp >= 0x2E80 && cp <= 0x303E) || (cp >= 0x3041 && cp <= 0x33FF) ||
      (cp >= 0x3400 && cp <= 0x4DBF) || (cp >= 0x4E00 && cp <= 0x9FFF) || (cp >= 0xA960 && cp <= 0xA97F) ||
      (cp >= 0xAC00 && cp <= 0xD7A3) || (cp >= 0xF900 && cp <= 0xFAFF) || (cp >= 0xFE30 && cp <= 0xFE4F) ||
      (cp >= 0xFF00 && cp <= 0xFF60) || (cp >= 0xFFE0 && cp <= 0xFFE6) || (cp >= 0x1F300 && cp <= 0x1F64F) ||
      (cp >= 0x1F900 && cp <= 0x1F9FF) || (cp >= 0x20000 && cp <= 0x3FFFD) || cp == 0x3131 || cp == 0x318E ||
      (cp >= 0x3130 && cp <= 0x318F))
    return 2;
  return 1;
}

// ESC [ ... 최종바이트, ESC ] ... BEL/ST 를 건너뛴다
static bool skip_ansi(std::string_view s, size_t& i) {
  if (s[i] != '\x1b' || i + 1 >= s.size()) return false;
  if (s[i + 1] == '[') {
    size_t j = i + 2;
    while (j < s.size() && !(s[j] >= 0x40 && s[j] <= 0x7E)) ++j;
    i = j < s.size() ? j + 1 : s.size();
    return true;
  }
  if (s[i + 1] == ']') {
    size_t j = i + 2;
    while (j < s.size() && s[j] != '\x07' && !(s[j] == '\x1b' && j + 1 < s.size() && s[j + 1] == '\\')) ++j;
    i = j < s.size() ? (s[j] == '\x07' ? j + 1 : j + 2) : s.size();
    return true;
  }
  i += 2;
  return true;
}

int display_width(std::string_view s) {
  int w = 0;
  size_t i = 0;
  while (i < s.size()) {
    if (s[i] == '\x1b') {
      skip_ansi(s, i);
      continue;
    }
    w += cp_width(next_cp(s, i));
  }
  return w;
}

std::string fit(std::string_view s, int width) {
  std::string out;
  int w = 0;
  size_t i = 0;
  bool had_ansi = false;
  while (i < s.size()) {
    if (s[i] == '\x1b') {
      const size_t a = i;
      skip_ansi(s, i);
      out.append(s.substr(a, i - a));
      had_ansi = true;
      continue;
    }
    const size_t a = i;
    const int cw = cp_width(next_cp(s, i));
    if (w + cw > width) break;
    out.append(s.substr(a, i - a));
    w += cw;
  }
  if (had_ansi) out += "\x1b[0m";
  if (w < width) out.append(static_cast<size_t>(width - w), ' ');
  return out;
}

std::string printable(std::string_view s, size_t max_len) {
  std::string out;
  for (size_t i = 0; i < s.size() && out.size() < max_len; ++i) {
    const unsigned char c = static_cast<unsigned char>(s[i]);
    if (c >= 32 && c != 127) {
      out.push_back(static_cast<char>(c));
    } else {
      char b[8];
      std::snprintf(b, sizeof(b), "\\x%02x", c);
      out += b;
    }
  }
  if (out.size() >= max_len) out += "…";
  return out;
}

std::string hex_dump_line(std::string_view s, size_t max_bytes) {
  std::string out;
  for (size_t i = 0; i < s.size() && i < max_bytes; ++i) {
    char b[4];
    std::snprintf(b, sizeof(b), "%02x", static_cast<unsigned char>(s[i]));
    out += b;
    if (i + 1 < s.size() && i + 1 < max_bytes) out += ' ';
  }
  if (s.size() > max_bytes) out += " …";
  return out;
}

std::string utf8_truncate(std::string_view s, size_t max_bytes) {
  if (s.size() <= max_bytes) return std::string(s);
  size_t n = max_bytes;
  while (n > 0 && (static_cast<unsigned char>(s[n]) & 0xC0) == 0x80) --n;  // back up to a character start
  return std::string(s.substr(0, n)) + "…";
}

const char* level_tag(Level l) {
  switch (l) {
    case Level::kInfo: return "[*]";
    case Level::kWarn: return "[!]";
    case Level::kError: return "[-]";
    case Level::kTx: return "[>]";
    case Level::kRx: return "[<]";
    case Level::kOk: return "[+]";
  }
  return "[?]";
}

void EventLog::add(Level level, std::string text) {
  // Messages can carry robot-supplied strings: bound the length and neutralize control characters.
  std::string clean;
  clean.reserve(std::min<size_t>(text.size(), 600));
  for (char c : utf8_truncate(text, 512)) clean.push_back((static_cast<unsigned char>(c) < 32 || c == 127) ? ' ' : c);
  std::lock_guard<std::mutex> lk(mu_);
  Event e{wall_clock_ms(), level, std::move(clean)};
  if (file_) {
    std::fprintf(file_, "%s %s %s\n", e.time.c_str(), level_tag(level), e.text.c_str());
    std::fflush(file_);
  }
  if (echo_) {
    const char* color = level == Level::kError ? "\x1b[31m" : level == Level::kWarn ? "\x1b[33m"
                        : level == Level::kOk ? "\x1b[32m" : "\x1b[34m";
    std::printf("%s%s\x1b[0m %s\n", color, level_tag(level), e.text.c_str());
    std::fflush(stdout);
  }
  events_.push_back(std::move(e));
  ++count_;
  while (events_.size() > 500) events_.pop_front();
}

std::vector<Event> EventLog::last(size_t n) const {
  std::lock_guard<std::mutex> lk(mu_);
  const size_t k = std::min(n, events_.size());
  return std::vector<Event>(events_.end() - static_cast<long>(k), events_.end());
}

uint64_t EventLog::count() const {
  std::lock_guard<std::mutex> lk(mu_);
  return count_;
}

bool EventLog::open_file(const std::string& path, std::string* err) {
  std::lock_guard<std::mutex> lk(mu_);
  if (file_) std::fclose(file_);
  file_ = std::fopen(path.c_str(), "a");
  if (!file_) {
    if (err) *err = std::strerror(errno);
    return false;
  }
  return true;
}

void EventLog::set_echo(bool on) {
  std::lock_guard<std::mutex> lk(mu_);
  echo_ = on;
}

EventLog::~EventLog() {
  if (file_) std::fclose(file_);
}

}  // namespace ally
