// 공용 도구: 시계, 문자열, 화면 폭 계산, 사건 기록
#pragma once

#include <cstdint>
#include <cstdio>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace ally {

// 단조 시계(초)
double mono_now();
// 벽시계 "HH:MM:SS.mmm"
std::string wall_clock_ms();
// 벽시계 파일 이름용 "YYYYmmdd_HHMMSS"
std::string wall_stamp();

std::string strf(const char* fmt, ...)
#if defined(__GNUC__)
    __attribute__((format(printf, 1, 2)))
#endif
    ;

std::string trim(std::string_view s);
std::string to_lower(std::string_view s);
std::vector<std::string> split_ws(std::string_view s);
bool starts_with(std::string_view s, std::string_view prefix);
std::optional<double> parse_double(std::string_view s);
std::optional<long long> parse_int(std::string_view s);

// 화면에 보이는 칸 수(ANSI 색 코드는 0칸, 한글·한자는 2칸)
int display_width(std::string_view s);
// 보이는 칸 수가 width 가 되게 자르거나 공백을 채운다(ANSI 코드 보존, 끝에 색 초기화)
std::string fit(std::string_view s, int width);
// 바이트 문자열을 사람이 읽을 수 있게(제어 문자는 \xNN)
std::string printable(std::string_view s, size_t max_len = 400);
std::string hex_dump_line(std::string_view s, size_t max_bytes = 32);
// Cut to at most max_bytes without splitting a UTF-8 character; appends "…" when cut.
std::string utf8_truncate(std::string_view s, size_t max_bytes);

constexpr double kPi = 3.14159265358979323846;
inline double deg(double rad) { return rad * 180.0 / kPi; }
inline double rad(double deg_) { return deg_ * kPi / 180.0; }
inline double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

enum class Level { kInfo, kWarn, kError, kTx, kRx, kOk };

struct Event {
  std::string time;
  Level level;
  std::string text;
};

// 화면에 보여 줄 최근 사건(스레드 안전). 파일을 열면 같은 줄을 파일에도 쓴다.
class EventLog {
 public:
  void add(Level level, std::string text);
  void info(std::string t) { add(Level::kInfo, std::move(t)); }
  void warn(std::string t) { add(Level::kWarn, std::move(t)); }
  void error(std::string t) { add(Level::kError, std::move(t)); }
  void ok(std::string t) { add(Level::kOk, std::move(t)); }
  std::vector<Event> last(size_t n) const;
  uint64_t count() const;
  bool open_file(const std::string& path, std::string* err);
  // 콘솔 모드: 새 사건을 곧바로 표준 출력에도 쓴다
  void set_echo(bool on);
  ~EventLog();

 private:
  mutable std::mutex mu_;
  std::deque<Event> events_;
  uint64_t count_{0};
  std::FILE* file_{nullptr};
  bool echo_{false};
};

const char* level_tag(Level l);

}  // namespace ally
