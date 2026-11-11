/// @file
/// Vrst_session_log and Vrst_log_record: JSON Lines session files.
/// Declarations and documentation are in session_log.h.
#include "session_log.h"

#include <algorithm>
#include <array>
#include <ctime>
#include <cstdio>
#include <stdexcept>
#include <system_error>

namespace nav {

namespace {

/// Prefix of session file names; files without it are never touched.
constexpr std::string_view kPrefix = "session-";
/// Extension of session files.
constexpr std::string_view kExtension = ".jsonl";

/// Whether a directory entry is a session file.
/// @param p Path.
/// @return true for `session-*.jsonl`.
bool is_session_file(const std::filesystem::path& p) {
  const std::string name = p.filename().string();
  return name.starts_with(kPrefix) && name.ends_with(kExtension);
}

/// A number with a fixed number of decimals. snprintf, not std::format: Apple's libc++
/// formats floating point with std::format only from iOS 16.3, and the app runs on 15.
/// @param v Value.
/// @param decimals Digits after the point.
/// @return Text.
std::string fixed(Vrst_float64 v, int decimals) {
  std::array<char, 64> buf{};
  const int n = std::snprintf(buf.data(), buf.size(), "%.*f", std::max(decimals, 0), v);
  return {buf.data(), static_cast<Vrst_size>(std::clamp(n, 0, static_cast<int>(buf.size()) - 1))};
}

/// Local time as `YYYYMMDD-HHMMSS`; names sort in time order.
/// @return Text.
std::string stamp_now() {
  const std::time_t now = std::time(nullptr);
  std::tm local{};
  localtime_r(&now, &local);  // C API
  std::array<char, 32> buf{};
  const Vrst_size n = std::strftime(buf.data(), buf.size(), "%Y%m%d-%H%M%S", &local);
  return {buf.data(), n};
}

}  // namespace

Vrst_log_record::Vrst_log_record(Vrst_session_log& log, std::string_view type, Vrst_float64 t) : m_log(log) {
  m_line.reserve(256);
  m_line += "{\"type\":\"";
  m_line += type;
  m_line += "\",\"t\":";
  m_line += fixed(t, 4);
}

Vrst_log_record::~Vrst_log_record() {
  m_line += '}';
  m_log.append(m_line);
}

void Vrst_log_record::key(std::string_view key) {
  m_line += ",\"";
  m_line += key;
  m_line += "\":";
}

Vrst_log_record& Vrst_log_record::num(std::string_view key, Vrst_float64 v, int decimals) {
  this->key(key);
  m_line += fixed(v, decimals);
  return *this;
}

Vrst_log_record& Vrst_log_record::count(std::string_view key, Vrst_uint64 v) {
  this->key(key);
  m_line += std::to_string(v);
  return *this;
}

Vrst_log_record& Vrst_log_record::text(std::string_view key, std::string_view v) {
  this->key(key);
  m_line += '"';
  for (const char c : v) {
    if (c == '"' || c == '\\') {
      m_line += '\\';
      m_line += c;
    } else if (static_cast<unsigned char>(c) < 0x20) {
      std::array<char, 8> esc{};
      std::snprintf(esc.data(), esc.size(), "\\u%04x", static_cast<unsigned>(c));
      m_line += esc.data();
    } else {
      m_line += c;
    }
  }
  m_line += '"';
  return *this;
}

Vrst_log_record& Vrst_log_record::flag(std::string_view key, bool v) {
  this->key(key);
  m_line += v ? "true" : "false";
  return *this;
}

Vrst_session_log::~Vrst_session_log() {
  flush();
}

void Vrst_session_log::open(const std::filesystem::path& dir, Vrst_size keep) {
  flush();
  m_file.close();
  m_dir = dir;
  std::filesystem::create_directories(dir);
  m_path = dir / (std::string(kPrefix) + stamp_now() + std::string(kExtension));
  m_file.open(m_path, std::ios::app);
  if (!m_file) throw std::runtime_error("cannot create " + m_path.string());
  std::vector<std::filesystem::path> all = files();
  const Vrst_size drop = all.size() > keep ? all.size() - keep : 0;
  std::error_code ec;  // a file that cannot be deleted is left; logging goes on
  for (Vrst_size i = 0; i < drop; ++i)
    if (all[i] != m_path) std::filesystem::remove(all[i], ec);
}

void Vrst_session_log::append(std::string_view line) {
  if (m_tail.size() == kTailLines) m_tail.pop_front();
  m_tail.emplace_back(line);
  ++m_appended;
  if (!m_file.is_open()) return;
  m_buffer += line;
  m_buffer += '\n';
}

void Vrst_session_log::flush_if_due(Vrst_float64 t) {
  if (t - m_flushed_t < kFlushSeconds) return;
  m_flushed_t = t;
  flush();
}

void Vrst_session_log::flush() {
  if (!m_file.is_open() || m_buffer.empty()) return;
  m_file.write(m_buffer.data(), static_cast<std::streamsize>(m_buffer.size()));
  m_file.flush();
  m_buffer.clear();
}

std::vector<std::filesystem::path> Vrst_session_log::files() const {
  std::vector<std::filesystem::path> out;
  std::error_code ec;
  if (m_dir.empty() || !std::filesystem::is_directory(m_dir, ec)) return out;
  for (const std::filesystem::directory_entry& e : std::filesystem::directory_iterator(m_dir, ec))
    if (e.is_regular_file(ec) && is_session_file(e.path())) out.push_back(e.path());
  std::ranges::sort(out);
  return out;
}

Vrst_uint64 Vrst_session_log::total_bytes() const {
  Vrst_uint64 sum = m_buffer.size();
  std::error_code ec;
  for (const std::filesystem::path& p : files()) {
    const std::uintmax_t n = std::filesystem::file_size(p, ec);
    if (!ec) sum += n;
  }
  return sum;
}

void Vrst_session_log::delete_old() {
  std::error_code ec;
  for (const std::filesystem::path& p : files())
    if (p != m_path) std::filesystem::remove(p, ec);
}

std::string readable_line(std::string_view json) {
  // Fields are written as "type", "t", then the rest: show `t type rest`.
  std::string type, t, rest;
  std::string* out = &rest;
  std::string key;
  bool in_string = false, in_key = false, escaped = false;
  for (const char c : json) {
    if (in_string) {
      if (escaped) {
        (in_key ? key : *out) += c;
        escaped = false;
      } else if (c == '\\') {
        escaped = true;
      } else if (c == '"') {
        in_string = false;
      } else {
        (in_key ? key : *out) += c;
      }
      continue;
    }
    if (c == '"') {
      in_string = true;
      continue;
    }
    if (c == '{' || c == ',') {
      in_key = true;
      key.clear();
    } else if (c == ':' && in_key) {
      in_key = false;
      if (key == "type") {
        out = &type;
      } else if (key == "t") {
        out = &t;
      } else {
        out = &rest;
        rest += rest.empty() ? "" : " ";
        rest += key;
        rest += '=';
      }
    } else if (c != '}' && !in_key) {
      *out += c;
    }
  }
  return t + ' ' + type + (rest.empty() ? "" : " ") + rest;
}

}  // namespace nav
