/// @file
/// Vrst_gpx_track: GPX parsing and playback queries.
/// Declarations and documentation are in gpx_track.h.
#include "gpx_track.h"

#include <algorithm>

#include <numbers>

namespace nav {

void Vrst_gpx_track::load(const std::string& path) {
  const std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error(path + ": cannot open");
  std::stringstream ss;
  ss << f.rdbuf();
  parse(ss.str(), path);
}

void Vrst_gpx_track::parse(std::string_view text, const std::string& name) {
  m_points.clear();
  m_times.clear();
  Vrst_float64 t0 = 0;
  for (Vrst_size at = text.find("<trkpt"); at != std::string_view::npos; at = text.find("<trkpt", at + 1)) {
    const Vrst_size tag_end = text.find('>', at);
    const Vrst_size end = text.find("</trkpt>", at);
    if (tag_end == std::string_view::npos || end == std::string_view::npos)
      throw std::runtime_error(name + ": unterminated <trkpt>");
    const std::string_view tag = text.substr(at, tag_end - at);
    const std::string_view body = text.substr(tag_end, end - tag_end);
    const std::optional<Vrst_float64> lat = attribute(tag, "lat"), lon = attribute(tag, "lon");
    if (!lat || !lon || std::abs(*lat) > 90 || std::abs(*lon) > 180)
      throw std::runtime_error(name + ": track point " + std::to_string(m_points.size()) + " has no valid lat/lon");
    const Vrst_size ts = body.find("<time>");
    const std::optional<Vrst_float64> time =
        ts == std::string_view::npos ? std::nullopt : parse_time(body.substr(ts + 6));
    if (!time)
      throw std::runtime_error(name + ": track point " + std::to_string(m_points.size()) + " has no valid <time>");
    Vrst_float64 t = *time;
    if (m_points.empty()) t0 = t;
    t -= t0;
    if (!m_times.empty()) t = std::max(t, m_times.back());
    m_points.push_back({*lat, *lon});
    m_times.push_back(t);
  }
  if (m_points.empty()) throw std::runtime_error(name + ": no <trkpt> track points");
}

Vrst_track_sample Vrst_gpx_track::sample(Vrst_float64 t) const {
  t = std::clamp(t, 0.0, duration());
  // Last point with time <= t; upper_bound finds the first one after it.
  const Vrst_size i = static_cast<Vrst_size>(std::ranges::upper_bound(m_times, t) - m_times.begin()) - 1;
  if (i + 1 >= m_points.size()) return {m_points.back(), m_points.size() - 1, 0};
  const Vrst_latlon a = m_points[i], b = m_points[i + 1];
  const Vrst_float64 dt = m_times[i + 1] - m_times[i];
  const Vrst_float64 f = dt > 0 ? (t - m_times[i]) / dt : 0;
  const Vrst_latlon p{a.lat + f * (b.lat - a.lat), a.lon + f * (b.lon - a.lon)};
  return {p, i, dt > 0 ? distance_m(a, b) / dt * 3.6 : 0};
}

Vrst_float64 Vrst_gpx_track::length_m() const {
  Vrst_float64 sum = 0;
  for (Vrst_size i = 1; i < m_points.size(); ++i) sum += distance_m(m_points[i - 1], m_points[i]);
  return sum;
}

Vrst_float64 Vrst_gpx_track::distance_m(Vrst_latlon a, Vrst_latlon b) {
  constexpr Vrst_float64 kRad = std::numbers::pi / 180;
  const Vrst_float64 la1 = a.lat * kRad, la2 = b.lat * kRad;
  const Vrst_float64 dla = la2 - la1, dlo = (b.lon - a.lon) * kRad;
  const Vrst_float64 h = std::sin(dla / 2) * std::sin(dla / 2) + std::cos(la1) * std::cos(la2) * std::sin(dlo / 2) * std::sin(dlo / 2);
  return 2 * 6371008.8 * std::asin(std::min(1.0, std::sqrt(h)));
}

std::optional<Vrst_float64> Vrst_gpx_track::parse_time(std::string_view s) {
  if (s.size() < 20 || s[4] != '-' || s[7] != '-' || s[10] != 'T' || s[13] != ':' || s[16] != ':') return std::nullopt;
  const auto num = [&](Vrst_size at, Vrst_size n, int& v) {
    v = 0;
    for (Vrst_size k = at; k < at + n; ++k) {
      if (s[k] < '0' || s[k] > '9') return false;
      v = v * 10 + (s[k] - '0');
    }
    return true;
  };
  int y = 0, mo = 0, d = 0, h = 0, mi = 0, sec = 0;
  if (!num(0, 4, y) || !num(5, 2, mo) || !num(8, 2, d) || !num(11, 2, h) || !num(14, 2, mi) ||
      !num(17, 2, sec) || mo < 1 || mo > 12 || d < 1 || d > 31)
    return std::nullopt;
  Vrst_float64 frac = 0;
  Vrst_size k = 19;
  if (s[k] == '.') {
    Vrst_float64 scale = 0.1;
    for (++k; k < s.size() && s[k] >= '0' && s[k] <= '9'; ++k, scale *= 0.1) frac += (s[k] - '0') * scale;
  }
  if (k >= s.size() || s[k] != 'Z') return std::nullopt;
  return static_cast<Vrst_float64>(days_from_civil(y, mo, d)) * 86400 + h * 3600 + mi * 60 + sec + frac;
}

std::optional<Vrst_float64> Vrst_gpx_track::attribute(std::string_view tag, std::string_view name) {
  for (Vrst_size at = tag.find(name); at != std::string_view::npos; at = tag.find(name, at + 1)) {
    // Must be a whole attribute name: preceded by a space and followed by =.
    if (at == 0 || tag[at - 1] != ' ' || at + name.size() + 1 >= tag.size() || tag[at + name.size()] != '=') continue;
    const char q = tag[at + name.size() + 1];
    if (q != '"' && q != '\'') return std::nullopt;
    return parse_float64(tag.substr(at + name.size() + 2, tag.find(q, at + name.size() + 2) - (at + name.size() + 2)));
  }
  return std::nullopt;
}

Vrst_int64 Vrst_gpx_track::days_from_civil(int y, int m, int d) {
  y -= m <= 2;
  const Vrst_int64 era = (y >= 0 ? y : y - 399) / 400;
  const Vrst_int64 yoe = y - era * 400;
  const Vrst_int64 doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const Vrst_int64 doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + doe - 719468;
}

}  // namespace nav
