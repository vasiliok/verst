/// @file
/// Recorded GPS track loaded from a GPX file, with time-based playback queries.
///
/// Platform independent like map_scene.h: the desktop viewer and the iOS app ask the
/// same Vrst_gpx_track where the vehicle was at a playback time and draw a dot there.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "format.h"

namespace nav {

/// Interpolated state of a track at one playback time (Vrst_gpx_track::sample()).
struct Vrst_track_sample {
  /// Position, linearly interpolated between the two surrounding track points.
  Vrst_coord pos;
  /// Index of the last track point at or before the time; the drawn "already driven"
  /// part of the track ends there.
  Vrst_size index;
  /// Ground speed over the current track interval in km/h; 0 when the GPS stood still
  /// or two points share a timestamp.
  Vrst_float64 speed_kmh;
};

/// GPS track of one GPX file: all `<trkpt>` points of all segments in file order,
/// with their times in seconds since the first point.
///
/// Needed to replay a recorded drive on the map: the time per point lets playback
/// move the dot at the real (or a multiplied) speed, including stops, instead of
/// stepping evenly through the points. The parser is a small text scanner, not an
/// XML library: it reads `lat`/`lon` attributes and the `<time>` child, which is all
/// GPX 1.0/1.1 trackers write for a track point.
class Vrst_gpx_track {
 public:
  /// Reads a GPX file.
  /// @param path GPX file.
  /// @throws std::runtime_error with the path as prefix when the file cannot be read,
  ///         has no track points, or a point has no valid lat/lon or time.
  void load(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error(path + ": cannot open");
    std::stringstream ss;
    ss << f.rdbuf();
    parse(ss.str(), path);
  }

  /// Parses GPX text. Points with a time earlier than their predecessor (clock jumps)
  /// get the predecessor's time, so times never go backwards.
  /// @param text Whole GPX document.
  /// @param name Label for error messages, usually the file path.
  /// @throws std::runtime_error when there are no track points or one is malformed.
  void parse(std::string_view text, const std::string& name) {
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
      Vrst_float64 lat = 0, lon = 0, t = 0;
      if (!attribute(tag, "lat", &lat) || !attribute(tag, "lon", &lon) || std::abs(lat) > 90 || std::abs(lon) > 180)
        throw std::runtime_error(name + ": track point " + std::to_string(m_points.size()) + " has no valid lat/lon");
      const Vrst_size ts = body.find("<time>");
      if (ts == std::string_view::npos || !parse_time(body.substr(ts + 6), &t))
        throw std::runtime_error(name + ": track point " + std::to_string(m_points.size()) + " has no valid <time>");
      if (m_points.empty()) t0 = t;
      t -= t0;
      if (!m_times.empty()) t = std::max(t, m_times.back());
      m_points.push_back({static_cast<Vrst_int32>(std::lround(lat * 1e7)), static_cast<Vrst_int32>(std::lround(lon * 1e7))});
      m_times.push_back(t);
    }
    if (m_points.empty()) throw std::runtime_error(name + ": no <trkpt> track points");
  }

  /// Track state at a playback time.
  /// @param t Seconds since the first point; clamped to [0, duration()].
  /// @return Interpolated position, current point index and speed.
  Vrst_track_sample sample(Vrst_float64 t) const {
    t = std::clamp(t, 0.0, duration());
    // Last point with time <= t; upper_bound finds the first one after it.
    const Vrst_size i = Vrst_size(std::upper_bound(m_times.begin(), m_times.end(), t) - m_times.begin()) - 1;
    if (i + 1 >= m_points.size()) return {m_points.back(), m_points.size() - 1, 0};
    const Vrst_coord a = m_points[i], b = m_points[i + 1];
    const Vrst_float64 dt = m_times[i + 1] - m_times[i];
    const Vrst_float64 f = dt > 0 ? (t - m_times[i]) / dt : 0;
    const Vrst_coord p{static_cast<Vrst_int32>(std::lround(a.lat_e7 + f * (b.lat_e7 - a.lat_e7))),
                       static_cast<Vrst_int32>(std::lround(a.lon_e7 + f * (b.lon_e7 - a.lon_e7)))};
    return {p, i, dt > 0 ? distance_m(a, b) / dt * 3.6 : 0};
  }

  /// Track length along all points.
  /// @return Metres.
  Vrst_float64 length_m() const {
    Vrst_float64 sum = 0;
    for (Vrst_size i = 1; i < m_points.size(); ++i) sum += distance_m(m_points[i - 1], m_points[i]);
    return sum;
  }

  /// Great-circle distance (haversine on the mean Earth radius).
  /// @param a First point.
  /// @param b Second point.
  /// @return Metres.
  static Vrst_float64 distance_m(Vrst_coord a, Vrst_coord b) {
    constexpr Vrst_float64 kRad = 3.14159265358979323846 / 180 * 1e-7;
    const Vrst_float64 la1 = a.lat_e7 * kRad, la2 = b.lat_e7 * kRad;
    const Vrst_float64 dla = la2 - la1, dlo = (b.lon_e7 - a.lon_e7) * kRad;
    const Vrst_float64 h = std::sin(dla / 2) * std::sin(dla / 2) + std::cos(la1) * std::cos(la2) * std::sin(dlo / 2) * std::sin(dlo / 2);
    return 2 * 6371008.8 * std::asin(std::min(1.0, std::sqrt(h)));
  }

  /// Time of the last point.
  /// @return Seconds since the first point.
  Vrst_float64 duration() const { return m_times.empty() ? 0 : m_times.back(); }
  /// @copydoc m_points
  /// @return Points in file order.
  const std::vector<Vrst_coord>& points() const { return m_points; }
  /// @copydoc m_times
  /// @return Seconds, same index as points().
  const std::vector<Vrst_float64>& times() const { return m_times; }
  /// Whether a track is loaded.
  /// @return true if there are no points.
  bool empty() const { return m_points.empty(); }

  /// Parses an ISO 8601 UTC time as GPX writes it: `YYYY-MM-DDTHH:MM:SS[.fff]Z`.
  /// Offsets other than Z are not used by GPX trackers and are rejected.
  /// @param s Text starting with the time.
  /// @param[out] out Seconds since 1970-01-01 UTC.
  /// @return true on success.
  static bool parse_time(std::string_view s, Vrst_float64* out) {
    if (s.size() < 20 || s[4] != '-' || s[7] != '-' || s[10] != 'T' || s[13] != ':' || s[16] != ':') return false;
    auto num = [&](Vrst_size at, Vrst_size n, int* v) {
      *v = 0;
      for (Vrst_size k = at; k < at + n; ++k) {
        if (s[k] < '0' || s[k] > '9') return false;
        *v = *v * 10 + (s[k] - '0');
      }
      return true;
    };
    int y, mo, d, h, mi, sec;
    if (!num(0, 4, &y) || !num(5, 2, &mo) || !num(8, 2, &d) || !num(11, 2, &h) || !num(14, 2, &mi) ||
        !num(17, 2, &sec) || mo < 1 || mo > 12 || d < 1 || d > 31)
      return false;
    Vrst_float64 frac = 0;
    Vrst_size k = 19;
    if (s[k] == '.') {
      Vrst_float64 scale = 0.1;
      for (++k; k < s.size() && s[k] >= '0' && s[k] <= '9'; ++k, scale *= 0.1) frac += (s[k] - '0') * scale;
    }
    if (k >= s.size() || s[k] != 'Z') return false;
    *out = Vrst_float64(days_from_civil(y, mo, d)) * 86400 + h * 3600 + mi * 60 + sec + frac;
    return true;
  }

 private:
  /// Reads a numeric XML attribute `name="value"` (or single quotes) from a start tag.
  /// @param tag Start tag text, from `<` to before `>`.
  /// @param name Attribute name.
  /// @param[out] out Parsed value.
  /// @return true if the attribute exists and is a number.
  static bool attribute(std::string_view tag, std::string_view name, Vrst_float64* out) {
    for (Vrst_size at = tag.find(name); at != std::string_view::npos; at = tag.find(name, at + 1)) {
      // Must be a whole attribute name: preceded by a space and followed by =.
      if (at == 0 || tag[at - 1] != ' ' || at + name.size() + 1 >= tag.size() || tag[at + name.size()] != '=') continue;
      const char q = tag[at + name.size() + 1];
      if (q != '"' && q != '\'') return false;
      const std::string value(tag.substr(at + name.size() + 2, tag.find(q, at + name.size() + 2) - (at + name.size() + 2)));
      char* end = nullptr;
      *out = std::strtod(value.c_str(), &end);
      return end != value.c_str() && *end == '\0';
    }
    return false;
  }

  /// Days since 1970-01-01 of a proleptic Gregorian date (H. Hinnant's algorithm).
  /// @param y Year.
  /// @param m Month 1..12.
  /// @param d Day 1..31.
  /// @return Day number; negative before 1970.
  static Vrst_int64 days_from_civil(int y, int m, int d) {
    y -= m <= 2;
    const Vrst_int64 era = (y >= 0 ? y : y - 399) / 400;
    const Vrst_int64 yoe = y - era * 400;
    const Vrst_int64 doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const Vrst_int64 doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + doe - 719468;
  }

  /// Track points in file order, segments joined. Fixed point like the map data, so a
  /// point is converted to the map's world coordinates the same way as a road point.
  std::vector<Vrst_coord> m_points;
  /// Seconds since the first point, never decreasing. The timing is what makes the
  /// replay realistic: stops at junctions and slow corners keep the dot in place.
  std::vector<Vrst_float64> m_times;
};

}  // namespace nav
