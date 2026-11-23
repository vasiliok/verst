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
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "geometry/geo.h"
#include "vrst_types.h"

namespace nav {

/// Parses a whole text as a decimal number, as strtod reads it.
/// @param text Number text, with nothing after the number.
/// @return The value, or nothing when the text is empty, not a number or has trailing characters.
inline std::optional<Vrst_float64> parse_float64(std::string_view text) {
  const std::string s(text);
  try {
    Vrst_size used = 0;
    const Vrst_float64 v = std::stod(s, &used);
    return used == s.size() ? std::optional(v) : std::nullopt;
  } catch (const std::exception&) {  // invalid_argument or out_of_range
    return std::nullopt;
  }
}

/// Interpolated state of a track at one playback time (Vrst_gpx_track::sample()).
struct Vrst_track_sample {
  /// Position, linearly interpolated between the two surrounding track points.
  Vrst_latlon pos;
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
  void load(const std::string& path);

  /// Parses GPX text. Points with a time earlier than their predecessor (clock jumps)
  /// get the predecessor's time, so times never go backwards.
  /// @param text Whole GPX document.
  /// @param name Label for error messages, usually the file path.
  /// @throws std::runtime_error when there are no track points or one is malformed.
  void parse(std::string_view text, const std::string& name);

  /// Track state at a playback time.
  /// @param t Seconds since the first point; clamped to [0, duration()].
  /// @return Interpolated position, current point index and speed.
  Vrst_track_sample sample(Vrst_float64 t) const;

  /// Track length along all points.
  /// @return Metres.
  Vrst_float64 length_m() const;

  /// Great-circle distance (haversine on the mean Earth radius).
  /// @param a First point.
  /// @param b Second point.
  /// @return Metres.
  static Vrst_float64 distance_m(Vrst_latlon a, Vrst_latlon b);

  /// Time of the last point.
  /// @return Seconds since the first point.
  Vrst_float64 duration() const { return m_times.empty() ? 0 : m_times.back(); }
  /// @copydoc m_points
  /// @return Points in file order.
  const std::vector<Vrst_latlon>& points() const { return m_points; }
  /// @copydoc m_times
  /// @return Seconds, same index as points().
  const std::vector<Vrst_float64>& times() const { return m_times; }
  /// Whether a track is loaded.
  /// @return true if there are no points.
  bool empty() const { return m_points.empty(); }

  /// Parses an ISO 8601 UTC time as GPX writes it: `YYYY-MM-DDTHH:MM:SS[.fff]Z`.
  /// Offsets other than Z are not used by GPX trackers and are rejected.
  /// @param s Text starting with the time.
  /// @return Seconds since 1970-01-01 UTC, or nothing when the text is not such a time.
  static std::optional<Vrst_float64> parse_time(std::string_view s);

 private:
  /// Reads a numeric XML attribute `name="value"` (or single quotes) from a start tag.
  /// @param tag Start tag text, from `<` to before `>`.
  /// @param name Attribute name.
  /// @return The value, or nothing if the attribute is missing or not a number.
  static std::optional<Vrst_float64> attribute(std::string_view tag, std::string_view name);

  /// Days since 1970-01-01 of a proleptic Gregorian date (H. Hinnant's algorithm).
  /// @param y Year.
  /// @param m Month 1..12.
  /// @param d Day 1..31.
  /// @return Day number; negative before 1970.
  static Vrst_int64 days_from_civil(int y, int m, int d);

  /// Track points in file order, segments joined. Fixed point like the map data, so a
  /// point is converted to the map's world coordinates the same way as a road point.
  std::vector<Vrst_latlon> m_points;
  /// Seconds since the first point, never decreasing. The timing is what makes the
  /// replay realistic: stops at junctions and slow corners keep the dot in place.
  std::vector<Vrst_float64> m_times;
};

}  // namespace nav
