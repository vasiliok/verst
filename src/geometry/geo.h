/// @file
/// Geographic and projected coordinates in the client's terms: degrees, Web Mercator
/// metres and boxes of both. No storage format: the nav files keep their own integer
/// coordinates (format.h) and convert at the data side.
#pragma once

#include <cmath>
#include <numbers>

#include "vrst_types.h"

namespace nav {

constexpr Vrst_float64 kPi = std::numbers::pi;  ///< Pi in double, for degree <-> radian conversion.
/// WGS84 equatorial radius in metres, the sphere radius of Web Mercator (EPSG:3857).
constexpr Vrst_float64 kEarthRadius = 6378137.0;

/// 2D point or vector in double precision: Web Mercator metres or screen pixels
/// depending on use.
/// All view math is done with it; only center-relative values are converted to
/// float for the GPU.
struct Vrst_vec2d {
  /// Double on purpose: Web Mercator metres reach 2e7, where float has ~2 m steps.
  /// Everything is computed in double and only center-relative values go to float.
  Vrst_float64 x;
  Vrst_float64 y;  ///< Second component; same units and precision as x.
};

/// Geographic point in degrees (WGS84).
struct Vrst_latlon {
  Vrst_float64 lat = 0;  ///< Latitude, degrees north.
  Vrst_float64 lon = 0;  ///< Longitude, degrees east.
};

/// Geographic box in degrees.
struct Vrst_geo_box {
  Vrst_latlon south_west;  ///< Smallest latitude and longitude.
  Vrst_latlon north_east;  ///< Largest latitude and longitude.
};

/// Axis-aligned box in Mercator metres (y north).
struct Vrst_box2d {
  /// South-west corner.
  Vrst_vec2d lo;
  /// North-east corner.
  Vrst_vec2d hi;
};

/// Web Mercator (EPSG:3857) projection.
/// @param p Geographic point.
/// @return Mercator metres (x east, y north) in double.
inline Vrst_vec2d mercator(Vrst_latlon p) {
  const Vrst_float64 lon = p.lon * kPi / 180.0;
  const Vrst_float64 lat = p.lat * kPi / 180.0;
  return {kEarthRadius * lon, kEarthRadius * std::log(std::tan(kPi / 4 + lat / 2))};
}

/// Inverse of mercator().
/// @param m Web Mercator metres.
/// @return Geographic point.
inline Vrst_latlon inverse_mercator(Vrst_vec2d m) {
  return {(2 * std::atan(std::exp(m.y / kEarthRadius)) - kPi / 2) * 180.0 / kPi, m.x / kEarthRadius * 180.0 / kPi};
}

/// Whether a point lies in a box (borders included).
/// @param b Box.
/// @param p Point.
/// @return true when inside.
inline bool contains(const Vrst_geo_box& b, Vrst_latlon p) {
  return p.lat >= b.south_west.lat && p.lat <= b.north_east.lat && p.lon >= b.south_west.lon &&
         p.lon <= b.north_east.lon;
}

}  // namespace nav
