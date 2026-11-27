/// @file
/// Vrst_route_tracker: position along a route and the off-route decision.
/// Declarations and documentation are in route_tracker.h.
#include "route/route_tracker.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace nav {

namespace {

/// Metres per degree of latitude on the Web Mercator sphere.
constexpr Vrst_float64 kMetresPerDegree = kEarthRadius * kPi / 180;

/// A point in flat metres east and north of an origin.
/// @param p Point.
/// @param origin Origin.
/// @param cos_lat Cosine of the origin's latitude.
/// @return Metres.
Vrst_vec2d local_m(Vrst_latlon p, Vrst_latlon origin, Vrst_float64 cos_lat) {
  return {(p.lon - origin.lon) * kMetresPerDegree * cos_lat, (p.lat - origin.lat) * kMetresPerDegree};
}

/// Flat distance between two points.
/// @param a First point.
/// @param b Second point.
/// @return Metres.
Vrst_float64 distance_m(Vrst_latlon a, Vrst_latlon b) {
  const Vrst_vec2d d = local_m(b, a, std::cos((a.lat + b.lat) / 2 * kPi / 180));
  return std::hypot(d.x, d.y);
}

}  // namespace

Vrst_route_tracker::Vrst_route_tracker(std::vector<Vrst_latlon> route, Vrst_travel_mode mode)
    : m_points(std::move(route)), m_mode(mode) {
  m_cum_m.reserve(m_points.size());
  Vrst_float64 total = 0;
  for (Vrst_size i = 0; i < m_points.size(); ++i) {
    if (i > 0) total += distance_m(m_points[i - 1], m_points[i]);
    m_cum_m.push_back(total);
  }
  m_last.left_m = total;
}

Vrst_float64 Vrst_route_tracker::tolerance_m(Vrst_travel_mode mode, Vrst_float64 accuracy_m) {
  return std::max(kBase_m[static_cast<Vrst_size>(mode)], kAccuracyFactor * accuracy_m);
}

Vrst_route_check Vrst_route_tracker::update(Vrst_latlon fix, Vrst_float64 accuracy_m, Vrst_float64 t) {
  if (m_points.size() < 2 || accuracy_m > kMaxAccuracy_m) {
    m_last.off_route = false;
    return m_last;
  }
  // The window: from kBehind_m behind the progress to kWindow_m ahead of it.
  Vrst_size first = 0, last = m_points.size() - 1;
  if (m_started) {
    first = m_piece;
    while (first > 0 && m_cum_m[m_piece] - m_cum_m[first - 1] <= kBehind_m) --first;
    last = m_piece + 1;
    while (last + 1 < m_points.size() && m_cum_m[last] - m_cum_m[m_piece] < kWindow_m) ++last;
  }
  const Vrst_float64 cos_lat = std::cos(fix.lat * kPi / 180);
  Vrst_float64 best_m = std::numeric_limits<Vrst_float64>::infinity(), best_along = 0;
  Vrst_size best_piece = first;
  for (Vrst_size i = first; i < last; ++i) {
    const Vrst_vec2d a = local_m(m_points[i], fix, cos_lat), b = local_m(m_points[i + 1], fix, cos_lat);
    const Vrst_vec2d ab{b.x - a.x, b.y - a.y};
    const Vrst_float64 len2 = ab.x * ab.x + ab.y * ab.y;
    // The fix is the origin: the nearest point of the piece is a + ab * s.
    const Vrst_float64 s = len2 > 0 ? std::clamp(-(a.x * ab.x + a.y * ab.y) / len2, 0.0, 1.0) : 0.0;
    const Vrst_float64 d = std::hypot(a.x + ab.x * s, a.y + ab.y * s);
    if (d < best_m) {
      best_m = d;
      best_piece = i;
      best_along = m_cum_m[i] + (m_cum_m[i + 1] - m_cum_m[i]) * s;
    }
  }
  if (!m_first_off_m) m_first_off_m = best_m;
  const Vrst_float64 tol = tolerance_m(m_mode, accuracy_m);
  const bool on = best_m <= tol;
  const bool off = m_started ? !on : best_m > *m_first_off_m + tol;
  Vrst_route_check c;
  c.off_m = best_m;
  c.left_m = m_cum_m.back() - best_along;
  if (on) {
    m_piece = best_piece;
    m_started = true;
    m_off_since.reset();
    if (c.left_m < kArrive_m) m_arrived = true;
  } else if (!off) {
    m_off_since.reset();  // near the start, not moving away
  } else if (!m_arrived) {
    if (!m_off_since) m_off_since = t;
    if (t - *m_off_since >= kOffTime_s) {
      c.off_route = true;
      m_off_since.reset();  // once per departure
    }
  }
  c.arrived = m_arrived;
  m_last = c;
  return c;
}

}  // namespace nav
