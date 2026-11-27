/// @file
/// Vrst_route_tracker: follows the GPS position along a shown route and says when the
/// traveller has left it, so the front end can lay a new route from where they are.
///
/// Each fix is measured against the route line in local flat metres (longitude scaled by
/// the cosine of the latitude, fine over the few hundred metres that matter). Only a
/// window of the line is searched: from a little behind the progress made so far to
/// kWindow_m ahead, so a route that passes the same place twice is not confused and a
/// 1 Hz fix costs a few hundred pieces at most; the first fix searches the whole line.
/// A fix farther than the tolerance (the mode's base, or more for a poor fix) for
/// kOffTime_s in a row is off route. Until a fix first reaches the line (a route starts
/// on the nearest road, which may be far from a parking place) the allowed distance is
/// that of the first fix plus the tolerance: moving away is off route, waiting is not.
/// Fixes worse than kMaxAccuracy_m decide nothing, and near the destination nothing is
/// off route any more. Client types only (route_result.h, geo.h), so front ends can use it.
#pragma once

#include <optional>
#include <vector>

#include "geometry/geo.h"
#include "provider/route_result.h"

namespace nav {

/// What one fix says about the route.
struct Vrst_route_check {
  Vrst_float64 off_m = 0;   ///< From the fix to the route line.
  Vrst_float64 left_m = 0;  ///< Route length still ahead of the fix.
  bool arrived = false;     ///< Within kArrive_m of the end; nothing is off route any more.
  bool off_route = false;   ///< Off the route long enough: lay a new one now.
};

/// Follows the position along one route; see the file comment.
class Vrst_route_tracker {
 public:
  /// Starts at the beginning of a route.
  /// @param route Route points, start to destination.
  /// @param mode Who travels (sets the base tolerance).
  Vrst_route_tracker(std::vector<Vrst_latlon> route, Vrst_travel_mode mode);

  /// Measures one fix. Reports off_route once per departure: the timer starts again
  /// after it.
  /// @param fix Position.
  /// @param accuracy_m Horizontal accuracy of the fix.
  /// @param t Time of the fix, seconds (any clock that only goes forward).
  /// @return What the fix says; the previous answer (off_route false) for a poor fix.
  Vrst_route_check update(Vrst_latlon fix, Vrst_float64 accuracy_m, Vrst_float64 t);

  /// Distance allowed from the line before a fix counts as off.
  /// @param mode Who travels.
  /// @param accuracy_m Horizontal accuracy of the fix.
  /// @return Metres.
  static Vrst_float64 tolerance_m(Vrst_travel_mode mode, Vrst_float64 accuracy_m);

  /// Base tolerance per mode (car, bike, foot), metres.
  static constexpr Vrst_float64 kBase_m[3] = {40, 30, 25};
  /// A poor fix widens the tolerance to this many times its accuracy.
  static constexpr Vrst_float64 kAccuracyFactor = 1.5;
  /// Fixes less accurate than this decide nothing, metres.
  static constexpr Vrst_float64 kMaxAccuracy_m = 60;
  /// Off the line this long in a row is off route, seconds.
  static constexpr Vrst_float64 kOffTime_s = 5;
  /// Closer than this to the end of the route is arrival, metres: the destination is
  /// often off the road (a parking place, a building) and the route ends on the nearest
  /// road, so the last metres are left on purpose.
  static constexpr Vrst_float64 kArrive_m = 200;
  /// The search window reaches this far ahead of the progress, metres.
  static constexpr Vrst_float64 kWindow_m = 2000;
  /// ... and this far behind it (a fix slightly behind the last one), metres.
  static constexpr Vrst_float64 kBehind_m = 200;

 private:
  /// Route points.
  std::vector<Vrst_latlon> m_points;
  /// Route length from the start to each point, metres.
  std::vector<Vrst_float64> m_cum_m;
  /// Who travels.
  Vrst_travel_mode m_mode;
  /// Piece (points i, i + 1) of the last fix that was on the route.
  Vrst_size m_piece = 0;
  /// A fix was on the route already (the window search may start).
  bool m_started = false;
  /// The destination was reached.
  bool m_arrived = false;
  /// Time of the first fix of the current departure; nullopt when on the route.
  std::optional<Vrst_float64> m_off_since;
  /// Distance of the first measured fix from the line, metres; nullopt before it.
  std::optional<Vrst_float64> m_first_off_m;
  /// Last answer, returned again for a poor fix.
  Vrst_route_check m_last;
};

}  // namespace nav
