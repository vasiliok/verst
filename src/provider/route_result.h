/// @file
/// Routes in the client's terms (Vrst_map_provider::route()): travel mode, the route as
/// points with its length and time, and the road at a point. No storage types, so front
/// ends can include it.
#pragma once

#include <string>
#include <vector>

#include "geometry/geo.h"
#include "vrst_types.h"

namespace nav {

/// Who travels: decides which ways may be used, in which direction, and how fast.
enum class Vrst_travel_mode : Vrst_uint8 {
  Car,   ///< Car driver: car roads, oneways, speed limits.
  Bike,  ///< Cyclist: no motorways, cycleways and paths, contraflow cycle lanes.
  Foot,  ///< Walker: footways, paths, steps, both ways of oneway streets.
};

/// A computed route.
struct Vrst_route {
  bool found = false;               ///< A route was found; otherwise see error.
  std::vector<Vrst_latlon> points;  ///< The route from start to destination.
  Vrst_float64 length_m = 0;        ///< Length.
  Vrst_float64 time_s = 0;          ///< Travel time for the mode.
  std::string error;                ///< Why there is no route ("no road near the start").
  Vrst_float64 shared_m = 0;        ///< Way back: metres on the outbound roads (0 for an outbound route).
  Vrst_size settled = 0;            ///< Graph vertices the search settled (for the log).
  Vrst_float64 ms = 0;              ///< Time the search took (for the log).
};

/// A route and its way back, computed together: the way back keeps to the outbound roads
/// wherever the mode may use them backwards and leaves them only where it must (a oneway).
struct Vrst_route_pair {
  Vrst_route there;  ///< From the start to the destination.
  Vrst_route back;   ///< From the destination back to the start; found = false when there is none.
};

/// The nearest road a mode may use at a point.
struct Vrst_road_at {
  Vrst_latlon pos;              ///< The nearest point on that road.
  std::string name;             ///< Road name or number; empty when it has none.
  Vrst_float64 distance_m = 0;  ///< From the asked point to pos.
};

}  // namespace nav
