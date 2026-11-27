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

/// How the car moves: where, which way and how fast (from GPS).
struct Vrst_motion {
  Vrst_latlon pos;              ///< Position.
  Vrst_float64 heading_deg = 0; ///< Direction of travel, degrees clockwise from north.
  Vrst_float64 speed_kmh = 0;   ///< Speed of travel.
};

/// Kind of road the car is on, for the reroute rules.
enum class Vrst_road_type : Vrst_uint8 {
  Highway,  ///< Motorway or trunk (and their ramps): no U-turns.
  Rural,    ///< Any other road outside a town.
  Town,     ///< Street in a town: U-turns only when nothing else works.
};

/// What comes up ahead on the road, in the direction of travel.
enum class Vrst_road_event_kind : Vrst_uint8 {
  Intersection,  ///< Three or more roads meet.
  Roundabout,    ///< The road reaches a roundabout: going round turns back without a U-turn.
  Interchange,   ///< A ramp leaves the road (an exit).
  DeadEnd,       ///< The road ends.
};

/// One thing ahead on the road the car is on.
struct Vrst_road_event {
  Vrst_road_event_kind kind = Vrst_road_event_kind::Intersection;  ///< What it is.
  Vrst_float64 ahead_m = 0;  ///< Distance along the road from the car.
  bool uturn_ok = false;     ///< The car may turn back there (a roundabout always).
  Vrst_latlon pos;           ///< Where it is.
};

/// Where the car is and what is ahead: the input of the reroute decision (no user action).
struct Vrst_road_situation {
  bool found = false;                  ///< The car is on a road the mode may use.
  Vrst_motion motion;                  ///< Position snapped onto its road, heading and speed.
  Vrst_road_type type = Vrst_road_type::Rural;  ///< Kind of road.
  std::string road;                    ///< Name or number of the road; empty when it has none.
  std::vector<Vrst_road_event> ahead;  ///< Next events within a few km, nearest first.
};

/// How a route from a moving car may turn back, chosen from the situation (decide()).
enum class Vrst_reroute_strategy : Vrst_uint8 {
  Any,         ///< Not moving: from the nearest road in either direction.
  Forward,     ///< Fast on a highway: on in the direction of travel to the next exit.
  UTurnAhead,  ///< Turn back at the roundabout or intersection ahead.
  NoUTurn,     ///< In town: round the blocks, a U-turn only when nothing else works.
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
  Vrst_size uturns = 0;             ///< U-turns in the route (for the log).
  Vrst_road_situation situation;    ///< Route from a moving car: its situation; otherwise found = false.
  Vrst_reroute_strategy strategy = Vrst_reroute_strategy::Any;  ///< Route from a moving car: the rule used.
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
