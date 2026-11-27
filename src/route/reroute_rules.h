/// @file
/// decide(): how a route from a moving car may turn back, chosen only from the situation
/// of the car (Vrst_road_situation: where it is, the kind of road, what is ahead and the
/// speed), never from a user action. The rules, in order:
/// 1. Highway above kHighwayFastKmh: the driver has not found a way to turn and is not
///    about to stop -> Forward, on in the direction of travel to the next exit. At
///    kHighwayFastKmh or less on a highway (slowing to turn or stop) rule 1 does not
///    apply.
/// 2. Town -> NoUTurn, whatever the speed: round the blocks, a U-turn only when nothing
///    else works.
/// 3. A roundabout or an intersection ahead where the car may turn back -> UTurnAhead
///    there; a roundabout wins over a nearer intersection by up to kRoundaboutPrefer_m.
/// 4. Anything else while moving -> Forward; not moving -> Any.
/// Client types only, so front ends can show the names.
#pragma once

#include <optional>

#include "provider/route_result.h"

namespace nav {

/// The chosen strategy and, for UTurnAhead, the event where the car turns back.
struct Vrst_reroute_plan {
  Vrst_reroute_strategy strategy = Vrst_reroute_strategy::Any;  ///< How the route may turn back.
  std::optional<Vrst_size> uturn_event;  ///< Index into Vrst_road_situation::ahead (UTurnAhead only).
};

/// Above this speed on a highway the car keeps going (rule 1), km/h.
inline constexpr Vrst_float64 kHighwayFastKmh = 50;
/// Below this speed the car is not moving: the heading means nothing, km/h.
inline constexpr Vrst_float64 kMovingKmh = 5;
/// A roundabout ahead wins over a nearer intersection by up to this much, metres.
inline constexpr Vrst_float64 kRoundaboutPrefer_m = 500;

/// Applies the rules of the file comment.
/// @param s Situation of the car.
/// @return Strategy, and the event to turn back at for UTurnAhead.
Vrst_reroute_plan decide(const Vrst_road_situation& s);

/// Name of a strategy, for the log and the debug box.
/// @param s Strategy.
/// @return "any", "forward", "uturn_ahead" or "no_uturn".
constexpr const char* strategy_name(Vrst_reroute_strategy s) {
  switch (s) {
    case Vrst_reroute_strategy::Any: return "any";
    case Vrst_reroute_strategy::Forward: return "forward";
    case Vrst_reroute_strategy::UTurnAhead: return "uturn_ahead";
    case Vrst_reroute_strategy::NoUTurn: return "no_uturn";
  }
  return "?";
}

/// Name of a road type, for the log and the debug box.
/// @param t Road type.
/// @return "highway", "rural" or "town".
constexpr const char* road_type_name(Vrst_road_type t) {
  switch (t) {
    case Vrst_road_type::Highway: return "highway";
    case Vrst_road_type::Rural: return "rural";
    case Vrst_road_type::Town: return "town";
  }
  return "?";
}

/// One-letter code of an event, for the log and the debug box.
/// @param k Event kind.
/// @return 'X' intersection, 'R' roundabout, 'E' interchange (exit), 'D' dead end.
constexpr char event_code(Vrst_road_event_kind k) {
  switch (k) {
    case Vrst_road_event_kind::Intersection: return 'X';
    case Vrst_road_event_kind::Roundabout: return 'R';
    case Vrst_road_event_kind::Interchange: return 'E';
    case Vrst_road_event_kind::DeadEnd: return 'D';
  }
  return '?';
}

}  // namespace nav
