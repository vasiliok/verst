/// @file
/// decide(): the reroute rules. Documentation is in reroute_rules.h.
#include "route/reroute_rules.h"

namespace nav {

Vrst_reroute_plan decide(const Vrst_road_situation& s) {
  if (!s.found || s.motion.speed_kmh < kMovingKmh) return {Vrst_reroute_strategy::Any, std::nullopt};
  if (s.type == Vrst_road_type::Highway && s.motion.speed_kmh > kHighwayFastKmh)
    return {Vrst_reroute_strategy::Forward, std::nullopt};
  if (s.type == Vrst_road_type::Town) return {Vrst_reroute_strategy::NoUTurn, std::nullopt};
  std::optional<Vrst_size> cross, round;  // nearest intersection and roundabout to turn back at
  for (Vrst_size i = 0; i < s.ahead.size(); ++i) {
    const Vrst_road_event& e = s.ahead[i];
    if (!e.uturn_ok) continue;
    if (e.kind == Vrst_road_event_kind::Roundabout && !round) round = i;
    if (e.kind == Vrst_road_event_kind::Intersection && !cross) cross = i;
  }
  if (round && (!cross || s.ahead[*round].ahead_m <= s.ahead[*cross].ahead_m + kRoundaboutPrefer_m))
    return {Vrst_reroute_strategy::UTurnAhead, round};
  if (cross) return {Vrst_reroute_strategy::UTurnAhead, cross};
  return {Vrst_reroute_strategy::Forward, std::nullopt};
}

}  // namespace nav
