/// @file
/// Tests of the reroute rules (reroute_rules.h) on hand-built situations.
#include <gtest/gtest.h>

#include "route/reroute_rules.h"

using namespace nav;

namespace {

/// A situation on a road of a type at a speed, with events ahead.
/// @param type Road type.
/// @param kmh Speed.
/// @param ahead Events, nearest first.
/// @return Situation.
Vrst_road_situation on(Vrst_road_type type, Vrst_float64 kmh, std::vector<Vrst_road_event> ahead = {}) {
  Vrst_road_situation s;
  s.found = true;
  s.type = type;
  s.motion.speed_kmh = kmh;
  s.ahead = std::move(ahead);
  return s;
}

/// An event.
/// @param kind Kind.
/// @param m Distance ahead.
/// @param ok U-turn allowed there.
/// @return Event.
Vrst_road_event at(Vrst_road_event_kind kind, Vrst_float64 m, bool ok = true) { return {kind, m, ok, {}}; }

}  // namespace

/// Rule 1: fast on a highway -> on to the next exit, whatever is ahead.
TEST(RerouteRules, FastHighwayGoesOn) {
  const Vrst_reroute_plan p = decide(on(Vrst_road_type::Highway, 110, {at(Vrst_road_event_kind::Interchange, 800, false)}));
  EXPECT_EQ(p.strategy, Vrst_reroute_strategy::Forward);
  EXPECT_FALSE(p.uturn_event);
}

/// Slow on a highway: rule 1 is off, a roundabout ahead is used.
TEST(RerouteRules, SlowHighwayUsesRoundabout) {
  const Vrst_reroute_plan p = decide(on(Vrst_road_type::Highway, 40, {at(Vrst_road_event_kind::Roundabout, 300)}));
  EXPECT_EQ(p.strategy, Vrst_reroute_strategy::UTurnAhead);
  EXPECT_EQ(p.uturn_event, 0u);
}

/// Town: no U-turn whatever the speed and whatever is ahead.
TEST(RerouteRules, TownNeverUTurns) {
  for (const Vrst_float64 kmh : {20.0, 50.0, 80.0})
    EXPECT_EQ(decide(on(Vrst_road_type::Town, kmh, {at(Vrst_road_event_kind::Intersection, 50)})).strategy,
              Vrst_reroute_strategy::NoUTurn);
}

/// Rural: turn back at the first intersection that allows it.
TEST(RerouteRules, RuralTurnsAtIntersection) {
  const Vrst_reroute_plan p = decide(on(Vrst_road_type::Rural, 80,
                                        {at(Vrst_road_event_kind::Intersection, 200, false),
                                         at(Vrst_road_event_kind::Intersection, 900)}));
  EXPECT_EQ(p.strategy, Vrst_reroute_strategy::UTurnAhead);
  EXPECT_EQ(p.uturn_event, 1u);
}

/// Rural: a roundabout a little farther wins over an intersection; a far one does not.
TEST(RerouteRules, RoundaboutPreferred) {
  EXPECT_EQ(decide(on(Vrst_road_type::Rural, 80,
                      {at(Vrst_road_event_kind::Intersection, 300), at(Vrst_road_event_kind::Roundabout, 700)}))
                .uturn_event,
            1u);
  EXPECT_EQ(decide(on(Vrst_road_type::Rural, 80,
                      {at(Vrst_road_event_kind::Intersection, 300), at(Vrst_road_event_kind::Roundabout, 2000)}))
                .uturn_event,
            0u);
}

/// Rural with nowhere to turn ahead: on; not moving: any.
TEST(RerouteRules, NothingAheadAndStanding) {
  EXPECT_EQ(decide(on(Vrst_road_type::Rural, 80)).strategy, Vrst_reroute_strategy::Forward);
  EXPECT_EQ(decide(on(Vrst_road_type::Rural, 2)).strategy, Vrst_reroute_strategy::Any);
  EXPECT_EQ(decide(Vrst_road_situation{}).strategy, Vrst_reroute_strategy::Any);
}
