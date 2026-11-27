/// @file
/// Vrst_router: shortest routes over the nav files for a car driver, a cyclist or a
/// walker, and the nearest road at a point.
///
/// A route starts and ends on the nearest road the mode may use that belongs to the
/// mode's main network (the largest connected part of the graph for that mode, so a
/// private parking lot or a fenced-off street is never chosen; computed once per mode
/// on first use), searched in the z14 tiles around the point (Vrst_tile_index), and
/// runs over the graph with A*: arcs the mode may not use are skipped (format 4 mode
/// bits; a format 3 graph is car only), a car arc costs its stored car time, a bike or
/// foot arc its segment length at the mode's speed, and the heuristic is the straight
/// distance at the mode's top speed. The path is unpacked into the segment shapes.
///
/// The search labels arcs, not vertices, so it knows the arc a car came in on when it
/// leaves a vertex: leaving on the same segment backwards is a U-turn, priced by
/// uturn_seconds() (never on a motorway or trunk or where no road meets, 60 s at an
/// intersection outside town, 1800 s in town; free for bikes and walkers). A route from
/// a moving car (route_moving()) starts on the road and in the direction it is moving,
/// and its U-turns follow the strategy decide() picks from the car's situation().
/// Const; the per-mode main network is computed once under a std::once_flag, so routes
/// may be computed on any thread.
#pragma once

#include <array>
#include <mutex>
#include <optional>
#include <vector>

#include "storage/map_tiles.h"
#include "storage/nav_files.h"
#include "provider/route_result.h"
#include "route/reroute_rules.h"

namespace nav {

/// Routes over one map; see the file comment.
class Vrst_router {
 public:
  /// Binds the router to a map's data (both must outlive it).
  /// @param data Nav files.
  /// @param index Tile index of the same data.
  Vrst_router(const Vrst_nav_data& data, const Vrst_tile_index& index) : m_data(data), m_index(index) {}

  /// Shortest route by travel time.
  /// @param from Start.
  /// @param to Destination.
  /// @param mode Who travels.
  /// @return The route, or found = false with the reason.
  Vrst_route route(Vrst_latlon from, Vrst_latlon to, Vrst_travel_mode mode) const;

  /// A route and its way back. The way back is searched with every road that is not on
  /// the outbound route kOffRouteFactor times more costly, so it follows the outbound
  /// roads in reverse wherever the mode may, and detours only where it must; its time and
  /// length are the real ones. A road with the same name or number next to an outbound
  /// road (the other carriageway of a motorway or a boulevard, mapped as two oneways)
  /// counts as an outbound road (twin_segments()).
  /// @param from Start.
  /// @param to Destination.
  /// @param mode Who travels.
  /// @return Both routes; back.found = false when there is no way back.
  Vrst_route_pair route_pair(Vrst_latlon from, Vrst_latlon to, Vrst_travel_mode mode) const;

  /// Route from a moving car: its situation() is built, decide() picks the strategy, and
  /// the route starts on the car's road in its direction of travel (unless the strategy
  /// is Any) with U-turns priced by the strategy. Bikes and walkers get route().
  /// @param m Position, heading and speed.
  /// @param to Destination.
  /// @param mode Who travels.
  /// @return The route with its situation and strategy.
  Vrst_route route_moving(const Vrst_motion& m, Vrst_latlon to, Vrst_travel_mode mode) const;

  /// Where a moving car is and what is ahead on its road: the road snapped in the
  /// direction of travel, its type, and the intersections, roundabouts, exits and dead
  /// ends within kAhead_m, following the road through plain nodes and on through
  /// intersections along the same name or number, else the straightest way.
  /// @param m Position, heading and speed.
  /// @param mode Who travels.
  /// @return The situation; found = false when no road is near.
  Vrst_road_situation situation(const Vrst_motion& m, Vrst_travel_mode mode) const;

  /// Cost factor of a road that is not on the outbound route, for the way back.
  static constexpr Vrst_float64 kOffRouteFactor = 3;
  /// Car U-turn at an intersection or dead end outside town, seconds.
  static constexpr Vrst_float64 kUTurnRural_s = 60;
  /// Car U-turn in town, or anywhere under the NoUTurn strategy, seconds: so high that a
  /// route takes one only when there is no other way.
  static constexpr Vrst_float64 kUTurnTown_s = 1800;
  /// situation() looks this far ahead, metres.
  static constexpr Vrst_float64 kAhead_m = 3000;
  /// ... and lists at most this many events.
  static constexpr Vrst_size kMaxEvents = 8;
  /// A segment within this many metres more than the nearest one wins when it runs in
  /// the direction of travel (the car's own carriageway).
  static constexpr Vrst_float64 kAlignSlack_m = 25;
  /// A segment runs in the direction of travel when they differ by at most this, degrees.
  static constexpr Vrst_float64 kAlignDeg = 60;

  /// The nearest road a mode may use.
  /// @param p Point.
  /// @param mode Who travels.
  /// @return The road, or nullopt when none is within about 3 km.
  std::optional<Vrst_road_at> road_at(Vrst_latlon p, Vrst_travel_mode mode) const;

 private:
  /// A point snapped onto a segment.
  struct Vrst_snap {
    Vrst_uint32 seg = 0;           ///< Segment id.
    Vrst_size piece = 0;           ///< Shape piece (points piece and piece + 1) holding the point.
    Vrst_latlon pos;               ///< The point on the segment.
    Vrst_float64 from_a = 0;       ///< Shape length from the segment's first point to pos, metres.
    Vrst_float64 length = 0;       ///< Shape length of the whole segment, metres.
    Vrst_float64 distance_m = 0;   ///< From the asked point to pos.
    Vrst_float64 bearing_deg = 0;  ///< Direction of the piece a -> b, degrees clockwise from north.
    std::optional<bool> heading_fwd;  ///< Snapped with a heading: travel runs a -> b (else b -> a).
  };

  /// How a search may start and turn back.
  struct Vrst_search_rules {
    std::optional<Vrst_float64> heading_deg;  ///< Start only in this direction (cars).
    Vrst_reroute_strategy strategy = Vrst_reroute_strategy::Any;  ///< Sets the U-turn costs.
    std::optional<Vrst_uint32> uturn_vertex;  ///< UTurnAhead: the vertex where turning back is free.
  };

  /// A situation and the graph vertices of its events (for the UTurnAhead vertex).
  struct Vrst_situation_at {
    Vrst_road_situation situation;        ///< The situation.
    std::vector<Vrst_uint32> event_vertex;  ///< Vertex of each event of situation.ahead.
  };

  /// A found route and the segments it runs on, in order.
  struct Vrst_found {
    Vrst_route route;                ///< The route.
    std::vector<Vrst_uint32> segs;   ///< Its segment ids, start to end.
  };

  /// A* from a point to a point; see the file comment.
  /// @param from Start.
  /// @param to Destination.
  /// @param mode Who travels.
  /// @param preferred Sorted segment ids that cost their real time; every other one costs
  ///        kOffRouteFactor times more. Empty = no preference.
  /// @param rules Start direction and U-turn costs.
  /// @return The route and its segments.
  Vrst_found search(Vrst_latlon from, Vrst_latlon to, Vrst_travel_mode mode, std::span<const Vrst_uint32> preferred,
                    const Vrst_search_rules& rules) const;

  /// situation() with the event vertices.
  /// @param m Position, heading and speed.
  /// @param mode Who travels.
  /// @return Situation and vertices.
  Vrst_situation_at situation_at(const Vrst_motion& m, Vrst_travel_mode mode) const;

  /// Cost of turning back at a vertex after arriving on a segment.
  /// @param v Vertex.
  /// @param seg Segment the car arrived on (and would leave on backwards).
  /// @param mode Who travels.
  /// @param rules Strategy and the free U-turn vertex.
  /// @return Seconds; infinity where a U-turn is not allowed.
  Vrst_float64 uturn_seconds(Vrst_uint32 v, Vrst_uint32 seg, Vrst_travel_mode mode, const Vrst_search_rules& rules) const;

  /// Distinct segments a mode may use that meet at a vertex.
  /// @param v Vertex.
  /// @param modes TravelModes bit.
  /// @return Their ids, sorted.
  std::vector<Vrst_uint32> segments_at(Vrst_uint32 v, Vrst_uint32 modes) const;

  /// Roads that meet at a vertex: its distinct neighbours over arcs a mode may use,
  /// counted up to 3 (two segments between the same two vertices count once).
  /// @param v Vertex.
  /// @param modes TravelModes bit.
  /// @return 0, 1, 2, or 3 for three or more.
  Vrst_size junction_size(Vrst_uint32 v, Vrst_uint32 modes) const;

  /// Kind of road of a segment from its class and speed limit alone.
  /// @param seg Segment id.
  /// @return Highway, Town or Rural.
  Vrst_road_type road_type(Vrst_uint32 seg) const;

  /// Whether a point is inside a settlement (near a city, town or village place).
  /// @param p Point.
  /// @return true when it is.
  bool in_settlement(Vrst_latlon p) const;

  /// Direction of a segment at one end, in a direction of travel.
  /// @param seg Segment id.
  /// @param reversed Travel runs b -> a.
  /// @param at_end The last piece in travel order (else the first).
  /// @return Degrees clockwise from north.
  Vrst_float64 seg_bearing(Vrst_uint32 seg, bool reversed, bool at_end) const;

  /// The segments of a route and, in the z14 tiles they cross, the segments with the
  /// same name or road number (the other carriageway of divided roads).
  /// @param segs Segments of the outbound route.
  /// @return Sorted unique segment ids.
  std::vector<Vrst_uint32> twin_segments(std::span<const Vrst_uint32> segs) const;

  /// Nearest point of a segment the mode may use.
  /// With a heading, a segment the mode may use in that direction wins over a nearer one
  /// by up to kAlignSlack_m, and heading_fwd says which way it runs.
  /// @param p Point.
  /// @param modes TravelModes bit of the mode.
  /// @param heading_deg Direction of travel; nullopt = plain nearest.
  /// @return Snap, or nullopt when no such segment is near.
  std::optional<Vrst_snap> snap(Vrst_latlon p, Vrst_uint32 modes, std::optional<Vrst_float64> heading_deg = std::nullopt) const;

  /// Vertices of a mode's main network (largest connected part, arc directions
  /// ignored); computed on first use.
  /// @param mode Who travels.
  /// @return One flag per vertex.
  const std::vector<Vrst_uint8>& main_network(Vrst_travel_mode mode) const;

  /// Whether a segment can be used by a mode in at least one direction.
  /// @param s Segment id.
  /// @param modes TravelModes bit.
  /// @return true when it can.
  bool usable(Vrst_uint32 s, Vrst_uint32 modes) const;

  /// Cost of one arc for a mode.
  /// @param weight Arc weight.
  /// @param seg Segment id of the arc.
  /// @param mode Who travels.
  /// @return Seconds.
  Vrst_float64 arc_seconds(Vrst_uint32 weight, Vrst_uint32 seg, Vrst_travel_mode mode) const;

  /// The arc that runs along a segment in one direction, if the mode may use it.
  /// @param s Segment id.
  /// @param reversed b -> a instead of a -> b.
  /// @param modes TravelModes bit.
  /// @return Index into the forward arcs, or nullopt.
  std::optional<Vrst_uint32> seg_arc(Vrst_uint32 s, bool reversed, Vrst_uint32 modes) const;

  /// Nav files.
  const Vrst_nav_data& m_data;
  /// Tile index, for snapping.
  const Vrst_tile_index& m_index;
  /// main_network() per mode, filled once each.
  mutable std::array<std::vector<Vrst_uint8>, 3> m_main;
  /// Guards the one computation of each m_main entry.
  mutable std::array<std::once_flag, 3> m_main_once;
};

/// TravelModes bit of a travel mode.
/// @param m Mode.
/// @return kModeCar, kModeBike or kModeFoot.
constexpr Vrst_uint32 mode_bit(Vrst_travel_mode m) {
  return m == Vrst_travel_mode::Car ? kModeCar : m == Vrst_travel_mode::Bike ? kModeBike : kModeFoot;
}

}  // namespace nav
