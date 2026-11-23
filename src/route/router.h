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

  /// Cost factor of a road that is not on the outbound route, for the way back.
  static constexpr Vrst_float64 kOffRouteFactor = 3;

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
  /// @return The route and its segments.
  Vrst_found search(Vrst_latlon from, Vrst_latlon to, Vrst_travel_mode mode, std::span<const Vrst_uint32> preferred) const;

  /// The segments of a route and, in the z14 tiles they cross, the segments with the
  /// same name or road number (the other carriageway of divided roads).
  /// @param segs Segments of the outbound route.
  /// @return Sorted unique segment ids.
  std::vector<Vrst_uint32> twin_segments(std::span<const Vrst_uint32> segs) const;

  /// Nearest point of a segment the mode may use.
  /// @param p Point.
  /// @param modes TravelModes bit of the mode.
  /// @return Snap, or nullopt when no such segment is near.
  std::optional<Vrst_snap> snap(Vrst_latlon p, Vrst_uint32 modes) const;

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
