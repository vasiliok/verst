/// @file
/// Vrst_router: snapping, the situation of a moving car and the arc-labelled A*.
/// Documentation is in router.h.
#include "route/router.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace nav {

namespace {

/// Metres per degree of latitude.
constexpr Vrst_float64 kMetresPerDegree = 111320;
/// Zoom of the tiles searched for the nearest road.
constexpr int kSnapZoom = 14;
/// Rings of tiles searched around a point (a z14 tile is about 1.6 km here).
constexpr int kSnapRings = 2;
/// Fastest car travel the heuristic assumes, km/h (no arc is faster, or barely).
constexpr Vrst_float64 kCarTopKmh = 150;
/// Walking speed, km/h.
constexpr Vrst_float64 kFootKmh = 4.8;
/// Cycling speed on a road, km/h; ways of other classes are slower or faster (bike_kmh()).
constexpr Vrst_float64 kBikeKmh = 16;
/// Fastest cycling speed of bike_kmh(), for the heuristic.
constexpr Vrst_float64 kBikeTopKmh = 18;
/// Predecessor marker: the arc is the start piece, from the start point to its head.
constexpr Vrst_uint32 kFromStart = std::numeric_limits<Vrst_uint32>::max();
/// Speed limits up to this are town streets, km/h.
constexpr Vrst_uint8 kTownMaxKmh = 50;
/// A point this close to a place node is in the settlement, metres (city, town, village).
constexpr Vrst_float64 kPlaceRadius_m[] = {3000, 1500, 700};

/// Degrees of a stored coordinate.
/// @param c Coordinate.
/// @return Point.
Vrst_latlon degrees(Vrst_coord c) { return {c.lat_e7 * 1e-7, c.lon_e7 * 1e-7}; }

/// Great-circle distance.
/// @param a First point.
/// @param b Second point.
/// @return Metres.
Vrst_float64 metres(Vrst_latlon a, Vrst_latlon b) {
  const Vrst_float64 r = kPi / 180, dlat = (b.lat - a.lat) * r, dlon = (b.lon - a.lon) * r;
  const Vrst_float64 h = std::sin(dlat / 2) * std::sin(dlat / 2) +
                         std::cos(a.lat * r) * std::cos(b.lat * r) * std::sin(dlon / 2) * std::sin(dlon / 2);
  return 2 * kEarthRadius * std::asin(std::min(1.0, std::sqrt(h)));
}

/// Direction from one point to another (flat, fine over a road piece).
/// @param a From.
/// @param b To.
/// @return Degrees clockwise from north, 0..360.
Vrst_float64 bearing(Vrst_latlon a, Vrst_latlon b) {
  const Vrst_float64 dx = (b.lon - a.lon) * std::cos((a.lat + b.lat) / 2 * kPi / 180), dy = b.lat - a.lat;
  const Vrst_float64 deg = std::atan2(dx, dy) * 180 / kPi;
  return deg < 0 ? deg + 360 : deg;
}

/// Angle between two directions.
/// @param a Degrees.
/// @param b Degrees.
/// @return 0..180.
Vrst_float64 angle_between(Vrst_float64 a, Vrst_float64 b) { return std::abs(std::fmod(a - b + 540.0, 360.0) - 180.0); }

/// Cycling speed on a way of a class.
/// @param c Road class.
/// @return km/h.
Vrst_float64 bike_kmh(RoadClass c) {
  switch (c) {
    case RoadClass::Cycleway: return kBikeTopKmh;
    case RoadClass::Track: return 12;
    case RoadClass::Footway: return 10;
    case RoadClass::Pedestrian: return 8;
    default: return kBikeKmh;
  }
}

/// Top speed of a mode, for the A* heuristic.
/// @param m Mode.
/// @return Metres per second.
Vrst_float64 top_speed(Vrst_travel_mode m) {
  const Vrst_float64 kmh = m == Vrst_travel_mode::Car ? kCarTopKmh : m == Vrst_travel_mode::Bike ? kBikeTopKmh : kFootKmh;
  return kmh / 3.6;
}

/// Whether a class is a motorway or trunk or one of their ramps.
/// @param c Road class.
/// @return true for the highway classes.
bool highway_class(RoadClass c) {
  return c == RoadClass::Motorway || c == RoadClass::MotorwayLink || c == RoadClass::Trunk || c == RoadClass::TrunkLink;
}

/// Whether a class is a ramp (_link).
/// @param c Road class.
/// @return true for the link classes.
bool link_class(RoadClass c) {
  return c == RoadClass::MotorwayLink || c == RoadClass::TrunkLink || c == RoadClass::PrimaryLink ||
         c == RoadClass::SecondaryLink || c == RoadClass::TertiaryLink;
}

/// Labels of the arc search: cost and predecessor per arc, and the arcs a search touched.
/// One per thread, kept between searches: a fresh pair of arrays the size of the graph
/// cost more than the search itself on a short route.
struct Vrst_arc_labels {
  std::vector<Vrst_float64> dist;     ///< Cost to the arc's head having come along it; infinity = not reached.
  std::vector<Vrst_uint32> pred;      ///< Arc before it, or kFromStart.
  std::vector<Vrst_uint32> touched;   ///< Arcs given a label, reset when the search ends.
};

/// Resets the arcs a search touched when it ends, so the next one starts clean.
class Vrst_labels_reset {
 public:
  /// Takes the thread's labels, sized for a graph.
  /// @param arcs Number of arcs of the graph.
  explicit Vrst_labels_reset(Vrst_size arcs) {
    thread_local Vrst_arc_labels labels;
    m_labels = &labels;
    if (labels.dist.size() != arcs) {
      labels.dist.assign(arcs, std::numeric_limits<Vrst_float64>::infinity());
      labels.pred.assign(arcs, kFromStart);
      labels.touched.clear();
    }
  }
  /// Puts the touched arcs back to "not reached".
  ~Vrst_labels_reset() {
    for (const Vrst_uint32 a : m_labels->touched) {
      m_labels->dist[a] = std::numeric_limits<Vrst_float64>::infinity();
      m_labels->pred[a] = kFromStart;
    }
    m_labels->touched.clear();
  }
  Vrst_labels_reset(const Vrst_labels_reset&) = delete;
  Vrst_labels_reset& operator=(const Vrst_labels_reset&) = delete;
  /// The labels.
  /// @return The thread's labels.
  Vrst_arc_labels& labels() const { return *m_labels; }

 private:
  /// The thread's labels (a thread_local object, outlives this).
  Vrst_arc_labels* m_labels;
};

}  // namespace

const std::vector<Vrst_uint8>& Vrst_router::main_network(Vrst_travel_mode mode) const {
  const auto m = static_cast<Vrst_size>(mode);
  std::call_once(m_main_once[m], [this, mode, m] {
    // Union-find over the arcs the mode may use, then the biggest set.
    const Vrst_graph& g = m_data.graph();
    const Vrst_uint32 V = g.num_vertices(), mb = mode_bit(mode);
    std::vector<Vrst_uint32> parent(V);
    for (Vrst_uint32 v = 0; v < V; ++v) parent[v] = v;
    const auto root = [&parent](Vrst_uint32 v) {
      while (parent[v] != v) v = parent[v] = parent[parent[v]];
      return v;
    };
    for (Vrst_uint32 v = 0; v < V; ++v)
      for (const Vrst_fwd_arc& a : g.out_arcs(v))
        if (arc_modes(a.weight) & mb) parent[root(v)] = root(a.head);
    std::vector<Vrst_uint32> size(V, 0);
    Vrst_uint32 biggest = 0;
    for (Vrst_uint32 v = 0; v < V; ++v)
      if (++size[root(v)] > size[biggest]) biggest = root(v);
    std::vector<Vrst_uint8> main(V);
    for (Vrst_uint32 v = 0; v < V; ++v) main[v] = root(v) == biggest;
    m_main[m] = std::move(main);
  });
  return m_main[m];
}

bool Vrst_router::usable(Vrst_uint32 s, Vrst_uint32 modes) const {
  if (m_data.graph().file().header().format_version < 4) return (modes & kModeCar) != 0;  // car-only graph
  return (m_data.edges().attr()[s].flags & (modes << kSegModeShift)) != 0;
}

Vrst_float64 Vrst_router::arc_seconds(Vrst_uint32 weight, Vrst_uint32 seg, Vrst_travel_mode mode) const {
  const Vrst_seg_attr& a = m_data.edges().attr()[seg];
  const auto cls = static_cast<RoadClass>(a.road_class);
  if (mode == Vrst_travel_mode::Car || cls == RoadClass::Ferry) return arc_time(weight) / 10.0;  // a ferry takes its time
  const Vrst_float64 kmh = mode == Vrst_travel_mode::Bike ? bike_kmh(cls) : kFootKmh;
  return a.length_dm / 10.0 / (kmh / 3.6);
}

std::optional<Vrst_uint32> Vrst_router::seg_arc(Vrst_uint32 s, bool reversed, Vrst_uint32 modes) const {
  const Vrst_graph& g = m_data.graph();
  const Vrst_seg_vertices sv = m_data.edges().vertices()[s];
  const Vrst_uint32 tail = reversed ? sv.b : sv.a;
  const Vrst_uint32 first = g.fwd_first_out()[tail];
  const std::span<const Vrst_fwd_arc> arcs = g.out_arcs(tail);
  for (Vrst_uint32 i = 0; i < arcs.size(); ++i)
    if ((arcs[i].seg & kSegIdMask) == s && ((arcs[i].seg & kSegReversed) != 0) == reversed &&
        (arc_modes(arcs[i].weight) & modes))
      return first + i;
  return std::nullopt;
}

std::vector<Vrst_uint32> Vrst_router::segments_at(Vrst_uint32 v, Vrst_uint32 modes) const {
  const Vrst_graph& g = m_data.graph();
  std::vector<Vrst_uint32> segs;
  for (const Vrst_fwd_arc& a : g.out_arcs(v))
    if (arc_modes(a.weight) & modes) segs.push_back(a.seg & kSegIdMask);
  for (const Vrst_bwd_arc& a : g.in_arcs(v))
    if (arc_modes(a.weight) & modes) segs.push_back(g.fwd_arcs()[a.fwd_arc].seg & kSegIdMask);
  std::ranges::sort(segs);
  segs.erase(std::ranges::unique(segs).begin(), segs.end());
  return segs;
}

Vrst_size Vrst_router::junction_size(Vrst_uint32 v, Vrst_uint32 modes) const {
  // Up to three distinct neighbours, without allocating and without leaving the two arc
  // rows of v (an incoming arc has its tail): this runs for every U-turn the search
  // considers. Two segments between the same two vertices count once.
  std::array<Vrst_uint32, 3> seen{};
  Vrst_size n = 0;
  const auto see = [&](Vrst_uint32 u) {
    if (std::find(seen.begin(), seen.begin() + n, u) == seen.begin() + n) seen[n++] = u;
    return n == seen.size();
  };
  const Vrst_graph& g = m_data.graph();
  for (const Vrst_fwd_arc& a : g.out_arcs(v))
    if ((arc_modes(a.weight) & modes) && see(a.head)) return n;
  for (const Vrst_bwd_arc& a : g.in_arcs(v))
    if ((arc_modes(a.weight) & modes) && see(a.tail)) return n;
  return n;
}

Vrst_road_type Vrst_router::road_type(Vrst_uint32 seg) const {
  const Vrst_seg_attr& a = m_data.edges().attr()[seg];
  const auto cls = static_cast<RoadClass>(a.road_class);
  if (highway_class(cls)) return Vrst_road_type::Highway;
  if (cls == RoadClass::Residential || cls == RoadClass::LivingStreet || cls == RoadClass::Service ||
      (a.maxspeed_kmh > 0 && a.maxspeed_kmh <= kTownMaxKmh))
    return Vrst_road_type::Town;
  return Vrst_road_type::Rural;
}

bool Vrst_router::in_settlement(Vrst_latlon p) const {
  for (const Vrst_place& pl : m_data.names().places()) {
    if (pl.kind >= std::size(kPlaceRadius_m)) continue;  // hamlets are not towns
    const Vrst_float64 r = kPlaceRadius_m[pl.kind];
    const Vrst_latlon q = degrees(pl.pos);
    if (std::abs(q.lat - p.lat) * kMetresPerDegree > r) continue;  // cheap reject first
    if (metres(p, q) <= r) return true;
  }
  return false;
}

Vrst_float64 Vrst_router::seg_bearing(Vrst_uint32 seg, bool reversed, bool at_end) const {
  const std::span<const Vrst_coord> pts = m_data.geom().segment_points(seg);
  const Vrst_size n = pts.size();
  if (n < 2) return 0;
  // In travel order the first piece is (0, 1), the last (n-2, n-1); reversed swaps the ends.
  const bool tail_piece = at_end != reversed;
  const Vrst_latlon p = degrees(pts[tail_piece ? n - 2 : 0]), q = degrees(pts[tail_piece ? n - 1 : 1]);
  return reversed ? bearing(q, p) : bearing(p, q);
}

Vrst_float64 Vrst_router::uturn_seconds(Vrst_uint32 v, Vrst_uint32 seg, Vrst_travel_mode mode,
                                        const Vrst_search_rules& rules) const {
  if (mode != Vrst_travel_mode::Car) return 0;  // a walker or cyclist turns anywhere
  const Vrst_float64 never = std::numeric_limits<Vrst_float64>::infinity();
  if (junction_size(v, kModeCar) == 2) return never;  // a plain road node, no intersection
  const Vrst_road_type type = road_type(seg);
  if (type == Vrst_road_type::Highway) return never;
  if (rules.uturn_vertex == v) return 0;
  if (rules.strategy == Vrst_reroute_strategy::NoUTurn || type == Vrst_road_type::Town) return kUTurnTown_s;
  return kUTurnRural_s;
}

std::optional<Vrst_router::Vrst_snap> Vrst_router::snap(Vrst_latlon p, Vrst_uint32 modes,
                                                        std::optional<Vrst_float64> heading_deg) const {
  const Vrst_float64 kx = std::cos(p.lat * kPi / 180) * kMetresPerDegree;
  const auto local = [&](Vrst_latlon q) { return Vrst_vec2d{(q.lon - p.lon) * kx, (q.lat - p.lat) * kMetresPerDegree}; };
  const Vrst_travel_mode mode = modes == kModeCar ? Vrst_travel_mode::Car : modes == kModeBike ? Vrst_travel_mode::Bike
                                                                                               : Vrst_travel_mode::Foot;
  const std::vector<Vrst_uint8>& main = main_network(mode);
  const Vrst_tile_id center = tile_at(kSnapZoom, mercator(p));
  const Vrst_float64 tile_m = tile_side(kSnapZoom) * std::cos(p.lat * kPi / 180);  // ground side of a tile
  std::optional<Vrst_snap> best, aligned;  // nearest, and nearest in the direction of travel
  for (int r = 0; r <= kSnapRings; ++r) {
    if (best && best->distance_m + kAlignSlack_m < (r - 1) * tile_m) break;  // nothing in ring r can be nearer
    for (int dx = -r; dx <= r; ++dx) {
      for (int dy = -r; dy <= r; ++dy) {
        if (std::max(std::abs(dx), std::abs(dy)) != r) continue;  // only the ring itself
        const Vrst_tile_id t{kSnapZoom, center.x + dx, center.y + dy};
        if (!m_index.contains(t)) continue;
        for (const Vrst_uint32 s : m_index.find(t).segs) {
          if (!usable(s, modes) || !main[m_data.edges().vertices()[s].a]) continue;
          const std::span<const Vrst_coord> pts = m_data.geom().segment_points(s);
          for (Vrst_size i = 0; i + 1 < pts.size(); ++i) {
            const Vrst_vec2d a = local(degrees(pts[i])), b = local(degrees(pts[i + 1]));
            const Vrst_vec2d d{b.x - a.x, b.y - a.y};
            const Vrst_float64 len2 = d.x * d.x + d.y * d.y;
            const Vrst_float64 u = len2 > 0 ? std::clamp(-(a.x * d.x + a.y * d.y) / len2, 0.0, 1.0) : 0.0;
            const Vrst_vec2d q{a.x + d.x * u, a.y + d.y * u};
            const Vrst_float64 dist = std::hypot(q.x, q.y);
            const bool nearer = !best || dist < best->distance_m;
            const bool nearer_aligned = heading_deg && (!aligned || dist < aligned->distance_m);
            if (!nearer && !nearer_aligned) continue;
            Vrst_float64 deg = std::atan2(d.x, d.y) * 180 / kPi;
            if (deg < 0) deg += 360;
            const Vrst_snap sn{s, i, {p.lat + q.y / kMetresPerDegree, p.lon + q.x / kx}, 0, 0, dist, deg, std::nullopt};
            if (nearer) best = sn;
            if (nearer_aligned) {
              const Vrst_float64 off = angle_between(*heading_deg, deg);
              if (off <= kAlignDeg && seg_arc(s, false, modes)) aligned = sn, aligned->heading_fwd = true;
              if (off >= 180 - kAlignDeg && seg_arc(s, true, modes)) aligned = sn, aligned->heading_fwd = false;
            }
          }
        }
      }
    }
  }
  if (!best) return std::nullopt;
  if (heading_deg) {
    if (aligned && aligned->distance_m <= best->distance_m + kAlignSlack_m) {
      best = aligned;
    } else {
      // Nothing runs that way (a noisy course): the nearest, in its nearer direction.
      const bool fwd = angle_between(*heading_deg, best->bearing_deg) < 90;
      best->heading_fwd = seg_arc(best->seg, !fwd, modes) ? fwd : !fwd;
    }
  }
  // Shape lengths: to the snapped point and of the whole segment.
  const std::span<const Vrst_coord> pts = m_data.geom().segment_points(best->seg);
  for (Vrst_size i = 0; i + 1 < pts.size(); ++i) {
    const Vrst_float64 piece = metres(degrees(pts[i]), degrees(pts[i + 1]));
    if (i < best->piece) best->from_a += piece;
    if (i == best->piece) best->from_a += metres(degrees(pts[i]), best->pos);
    best->length += piece;
  }
  return best;
}

std::optional<Vrst_road_at> Vrst_router::road_at(Vrst_latlon p, Vrst_travel_mode mode) const {
  const std::optional<Vrst_snap> s = snap(p, mode_bit(mode));
  if (!s) return std::nullopt;
  const Vrst_seg_attr& a = m_data.edges().attr()[s->seg];
  std::string name(m_data.names().get(a.name_id));
  if (name.empty()) name = m_data.names().get(a.ref_id);
  return Vrst_road_at{s->pos, std::move(name), s->distance_m};
}

Vrst_route Vrst_router::route(Vrst_latlon from, Vrst_latlon to, Vrst_travel_mode mode) const {
  return search(from, to, mode, {}, {}).route;
}

Vrst_route_pair Vrst_router::route_pair(Vrst_latlon from, Vrst_latlon to, Vrst_travel_mode mode) const {
  Vrst_found there = search(from, to, mode, {}, {});
  if (!there.route.found) return {std::move(there.route), {}};
  const std::vector<Vrst_uint32> prefer = twin_segments(there.segs);
  return {std::move(there.route), search(to, from, mode, prefer, {}).route};
}

Vrst_route Vrst_router::route_moving(const Vrst_motion& m, Vrst_latlon to, Vrst_travel_mode mode) const {
  if (mode != Vrst_travel_mode::Car) return route(m.pos, to, mode);
  const Vrst_situation_at at = situation_at(m, mode);
  const Vrst_reroute_plan plan = decide(at.situation);
  Vrst_search_rules rules;
  rules.strategy = plan.strategy;
  if (plan.strategy != Vrst_reroute_strategy::Any) rules.heading_deg = m.heading_deg;
  if (plan.uturn_event) rules.uturn_vertex = at.event_vertex[*plan.uturn_event];
  Vrst_route r = search(m.pos, to, mode, {}, rules).route;
  r.situation = at.situation;
  r.strategy = plan.strategy;
  return r;
}

Vrst_road_situation Vrst_router::situation(const Vrst_motion& m, Vrst_travel_mode mode) const {
  return situation_at(m, mode).situation;
}

Vrst_router::Vrst_situation_at Vrst_router::situation_at(const Vrst_motion& m, Vrst_travel_mode mode) const {
  Vrst_situation_at out;
  Vrst_road_situation& sit = out.situation;
  const Vrst_uint32 mb = mode_bit(mode);
  const std::optional<Vrst_snap> s = snap(m.pos, mb, m.heading_deg);
  if (!s) return out;
  const Vrst_graph& g = m_data.graph();
  const Vrst_edges& e = m_data.edges();
  const std::span<const Vrst_seg_attr> attr = e.attr();
  sit.found = true;
  sit.motion = {s->pos, m.heading_deg, m.speed_kmh};
  sit.type = road_type(s->seg);
  if (sit.type == Vrst_road_type::Rural && in_settlement(s->pos)) sit.type = Vrst_road_type::Town;
  sit.road = m_data.names().get(attr[s->seg].name_id);
  if (sit.road.empty()) sit.road = m_data.names().get(attr[s->seg].ref_id);

  // Walk the road forward: through plain nodes, on through intersections along the same
  // name or number, else the straightest way.
  Vrst_uint32 seg = s->seg;
  bool reversed = !s->heading_fwd.value_or(true);
  Vrst_float64 ahead = reversed ? s->from_a : s->length - s->from_a;
  const Vrst_search_rules plain;
  while (ahead < kAhead_m && sit.ahead.size() < kMaxEvents) {
    const Vrst_seg_vertices sv = e.vertices()[seg];
    const Vrst_uint32 v = reversed ? sv.a : sv.b;
    const std::vector<Vrst_uint32> segs = segments_at(v, mb);
    const Vrst_latlon at = degrees(g.vertex_coord()[v]);
    const auto cls = static_cast<RoadClass>(attr[seg].road_class);
    const auto event = [&](Vrst_road_event_kind kind, bool uturn_ok) {
      sit.ahead.push_back({kind, ahead, uturn_ok, at});
      out.event_vertex.push_back(v);
    };
    if (segs.size() <= 1) {
      event(Vrst_road_event_kind::DeadEnd, true);
      break;
    }
    if (segs.size() >= 3) {
      const auto other_is = [&](auto pred) {
        return std::ranges::any_of(segs, [&](Vrst_uint32 o) { return o != seg && pred(o); });
      };
      const bool in_roundabout = attr[seg].flags & kSegRoundabout;
      if (!in_roundabout && other_is([&](Vrst_uint32 o) { return (attr[o].flags & kSegRoundabout) != 0; })) {
        event(Vrst_road_event_kind::Roundabout, true);
        break;  // where the car leaves the roundabout is its choice
      }
      const bool uturn_ok = std::isfinite(uturn_seconds(v, seg, mode, plain));
      if (!link_class(cls) && other_is([&](Vrst_uint32 o) { return link_class(static_cast<RoadClass>(attr[o].road_class)); })) {
        event(Vrst_road_event_kind::Interchange, uturn_ok);
      } else {
        event(Vrst_road_event_kind::Intersection, uturn_ok);
      }
    }
    // The way on: an arc out of v on another segment.
    const Vrst_float64 in_deg = seg_bearing(seg, reversed, true);
    std::optional<Vrst_fwd_arc> next;
    Vrst_float64 next_score = std::numeric_limits<Vrst_float64>::infinity();
    for (const Vrst_fwd_arc& a : g.out_arcs(v)) {
      const Vrst_uint32 o = a.seg & kSegIdMask;
      if (o == seg || !(arc_modes(a.weight) & mb)) continue;
      const bool same_road = (attr[seg].name_id && attr[o].name_id == attr[seg].name_id) ||
                             (attr[seg].ref_id && attr[o].ref_id == attr[seg].ref_id);
      const Vrst_float64 score = angle_between(in_deg, seg_bearing(o, (a.seg & kSegReversed) != 0, false)) -
                                 (same_road ? 360 : 0);
      if (score < next_score) next = a, next_score = score;
    }
    if (!next) break;  // only a way back (a oneway's end)
    seg = next->seg & kSegIdMask;
    reversed = (next->seg & kSegReversed) != 0;
    ahead += attr[seg].length_dm / 10.0;
  }
  return out;
}

std::vector<Vrst_uint32> Vrst_router::twin_segments(std::span<const Vrst_uint32> segs) const {
  const std::span<const Vrst_seg_attr> attr = m_data.edges().attr();
  std::vector<Vrst_uint32> out(segs.begin(), segs.end());
  // Names and numbers of the route's roads, and the tiles it crosses.
  std::vector<Vrst_uint32> names, refs;
  std::vector<Vrst_uint64> tiles;
  for (const Vrst_uint32 s : segs) {
    if (attr[s].name_id) names.push_back(attr[s].name_id);
    if (attr[s].ref_id) refs.push_back(attr[s].ref_id);
    for (const Vrst_coord c : m_data.geom().segment_points(s)) tiles.push_back(tile_key(tile_at(kSnapZoom, mercator(degrees(c)))));
  }
  const auto sort_unique = [](auto& v) {
    std::ranges::sort(v);
    v.erase(std::ranges::unique(v).begin(), v.end());
  };
  sort_unique(names);
  sort_unique(refs);
  sort_unique(tiles);
  for (const Vrst_uint64 k : tiles) {
    const Vrst_tile_id t = tile_of_key(k);
    if (!m_index.contains(t)) continue;
    for (const Vrst_uint32 c : m_index.find(t).segs)
      if ((attr[c].ref_id && std::ranges::binary_search(refs, attr[c].ref_id)) ||
          (attr[c].name_id && std::ranges::binary_search(names, attr[c].name_id)))
        out.push_back(c);
  }
  sort_unique(out);
  return out;
}

Vrst_router::Vrst_found Vrst_router::search(Vrst_latlon from, Vrst_latlon to, Vrst_travel_mode mode,
                                            std::span<const Vrst_uint32> preferred, const Vrst_search_rules& rules) const {
  const auto t0 = std::chrono::steady_clock::now();
  Vrst_found found;
  Vrst_route& out = found.route;
  const Vrst_uint32 mb = mode_bit(mode);
  const std::optional<Vrst_snap> s = snap(from, mb, rules.heading_deg), t = snap(to, mb);
  if (!s || !t) {
    out.error = !s ? "no road near the start" : "no road near the destination";
    return found;
  }
  const Vrst_graph& g = m_data.graph();
  const Vrst_edges& e = m_data.edges();
  const std::span<const Vrst_fwd_arc> arcs = g.fwd_arcs();
  const Vrst_float64 inf = std::numeric_limits<Vrst_float64>::infinity();
  const Vrst_float64 speed = top_speed(mode);
  // Search cost of real seconds on a segment: raised off the preferred roads.
  const auto off = [&](Vrst_uint32 seg) {
    return !preferred.empty() && !std::ranges::binary_search(preferred, seg) ? kOffRouteFactor : 1.0;
  };
  // Real seconds per metre along a segment in one direction, or nullopt when the mode may not.
  const auto per_metre = [&](const Vrst_snap& sn, bool reversed) -> std::optional<Vrst_float64> {
    const std::optional<Vrst_uint32> a = seg_arc(sn.seg, reversed, mb);
    if (!a) return std::nullopt;
    return sn.length > 0 ? arc_seconds(arcs[*a].weight, sn.seg, mode) / sn.length : 0.0;
  };
  std::optional<Vrst_float64> s_fwd = per_metre(*s, false), s_rev = per_metre(*s, true);
  if (rules.heading_deg && s->heading_fwd) {  // start in the direction of travel only
    if (*s->heading_fwd) s_rev.reset();
    else s_fwd.reset();
  }
  const std::optional<Vrst_float64> t_fwd = per_metre(*t, false), t_rev = per_metre(*t, true);
  const Vrst_seg_vertices tv = e.vertices()[t->seg];

  // Labels per arc: the cost to the arc's head having come along it, so the arc a car
  // arrived on is known when it turns.
  const Vrst_labels_reset reset(g.num_arcs());
  std::vector<Vrst_float64>& dist = reset.labels().dist;
  std::vector<Vrst_uint32>& pred = reset.labels().pred;
  std::vector<Vrst_uint32>& touched = reset.labels().touched;
  using Vrst_entry = std::pair<Vrst_float64, Vrst_uint32>;  // estimated total, arc
  std::priority_queue<Vrst_entry, std::vector<Vrst_entry>, std::greater<>> open;
  const auto h = [&](Vrst_uint32 v) { return metres(degrees(g.vertex_coord()[v]), t->pos) / speed; };
  const auto reach = [&](Vrst_uint32 arc, Vrst_float64 d, Vrst_uint32 via) {
    if (d >= dist[arc]) return;
    if (dist[arc] == inf) touched.push_back(arc);
    dist[arc] = d;
    pred[arc] = via;
    open.push({d + h(arcs[arc].head), arc});
  };
  const Vrst_float64 s_off = off(s->seg), t_off = off(t->seg);
  // The start pieces: from the start point to either end of its segment.
  if (s_fwd) reach(*seg_arc(s->seg, false, mb), (s->length - s->from_a) * *s_fwd * s_off, kFromStart);
  if (s_rev) reach(*seg_arc(s->seg, true, mb), s->from_a * *s_rev * s_off, kFromStart);

  // The best way to the destination point so far: straight along a shared segment, or
  // along the destination segment after an arc.
  Vrst_float64 best = inf;
  enum class Vrst_end { None, Direct, ViaA, ViaB } end = Vrst_end::None;
  Vrst_uint32 last_arc = 0;
  if (t->seg == s->seg) {
    if (s_fwd && t->from_a >= s->from_a) best = (t->from_a - s->from_a) * *s_fwd * s_off, end = Vrst_end::Direct;
    if (s_rev && t->from_a <= s->from_a && (s->from_a - t->from_a) * *s_rev * s_off < best)
      best = (s->from_a - t->from_a) * *s_rev * s_off, end = Vrst_end::Direct;
  }
  while (!open.empty()) {
    const auto [f, a] = open.top();
    open.pop();
    if (f >= best) break;
    const Vrst_uint32 v = arcs[a].head;
    if (f > dist[a] + h(v) + 1e-9) continue;  // an older, longer entry
    ++out.settled;
    const Vrst_uint32 sa = arcs[a].seg & kSegIdMask;
    // On along the destination segment; turning back onto it is a U-turn.
    const Vrst_float64 turn_t = sa == t->seg ? uturn_seconds(v, sa, mode, rules) : 0;
    if (v == tv.a && t_fwd && dist[a] + turn_t + t->from_a * *t_fwd * t_off < best)
      best = dist[a] + turn_t + t->from_a * *t_fwd * t_off, end = Vrst_end::ViaA, last_arc = a;
    if (v == tv.b && t_rev && dist[a] + turn_t + (t->length - t->from_a) * *t_rev * t_off < best)
      best = dist[a] + turn_t + (t->length - t->from_a) * *t_rev * t_off, end = Vrst_end::ViaB, last_arc = a;
    const Vrst_uint32 first = g.fwd_first_out()[v];
    const std::span<const Vrst_fwd_arc> out_arcs = g.out_arcs(v);
    for (Vrst_uint32 i = 0; i < out_arcs.size(); ++i) {
      const Vrst_fwd_arc& b = out_arcs[i];
      if (!(arc_modes(b.weight) & mb)) continue;
      const Vrst_uint32 seg = b.seg & kSegIdMask;
      const Vrst_float64 turn = seg == sa ? uturn_seconds(v, sa, mode, rules) : 0;
      if (!std::isfinite(turn)) continue;
      reach(first + i, dist[a] + turn + arc_seconds(b.weight, seg, mode) * off(seg), a);
    }
  }
  if (end == Vrst_end::None) {
    out.error = "no route between these points";
    out.ms = std::chrono::duration<Vrst_float64, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return found;
  }
  // Real time and the metres on preferred roads, piece by piece as the path is unpacked.
  Vrst_float64 real = 0;
  const auto piece = [&](Vrst_uint32 seg, Vrst_float64 seconds, Vrst_float64 m) {
    real += seconds;
    if (!preferred.empty() && off(seg) == 1.0) out.shared_m += m;
    if (found.segs.empty() || found.segs.back() != seg) found.segs.push_back(seg);
  };

  // Unpack: start piece, the arcs, end piece; shape points in travel order.
  std::vector<Vrst_latlon>& pts = out.points;
  const auto shape = [&](Vrst_uint32 seg) { return m_data.geom().segment_points(seg); };
  const auto add = [&pts](Vrst_latlon p) {
    if (pts.empty() || pts.back().lat != p.lat || pts.back().lon != p.lon) pts.push_back(p);
  };
  pts.push_back(s->pos);
  if (end == Vrst_end::Direct) {
    const std::span<const Vrst_coord> sh = shape(s->seg);
    const Vrst_float64 m = std::abs(t->from_a - s->from_a);
    piece(s->seg, m * (t->from_a >= s->from_a ? *s_fwd : *s_rev), m);
    if (t->from_a >= s->from_a) {
      for (Vrst_size i = s->piece + 1; i <= t->piece; ++i) add(degrees(sh[i]));
    } else {
      for (Vrst_size i = s->piece; i > t->piece; --i) add(degrees(sh[i]));
    }
  } else {
    std::vector<Vrst_uint32> path;  // arcs after the start piece, last first
    Vrst_uint32 a = last_arc;
    while (pred[a] != kFromStart) {
      path.push_back(a);
      a = pred[a];
    }
    const bool start_fwd = (arcs[a].seg & kSegReversed) == 0;  // a is the start piece
    Vrst_uint32 prev = s->seg;
    for (auto it = path.rbegin(); it != path.rend(); ++it) {
      const Vrst_uint32 seg = arcs[*it].seg & kSegIdMask;
      if (seg == prev) ++out.uturns;
      prev = seg;
    }
    if (prev == t->seg) ++out.uturns;  // came along the destination segment and turns back onto it
    const std::span<const Vrst_coord> ss = shape(s->seg);
    const Vrst_float64 sm = start_fwd ? s->length - s->from_a : s->from_a;
    piece(s->seg, sm * (start_fwd ? *s_fwd : *s_rev), sm);
    if (start_fwd) {
      for (Vrst_size i = s->piece + 1; i < ss.size(); ++i) add(degrees(ss[i]));
    } else {
      for (Vrst_size i = s->piece + 1; i-- > 0;) add(degrees(ss[i]));
    }
    for (auto it = path.rbegin(); it != path.rend(); ++it) {
      const Vrst_uint32 seg = arcs[*it].seg & kSegIdMask;
      piece(seg, arc_seconds(arcs[*it].weight, seg, mode), e.attr()[seg].length_dm / 10.0);
      const std::span<const Vrst_coord> sh = shape(seg);
      if (arcs[*it].seg & kSegReversed) {
        for (Vrst_size i = sh.size(); i-- > 0;) add(degrees(sh[i]));
      } else {
        for (const Vrst_coord c : sh) add(degrees(c));
      }
    }
    const std::span<const Vrst_coord> ts = shape(t->seg);
    const Vrst_float64 tm = end == Vrst_end::ViaA ? t->from_a : t->length - t->from_a;
    piece(t->seg, tm * (end == Vrst_end::ViaA ? *t_fwd : *t_rev), tm);
    if (end == Vrst_end::ViaA) {
      for (Vrst_size i = 0; i <= t->piece; ++i) add(degrees(ts[i]));
    } else {
      for (Vrst_size i = ts.size() - 1; i > t->piece; --i) add(degrees(ts[i]));
    }
  }
  add(t->pos);
  for (Vrst_size i = 1; i < pts.size(); ++i) out.length_m += metres(pts[i - 1], pts[i]);
  out.time_s = real;
  out.found = true;
  out.ms = std::chrono::duration<Vrst_float64, std::milli>(std::chrono::steady_clock::now() - t0).count();
  return found;
}

}  // namespace nav
