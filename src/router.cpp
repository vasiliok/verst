/// @file
/// Vrst_router: snapping and A*. Documentation is in router.h.
#include "router.h"

#include <algorithm>
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
/// Predecessor markers: the vertex was reached from the start point along the start
/// segment (a -> b or b -> a), not by an arc.
constexpr Vrst_uint32 kFromStartFwd = std::numeric_limits<Vrst_uint32>::max();
constexpr Vrst_uint32 kFromStartRev = kFromStartFwd - 1;  ///< See kFromStartFwd.

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

std::optional<Vrst_router::Vrst_snap> Vrst_router::snap(Vrst_latlon p, Vrst_uint32 modes) const {
  const Vrst_float64 kx = std::cos(p.lat * kPi / 180) * kMetresPerDegree;
  const auto local = [&](Vrst_latlon q) { return Vrst_vec2d{(q.lon - p.lon) * kx, (q.lat - p.lat) * kMetresPerDegree}; };
  const Vrst_travel_mode mode = modes == kModeCar ? Vrst_travel_mode::Car : modes == kModeBike ? Vrst_travel_mode::Bike
                                                                                               : Vrst_travel_mode::Foot;
  const std::vector<Vrst_uint8>& main = main_network(mode);
  const Vrst_tile_id center = tile_at(kSnapZoom, mercator(p));
  const Vrst_float64 tile_m = tile_side(kSnapZoom) * std::cos(p.lat * kPi / 180);  // ground side of a tile
  std::optional<Vrst_snap> best;
  for (int r = 0; r <= kSnapRings; ++r) {
    if (best && best->distance_m < (r - 1) * tile_m) break;  // nothing in ring r can be nearer
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
            if (best && dist >= best->distance_m) continue;
            best = Vrst_snap{s, i, {p.lat + q.y / kMetresPerDegree, p.lon + q.x / kx}, 0, 0, dist};
          }
        }
      }
    }
  }
  if (!best) return std::nullopt;
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
  const auto t0 = std::chrono::steady_clock::now();
  Vrst_route out;
  const Vrst_uint32 mb = mode_bit(mode);
  const std::optional<Vrst_snap> s = snap(from, mb), t = snap(to, mb);
  if (!s || !t) {
    out.error = !s ? "no road near the start" : "no road near the destination";
    return out;
  }
  const Vrst_graph& g = m_data.graph();
  const Vrst_edges& e = m_data.edges();
  const std::span<const Vrst_fwd_arc> arcs = g.fwd_arcs();
  const Vrst_float64 inf = std::numeric_limits<Vrst_float64>::infinity();
  const Vrst_float64 speed = top_speed(mode);
  // Seconds per metre along a segment in one direction, or nullopt when the mode may not.
  const auto per_metre = [&](const Vrst_snap& sn, bool reversed) -> std::optional<Vrst_float64> {
    const std::optional<Vrst_uint32> a = seg_arc(sn.seg, reversed, mb);
    if (!a) return std::nullopt;
    return sn.length > 0 ? arc_seconds(arcs[*a].weight, sn.seg, mode) / sn.length : 0.0;
  };
  const std::optional<Vrst_float64> s_fwd = per_metre(*s, false), s_rev = per_metre(*s, true);
  const std::optional<Vrst_float64> t_fwd = per_metre(*t, false), t_rev = per_metre(*t, true);
  const Vrst_seg_vertices sv = e.vertices()[s->seg], tv = e.vertices()[t->seg];

  std::vector<Vrst_float64> dist(g.num_vertices(), inf);
  std::vector<Vrst_uint32> pred(g.num_vertices(), kFromStartFwd);
  using Vrst_entry = std::pair<Vrst_float64, Vrst_uint32>;  // estimated total, vertex
  std::priority_queue<Vrst_entry, std::vector<Vrst_entry>, std::greater<>> open;
  const auto h = [&](Vrst_uint32 v) { return metres(degrees(g.vertex_coord()[v]), t->pos) / speed; };
  const auto reach = [&](Vrst_uint32 v, Vrst_float64 d, Vrst_uint32 via) {
    if (d >= dist[v]) return;
    dist[v] = d;
    pred[v] = via;
    open.push({d + h(v), v});
  };
  if (s_fwd) reach(sv.b, (s->length - s->from_a) * *s_fwd, kFromStartFwd);
  if (s_rev) reach(sv.a, s->from_a * *s_rev, kFromStartRev);

  // The best way to the destination point so far: straight along a shared segment, or
  // from one end of the destination segment.
  Vrst_float64 best = inf;
  enum class Vrst_end { None, Direct, ViaA, ViaB } end = Vrst_end::None;
  if (t->seg == s->seg) {
    if (s_fwd && t->from_a >= s->from_a) best = (t->from_a - s->from_a) * *s_fwd, end = Vrst_end::Direct;
    if (s_rev && t->from_a <= s->from_a && (s->from_a - t->from_a) * *s_rev < best)
      best = (s->from_a - t->from_a) * *s_rev, end = Vrst_end::Direct;
  }
  while (!open.empty()) {
    const auto [f, v] = open.top();
    open.pop();
    if (f >= best) break;
    if (f > dist[v] + h(v) + 1e-9) continue;  // an older, longer entry
    ++out.settled;
    if (v == tv.a && t_fwd && dist[v] + t->from_a * *t_fwd < best) best = dist[v] + t->from_a * *t_fwd, end = Vrst_end::ViaA;
    if (v == tv.b && t_rev && dist[v] + (t->length - t->from_a) * *t_rev < best)
      best = dist[v] + (t->length - t->from_a) * *t_rev, end = Vrst_end::ViaB;
    const Vrst_uint32 first = g.fwd_first_out()[v];
    const std::span<const Vrst_fwd_arc> out_arcs = g.out_arcs(v);
    for (Vrst_uint32 i = 0; i < out_arcs.size(); ++i) {
      const Vrst_fwd_arc& a = out_arcs[i];
      if (!(arc_modes(a.weight) & mb)) continue;
      reach(a.head, dist[v] + arc_seconds(a.weight, a.seg & kSegIdMask, mode), first + i);
    }
  }
  if (end == Vrst_end::None) {
    out.error = "no route between these points";
    out.ms = std::chrono::duration<Vrst_float64, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return out;
  }

  // Unpack: start piece, the arcs, end piece; shape points in travel order.
  std::vector<Vrst_latlon>& pts = out.points;
  const auto shape = [&](Vrst_uint32 seg) { return m_data.geom().segment_points(seg); };
  const auto add = [&pts](Vrst_latlon p) {
    if (pts.empty() || pts.back().lat != p.lat || pts.back().lon != p.lon) pts.push_back(p);
  };
  pts.push_back(s->pos);
  if (end == Vrst_end::Direct) {
    const std::span<const Vrst_coord> sh = shape(s->seg);
    if (t->from_a >= s->from_a) {
      for (Vrst_size i = s->piece + 1; i <= t->piece; ++i) add(degrees(sh[i]));
    } else {
      for (Vrst_size i = s->piece; i > t->piece; --i) add(degrees(sh[i]));
    }
  } else {
    const Vrst_uint32 last = end == Vrst_end::ViaA ? tv.a : tv.b;
    std::vector<Vrst_uint32> path;  // arcs, last first
    Vrst_uint32 v = last;
    while (pred[v] != kFromStartFwd && pred[v] != kFromStartRev) {
      const Vrst_uint32 arc = pred[v];
      path.push_back(arc);
      const Vrst_seg_vertices av = e.vertices()[arcs[arc].seg & kSegIdMask];
      v = (arcs[arc].seg & kSegReversed) ? av.b : av.a;  // the arc's tail
    }
    const std::span<const Vrst_coord> ss = shape(s->seg);
    if (pred[v] == kFromStartFwd) {
      for (Vrst_size i = s->piece + 1; i < ss.size(); ++i) add(degrees(ss[i]));
    } else {
      for (Vrst_size i = s->piece + 1; i-- > 0;) add(degrees(ss[i]));
    }
    for (auto it = path.rbegin(); it != path.rend(); ++it) {
      const Vrst_uint32 seg = arcs[*it].seg & kSegIdMask;
      const std::span<const Vrst_coord> sh = shape(seg);
      if (arcs[*it].seg & kSegReversed) {
        for (Vrst_size i = sh.size(); i-- > 0;) add(degrees(sh[i]));
      } else {
        for (const Vrst_coord c : sh) add(degrees(c));
      }
    }
    const std::span<const Vrst_coord> ts = shape(t->seg);
    if (end == Vrst_end::ViaA) {
      for (Vrst_size i = 0; i <= t->piece; ++i) add(degrees(ts[i]));
    } else {
      for (Vrst_size i = ts.size() - 1; i > t->piece; --i) add(degrees(ts[i]));
    }
  }
  add(t->pos);
  for (Vrst_size i = 1; i < pts.size(); ++i) out.length_m += metres(pts[i - 1], pts[i]);
  out.time_s = best;
  out.found = true;
  out.ms = std::chrono::duration<Vrst_float64, std::milli>(std::chrono::steady_clock::now() - t0).count();
  return out;
}

}  // namespace nav
