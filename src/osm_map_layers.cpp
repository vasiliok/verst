/// @file
/// Vrst_map_layer_builder: reading water, forest, buildings and railways from OSM; tag rules.
/// Declarations and documentation are in osm_map_layers.h.
#include "osm_map_layers.h"

#include <algorithm>
#include <mapbox/earcut.hpp>
#include <numbers>
#include <osmium/area/assembler.hpp>
#include <osmium/area/multipolygon_manager.hpp>
#include <osmium/handler/node_locations_for_ways.hpp>
#include <osmium/index/map/flex_mem.hpp>
#include <osmium/io/pbf_input.hpp>
#include <osmium/relations/manager_util.hpp>
#include <osmium/relations/relations_manager.hpp>
#include <osmium/tags/tags_filter.hpp>
#include <osmium/visitor.hpp>

namespace nav {

void Vrst_map_layer_builder::read(const std::string& path) {
  const osmium::area::Assembler::config_type cfg;
  osmium::TagsFilter filter{false};
  for (const std::string k : {"building", "landuse", "natural", "leisure", "waterway"}) filter.add_rule(true, k);
  osmium::area::MultipolygonManager<osmium::area::Assembler> mp{cfg, filter};
  osmium::relations::read_relations(osmium::io::File{path}, mp);

  using Index = osmium::index::map::FlexMem<osmium::unsigned_object_id_type, osmium::Location>;
  Index index;
  osmium::handler::NodeLocationsForWays<Index> locations{index};
  locations.ignore_errors();
  Vrst_handler handler{*this};
  osmium::io::Reader reader{path};
  osmium::apply(reader, locations, handler,
                mp.handler([&handler](osmium::memory::Buffer&& buffer) { osmium::apply(buffer, handler); }));
  reader.close();
}

void Vrst_map_layer_builder::finish() {
  std::vector<Vrst_uint32> order(m_areas.size());
  std::iota(order.begin(), order.end(), 0);
  std::ranges::stable_sort(order, [&](Vrst_uint32 a, Vrst_uint32 b) { return m_areas[a].kind < m_areas[b].kind; });
  Vrst_map_layer_data& d = m_out;
  for (const Vrst_uint32 i : order) {
    const Vrst_area_tmp& a = m_areas[i];
    d.area_kinds.push_back(static_cast<Vrst_uint8>(a.kind));
    d.area_outer_size.push_back(a.outer_size);
    d.area_points.insert(d.area_points.end(), a.points.begin(), a.points.end());
    d.area_indices.insert(d.area_indices.end(), a.indices.begin(), a.indices.end());
    d.area_first_point.push_back(static_cast<Vrst_uint32>(d.area_points.size()));
    d.area_first_index.push_back(static_cast<Vrst_uint32>(d.area_indices.size()));
  }
  std::vector<Vrst_area_tmp>().swap(m_areas);
}

std::vector<Vrst_map_layer_builder::Vrst_local> Vrst_map_layer_builder::simplify(const std::vector<Vrst_local>& pts,
                                                                                 Vrst_float64 tol) {
  // Douglas-Peucker, iterative: an explicit stack instead of recursion, so an outline with
  // 100 000 points cannot overflow the call stack.
  //  - keep[] marks the points that survive; the two ends always do.
  //  - Each stack entry is a span [i, j] whose two ends are kept. Find the point between
  //    them farthest from the straight line i-j: the perpendicular distance, or the plain
  //    distance to i when i and j are the same point (a closed ring starts and ends there).
  //  - Farther than tol: the line i-j would cut off a real detail of the shape, so keep
  //    that point and check both halves [i, at] and [at, j]. Otherwise every point between
  //    i and j lies within tol of the line and is dropped.
  // Cost: O(n log n) on usual shapes, O(n^2) at worst (a spiral). Points are local metres
  // (Vrst_local), so tol is metres on the ground.
  // The result keeps the shape within tol but not its topology: a simplified ring can
  // cross itself, which earcut handles only slowly (see docs/storage, drawing data).
  if (pts.size() <= 2) return pts;
  std::vector<Vrst_uint8> keep(pts.size(), 0);
  keep.front() = keep.back() = 1;
  std::vector<std::pair<Vrst_size, Vrst_size>> stack{{0, pts.size() - 1}};
  while (!stack.empty()) {
    const auto [i, j] = stack.back();
    stack.pop_back();
    const Vrst_local a = pts[i], b = pts[j];
    // (dx, dy): the line a->b that would replace the points between; len2: its squared length.
    // A point's distance to the line is |cross(p - a, b - a)| / sqrt(len2);
    // len2 == 0 when a and b coincide (closed ring), then the plain distance to a is used.
    const Vrst_float64 dx = b.x - a.x, dy = b.y - a.y, len2 = dx * dx + dy * dy;
    Vrst_float64 best = -1;
    Vrst_size at = i;
    for (Vrst_size k = i + 1; k < j; ++k) {
      // (px, py): vector from a to the candidate point k.
      const Vrst_float64 px = pts[k].x - a.x, py = pts[k].y - a.y;
      // Its distance to the line a->b: cross product / line length; to a itself when a == b.
      const Vrst_float64 d = len2 > 0 ? std::abs(px * dy - py * dx) / std::sqrt(len2) : std::hypot(px, py);
      // Remember the farthest point so far: the one that may have to be kept.
      if (d > best) best = d, at = k;
    }
    if (best > tol) {
      keep[at] = 1;
      stack.emplace_back(i, at);
      stack.emplace_back(at, j);
    }
  }
  std::vector<Vrst_local> out;
  for (Vrst_size k = 0; k < pts.size(); ++k)
    if (keep[k]) out.push_back(pts[k]);
  return out;
}

Vrst_float64 Vrst_map_layer_builder::ring_area(const std::vector<Vrst_local>& r) {
  Vrst_float64 s = 0;
  for (Vrst_size i = 0, j = r.size() - 1; i < r.size(); j = i++) 
  {
    s += r[j].x * r[i].y - r[i].x * r[j].y;
  }
  return s / 2;
}

std::vector<Vrst_map_layer_builder::Vrst_local> Vrst_map_layer_builder::min_rectangle(std::vector<Vrst_local> r) {
  // 1. Convex hull, Andrew's monotone chain: sort the points left to right, then walk them
  //    twice (lower hull left to right, upper hull back) keeping only left turns.
  std::ranges::sort(r, [](Vrst_local a, Vrst_local b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
  // cross > 0: o -> a -> b turns left (counter-clockwise); <= 0: right turn or straight.
  const auto cross = [](Vrst_local o, Vrst_local a, Vrst_local b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
  };
  std::vector<Vrst_local> h(2 * r.size());  // hull under construction, k points used
  Vrst_size k = 0;
  // Lower hull: drop the last hull point while it would make a right turn.
  for (const Vrst_local p : r) {
    while (k >= 2 && cross(h[k - 2], h[k - 1], p) <= 0) --k;
    h[k++] = p;
  }
  // Upper hull: same, walking back; t keeps the lower hull from being popped.
  for (Vrst_size i = r.size() - 1, t = k + 1; i-- > 0;) {
    while (k >= t && cross(h[k - 2], h[k - 1], r[i]) <= 0) --k;
    h[k++] = r[i];
  }
  h.resize(k > 1 ? k - 1 : k);  // the last point repeats the first
  // 2. One rectangle per hull edge, its sides along and across the edge; keep the smallest.
  Vrst_float64 best = 1e300;
  std::vector<Vrst_local> out(4);
  for (Vrst_size i = 0; i < h.size(); ++i) {
    const Vrst_local a = h[i], b = h[(i + 1) % h.size()];
    const Vrst_float64 len = std::hypot(b.x - a.x, b.y - a.y);
    if (len <= 0) continue;
    const Vrst_float64 ux = (b.x - a.x) / len, uy = (b.y - a.y) / len;  // edge direction
    // Extent of the hull in the edge's frame: u along the edge, v across it.
    Vrst_float64 u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300;
    for (const Vrst_local& p : h) {
      const Vrst_float64 u = p.x * ux + p.y * uy, v = -p.x * uy + p.y * ux;
      u0 = std::min(u0, u), u1 = std::max(u1, u), v0 = std::min(v0, v), v1 = std::max(v1, v);
    }
    const Vrst_float64 area = (u1 - u0) * (v1 - v0);
    if (area < best) {
      best = area;
      // Back from (u, v) to local x, y: rotate by the edge direction.
      const auto pt = [&](Vrst_float64 u, Vrst_float64 v) { return Vrst_local{u * ux - v * uy, u * uy + v * ux}; };
      out = {pt(u0, v0), pt(u1, v0), pt(u1, v1), pt(u0, v1)};
    }
  }
  return out;
}

void Vrst_map_layer_builder::add_area(const osmium::Area& a) {
  const std::optional<AreaKind> kind = map_area_kind(a.tags());
  if (!kind) return;
  for (const osmium::OuterRing& outer : a.outer_rings()) {
    if (outer.size() < 4) continue;
    const osmium::Location ref = outer.front().location();
    const Vrst_float64 kx = 111320.0 * std::cos(ref.lat() * std::numbers::pi / 180);
    const auto ring_of = [&](const osmium::NodeRefList& nodes) {
      std::vector<Vrst_local> r;
      for (const osmium::NodeRef& n : nodes) r.push_back(local(n.location(), ref, kx));
      r = simplify(r, kToleranceM);
      if (r.size() > 1) r.pop_back();  // rings repeat the first point at the end
      return r;
    };
    std::vector<std::vector<Vrst_local>> rings{ring_of(outer)};
    if (rings[0].size() < 3) continue;
    const Vrst_float64 area = std::abs(ring_area(rings[0]));
    if (area < (*kind == AreaKind::Building ? kMinBuildingM2 : kMinAreaM2)) continue;
    if (*kind == AreaKind::Building) {
      // Outline holes (courtyards) are left out for buildings: one ring, and the
      // outline the renderer draws around it stays a single loop.
      std::vector<Vrst_local> rect = min_rectangle(rings[0]);
      if (std::abs(ring_area(rect)) <= kRectFactor * area) {
        rings[0] = std::move(rect);
        ++m_rectangles;
      } else {
        ++m_outlines;
      }
    } else {
      for (const osmium::InnerRing& inner : a.inner_rings(outer)) {
        std::vector<Vrst_local> r = ring_of(inner);
        if (r.size() >= 3) rings.push_back(std::move(r));
      }
    }
    std::vector<std::vector<std::array<Vrst_float64, 2>>> poly;
    Vrst_area_tmp out{*kind, {}, static_cast<Vrst_uint32>(rings[0].size()), {}};
    for (const std::vector<Vrst_local>& r : rings) {
      poly.emplace_back();
      for (const Vrst_local& p : r) {
        poly.back().push_back({p.x, p.y});
        out.points.push_back(to_coord(p, ref, kx));
      }
    }
    out.indices = mapbox::earcut<Vrst_uint32>(poly);
    if (out.indices.empty()) continue;
    m_areas.push_back(std::move(out));
  }
}

void Vrst_map_layer_builder::add_line(const osmium::Way& w) {
  const std::optional<LineKind> kind = map_line_kind(w.tags());
  if (!kind || w.nodes().size() < 2) return;
  for (const osmium::NodeRef& n : w.nodes())
    if (!n.location().valid()) return;
  const osmium::Location ref = w.nodes().front().location();
  const Vrst_float64 kx = 111320.0 * std::cos(ref.lat() * std::numbers::pi / 180);
  std::vector<Vrst_local> pts;
  for (const osmium::NodeRef& n : w.nodes()) pts.push_back(local(n.location(), ref, kx));
  pts = simplify(pts, kToleranceM);
  const auto set = [&]<Vrst_size N>(const char (&k)[N]) {
    const std::string_view v = tag_value(w.tags(), k);
    return !v.empty() && v != "no";
  };
  const Vrst_uint8 flags =
      static_cast<Vrst_uint8>((set("bridge") ? kLineBridge : 0) | (set("tunnel") ? kLineTunnel : 0));
  m_out.line_attr.push_back({static_cast<Vrst_uint8>(*kind), flags, 0});
  for (const Vrst_local& p : pts) m_out.line_points.push_back(to_coord(p, ref, kx));
  m_out.line_first_point.push_back(static_cast<Vrst_uint32>(m_out.line_points.size()));
}

std::optional<AreaKind> map_area_kind(const osmium::TagList& tags) {
  const auto is = [&]<Vrst_size N>(const char (&k)[N], std::initializer_list<std::string_view> values) {
    return std::find(values.begin(), values.end(), tag_value(tags, k)) != values.end();
  };
  if (const std::string_view b = tag_value(tags, "building"); !b.empty() && b != "no") return AreaKind::Building;
  if (is("natural", {"water"}) || is("waterway", {"riverbank"}) || is("landuse", {"reservoir", "basin"}))
    return AreaKind::Water;
  if (is("landuse", {"forest"}) || is("natural", {"wood"})) return AreaKind::Forest;
  if (is("leisure", {"park", "garden", "pitch"}) || is("landuse", {"cemetery", "recreation_ground"}))
    return AreaKind::Park;
  if (is("landuse", {"residential"})) return AreaKind::Residential;
  if (is("landuse", {"industrial", "commercial", "retail", "railway"})) return AreaKind::Industrial;
  if (is("landuse", {"farmland", "farmyard", "orchard", "vineyard"})) return AreaKind::Farmland;
  if (is("landuse", {"meadow", "grass"}) || is("natural", {"grassland", "scrub", "heath"})) return AreaKind::Meadow;
  return std::nullopt;
}

std::optional<LineKind> map_line_kind(const osmium::TagList& tags) {
  if (const std::string_view v = tag_value(tags, "waterway"); !v.empty()) {
    if (v == "river" || v == "canal") return LineKind::River;
    if (v == "stream" || v == "ditch" || v == "drain") return LineKind::Stream;
  }
  if (const std::string_view v = tag_value(tags, "railway"); !v.empty()) {
    // Sidings and yard tracks (service=*) are drawn thin like trams, so a station
    // yard does not become a solid grey block.
    if (v == "rail" || v == "narrow_gauge")
      return tag_value(tags, "service").empty() ? LineKind::Rail : LineKind::MinorRail;
    if (v == "light_rail" || v == "tram" || v == "subway") return LineKind::MinorRail;
  }
  return std::nullopt;
}

}  // namespace nav
