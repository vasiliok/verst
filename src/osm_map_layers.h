/// @file
/// Import of the drawing-only map layers (map.dat) for osm2nav: water, forest, land
/// use and buildings as triangulated areas, rivers and railways as lines.
///
/// Uses libosmium directly, not the Vrst_osm_source interface: areas need libosmium's
/// multipolygon assembler, which joins closed ways and type=multipolygon relations into
/// rings with holes. Polygons are cut into triangles here with earcut, once, so the
/// apps only upload triangles.
#pragma once

#include <mapbox/earcut.hpp>

#include <osmium/area/assembler.hpp>
#include <osmium/area/multipolygon_manager.hpp>
#include <osmium/handler.hpp>
#include <osmium/handler/node_locations_for_ways.hpp>
#include <osmium/index/map/flex_mem.hpp>
#include <osmium/io/pbf_input.hpp>
#include <osmium/osm/area.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/relations/manager_util.hpp>
#include <osmium/relations/relations_manager.hpp>
#include <osmium/tags/tags_filter.hpp>
#include <osmium/visitor.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <numeric>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "format.h"

namespace nav {

/// Area kind of an OSM area, from its tags.
/// @param tags Tags of the closed way or multipolygon relation.
/// @return Kind, or nullopt when the area is not drawn.
inline std::optional<AreaKind> map_area_kind(const osmium::TagList& tags) {
  auto is = [&](const char* k, std::initializer_list<const char*> values) {
    const char* v = tags[k];
    if (!v) return false;
    for (const char* x : values)
      if (std::strcmp(v, x) == 0) return true;
    return false;
  };
  if (const char* b = tags["building"]; b && std::strcmp(b, "no") != 0) return AreaKind::Building;
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

/// Line kind of an OSM way, from its tags.
/// @param tags Tags of the way.
/// @return Kind, or nullopt when the way is not a drawn river or railway.
inline std::optional<LineKind> map_line_kind(const osmium::TagList& tags) {
  if (const char* w = tags["waterway"]) {
    const std::string_view v = w;
    if (v == "river" || v == "canal") return LineKind::River;
    if (v == "stream" || v == "ditch" || v == "drain") return LineKind::Stream;
  }
  if (const char* r = tags["railway"]) {
    const std::string_view v = r;
    // Sidings and yard tracks (service=*) are drawn thin like trams, so a station
    // yard does not become a solid grey block.
    if (v == "rail" || v == "narrow_gauge") return tags["service"] ? LineKind::MinorRail : LineKind::Rail;
    if (v == "light_rail" || v == "tram" || v == "subway") return LineKind::MinorRail;
  }
  return std::nullopt;
}

/// Packed map.dat arrays, named like the sections; osm2nav writes them as they are.
struct Vrst_map_layer_data {
  std::vector<Vrst_uint32> area_first_point{0};  ///< AREA_FIRST_POINT, R+1 entries.
  std::vector<Vrst_uint32> area_first_index{0};  ///< AREA_FIRST_INDEX, R+1 entries.
  std::vector<Vrst_uint8> area_kinds;            ///< AREA_KIND, sorted ascending.
  std::vector<Vrst_coord> area_points;           ///< AREA_POINTS.
  std::vector<Vrst_uint32> area_indices;         ///< AREA_INDICES.
  std::vector<Vrst_uint32> area_outer_size;      ///< AREA_OUTER_SIZE.
  std::vector<Vrst_uint32> line_first_point{0};  ///< LINE_FIRST_POINT, L+1 entries.
  std::vector<Vrst_line_attr> line_attr;         ///< LINE_ATTR.
  std::vector<Vrst_coord> line_points;           ///< LINE_POINTS.
};

/// Collects map.dat content while libosmium reads the file, then sorts and packs it.
///
/// Geometry is simplified on the way in (Douglas–Peucker, kToleranceM) and tiny areas
/// are dropped: a forest outline with 10 000 points costs triangles on every frame
/// that shows it, and detail below half a metre is never visible.
class Vrst_map_layer_builder {
 public:
  /// Douglas–Peucker tolerance in metres. Half a metre is below a pixel even at the
  /// closest zoom the viewer allows for buildings.
  static constexpr Vrst_float64 kToleranceM = 0.5;
  /// Areas smaller than this many square metres are dropped (except buildings).
  static constexpr Vrst_float64 kMinAreaM2 = 100;
  /// Buildings smaller than this are dropped (sheds, wrong tags).
  static constexpr Vrst_float64 kMinBuildingM2 = 4;
  /// A building becomes its smallest rotated rectangle when that rectangle is at most
  /// this factor bigger than the real outline; otherwise the outline is kept.
  static constexpr Vrst_float64 kRectFactor = 1.25;

  /// Reads one extract: relations for the multipolygons, then all objects with node
  /// locations.
  /// @param path .osm.pbf file.
  void read(const std::string& path) {
    osmium::area::Assembler::config_type cfg;
    osmium::TagsFilter filter{false};
    for (const char* k : {"building", "landuse", "natural", "leisure", "waterway"}) filter.add_rule(true, k);
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

  /// Sorts areas by kind (the draw order) and packs everything into data(). Call once,
  /// after read().
  void finish() {
    std::vector<Vrst_uint32> order(m_areas.size());
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&](Vrst_uint32 a, Vrst_uint32 b) { return m_areas[a].kind < m_areas[b].kind; });
    Vrst_map_layer_data& d = m_out;
    for (const Vrst_uint32 i : order) {
      const Vrst_area_tmp& a = m_areas[i];
      d.area_kinds.push_back(Vrst_uint8(a.kind));
      d.area_outer_size.push_back(a.outer_size);
      d.area_points.insert(d.area_points.end(), a.points.begin(), a.points.end());
      d.area_indices.insert(d.area_indices.end(), a.indices.begin(), a.indices.end());
      d.area_first_point.push_back(Vrst_uint32(d.area_points.size()));
      d.area_first_index.push_back(Vrst_uint32(d.area_indices.size()));
    }
    std::vector<Vrst_area_tmp>().swap(m_areas);
  }

  /// Number of buildings reduced to a rectangle.
  /// @return Count.
  Vrst_size rectangles() const { return m_rectangles; }
  /// Number of buildings that kept their (simplified) outline.
  /// @return Count.
  Vrst_size outlines() const { return m_outlines; }

  /// @copydoc m_out
  /// @return Arrays; complete after finish().
  const Vrst_map_layer_data& data() const { return m_out; }

 private:
  /// Point in local metres around a reference point (equirectangular). Accurate
  /// enough over one building or forest for simplification, areas and rectangles.
  struct Vrst_local {
    Vrst_float64 x;  ///< East, metres.
    Vrst_float64 y;  ///< North, metres.
  };

  /// One area before sorting: kind, points and triangle indices.
  struct Vrst_area_tmp {
    AreaKind kind;                     ///< What it is.
    std::vector<Vrst_coord> points;    ///< Outer ring, then holes.
    Vrst_uint32 outer_size;            ///< Points of the outer ring.
    std::vector<Vrst_uint32> indices;  ///< Triangles into points.
  };

  /// libosmium handler: one callback per assembled area and per way.
  class Vrst_handler : public osmium::handler::Handler {
   public:
    /// Binds the handler to its builder.
    /// @param owner Builder that receives the geometry.
    explicit Vrst_handler(Vrst_map_layer_builder& owner) : m_owner(owner) {}
    /// Area from the assembler (closed way or multipolygon relation).
    /// @param a Assembled area.
    void area(const osmium::Area& a) { m_owner.add_area(a); }
    /// Way with node locations; rivers and railways become lines.
    /// @param w Way.
    void way(const osmium::Way& w) { m_owner.add_line(w); }

   private:
    /// Builder that receives the geometry.
    Vrst_map_layer_builder& m_owner;
  };

  /// Converts to local metres around a reference latitude/longitude.
  /// @param l Location.
  /// @param ref Reference point (first point of the object).
  /// @param kx Metres per degree of longitude at the reference latitude.
  /// @return Local point.
  static Vrst_local local(const osmium::Location& l, const osmium::Location& ref, Vrst_float64 kx) {
    return {(l.lon() - ref.lon()) * kx, (l.lat() - ref.lat()) * 111320.0};
  }

  /// Converts a local point back to a fixed-point coordinate.
  /// @param p Local point.
  /// @param ref Reference point used by local().
  /// @param kx Metres per degree of longitude at the reference latitude.
  /// @return Coordinate in 1e-7 degrees.
  static Vrst_coord to_coord(Vrst_local p, const osmium::Location& ref, Vrst_float64 kx) {
    return {Vrst_int32(std::lround((ref.lat() + p.y / 111320.0) * 1e7)), Vrst_int32(std::lround((ref.lon() + p.x / kx) * 1e7))};
  }

  /// Douglas–Peucker simplification of an open polyline.
  /// @param pts Points; the first and last are always kept.
  /// @param tol Tolerance in metres.
  /// @return Kept points.
  static std::vector<Vrst_local> simplify(const std::vector<Vrst_local>& pts, Vrst_float64 tol) {
    if (pts.size() <= 2) return pts;
    std::vector<Vrst_uint8> keep(pts.size(), 0);
    keep.front() = keep.back() = 1;
    std::vector<std::pair<Vrst_size, Vrst_size>> stack{{0, pts.size() - 1}};
    while (!stack.empty()) {
      const auto [i, j] = stack.back();
      stack.pop_back();
      const Vrst_local a = pts[i], b = pts[j];
      const Vrst_float64 dx = b.x - a.x, dy = b.y - a.y, len2 = dx * dx + dy * dy;
      Vrst_float64 best = -1;
      Vrst_size at = i;
      for (Vrst_size k = i + 1; k < j; ++k) {
        const Vrst_float64 px = pts[k].x - a.x, py = pts[k].y - a.y;
        const Vrst_float64 d = len2 > 0 ? std::abs(px * dy - py * dx) / std::sqrt(len2) : std::hypot(px, py);
        if (d > best) best = d, at = k;
      }
      if (best > tol) {
        keep[at] = 1;
        stack.push_back({i, at});
        stack.push_back({at, j});
      }
    }
    std::vector<Vrst_local> out;
    for (Vrst_size k = 0; k < pts.size(); ++k)
      if (keep[k]) out.push_back(pts[k]);
    return out;
  }

  /// Signed area of a ring (shoelace).
  /// @param r Ring without the closing point.
  /// @return Square metres, positive when counter-clockwise.
  static Vrst_float64 ring_area(const std::vector<Vrst_local>& r) {
    Vrst_float64 s = 0;
    for (Vrst_size i = 0, j = r.size() - 1; i < r.size(); j = i++) s += r[j].x * r[i].y - r[i].x * r[j].y;
    return s / 2;
  }

  /// Smallest rotated rectangle around a ring (rotating calipers on the convex hull).
  /// @param r Ring without the closing point, at least 3 points.
  /// @return 4 corners, counter-clockwise.
  static std::vector<Vrst_local> min_rectangle(std::vector<Vrst_local> r) {
    // Convex hull, Andrew's monotone chain.
    std::sort(r.begin(), r.end(), [](Vrst_local a, Vrst_local b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
    auto cross = [](Vrst_local o, Vrst_local a, Vrst_local b) { return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x); };
    std::vector<Vrst_local> h(2 * r.size());
    Vrst_size k = 0;
    for (Vrst_size i = 0; i < r.size(); ++i) {
      while (k >= 2 && cross(h[k - 2], h[k - 1], r[i]) <= 0) --k;
      h[k++] = r[i];
    }
    for (Vrst_size i = r.size() - 1, t = k + 1; i-- > 0;) {
      while (k >= t && cross(h[k - 2], h[k - 1], r[i]) <= 0) --k;
      h[k++] = r[i];
    }
    h.resize(k > 1 ? k - 1 : k);
    Vrst_float64 best = 1e300;
    std::vector<Vrst_local> out(4);
    for (Vrst_size i = 0; i < h.size(); ++i) {
      const Vrst_local a = h[i], b = h[(i + 1) % h.size()];
      const Vrst_float64 len = std::hypot(b.x - a.x, b.y - a.y);
      if (len <= 0) continue;
      const Vrst_float64 ux = (b.x - a.x) / len, uy = (b.y - a.y) / len;  // edge direction
      Vrst_float64 u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300;
      for (const Vrst_local& p : h) {
        const Vrst_float64 u = p.x * ux + p.y * uy, v = -p.x * uy + p.y * ux;
        u0 = std::min(u0, u), u1 = std::max(u1, u), v0 = std::min(v0, v), v1 = std::max(v1, v);
      }
      const Vrst_float64 area = (u1 - u0) * (v1 - v0);
      if (area < best) {
        best = area;
        auto pt = [&](Vrst_float64 u, Vrst_float64 v) { return Vrst_local{u * ux - v * uy, u * uy + v * ux}; };
        out = {pt(u0, v0), pt(u1, v0), pt(u1, v1), pt(u0, v1)};
      }
    }
    return out;
  }

  /// Adds one assembled area: simplifies the rings, reduces buildings to rectangles
  /// where they fit, triangulates and stores it. Each outer ring is its own area.
  /// @param a Area from the assembler.
  void add_area(const osmium::Area& a) {
    const std::optional<AreaKind> kind = map_area_kind(a.tags());
    if (!kind) return;
    for (const osmium::OuterRing& outer : a.outer_rings()) {
      if (outer.size() < 4) continue;
      const osmium::Location ref = outer.front().location();
      const Vrst_float64 kx = 111320.0 * std::cos(ref.lat() * 3.14159265358979323846 / 180);
      auto ring_of = [&](const osmium::NodeRefList& nodes) {
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
      Vrst_area_tmp out{*kind, {}, Vrst_uint32(rings[0].size()), {}};
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

  /// Adds one river or railway way, simplified.
  /// @param w Way with node locations.
  void add_line(const osmium::Way& w) {
    const std::optional<LineKind> kind = map_line_kind(w.tags());
    if (!kind || w.nodes().size() < 2) return;
    for (const osmium::NodeRef& n : w.nodes())
      if (!n.location().valid()) return;
    const osmium::Location ref = w.nodes().front().location();
    const Vrst_float64 kx = 111320.0 * std::cos(ref.lat() * 3.14159265358979323846 / 180);
    std::vector<Vrst_local> pts;
    for (const osmium::NodeRef& n : w.nodes()) pts.push_back(local(n.location(), ref, kx));
    pts = simplify(pts, kToleranceM);
    auto set = [&](const char* k) { const char* v = w.tags()[k]; return v && std::strcmp(v, "no") != 0; };
    const Vrst_uint8 flags = Vrst_uint8((set("bridge") ? kLineBridge : 0) | (set("tunnel") ? kLineTunnel : 0));
    m_out.line_attr.push_back({Vrst_uint8(*kind), flags, 0});
    for (const Vrst_local& p : pts) m_out.line_points.push_back(to_coord(p, ref, kx));
    m_out.line_first_point.push_back(Vrst_uint32(m_out.line_points.size()));
  }

  /// Output arrays: lines as they are read, areas after finish().
  Vrst_map_layer_data m_out;
  /// Areas in file order until finish() sorts them.
  std::vector<Vrst_area_tmp> m_areas;
  /// Buildings stored as rectangles.
  Vrst_size m_rectangles = 0;
  /// Buildings stored with their own outline.
  Vrst_size m_outlines = 0;
};

}  // namespace nav
