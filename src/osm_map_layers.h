/// @file
/// Import of the drawing-only map layers (map.dat) for osm2nav: water, forest, land
/// use and buildings as triangulated areas, rivers and railways as lines.
///
/// Uses libosmium directly, not the Vrst_osm_source interface: areas need libosmium's
/// multipolygon assembler, which joins closed ways and type=multipolygon relations into
/// rings with holes. Polygons are cut into triangles here with earcut, once, so the
/// apps only upload triangles.
#pragma once

#include <osmium/handler.hpp>
#include <osmium/osm/area.hpp>
#include <osmium/osm/way.hpp>

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

/// Value of one OSM tag.
/// @tparam N Length of the key literal.
/// @param tags Tags of an object.
/// @param key Tag key, a string literal (osmium wants a C string).
/// @return The value; empty when the tag is missing.
template <Vrst_size N>
inline std::string_view tag_value(const osmium::TagList& tags, const char (&key)[N]) {
  return tags.get_value_by_key(key, "");
}

/// Area kind of an OSM area, from its tags.
/// @param tags Tags of the closed way or multipolygon relation.
/// @return Kind, or nullopt when the area is not drawn.
std::optional<AreaKind> map_area_kind(const osmium::TagList& tags);

/// Line kind of an OSM way, from its tags.
/// @param tags Tags of the way.
/// @return Kind, or nullopt when the way is not a drawn river or railway.
std::optional<LineKind> map_line_kind(const osmium::TagList& tags);

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
/// Geometry is simplified on the way in (Douglas-Peucker, kToleranceM) and tiny areas
/// are dropped: a forest outline with 10 000 points costs triangles on every frame
/// that shows it, and detail below half a metre is never visible.
class Vrst_map_layer_builder {
 public:
  /// Douglas-Peucker tolerance in metres. Half a metre is below a pixel even at the
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
  void read(const std::string& path);

  /// Sorts areas by kind (the draw order) and packs everything into data(). Call once,
  /// after read().
  void finish();

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
    return {static_cast<Vrst_int32>(std::lround((ref.lat() + p.y / 111320.0) * 1e7)),
            static_cast<Vrst_int32>(std::lround((ref.lon() + p.x / kx) * 1e7))};
  }

  /// Douglas-Peucker simplification of a polyline or a closed ring (first point repeated
  /// at the end): drops every point that lies within tol of the simplified line. Used for
  /// the map.dat outlines and lines before triangulation. Iterative, so very long
  /// outlines are safe; the result may cross itself (topology is not kept).
  /// @param pts Points in local metres; the first and last are always kept.
  /// @param tol Tolerance in metres.
  /// @return Kept points, in the original order.
  static std::vector<Vrst_local> simplify(const std::vector<Vrst_local>& pts, Vrst_float64 tol);

  /// Signed area of a ring (shoelace).
  /// @param r Ring without the closing point.
  /// @return Square metres, positive when counter-clockwise.
  static Vrst_float64 ring_area(const std::vector<Vrst_local>& r);

  /// Smallest rectangle, at any rotation, that contains a ring. Used for buildings: when
  /// the rectangle is barely bigger than the outline (kRectFactor), the building is stored
  /// as these 4 corners instead of its full outline, so the map is smaller and edges look
  /// clean at street zoom.
  ///
  /// Method: the convex hull of the points (Andrew's monotone chain), then one candidate
  /// rectangle per hull edge, aligned with that edge; the smallest wins. This is enough
  /// because the minimum-area enclosing rectangle always has a side on a hull edge
  /// (Freeman and Shapira, 1975). Each candidate measures the whole hull, so the cost is
  /// O(n log n) for the hull plus O(h^2) for h hull points (rotating calipers would make
  /// the second part O(h); building hulls are small, so this simpler loop is kept).
  /// @param r Ring without the closing point, at least 3 points, local metres; taken by
  ///        value because it is sorted in place.
  /// @return 4 corners, counter-clockwise, local metres; all zero when every point is
  ///         the same (no hull edge has a length).
  static std::vector<Vrst_local> min_rectangle(std::vector<Vrst_local> r);

  /// Adds one assembled area: simplifies the rings, reduces buildings to rectangles
  /// where they fit, triangulates and stores it. Each outer ring is its own area.
  /// @param a Area from the assembler.
  void add_area(const osmium::Area& a);

  /// Adds one river or railway way, simplified.
  /// @param w Way with node locations.
  void add_line(const osmium::Way& w);

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
