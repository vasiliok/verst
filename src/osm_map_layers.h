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
#include <osmium/io/reader.hpp>
#include <osmium/osm/area.hpp>
#include <osmium/osm/way.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <numeric>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "convert_progress.h"
#include "format.h"

namespace nav {

/// Reports how far a reader is through its file and stops when cancelled; called after
/// every buffer of every pass.
/// @param r Reader.
/// @param p Progress.
/// @throws Vrst_convert_cancelled after Vrst_convert_progress::cancel().
inline void report_read(const osmium::io::Reader& r, Vrst_convert_progress& p) {
  if (r.file_size() > 0) p.set_fraction(static_cast<Vrst_float64>(r.offset()) / static_cast<Vrst_float64>(r.file_size()));
  p.check();
}

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

/// map.dat content, named like the sections. The two big area sections (points and
/// triangle indices, most of the file) are not held in memory: they are in one
/// temporary file per AreaKind, which written one after the other in kind order are the
/// section, so areas are neither kept nor copied to sort them by kind.
struct Vrst_map_layer_data {
  std::vector<Vrst_uint32> area_first_point{0};  ///< AREA_FIRST_POINT, R+1 entries.
  std::vector<Vrst_uint32> area_first_index{0};  ///< AREA_FIRST_INDEX, R+1 entries.
  std::vector<Vrst_uint8> area_kinds;            ///< AREA_KIND, sorted ascending.
  /// AREA_POINTS (Vrst_coord), one temporary file per AreaKind in kind order.
  std::array<std::filesystem::path, kAreaKindCount> area_point_files;
  /// AREA_INDICES (Vrst_uint32), one temporary file per AreaKind in kind order.
  std::array<std::filesystem::path, kAreaKindCount> area_index_files;
  Vrst_size area_points = 0;                     ///< Elements of AREA_POINTS.
  Vrst_size area_indices = 0;                    ///< Elements of AREA_INDICES.
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

  /// Starts a builder that keeps area points and triangles in temporary files.
  /// @param temp_dir Existing folder for them (the conversion's .tmp folder).
  /// @throws std::runtime_error when a file cannot be created.
  explicit Vrst_map_layer_builder(const std::filesystem::path& temp_dir);
  ~Vrst_map_layer_builder() = default;                                         ///< Closes the files.
  Vrst_map_layer_builder(const Vrst_map_layer_builder&) = delete;             ///< Owns files: not copyable.
  Vrst_map_layer_builder& operator=(const Vrst_map_layer_builder&) = delete;  ///< Owns files: not copyable.
  Vrst_map_layer_builder(Vrst_map_layer_builder&&) = delete;                  ///< Handlers point at it: not movable.
  Vrst_map_layer_builder& operator=(Vrst_map_layer_builder&&) = delete;       ///< Handlers point at it: not movable.

  /// Reads one extract in four streaming passes, keeping only the node locations that
  /// can be needed:
  /// 1. relations: the multipolygons (the assembler's own selection);
  /// 2. ways: ids of the nodes of ways that may become an area or a line, and of the
  ///    member ways of those multipolygons (a superset of what is drawn);
  /// 3. nodes: the locations of just those nodes;
  /// 4. ways: locations filled in, areas assembled, lines taken.
  /// About 16 bytes per needed node instead of an index over every node in the file.
  /// @param path .osm.pbf file.
  /// @param progress Phases Relations, AreaWays, AreaNodes, Areas with their fraction.
  /// @throws Vrst_convert_cancelled after progress.cancel().
  void read(const std::string& path, Vrst_convert_progress& progress);

  /// Fills the per-area tables (first point and index, kind, outer size) in kind order,
  /// the draw order, and closes the per-kind files. Call once, after read().
  /// @throws std::runtime_error when a temporary file could not be written.
  void finish();

  /// Number of buildings reduced to a rectangle.
  /// @return Count.
  Vrst_size rectangles() const { return m_rectangles; }
  /// Number of buildings that kept their (simplified) outline.
  /// @return Count.
  Vrst_size outlines() const { return m_outlines; }
  /// Number of nodes whose location was kept for the assembly (read()).
  /// @return Count.
  Vrst_size needed_nodes() const { return m_needed_nodes; }

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

  /// Sizes of the areas of one kind, in the order they were read; their points and
  /// indices are in the kind's part of m_out.
  struct Vrst_kind_areas {
    std::vector<Vrst_uint32> points;      ///< Points of each area.
    std::vector<Vrst_uint32> indices;     ///< Triangle indices of each area.
    std::vector<Vrst_uint32> outer_size;  ///< Points of each area's outer ring.
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

  /// Adds one assembled area: simplifies the rings, reduces buildings to rectangles
  /// where they fit, triangulates and stores it. Each outer ring is its own area.
  /// @param a Area from the assembler.
  void add_area(const osmium::Area& a);

  /// Adds one river or railway way, simplified.
  /// @param w Way with node locations.
  void add_line(const osmium::Way& w);

  /// Output arrays: lines as they are read, area tables after finish().
  Vrst_map_layer_data m_out;
  /// Open per-kind files of AREA_POINTS.
  std::array<std::ofstream, kAreaKindCount> m_point_files;
  /// Open per-kind files of AREA_INDICES.
  std::array<std::ofstream, kAreaKindCount> m_index_files;
  /// Area sizes per kind until finish() turns them into the area tables.
  std::array<Vrst_kind_areas, kAreaKindCount> m_kind_areas;
  /// Buildings stored as rectangles.
  Vrst_size m_rectangles = 0;
  /// Buildings stored with their own outline.
  Vrst_size m_outlines = 0;
  /// Nodes whose location read() kept.
  Vrst_size m_needed_nodes = 0;
};

}  // namespace nav
