/// @file
/// Vrst_map_layer_builder: reading water, forest, buildings and railways from OSM; tag rules.
/// Declarations and documentation are in osm_map_layers.h.
#include "convert/osm_map_layers.h"

#include <algorithm>
#include <span>
#include <stdexcept>
#include <mapbox/earcut.hpp>
#include <numbers>
#include <osmium/area/assembler.hpp>
#include <osmium/area/multipolygon_manager.hpp>
#include <osmium/osm/node.hpp>
#include <osmium/osm/relation.hpp>
#include <osmium/io/pbf_input.hpp>
#include <osmium/relations/manager_util.hpp>
#include <osmium/relations/relations_manager.hpp>
#include <osmium/tags/tags_filter.hpp>
#include <osmium/visitor.hpp>

#include "geometry/geometry.h"

namespace nav {

namespace {

/// Appends plain data to a binary file.
/// @tparam T Element type, written as raw bytes.
/// @param out File.
/// @param data Elements.
template <Vrst_plain_data T>
void write_raw(std::ofstream& out, std::span<const T> data) {
  const std::span<const std::byte> bytes = std::as_bytes(data);
  // ostream::write takes chars; the bytes are only reinterpreted here.
  out.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
}

/// The assembler's area tags: ways and multipolygons with one of these keys can become areas.
/// @return Filter.
osmium::TagsFilter area_filter() {
  osmium::TagsFilter filter{false};
  for (const std::string k : {"building", "landuse", "natural", "leisure", "waterway"}) filter.add_rule(true, k);
  return filter;
}

/// Sorts ids and removes repeats.
/// @param ids Ids; changed in place.
void sort_unique(std::vector<Vrst_uint64>& ids) {
  std::ranges::sort(ids);
  ids.erase(std::ranges::unique(ids).begin(), ids.end());
}

/// Location of every needed node, looked up by id: sorted ids and their locations.
/// Fills in the node locations of each way (libosmium handler), like
/// NodeLocationsForWays with ignore_errors(): unknown nodes keep an undefined location.
class Vrst_needed_locations : public osmium::handler::Handler {
 public:
  /// Reads the locations of the given nodes (one pass over the nodes of the file).
  /// @param path .osm.pbf file.
  /// @param ids Needed node ids, sorted, unique; moved in.
  /// @param progress Fraction of the pass; stops when cancelled.
  Vrst_needed_locations(const std::string& path, std::vector<Vrst_uint64> ids, Vrst_convert_progress& progress)
      : m_ids(std::move(ids)), m_locations(m_ids.size()) {
    osmium::io::Reader reader{path, osmium::osm_entity_bits::node};
    Vrst_size next = 0;  // nodes come sorted by id: walk the ids instead of searching
    while (const osmium::memory::Buffer buffer = reader.read()) {
      for (const osmium::Node& n : buffer.select<osmium::Node>()) {
        const auto id = static_cast<Vrst_uint64>(n.positive_id());
        if (next > 0 && m_ids[next - 1] > id) next = 0;  // not sorted: start over
        while (next < m_ids.size() && m_ids[next] < id) ++next;
        if (next < m_ids.size() && m_ids[next] == id) m_locations[next] = n.location();
      }
      report_read(reader, progress);
    }
    reader.close();
  }

  /// Sets the location of every node of a way that is known.
  /// @param way Way, changed in place.
  void way(osmium::Way& way) const {
    for (osmium::NodeRef& nr : way.nodes()) {
      const auto it = std::ranges::lower_bound(m_ids, static_cast<Vrst_uint64>(nr.positive_ref()));
      if (it != m_ids.end() && *it == static_cast<Vrst_uint64>(nr.positive_ref()))
        nr.set_location(m_locations[static_cast<Vrst_size>(it - m_ids.begin())]);
    }
  }

  /// @return Number of nodes kept.
  Vrst_size size() const { return m_ids.size(); }

 private:
  /// Needed node ids, sorted.
  std::vector<Vrst_uint64> m_ids;
  /// Location of each id; undefined when the node is not in the file.
  std::vector<osmium::Location> m_locations;
};

}  // namespace

Vrst_map_layer_builder::Vrst_map_layer_builder(const std::filesystem::path& temp_dir,
                                               Vrst_address_collector& addresses)
    : m_addresses(addresses) {
  for (int k = 0; k < kAreaKindCount; ++k) {
    const auto i = static_cast<Vrst_size>(k);
    m_out.area_point_files[i] = temp_dir / ("area-points-" + std::to_string(k) + ".tmp");
    m_out.area_index_files[i] = temp_dir / ("area-indices-" + std::to_string(k) + ".tmp");
    m_point_files[i].open(m_out.area_point_files[i], std::ios::binary | std::ios::trunc);
    m_index_files[i].open(m_out.area_index_files[i], std::ios::binary | std::ios::trunc);
    if (!m_point_files[i] || !m_index_files[i]) throw std::runtime_error("cannot create " + temp_dir.string());
  }
}

void Vrst_map_layer_builder::read(const std::string& path, Vrst_convert_progress& progress) {
  const osmium::area::Assembler::config_type cfg;
  const osmium::TagsFilter filter = area_filter();
  osmium::area::MultipolygonManager<osmium::area::Assembler> mp{cfg, filter};
  progress.begin(Vrst_convert_phase::Relations);
  osmium::relations::read_relations(osmium::io::File{path}, mp);

  // Member ways of the multipolygons the manager kept (its own new_relation() test).
  std::vector<Vrst_uint64> members;
  {
    osmium::io::Reader reader{path, osmium::osm_entity_bits::relation};
    while (const osmium::memory::Buffer buffer = reader.read()) {
      for (const osmium::Relation& r : buffer.select<osmium::Relation>())
        if (mp.new_relation(r))
          for (const osmium::RelationMember& m : r.members())
            if (m.type() == osmium::item_type::way) members.push_back(static_cast<Vrst_uint64>(m.positive_ref()));
      report_read(reader, progress);
    }
    reader.close();
  }
  sort_unique(members);

  // Nodes of every way that may become an area (more than 3 nodes, an area key) or a
  // line, or is a multipolygon member.
  progress.begin(Vrst_convert_phase::AreaWays);
  std::vector<Vrst_uint64> needed;
  {
    osmium::io::Reader reader{path, osmium::osm_entity_bits::way};
    Vrst_size unique = 0;  // size after the last sort_unique()
    while (const osmium::memory::Buffer buffer = reader.read()) {
      for (const osmium::Way& w : buffer.select<osmium::Way>()) {
        const bool area = w.nodes().size() > 3 && osmium::tags::match_any_of(w.tags(), filter);
        if (area || map_line_kind(w.tags()) ||
            std::ranges::binary_search(members, static_cast<Vrst_uint64>(w.positive_id())))
          for (const osmium::NodeRef& nr : w.nodes()) needed.push_back(static_cast<Vrst_uint64>(nr.positive_ref()));
      }
      // Neighbouring ways share nodes: drop repeats now and then, so the list does not
      // grow to every reference of every way before it is made unique.
      if (needed.size() > 2 * unique + (1u << 20)) {
        sort_unique(needed);
        unique = needed.size();
      }
      report_read(reader, progress);
    }
    reader.close();
  }
  std::vector<Vrst_uint64>().swap(members);
  sort_unique(needed);
  needed.shrink_to_fit();
  m_needed_nodes = needed.size();

  progress.begin(Vrst_convert_phase::AreaNodes);
  const Vrst_needed_locations locations{path, std::move(needed), progress};
  progress.begin(Vrst_convert_phase::Areas);
  Vrst_handler handler{*this};
  auto& second_pass = mp.handler([&handler](osmium::memory::Buffer&& buffer) { osmium::apply(buffer, handler); });
  osmium::io::Reader reader{path, osmium::osm_entity_bits::way};
  // Item by item and one flush at the end, exactly like osmium::apply(reader, ...),
  // with a progress report between buffers (apply(buffer, ...) would flush each one).
  while (osmium::memory::Buffer buffer = reader.read()) {
    for (osmium::memory::Item& item : buffer) osmium::apply_item(item, locations, handler, second_pass);
    report_read(reader, progress);
  }
  osmium::apply_flush(locations, handler, second_pass);
  reader.close();
}

void Vrst_map_layer_builder::finish() {
  Vrst_map_layer_data& d = m_out;
  for (int k = 0; k < kAreaKindCount; ++k) {
    const auto f = static_cast<Vrst_size>(k);
    m_point_files[f].close();
    m_index_files[f].close();
    if (m_point_files[f].fail() || m_index_files[f].fail())
      throw std::runtime_error("write failed: " + d.area_point_files[f].parent_path().string());
    Vrst_kind_areas& ka = m_kind_areas[static_cast<Vrst_size>(k)];
    for (Vrst_size i = 0; i < ka.points.size(); ++i) {
      d.area_kinds.push_back(static_cast<Vrst_uint8>(k));
      d.area_outer_size.push_back(ka.outer_size[i]);
      d.area_first_point.push_back(d.area_first_point.back() + ka.points[i]);
      d.area_first_index.push_back(d.area_first_index.back() + ka.indices[i]);
    }
    ka = {};
  }
  d.area_points = d.area_first_point.back();
  d.area_indices = d.area_first_index.back();
}

void Vrst_map_layer_builder::add_area(const osmium::Area& a) {
  const std::optional<AreaKind> kind = map_area_kind(a.tags());
  if (!kind) return;
  if (*kind == AreaKind::Building) add_building_address(a);
  for (const osmium::OuterRing& outer : a.outer_rings()) {
    if (outer.size() < 4) continue;
    const osmium::Location ref = outer.front().location();
    const Vrst_float64 kx = 111320.0 * std::cos(ref.lat() * std::numbers::pi / 180);
    const auto ring_of = [&](const osmium::NodeRefList& nodes) {
      std::vector<Vrst_local> r;
      for (const osmium::NodeRef& n : nodes) r.push_back(local(n.location(), ref, kx));
      simplify_line(r, kToleranceM);
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
      std::vector<Vrst_local> rect = min_area_rectangle(rings[0]);
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
    Vrst_size points = 0;
    for (const std::vector<Vrst_local>& r : rings) {
      poly.emplace_back();
      for (const Vrst_local& p : r) poly.back().push_back({p.x, p.y});
      points += r.size();
    }
    const std::vector<Vrst_uint32> indices = mapbox::earcut<Vrst_uint32>(poly);
    if (indices.empty()) continue;
    // Straight into this kind's file: nothing kept per area, no copy to sort by kind.
    const auto k = static_cast<Vrst_size>(*kind);
    std::vector<Vrst_coord> coords;
    coords.reserve(points);
    for (const std::vector<Vrst_local>& r : rings)
      for (const Vrst_local& p : r) coords.push_back(to_coord(p, ref, kx));
    write_raw(m_point_files[k], std::span<const Vrst_coord>(coords));
    write_raw(m_index_files[k], std::span<const Vrst_uint32>(indices));
    m_kind_areas[k].points.push_back(static_cast<Vrst_uint32>(points));
    m_kind_areas[k].indices.push_back(static_cast<Vrst_uint32>(indices.size()));
    m_kind_areas[k].outer_size.push_back(static_cast<Vrst_uint32>(rings[0].size()));
  }
}

void Vrst_map_layer_builder::add_building_address(const osmium::Area& a) {
  const osmium::TagList& tags = a.tags();
  const Vrst_address_tags t{tag_value(tags, "addr:housenumber"), tag_value(tags, "addr:street"),
                            tag_value(tags, "addr:place"), tag_value(tags, "addr:city"),
                            tag_value(tags, "addr:postcode")};
  if (t.number.empty()) return;
  const auto outer = a.outer_rings();
  if (outer.begin() == outer.end()) return;
  const osmium::OuterRing& ring = *outer.begin();
  if (ring.size() < 2) return;
  Vrst_int64 lat = 0, lon = 0;
  const Vrst_size n = ring.size() - 1;  // the last point repeats the first
  for (Vrst_size i = 0; i < n; ++i) {
    lat += ring[i].location().y();
    lon += ring[i].location().x();
  }
  const auto count = static_cast<Vrst_int64>(n);
  m_addresses.add_address({static_cast<Vrst_int32>(lat / count), static_cast<Vrst_int32>(lon / count)}, t);
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
  simplify_line(pts, kToleranceM);
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
