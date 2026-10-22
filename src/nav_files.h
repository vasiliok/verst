/// @file
/// Typed, zero-copy views over the mapped nav files. Throws std::runtime_error.
///
/// Loading checks magic, version, file type, section bounds/alignment,
/// elem_size and O(1) count consistency. Deep checks live in navcheck.
#pragma once

#include <cstring>
#include <filesystem>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

#include "format.h"
#include "mapped_file.h"

namespace nav {

/// One mapped nav file with its header and section table checked.
/// The only place where raw bytes become typed arrays: open() validates magic,
/// version, type and every section's bounds and alignment, and `section<T>()` checks
/// elem_size against sizeof(T). Everything above it can then use plain spans.
class Vrst_nav_file {
 public:
  void open(const std::string& path, FileType type) {
    file_.open(path);
    const Vrst_size size = file_.size();
    if (size < sizeof(Vrst_file_header)) fail("file too small");
    const auto* h = header_ptr();
    if (std::memcmp(h->magic, magic_for(type), 8) != 0) fail("bad magic");
    if (h->format_version != kFormatVersion)
      fail("unsupported format_version " + std::to_string(h->format_version));
    if (h->file_type != static_cast<Vrst_uint32>(type)) fail("bad file_type");

    const Vrst_uint64 table_end = sizeof(Vrst_file_header) + Vrst_uint64(h->section_count) * sizeof(Vrst_section_entry);
    if (table_end > size) fail("section table out of bounds");
    table_ = {reinterpret_cast<const Vrst_section_entry*>(file_.data() + sizeof(Vrst_file_header)), h->section_count};

    for (const Vrst_section_entry& s : table_) {
      const std::string sid = "section " + std::to_string(s.id);
      if (s.elem_size == 0) fail(sid + ": elem_size 0");
      if (s.offset % kSectionAlign != 0) fail(sid + ": offset not 64-aligned");
      if (s.offset < table_end || s.offset > size) fail(sid + ": offset out of bounds");
      if (s.count > (size - s.offset) / s.elem_size) fail(sid + ": data out of bounds");
    }
  }

  const Vrst_file_header& header() const { return *header_ptr(); }
  Vrst_uint64 build_id() const { return header().build_id; }
  const std::string& path() const { return file_.path(); }
  Vrst_size file_size() const { return file_.size(); }

  template <class T>
  std::span<const T> section(SectionId id) const {
    for (const Vrst_section_entry& s : table_) {
      if (s.id != static_cast<Vrst_uint32>(id)) continue;
      if (s.elem_size != sizeof(T))
        fail("section " + std::to_string(s.id) + ": elem_size " + std::to_string(s.elem_size) +
             " != " + std::to_string(sizeof(T)));
      return {reinterpret_cast<const T*>(file_.data() + s.offset), static_cast<Vrst_size>(s.count)};
    }
    fail("missing section " + std::to_string(static_cast<Vrst_uint32>(id)));
  }

 private:
  const Vrst_file_header* header_ptr() const { return reinterpret_cast<const Vrst_file_header*>(file_.data()); }
  [[noreturn]] void fail(const std::string& msg) const { throw std::runtime_error(file_.path() + ": " + msg); }

  /// Owns the mapping that every span returned by section() points into.
  Vrst_mapped_file file_;
  /// Validated section table inside the mapping. Kept so section() is a short linear
  /// search with no copy; all entries were bounds-checked in open().
  std::span<const Vrst_section_entry> table_;
};

inline void require(bool ok, const std::string& path, const char* what) {
  if (!ok) throw std::runtime_error(path + ": " + what);
}

/// Typed view of graph.dat: vertex positions plus forward and backward CSR.
/// The routing hot data, kept in its own file so a route query touches only these
/// pages and never the attributes, geometry or names.
struct Vrst_graph {
  /// Keeps graph.dat mapped; all spans below point into it.
  Vrst_nav_file file;
  /// Position of each vertex. A* needs it for the distance heuristic, and snapping
  /// needs it without opening geom.dat.
  std::span<const Vrst_coord> vertex_coord;
  /// CSR row starts, V+1 entries: out-arcs of v are [fwd_first_out[v], [v+1]). The
  /// extra last entry (= A) removes the special case for the last vertex.
  std::span<const Vrst_uint32> fwd_first_out;
  /// All arcs sorted by tail; the forward search reads one contiguous run per vertex.
  std::span<const Vrst_fwd_arc> fwd_arcs;
  /// Same as fwd_first_out for incoming arcs. Without a reverse index, finding the
  /// arcs into v means scanning all A arcs, which kills backward and bidirectional
  /// search.
  std::span<const Vrst_uint32> bwd_first_in;
  /// Incoming arcs sorted by head.
  std::span<const Vrst_bwd_arc> bwd_arcs;
  /// Extent of the source data (copied, it is one element).
  Vrst_bbox bbox{};

  Vrst_uint32 num_vertices() const { return static_cast<Vrst_uint32>(vertex_coord.size()); }
  Vrst_uint32 num_arcs() const { return static_cast<Vrst_uint32>(fwd_arcs.size()); }
  std::span<const Vrst_fwd_arc> out_arcs(Vrst_uint32 v) const {
    return fwd_arcs.subspan(fwd_first_out[v], fwd_first_out[v + 1] - fwd_first_out[v]);
  }
  std::span<const Vrst_bwd_arc> in_arcs(Vrst_uint32 v) const {
    return bwd_arcs.subspan(bwd_first_in[v], bwd_first_in[v + 1] - bwd_first_in[v]);
  }

  void open(const std::string& path) {
    file.open(path, FileType::Graph);
    vertex_coord = file.section<Vrst_coord>(SectionId::VertexCoord);
    fwd_first_out = file.section<Vrst_uint32>(SectionId::FwdFirstOut);
    fwd_arcs = file.section<Vrst_fwd_arc>(SectionId::FwdArcs);
    bwd_first_in = file.section<Vrst_uint32>(SectionId::BwdFirstIn);
    bwd_arcs = file.section<Vrst_bwd_arc>(SectionId::BwdArcs);
    auto bb = file.section<Vrst_bbox>(SectionId::Bbox);
    const Vrst_size V = vertex_coord.size();
    require(fwd_first_out.size() == V + 1, path, "FWD_FIRST_OUT size != V+1");
    require(bwd_first_in.size() == V + 1, path, "BWD_FIRST_IN size != V+1");
    require(bwd_arcs.size() == fwd_arcs.size(), path, "BWD_ARCS size != FWD_ARCS size");
    require(bb.size() == 1, path, "BBOX count != 1");
    bbox = bb[0];
  }
};

/// Typed view of edges.dat: per-segment attributes, end vertices and OSM way id,
/// all indexed by segment id (Vrst_fwd_arc::seg & kSegIdMask).
/// Used after routing (guidance, display) and for snapping, not during the search.
struct Vrst_edges {
  /// Keeps edges.dat mapped.
  Vrst_nav_file file;
  /// Per-segment attributes, indexed by segment id.
  std::span<const Vrst_seg_attr> attr;
  /// Segment end vertices (a, b), indexed by segment id.
  std::span<const Vrst_seg_vertices> vertices;
  /// Source OSM way per segment. OSM turn restrictions reference way ids, so mapping
  /// them onto arcs later is impossible without it; also used to compare with
  /// openstreetmap.org when debugging.
  std::span<const Vrst_uint64> osm_way;

  Vrst_uint32 num_segments() const { return static_cast<Vrst_uint32>(attr.size()); }

  void open(const std::string& path) {
    file.open(path, FileType::Edges);
    attr = file.section<Vrst_seg_attr>(SectionId::SegAttr);
    vertices = file.section<Vrst_seg_vertices>(SectionId::SegVertices);
    osm_way = file.section<Vrst_uint64>(SectionId::SegOsmWay);
    require(vertices.size() == attr.size(), path, "SEG_VERTICES size != S");
    require(osm_way.size() == attr.size(), path, "SEG_OSM_WAY size != S");
  }
};

/// Typed view of geom.dat: the shape points of every segment.
/// Shapes are by far the largest data and are needed only for drawing, snapping and
/// distance along a road, so they live apart from the graph.
struct Vrst_geom {
  /// Keeps geom.dat mapped. Separate file so the router never pages in shape points.
  Vrst_nav_file file;
  /// Start of each segment's point run, S+1 entries (last = P). Segments have a
  /// variable number of points, so a prefix array is the only O(1) lookup.
  std::span<const Vrst_uint32> seg_first_point;
  /// All shape points, segment after segment, both end points included so a segment
  /// can be drawn or measured without looking up its vertices.
  std::span<const Vrst_coord> points;

  /// Points of segment s in stored (a -> b) order.
  std::span<const Vrst_coord> segment_points(Vrst_uint32 s) const {
    return points.subspan(seg_first_point[s], seg_first_point[s + 1] - seg_first_point[s]);
  }

  void open(const std::string& path) {
    file.open(path, FileType::Geom);
    seg_first_point = file.section<Vrst_uint32>(SectionId::SegFirstPoint);
    points = file.section<Vrst_coord>(SectionId::Points);
    require(!seg_first_point.empty(), path, "SEG_FIRST_POINT empty");
  }
};

/// Typed view of names.dat: street names and road refs, looked up by id (0 = "").
/// Ids keep Vrst_seg_attr fixed-size; each distinct string is stored once.
struct Vrst_names {
  /// Keeps names.dat mapped.
  Vrst_nav_file file;
  /// Start of string i, N+1 entries. Length is offsets[i+1] - offsets[i], so strings
  /// need no terminator and may contain any UTF-8.
  std::span<const Vrst_uint32> offsets;
  /// All strings back to back, deduplicated (one copy of "Avinguda Meritxell" for
  /// all its segments).
  std::span<const char> data;

  Vrst_uint32 size() const { return static_cast<Vrst_uint32>(offsets.size() - 1); }
  std::string_view get(Vrst_uint32 id) const {
    return {data.data() + offsets[id], offsets[id + 1] - offsets[id]};
  }

  void open(const std::string& path) {
    file.open(path, FileType::Names);
    offsets = file.section<Vrst_uint32>(SectionId::StrOffsets);
    data = file.section<char>(SectionId::StrData);
    require(offsets.size() >= 2, path, "STR_OFFSETS needs at least 2 entries");
    require(offsets.back() == data.size(), path, "STR_OFFSETS last != data size");
  }
};

/// All four files of one import, opened together.
/// The normal entry point for tools: besides opening each file it checks that all
/// share one build_id and agree on the segment count, the mismatches that each file
/// on its own cannot detect.
struct Vrst_nav_data {
  /// Routing hot data.
  Vrst_graph graph;
  /// Segment attributes, needed after routing (guidance, display).
  Vrst_edges edges;
  /// Shape points, needed for drawing and snapping.
  Vrst_geom geom;
  /// Street names and refs.
  Vrst_names names;

  void open(const std::filesystem::path& dir) {
    graph.open((dir / file_name_for(FileType::Graph)).string());
    edges.open((dir / file_name_for(FileType::Edges)).string());
    geom.open((dir / file_name_for(FileType::Geom)).string());
    names.open((dir / file_name_for(FileType::Names)).string());
    const Vrst_uint64 id = graph.file.build_id();
    if (edges.file.build_id() != id || geom.file.build_id() != id || names.file.build_id() != id)
      throw std::runtime_error(dir.string() + ": files come from different builds (build_id mismatch)");
    if (geom.seg_first_point.size() != Vrst_size(edges.num_segments()) + 1)
      throw std::runtime_error(dir.string() + ": SEG_FIRST_POINT size != S+1");
  }
};

}  // namespace nav
