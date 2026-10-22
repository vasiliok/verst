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
  /// Maps a nav file and validates its header and section table: magic, format_version
  /// and file_type must match, and every section must be 64-aligned and inside the file.
  /// @param path File to open.
  /// @param type Expected file type.
  /// @throws std::runtime_error naming the file and the failed check.
  void open(const std::string& path, FileType type) {
    m_file.open(path);
    const Vrst_size size = m_file.size();
    if (size < sizeof(Vrst_file_header)) fail("file too small");
    const auto* h = header_ptr();
    if (std::memcmp(h->magic, magic_for(type), 8) != 0) fail("bad magic");
    if (h->format_version != kFormatVersion)
      fail("unsupported format_version " + std::to_string(h->format_version));
    if (h->file_type != static_cast<Vrst_uint32>(type)) fail("bad file_type");

    const Vrst_uint64 table_end = sizeof(Vrst_file_header) + Vrst_uint64(h->section_count) * sizeof(Vrst_section_entry);
    if (table_end > size) fail("section table out of bounds");
    m_table = {reinterpret_cast<const Vrst_section_entry*>(m_file.data() + sizeof(Vrst_file_header)), h->section_count};

    for (const Vrst_section_entry& s : m_table) {
      const std::string sid = "section " + std::to_string(s.id);
      if (s.elem_size == 0) fail(sid + ": elem_size 0");
      if (s.offset % kSectionAlign != 0) fail(sid + ": offset not 64-aligned");
      if (s.offset < table_end || s.offset > size) fail(sid + ": offset out of bounds");
      if (s.count > (size - s.offset) / s.elem_size) fail(sid + ": data out of bounds");
    }
  }

  /// File header.
  /// @return Header at the start of the mapping; valid after open().
  const Vrst_file_header& header() const { return *header_ptr(); }
  /// Import id of this file.
  /// @return Id shared by the four files written by one osm2nav run.
  Vrst_uint64 build_id() const { return header().build_id; }
  /// Path of the file, for error messages.
  /// @return Path given to open().
  const std::string& path() const { return m_file.path(); }
  /// Size of the file.
  /// @return Size in bytes.
  Vrst_size file_size() const { return m_file.size(); }

  /// Typed view of one section, without copying.
  /// @tparam T Element type; sizeof(T) must equal the stored elem_size.
  /// @param id Section to look up.
  /// @return Span over the section inside the mapping.
  /// @throws std::runtime_error if the section is missing or its elem_size differs.
  template <class T>
  std::span<const T> section(SectionId id) const {
    for (const Vrst_section_entry& s : m_table) {
      if (s.id != static_cast<Vrst_uint32>(id)) continue;
      if (s.elem_size != sizeof(T))
        fail("section " + std::to_string(s.id) + ": elem_size " + std::to_string(s.elem_size) +
             " != " + std::to_string(sizeof(T)));
      return {reinterpret_cast<const T*>(m_file.data() + s.offset), static_cast<Vrst_size>(s.count)};
    }
    fail("missing section " + std::to_string(static_cast<Vrst_uint32>(id)));
  }

 private:
  /// Header pointer without checks; open() checks the size before using it.
  /// @return Start of the mapping as a header.
  const Vrst_file_header* header_ptr() const { return reinterpret_cast<const Vrst_file_header*>(m_file.data()); }
  /// Reports a broken file.
  /// @param msg What is wrong.
  /// @throws std::runtime_error with the file path as prefix, always.
  [[noreturn]] void fail(const std::string& msg) const { throw std::runtime_error(m_file.path() + ": " + msg); }

  /// Owns the mapping that every span returned by section() points into.
  Vrst_mapped_file m_file;
  /// Validated section table inside the mapping. Kept so section() is a short linear
  /// search with no copy; all entries were bounds-checked in open().
  std::span<const Vrst_section_entry> m_table;
};

/// Throws when a consistency check fails.
/// @param ok Check result; nothing happens when true.
/// @param path File the check is about, for the message.
/// @param what Description of the failed check.
/// @throws std::runtime_error when ok is false.
inline void require(bool ok, const std::string& path, const char* what) {
  if (!ok) throw std::runtime_error(path + ": " + what);
}

/// Typed view of graph.dat: vertex positions plus forward and backward CSR.
/// The routing hot data, kept in its own file so a route query touches only these
/// pages and never the attributes, geometry or names.
class Vrst_graph {
 public:
  /// Maps graph.dat and checks that the CSR arrays agree in size.
  /// @param path Path of graph.dat.
  /// @throws std::runtime_error on a header, section or size mismatch.
  void open(const std::string& path) {
    m_file.open(path, FileType::Graph);
    m_vertex_coord = m_file.section<Vrst_coord>(SectionId::VertexCoord);
    m_fwd_first_out = m_file.section<Vrst_uint32>(SectionId::FwdFirstOut);
    m_fwd_arcs = m_file.section<Vrst_fwd_arc>(SectionId::FwdArcs);
    m_bwd_first_in = m_file.section<Vrst_uint32>(SectionId::BwdFirstIn);
    m_bwd_arcs = m_file.section<Vrst_bwd_arc>(SectionId::BwdArcs);
    auto bb = m_file.section<Vrst_bbox>(SectionId::Bbox);
    const Vrst_size V = m_vertex_coord.size();
    require(m_fwd_first_out.size() == V + 1, path, "FWD_FIRST_OUT size != V+1");
    require(m_bwd_first_in.size() == V + 1, path, "BWD_FIRST_IN size != V+1");
    require(m_bwd_arcs.size() == m_fwd_arcs.size(), path, "BWD_ARCS size != FWD_ARCS size");
    require(bb.size() == 1, path, "BBOX count != 1");
    m_bbox = bb[0];
  }

  /// Number of vertices.
  /// @return V.
  Vrst_uint32 num_vertices() const { return static_cast<Vrst_uint32>(m_vertex_coord.size()); }
  /// Number of directed arcs.
  /// @return A.
  Vrst_uint32 num_arcs() const { return static_cast<Vrst_uint32>(m_fwd_arcs.size()); }
  /// Arcs leaving a vertex.
  /// @param v Vertex id, less than V.
  /// @return Contiguous run of forward arcs with tail v.
  std::span<const Vrst_fwd_arc> out_arcs(Vrst_uint32 v) const {
    return m_fwd_arcs.subspan(m_fwd_first_out[v], m_fwd_first_out[v + 1] - m_fwd_first_out[v]);
  }
  /// Arcs entering a vertex.
  /// @param v Vertex id, less than V.
  /// @return Contiguous run of backward arcs with head v.
  std::span<const Vrst_bwd_arc> in_arcs(Vrst_uint32 v) const {
    return m_bwd_arcs.subspan(m_bwd_first_in[v], m_bwd_first_in[v + 1] - m_bwd_first_in[v]);
  }

  /// @copydoc m_file
  /// @return The mapped file.
  const Vrst_nav_file& file() const { return m_file; }
  /// @copydoc m_vertex_coord
  /// @return Span into the mapping, valid while this object is open.
  std::span<const Vrst_coord> vertex_coord() const { return m_vertex_coord; }
  /// @copydoc m_fwd_first_out
  /// @return Span into the mapping, valid while this object is open.
  std::span<const Vrst_uint32> fwd_first_out() const { return m_fwd_first_out; }
  /// @copydoc m_fwd_arcs
  /// @return Span into the mapping, valid while this object is open.
  std::span<const Vrst_fwd_arc> fwd_arcs() const { return m_fwd_arcs; }
  /// @copydoc m_bwd_first_in
  /// @return Span into the mapping, valid while this object is open.
  std::span<const Vrst_uint32> bwd_first_in() const { return m_bwd_first_in; }
  /// @copydoc m_bwd_arcs
  /// @return Span into the mapping, valid while this object is open.
  std::span<const Vrst_bwd_arc> bwd_arcs() const { return m_bwd_arcs; }
  /// @copydoc m_bbox
  /// @return The extent.
  const Vrst_bbox& bbox() const { return m_bbox; }

 private:
  /// Keeps graph.dat mapped; all spans below point into it.
  Vrst_nav_file m_file;
  /// Position of each vertex. A* needs it for the distance heuristic, and snapping
  /// needs it without opening geom.dat.
  std::span<const Vrst_coord> m_vertex_coord;
  /// CSR row starts, V+1 entries: out-arcs of v are [fwd_first_out[v], [v+1]). The
  /// extra last entry (= A) removes the special case for the last vertex.
  std::span<const Vrst_uint32> m_fwd_first_out;
  /// All arcs sorted by tail; the forward search reads one contiguous run per vertex.
  std::span<const Vrst_fwd_arc> m_fwd_arcs;
  /// Same as fwd_first_out for incoming arcs. Without a reverse index, finding the
  /// arcs into v means scanning all A arcs, which kills backward and bidirectional
  /// search.
  std::span<const Vrst_uint32> m_bwd_first_in;
  /// Incoming arcs sorted by head.
  std::span<const Vrst_bwd_arc> m_bwd_arcs;
  /// Extent of the source data (copied, it is one element).
  Vrst_bbox m_bbox{};
};

/// Typed view of edges.dat: per-segment attributes, end vertices and OSM way id,
/// all indexed by segment id (Vrst_fwd_arc::seg & kSegIdMask).
/// Used after routing (guidance, display) and for snapping, not during the search.
class Vrst_edges {
 public:
  /// Maps edges.dat and checks that all per-segment arrays have S entries.
  /// @param path Path of edges.dat.
  /// @throws std::runtime_error on a header, section or size mismatch.
  void open(const std::string& path) {
    m_file.open(path, FileType::Edges);
    m_attr = m_file.section<Vrst_seg_attr>(SectionId::SegAttr);
    m_vertices = m_file.section<Vrst_seg_vertices>(SectionId::SegVertices);
    m_osm_way = m_file.section<Vrst_uint64>(SectionId::SegOsmWay);
    require(m_vertices.size() == m_attr.size(), path, "SEG_VERTICES size != S");
    require(m_osm_way.size() == m_attr.size(), path, "SEG_OSM_WAY size != S");
  }

  /// Number of segments.
  /// @return S.
  Vrst_uint32 num_segments() const { return static_cast<Vrst_uint32>(m_attr.size()); }

  /// @copydoc m_file
  /// @return The mapped file.
  const Vrst_nav_file& file() const { return m_file; }
  /// @copydoc m_attr
  /// @return Span into the mapping, valid while this object is open.
  std::span<const Vrst_seg_attr> attr() const { return m_attr; }
  /// @copydoc m_vertices
  /// @return Span into the mapping, valid while this object is open.
  std::span<const Vrst_seg_vertices> vertices() const { return m_vertices; }
  /// @copydoc m_osm_way
  /// @return Span into the mapping, valid while this object is open.
  std::span<const Vrst_uint64> osm_way() const { return m_osm_way; }

 private:
  /// Keeps edges.dat mapped.
  Vrst_nav_file m_file;
  /// Per-segment attributes, indexed by segment id.
  std::span<const Vrst_seg_attr> m_attr;
  /// Segment end vertices (a, b), indexed by segment id.
  std::span<const Vrst_seg_vertices> m_vertices;
  /// Source OSM way per segment. OSM turn restrictions reference way ids, so mapping
  /// them onto arcs later is impossible without it; also used to compare with
  /// openstreetmap.org when debugging.
  std::span<const Vrst_uint64> m_osm_way;
};

/// Typed view of geom.dat: the shape points of every segment.
/// Shapes are by far the largest data and are needed only for drawing, snapping and
/// distance along a road, so they live apart from the graph.
class Vrst_geom {
 public:
  /// Maps geom.dat.
  /// @param path Path of geom.dat.
  /// @throws std::runtime_error on a header or section mismatch, or an empty SEG_FIRST_POINT.
  void open(const std::string& path) {
    m_file.open(path, FileType::Geom);
    m_seg_first_point = m_file.section<Vrst_uint32>(SectionId::SegFirstPoint);
    m_points = m_file.section<Vrst_coord>(SectionId::Points);
    require(!m_seg_first_point.empty(), path, "SEG_FIRST_POINT empty");
  }

  /// Shape of one segment.
  /// @param s Segment id, less than S.
  /// @return Points in stored (a -> b) order; the first is vertex a, the last vertex b.
  std::span<const Vrst_coord> segment_points(Vrst_uint32 s) const {
    return m_points.subspan(m_seg_first_point[s], m_seg_first_point[s + 1] - m_seg_first_point[s]);
  }

  /// @copydoc m_file
  /// @return The mapped file.
  const Vrst_nav_file& file() const { return m_file; }
  /// @copydoc m_seg_first_point
  /// @return Span into the mapping, valid while this object is open.
  std::span<const Vrst_uint32> seg_first_point() const { return m_seg_first_point; }
  /// @copydoc m_points
  /// @return Span into the mapping, valid while this object is open.
  std::span<const Vrst_coord> points() const { return m_points; }

 private:
  /// Keeps geom.dat mapped. Separate file so the router never pages in shape points.
  Vrst_nav_file m_file;
  /// Start of each segment's point run, S+1 entries (last = P). Segments have a
  /// variable number of points, so a prefix array is the only O(1) lookup.
  std::span<const Vrst_uint32> m_seg_first_point;
  /// All shape points, segment after segment, both end points included so a segment
  /// can be drawn or measured without looking up its vertices.
  std::span<const Vrst_coord> m_points;
};

/// Typed view of names.dat: street names and road refs, looked up by id (0 = "").
/// Ids keep Vrst_seg_attr fixed-size; each distinct string is stored once.
class Vrst_names {
 public:
  /// Maps names.dat and checks that the offsets cover the string data exactly.
  /// @param path Path of names.dat.
  /// @throws std::runtime_error on a header, section or offset mismatch.
  void open(const std::string& path) {
    m_file.open(path, FileType::Names);
    m_offsets = m_file.section<Vrst_uint32>(SectionId::StrOffsets);
    m_data = m_file.section<char>(SectionId::StrData);
    require(m_offsets.size() >= 2, path, "STR_OFFSETS needs at least 2 entries");
    require(m_offsets.back() == m_data.size(), path, "STR_OFFSETS last != data size");
  }

  /// Number of strings.
  /// @return N, including the empty string 0.
  Vrst_uint32 size() const { return static_cast<Vrst_uint32>(m_offsets.size() - 1); }
  /// String by id.
  /// @param id String id, less than size(); 0 is "".
  /// @return View into the mapping; not NUL-terminated.
  std::string_view get(Vrst_uint32 id) const {
    return {m_data.data() + m_offsets[id], m_offsets[id + 1] - m_offsets[id]};
  }

  /// @copydoc m_file
  /// @return The mapped file.
  const Vrst_nav_file& file() const { return m_file; }
  /// @copydoc m_offsets
  /// @return Span into the mapping, valid while this object is open.
  std::span<const Vrst_uint32> offsets() const { return m_offsets; }
  /// @copydoc m_data
  /// @return Span into the mapping, valid while this object is open.
  std::span<const char> data() const { return m_data; }

 private:
  /// Keeps names.dat mapped.
  Vrst_nav_file m_file;
  /// Start of string i, N+1 entries. Length is offsets[i+1] - offsets[i], so strings
  /// need no terminator and may contain any UTF-8.
  std::span<const Vrst_uint32> m_offsets;
  /// All strings back to back, deduplicated (one copy of "Avinguda Meritxell" for
  /// all its segments).
  std::span<const char> m_data;
};

/// All four files of one import, opened together.
/// The normal entry point for tools: besides opening each file it checks that all
/// share one build_id and agree on the segment count, the mismatches that each file
/// on its own cannot detect.
class Vrst_nav_data {
 public:
  /// Opens all four files and checks that they belong together.
  /// @param dir Directory with graph.dat, edges.dat, geom.dat and names.dat.
  /// @throws std::runtime_error if a file is broken, the build_ids differ or the
  ///         segment counts disagree.
  void open(const std::filesystem::path& dir) {
    open_file(FileType::Graph, dir);
    open_file(FileType::Edges, dir);
    open_file(FileType::Geom, dir);
    open_file(FileType::Names, dir);
    const Vrst_uint64 id = m_graph.file().build_id();
    if (m_edges.file().build_id() != id || m_geom.file().build_id() != id || m_names.file().build_id() != id)
      throw std::runtime_error(dir.string() + ": files come from different builds (build_id mismatch)");
    if (m_geom.seg_first_point().size() != Vrst_size(m_edges.num_segments()) + 1)
      throw std::runtime_error(dir.string() + ": SEG_FIRST_POINT size != S+1");
  }

  /// Opens one file without the cross-file checks. For navcheck, which opens each file
  /// on its own so that every broken file is reported, not only the first.
  /// @param type Which file to open.
  /// @param dir Directory with the nav files.
  /// @throws std::runtime_error if the file is broken.
  void open_file(FileType type, const std::filesystem::path& dir) {
    const std::string path = (dir / file_name_for(type)).string();
    switch (type) {
      case FileType::Graph: m_graph.open(path); break;
      case FileType::Edges: m_edges.open(path); break;
      case FileType::Geom: m_geom.open(path); break;
      case FileType::Names: m_names.open(path); break;
    }
  }

  /// @copydoc m_graph
  /// @return The graph.dat view.
  const Vrst_graph& graph() const { return m_graph; }
  /// @copydoc m_edges
  /// @return The edges.dat view.
  const Vrst_edges& edges() const { return m_edges; }
  /// @copydoc m_geom
  /// @return The geom.dat view.
  const Vrst_geom& geom() const { return m_geom; }
  /// @copydoc m_names
  /// @return The names.dat view.
  const Vrst_names& names() const { return m_names; }

 private:
  /// Routing hot data.
  Vrst_graph m_graph;
  /// Segment attributes, needed after routing (guidance, display).
  Vrst_edges m_edges;
  /// Shape points, needed for drawing and snapping.
  Vrst_geom m_geom;
  /// Street names and refs.
  Vrst_names m_names;
};

}  // namespace nav
