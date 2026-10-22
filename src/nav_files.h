// Typed, zero-copy views over the mapped nav files. Throws std::runtime_error.
//
// Loading checks magic, version, file type, section bounds/alignment,
// elem_size and O(1) count consistency. Deep checks live in navcheck.
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

  Vrst_mapped_file file_;
  std::span<const Vrst_section_entry> table_;
};

inline void require(bool ok, const std::string& path, const char* what) {
  if (!ok) throw std::runtime_error(path + ": " + what);
}

struct Vrst_graph {
  Vrst_nav_file file;
  std::span<const Vrst_coord> vertex_coord;
  std::span<const Vrst_uint32> fwd_first_out;
  std::span<const Vrst_fwd_arc> fwd_arcs;
  std::span<const Vrst_uint32> bwd_first_in;
  std::span<const Vrst_bwd_arc> bwd_arcs;
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

struct Vrst_edges {
  Vrst_nav_file file;
  std::span<const Vrst_seg_attr> attr;
  std::span<const Vrst_seg_vertices> vertices;
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

struct Vrst_geom {
  Vrst_nav_file file;
  std::span<const Vrst_uint32> seg_first_point;
  std::span<const Vrst_coord> points;

  // Points of segment s in stored (a -> b) order.
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

struct Vrst_names {
  Vrst_nav_file file;
  std::span<const Vrst_uint32> offsets;
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

// All four files of one import.
struct Vrst_nav_data {
  Vrst_graph graph;
  Vrst_edges edges;
  Vrst_geom geom;
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
