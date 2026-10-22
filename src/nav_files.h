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

class NavFile {
 public:
  void open(const std::string& path, FileType type) {
    file_.open(path);
    const size_t size = file_.size();
    if (size < sizeof(FileHeader)) fail("file too small");
    const auto* h = header_ptr();
    if (std::memcmp(h->magic, magic_for(type), 8) != 0) fail("bad magic");
    if (h->format_version != kFormatVersion)
      fail("unsupported format_version " + std::to_string(h->format_version));
    if (h->file_type != static_cast<uint32_t>(type)) fail("bad file_type");

    const uint64_t table_end = sizeof(FileHeader) + uint64_t(h->section_count) * sizeof(SectionEntry);
    if (table_end > size) fail("section table out of bounds");
    table_ = {reinterpret_cast<const SectionEntry*>(file_.data() + sizeof(FileHeader)), h->section_count};

    for (const SectionEntry& s : table_) {
      const std::string sid = "section " + std::to_string(s.id);
      if (s.elem_size == 0) fail(sid + ": elem_size 0");
      if (s.offset % kSectionAlign != 0) fail(sid + ": offset not 64-aligned");
      if (s.offset < table_end || s.offset > size) fail(sid + ": offset out of bounds");
      if (s.count > (size - s.offset) / s.elem_size) fail(sid + ": data out of bounds");
    }
  }

  const FileHeader& header() const { return *header_ptr(); }
  uint64_t build_id() const { return header().build_id; }
  const std::string& path() const { return file_.path(); }
  size_t file_size() const { return file_.size(); }

  template <class T>
  std::span<const T> section(SectionId id) const {
    for (const SectionEntry& s : table_) {
      if (s.id != static_cast<uint32_t>(id)) continue;
      if (s.elem_size != sizeof(T))
        fail("section " + std::to_string(s.id) + ": elem_size " + std::to_string(s.elem_size) +
             " != " + std::to_string(sizeof(T)));
      return {reinterpret_cast<const T*>(file_.data() + s.offset), static_cast<size_t>(s.count)};
    }
    fail("missing section " + std::to_string(static_cast<uint32_t>(id)));
  }

 private:
  const FileHeader* header_ptr() const { return reinterpret_cast<const FileHeader*>(file_.data()); }
  [[noreturn]] void fail(const std::string& msg) const { throw std::runtime_error(file_.path() + ": " + msg); }

  MappedFile file_;
  std::span<const SectionEntry> table_;
};

inline void require(bool ok, const std::string& path, const char* what) {
  if (!ok) throw std::runtime_error(path + ": " + what);
}

struct Graph {
  NavFile file;
  std::span<const Coord> vertex_coord;
  std::span<const uint32_t> fwd_first_out;
  std::span<const FwdArc> fwd_arcs;
  std::span<const uint32_t> bwd_first_in;
  std::span<const BwdArc> bwd_arcs;
  BBox bbox{};

  uint32_t num_vertices() const { return static_cast<uint32_t>(vertex_coord.size()); }
  uint32_t num_arcs() const { return static_cast<uint32_t>(fwd_arcs.size()); }
  std::span<const FwdArc> out_arcs(uint32_t v) const {
    return fwd_arcs.subspan(fwd_first_out[v], fwd_first_out[v + 1] - fwd_first_out[v]);
  }
  std::span<const BwdArc> in_arcs(uint32_t v) const {
    return bwd_arcs.subspan(bwd_first_in[v], bwd_first_in[v + 1] - bwd_first_in[v]);
  }

  void open(const std::string& path) {
    file.open(path, FileType::Graph);
    vertex_coord = file.section<Coord>(SectionId::VertexCoord);
    fwd_first_out = file.section<uint32_t>(SectionId::FwdFirstOut);
    fwd_arcs = file.section<FwdArc>(SectionId::FwdArcs);
    bwd_first_in = file.section<uint32_t>(SectionId::BwdFirstIn);
    bwd_arcs = file.section<BwdArc>(SectionId::BwdArcs);
    auto bb = file.section<BBox>(SectionId::Bbox);
    const size_t V = vertex_coord.size();
    require(fwd_first_out.size() == V + 1, path, "FWD_FIRST_OUT size != V+1");
    require(bwd_first_in.size() == V + 1, path, "BWD_FIRST_IN size != V+1");
    require(bwd_arcs.size() == fwd_arcs.size(), path, "BWD_ARCS size != FWD_ARCS size");
    require(bb.size() == 1, path, "BBOX count != 1");
    bbox = bb[0];
  }
};

struct Edges {
  NavFile file;
  std::span<const SegAttr> attr;
  std::span<const SegVertices> vertices;
  std::span<const uint64_t> osm_way;

  uint32_t num_segments() const { return static_cast<uint32_t>(attr.size()); }

  void open(const std::string& path) {
    file.open(path, FileType::Edges);
    attr = file.section<SegAttr>(SectionId::SegAttr);
    vertices = file.section<SegVertices>(SectionId::SegVertices);
    osm_way = file.section<uint64_t>(SectionId::SegOsmWay);
    require(vertices.size() == attr.size(), path, "SEG_VERTICES size != S");
    require(osm_way.size() == attr.size(), path, "SEG_OSM_WAY size != S");
  }
};

struct Geom {
  NavFile file;
  std::span<const uint32_t> seg_first_point;
  std::span<const Coord> points;

  // Points of segment s in stored (a -> b) order.
  std::span<const Coord> segment_points(uint32_t s) const {
    return points.subspan(seg_first_point[s], seg_first_point[s + 1] - seg_first_point[s]);
  }

  void open(const std::string& path) {
    file.open(path, FileType::Geom);
    seg_first_point = file.section<uint32_t>(SectionId::SegFirstPoint);
    points = file.section<Coord>(SectionId::Points);
    require(!seg_first_point.empty(), path, "SEG_FIRST_POINT empty");
  }
};

struct Names {
  NavFile file;
  std::span<const uint32_t> offsets;
  std::span<const char> data;

  uint32_t size() const { return static_cast<uint32_t>(offsets.size() - 1); }
  std::string_view get(uint32_t id) const {
    return {data.data() + offsets[id], offsets[id + 1] - offsets[id]};
  }

  void open(const std::string& path) {
    file.open(path, FileType::Names);
    offsets = file.section<uint32_t>(SectionId::StrOffsets);
    data = file.section<char>(SectionId::StrData);
    require(offsets.size() >= 2, path, "STR_OFFSETS needs at least 2 entries");
    require(offsets.back() == data.size(), path, "STR_OFFSETS last != data size");
  }
};

// All four files of one import.
struct NavData {
  Graph graph;
  Edges edges;
  Geom geom;
  Names names;

  void open(const std::filesystem::path& dir) {
    graph.open((dir / file_name_for(FileType::Graph)).string());
    edges.open((dir / file_name_for(FileType::Edges)).string());
    geom.open((dir / file_name_for(FileType::Geom)).string());
    names.open((dir / file_name_for(FileType::Names)).string());
    const uint64_t id = graph.file.build_id();
    if (edges.file.build_id() != id || geom.file.build_id() != id || names.file.build_id() != id)
      throw std::runtime_error(dir.string() + ": files come from different builds (build_id mismatch)");
    if (geom.seg_first_point.size() != size_t(edges.num_segments()) + 1)
      throw std::runtime_error(dir.string() + ": SEG_FIRST_POINT size != S+1");
  }
};

}  // namespace nav
