/// @file
/// Opening and checking the nav files: Vrst_nav_file, Vrst_graph, Vrst_map_layers, Vrst_nav_data.
/// Declarations and documentation are in nav_files.h.
#include "nav_files.h"

namespace nav {

void Vrst_nav_file::open(const std::string& path, FileType type) {
  m_file.open(path);
  const Vrst_size size = m_file.size();
  if (size < sizeof(Vrst_file_header)) fail("file too small");
  std::memcpy(&m_header, m_file.bytes().data(), sizeof m_header);
  const Vrst_file_header& h = m_header;
  if (std::string_view(h.magic, 8) != magic_for(type)) fail("bad magic");
  if (h.format_version != kFormatVersion)
    fail("unsupported format_version " + std::to_string(h.format_version));
  if (h.file_type != static_cast<Vrst_uint32>(type)) fail("bad file_type");

  const Vrst_uint64 table_end = sizeof(Vrst_file_header) + Vrst_uint64(h.section_count) * sizeof(Vrst_section_entry);
  if (table_end > size) fail("section table out of bounds");
  m_table = typed<Vrst_section_entry>(sizeof(Vrst_file_header), h.section_count);

  for (const Vrst_section_entry& s : m_table) {
    const std::string sid = "section " + std::to_string(s.id);
    if (s.elem_size == 0) fail(sid + ": elem_size 0");
    if (s.offset % kSectionAlign != 0) fail(sid + ": offset not 64-aligned");
    if (s.offset < table_end || s.offset > size) fail(sid + ": offset out of bounds");
    if (s.count > (size - s.offset) / s.elem_size) fail(sid + ": data out of bounds");
  }
}

void Vrst_graph::open(const std::string& path) {
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

void Vrst_map_layers::open(const std::string& path) {
  m_file.open(path, FileType::Map);
  m_area_first_point = m_file.section<Vrst_uint32>(SectionId::AreaFirstPoint);
  m_area_first_index = m_file.section<Vrst_uint32>(SectionId::AreaFirstIndex);
  m_area_kind = m_file.section<Vrst_uint8>(SectionId::AreaKinds);
  m_area_points = m_file.section<Vrst_coord>(SectionId::AreaPoints);
  m_area_indices = m_file.section<Vrst_uint32>(SectionId::AreaIndices);
  m_line_first_point = m_file.section<Vrst_uint32>(SectionId::LineFirstPoint);
  m_line_attr = m_file.section<Vrst_line_attr>(SectionId::LineAttr);
  m_line_points = m_file.section<Vrst_coord>(SectionId::LinePoints);
  m_area_outer_size = m_file.section<Vrst_uint32>(SectionId::AreaOuterSize);
  const Vrst_size R = m_area_kind.size(), L = m_line_attr.size();
  require(m_area_first_point.size() == R + 1, path, "AREA_FIRST_POINT size != R+1");
  require(m_area_first_index.size() == R + 1, path, "AREA_FIRST_INDEX size != R+1");
  require(m_area_outer_size.size() == R, path, "AREA_OUTER_SIZE size != R");
  require(m_area_first_point.back() == m_area_points.size(), path, "AREA_FIRST_POINT last != points");
  require(m_area_first_index.back() == m_area_indices.size(), path, "AREA_FIRST_INDEX last != indices");
  require(m_line_first_point.size() == L + 1, path, "LINE_FIRST_POINT size != L+1");
  require(m_line_first_point.back() == m_line_points.size(), path, "LINE_FIRST_POINT last != points");
}

void Vrst_nav_data::open(const std::filesystem::path& dir) {
  open_file(FileType::Graph, dir);
  open_file(FileType::Edges, dir);
  open_file(FileType::Geom, dir);
  open_file(FileType::Names, dir);
  m_has_map = std::filesystem::exists(dir / file_name_for(FileType::Map));
  if (m_has_map) open_file(FileType::Map, dir);
  const Vrst_uint64 id = m_graph.file().build_id();
  if (m_edges.file().build_id() != id || m_geom.file().build_id() != id || m_names.file().build_id() != id ||
      (m_has_map && m_map.file().build_id() != id))
    throw std::runtime_error(dir.string() + ": files come from different builds (build_id mismatch)");
  if (m_geom.seg_first_point().size() != Vrst_size(m_edges.num_segments()) + 1)
    throw std::runtime_error(dir.string() + ": SEG_FIRST_POINT size != S+1");
}

void Vrst_nav_data::open_file(FileType type, const std::filesystem::path& dir) {
  const std::string path = (dir / file_name_for(type)).string();
  switch (type) {
    case FileType::Graph: m_graph.open(path); break;
    case FileType::Edges: m_edges.open(path); break;
    case FileType::Geom: m_geom.open(path); break;
    case FileType::Names: m_names.open(path); break;
    case FileType::Map: m_map.open(path); break;
  }
}

}  // namespace nav
