/// @file
/// Vrst_map_mesh: building the projected road mesh and the start views.
/// Declarations and documentation are in map_scene.h.
#include "map_scene.h"

#include <utility>

namespace nav {

void Vrst_map_mesh::build(const Vrst_nav_data& d, bool gpu_arrays) {
  const Vrst_graph& g = d.graph();
  const Vrst_edges& e = d.edges();
  const Vrst_geom& geo = d.geom();
  const Vrst_uint32 V = g.num_vertices(), S = e.num_segments();
  const Vrst_size P = geo.points().size();

  // build() works in four steps:
  //  1. Projection and extent (always): every shape point of every road segment is
  //     projected to Web Mercator metres, in double. Their bounding box gives the map
  //     center, the world origin of everything drawn (float vertices relative to it keep
  //     about 1 cm precision across a region), and the half extent used by fit_scale().
  //     With gpu_arrays false (front ends that draw tiles) the function stops here.
  //  2. Line vertices: one per shape point, relative to the center, with the road class
  //     in the low 16 bits of attr and the segment flags (oneway, bridge, ...) in the high 16.
  //  3. Line indices: GL_LINES pairs between neighbouring points of one segment, never
  //     across segments; minor classes first so major roads are drawn on top.
  //  4. Vertex dots: one point per graph vertex; vertices with one segment end (dead
  //     ends) are counted and listed last, so they are drawn on top.

  // 1. Projection and extent.
  Vrst_vec2d lo{1e300, 1e300}, hi{-1e300, -1e300};
  std::vector<Vrst_vec2d> merc(P);
  for (Vrst_size i = 0; i < P; ++i) {
    merc[i] = mercator(geo.points()[i]);
    lo = {std::min(lo.x, merc[i].x), std::min(lo.y, merc[i].y)};
    hi = {std::max(hi.x, merc[i].x), std::max(hi.y, merc[i].y)};
  }
  if (P == 0) lo = hi = {0, 0};
  m_center = {(lo.x + hi.x) / 2, (lo.y + hi.y) / 2};
  m_half_extent = {(hi.x - lo.x) / 2, (hi.y - lo.y) / 2};
  m_line_verts.clear();
  m_line_indices.clear();
  m_point_verts.clear();
  m_dead_ends = 0;
  if (!gpu_arrays) return;

  // 2. Line vertices.
  m_line_verts.assign(P, {});
  for (Vrst_uint32 s = 0; s < S; ++s) {
    const Vrst_uint32 attr = e.attr()[s].road_class | static_cast<Vrst_uint32>(e.attr()[s].flags) << 16;
    for (Vrst_uint32 i = geo.seg_first_point()[s]; i < geo.seg_first_point()[s + 1]; ++i)
      m_line_verts[i] = {static_cast<Vrst_float32>(merc[i].x - m_center.x),
                         static_cast<Vrst_float32>(merc[i].y - m_center.y), attr};
  }

  // 3. Line indices: pairs only inside a segment; minor classes first so major roads draw on top.
  m_line_indices.clear();
  m_line_indices.reserve((P - S) * 2);
  for (int cls = kRoadClassCount - 1; cls >= 0; --cls)
    for (Vrst_uint32 s = 0; s < S; ++s) {
      if (std::cmp_not_equal(e.attr()[s].road_class, cls)) continue;
      for (Vrst_uint32 i = geo.seg_first_point()[s]; i + 1 < geo.seg_first_point()[s + 1]; ++i) {
        m_line_indices.push_back(i);
        m_line_indices.push_back(i + 1);
      }
    }

  // 4. Vertex dots. degree = segment ends at the vertex; 1 = dead end.
  std::vector<Vrst_uint32> degree(V, 0);
  for (const Vrst_seg_vertices& sv : e.vertices()) ++degree[sv.a], ++degree[sv.b];
  m_point_verts.clear();
  m_point_verts.reserve(V);
  m_dead_ends = 0;
  for (int pass = 0; pass < 2; ++pass)  // dead ends last, on top
    for (Vrst_uint32 v = 0; v < V; ++v) {
      const Vrst_uint32 dead = degree[v] == 1;
      if (std::cmp_not_equal(dead, pass)) continue;
      m_dead_ends += dead;
      const Vrst_vec2d m = mercator(g.vertex_coord()[v]);
      m_point_verts.push_back({static_cast<Vrst_float32>(m.x - m_center.x),
                               static_cast<Vrst_float32>(m.y - m_center.y), dead});
    }
}

Vrst_float64 Vrst_map_mesh::fit_scale(int fb_w, int fb_h) const {
  const Vrst_float64 sx = fb_w / (2 * std::max(m_half_extent.x, 1.0));
  const Vrst_float64 sy = fb_h / (2 * std::max(m_half_extent.y, 1.0));
  return 0.95 * std::min(sx, sy);
}

Vrst_view Vrst_map_mesh::view_at(Vrst_coord p, Vrst_float64 metres_per_point, Vrst_float64 px_per_point) const {
  const Vrst_vec2d m = mercator(p);
  const Vrst_float64 lat = p.lat_e7 * 1e-7 * kPi / 180;
  // Mercator metres are 1/cos(lat) ground metres; scale is framebuffer px per Mercator metre.
  return {{m.x - m_center.x, m.y - m_center.y}, px_per_point * std::cos(lat) / metres_per_point, 0};
}

Vrst_view Vrst_map_mesh::start_view(const Vrst_bbox& bbox, Vrst_coord p, Vrst_float64 metres_per_point,
                                    Vrst_float64 px_per_point, int fb_w, int fb_h) const {
  const bool inside = p.lat_e7 >= bbox.min.lat_e7 && p.lat_e7 <= bbox.max.lat_e7 &&
                      p.lon_e7 >= bbox.min.lon_e7 && p.lon_e7 <= bbox.max.lon_e7;
  if (!inside) return {{0, 0}, fit_scale(fb_w, fb_h), 0};
  return view_at(p, metres_per_point, px_per_point);
}

}  // namespace nav
