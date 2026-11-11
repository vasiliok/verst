/// @file
/// Vrst_map_frame: world origin, start views and the area a camera shows.
/// Declarations and documentation are in map_scene.h.
#include "map_scene.h"

namespace nav {

Vrst_map_frame::Vrst_map_frame(const Vrst_geo_box& extent) : m_extent(extent) {
  const Vrst_vec2d lo = mercator(extent.south_west), hi = mercator(extent.north_east);
  m_center = {(lo.x + hi.x) / 2, (lo.y + hi.y) / 2};
  m_half_extent = {(hi.x - lo.x) / 2, (hi.y - lo.y) / 2};
}

Vrst_float64 Vrst_map_frame::fit_scale(int fb_w, int fb_h) const {
  const Vrst_float64 sx = fb_w / (2 * std::max(m_half_extent.x, 1.0));
  const Vrst_float64 sy = fb_h / (2 * std::max(m_half_extent.y, 1.0));
  return 0.95 * std::min(sx, sy);
}

Vrst_view Vrst_map_frame::view_at(Vrst_latlon p, Vrst_float64 metres_per_point, Vrst_float64 px_per_point) const {
  // Mercator metres are 1/cos(lat) ground metres; scale is framebuffer px per Mercator metre.
  return {to_world(p), px_per_point * std::cos(p.lat * kPi / 180) / metres_per_point, 0};
}

Vrst_view Vrst_map_frame::start_view(Vrst_latlon p, Vrst_float64 metres_per_point, Vrst_float64 px_per_point,
                                     int fb_w, int fb_h) const {
  if (!contains(m_extent, p)) return {{0, 0}, fit_scale(fb_w, fb_h), 0};
  return view_at(p, metres_per_point, px_per_point);
}

Vrst_map_area Vrst_map_frame::area_of(const Vrst_view& v, int fb_w, int fb_h) const {
  Vrst_map_area a;
  const Vrst_float64 w = static_cast<Vrst_float64>(fb_w), h = static_cast<Vrst_float64>(fb_h);
  const std::array<Vrst_vec2d, 4> screen = {{{0, 0}, {w, 0}, {0, h}, {w, h}}};
  for (Vrst_size i = 0; i < screen.size(); ++i) {
    const Vrst_vec2d c = screen_to_world(v, fb_w, fb_h, screen[i].x, screen[i].y);
    a.corners[i] = {c.x + m_center.x, c.y + m_center.y};
  }
  a.center = {v.center.x + m_center.x, v.center.y + m_center.y};
  a.metres_per_px = 1.0 / v.scale;
  return a;
}

}  // namespace nav
