/// @file
/// Vrst_map_labels: collecting places and laying out label glyphs per frame.
/// Declarations and documentation are in map_labels.h.
#include "map_labels.h"

#include <algorithm>

namespace nav {

void Vrst_map_labels::build(const std::vector<Vrst_map_place>& places, const Vrst_map_frame& frame) {
  m_kind.clear();
  m_world.clear();
  m_cos_lat.clear();
  m_text.clear();
  m_text_first.assign(1, 0);
  for (const Vrst_map_place& p : places) {
    m_kind.push_back(static_cast<Vrst_uint8>(p.kind));
    m_world.push_back(frame.to_world(p.pos));
    m_cos_lat.push_back(std::cos(p.pos.lat * kPi / 180));
    const std::vector<char32_t> t = decode_utf8(p.name);
    m_text.insert(m_text.end(), t.begin(), t.end());
    m_text_first.push_back(static_cast<Vrst_uint32>(m_text.size()));
  }
  m_width.assign(m_kind.size(), 0);
}

const std::vector<Vrst_text_vertex>& Vrst_map_labels::layout(const Vrst_view& v, int fb_w, int fb_h,
                                                             Vrst_float64 px_per_point, const Vrst_glyph_atlas& atlas) {
  m_verts.clear();
  m_placed.clear();
  m_count = 0;
  const Vrst_float64 pad = 4 * px_per_point;
  // Greedy placement, one place per pass. Places come most important first (osm2nav
  // sorts them by kind, then population), so a city always wins over a village nearby.
  // For each place:
  //  1. skip it when zoomed out past its kind's max_metres_per_point;
  //  2. project it to the screen and size its box from the text width and font size,
  //     plus `pad` so labels keep a gap between them;
  //  3. skip it when the box is off screen or overlaps a label already placed;
  //  4. otherwise keep the box and emit two triangles per glyph, walking the pen along
  //     the baseline by each glyph's advance.
  for (Vrst_size i = 0; i < m_kind.size(); ++i) {
    const Vrst_label_style& st = kLabelStyles[m_kind[i]];
    if (px_per_point * m_cos_lat[i] / v.scale > st.max_metres_per_point) continue;
    const Vrst_vec2d p = world_to_screen(v, fb_w, fb_h, m_world[i]);
    const Vrst_float64 size_px = st.size_pt * px_per_point;
    const Vrst_float64 s = size_px / Vrst_glyph_atlas::kBasePx;
    const Vrst_float64 half_w = m_width[i] * s / 2, half_h = size_px * 0.6;
    const Vrst_rect r{p.x - half_w - pad, p.y - half_h - pad, p.x + half_w + pad, p.y + half_h + pad};
    if (r.x1 < 0 || r.y1 < 0 || r.x0 > fb_w || r.y0 > fb_h) continue;
    if (std::ranges::any_of(m_placed, [&](const Vrst_rect& o) { return overlaps(r, o); })) continue;
    m_placed.push_back(r);
    ++m_count;

    // Baseline a bit below the anchor so the capitals are centered on the place.
    const Vrst_float64 baseline = p.y + size_px * 0.36;
    Vrst_float64 pen = p.x - half_w;
    for (const char32_t c : text(i)) {
      const std::optional<Vrst_glyph> g = atlas.find(st.font, c);
      if (!g) continue;
      if (g->w > 0) {
        const auto x0 = static_cast<Vrst_float32>(pen + g->left * s);
        const auto y0 = static_cast<Vrst_float32>(baseline - g->top * s);
        const auto x1 = static_cast<Vrst_float32>(x0 + g->w * s);
        const auto y1 = static_cast<Vrst_float32>(y0 + g->h * s);
        m_verts.insert(m_verts.end(), {{x0, y0, g->u0, g->v0}, {x1, y0, g->u1, g->v0}, {x0, y1, g->u0, g->v1},
                                       {x1, y0, g->u1, g->v0}, {x1, y1, g->u1, g->v1}, {x0, y1, g->u0, g->v1}});
      }
      pen += g->advance * s;
    }
  }
  return m_verts;
}

}  // namespace nav
