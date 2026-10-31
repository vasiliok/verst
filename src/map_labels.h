/// @file
/// Place name labels (cities, towns, villages, hamlets): which ones to show at the
/// current zoom, where, and the glyph quads to draw them. Shared by navview and the
/// iOS app; no graphics API calls.
///
/// Usage: build() after the map is loaded, then Vrst_glyph_atlas::build() with
/// codepoints(), then measure() with the atlas, then layout() every frame and hand
/// the vertices to Vrst_map_renderer::draw_labels().
#pragma once

#include <algorithm>
#include <cmath>
#include <span>
#include <vector>

#include "glyph_atlas.h"
#include "map_scene.h"
#include "nav_files.h"

namespace nav {

/// Font id of regular text in the label atlas (index into kLabelFontFiles).
inline constexpr int kFontRegular = 0;
/// Font id of bold text in the label atlas.
inline constexpr int kFontBold = 1;
/// Font files of the label atlas, by font id; looked up in the front end's font dir
/// (CMake downloads them into `<build>/fonts`, the iOS app bundles them).
inline constexpr const char* kLabelFontFiles[] = {"DejaVuSans.ttf", "DejaVuSans-Bold.ttf"};

/// How one place kind is labelled.
struct Vrst_label_style {
  /// Font id (kFontRegular / kFontBold).
  int font;
  /// Text height in screen points (scaled by the Retina factor).
  Vrst_float32 size_pt;
  /// Shown only while the map shows at most this many ground metres per screen
  /// point; keeps small places from crowding a zoomed-out map.
  Vrst_float64 max_metres_per_point;
};

/// Label style per PlaceKind: bigger, bolder and visible from further out for larger places.
inline constexpr Vrst_label_style kLabelStyles[kPlaceKindCount] = {
    {kFontBold, 19, 1e9},   // City: always
    {kFontBold, 16, 250},   // Town
    {kFontRegular, 14, 45}, // Village
    {kFontRegular, 12, 9},  // Hamlet
};

/// Vertex of a label glyph quad: framebuffer position and atlas texture coordinate.
/// Labels are laid out in screen space every frame so they stay upright and the
/// same size while the map is zoomed and rotated.
struct Vrst_text_vertex {
  /// Framebuffer px from the left.
  Vrst_float32 x;
  /// Framebuffer px from the top.
  Vrst_float32 y;
  /// Atlas texture coordinate, 0..1.
  Vrst_float32 u;
  Vrst_float32 v;  ///< Atlas texture coordinate, 0..1, from the top.
};
static_assert(sizeof(Vrst_text_vertex) == 16);

/// Labels of all places of a map and the per-frame layout.
///
/// Without a collision test the thousands of village names of a region would print
/// over each other; the layout walks places by priority (Vrst_place order: cities
/// first, then by population) and drops every label that would overlap one already
/// placed, so the important names always win.
class Vrst_map_labels {
 public:
  /// Collects places, their world positions and decoded names.
  /// @param d Opened nav files; places and names are copied, d may close afterwards.
  /// @param mesh Built mesh, for the world coordinates.
  void build(const Vrst_nav_data& d, const Vrst_map_mesh& mesh) {
    const std::span<const Vrst_place> places = d.names().places();
    m_kind.clear();
    m_world.clear();
    m_cos_lat.clear();
    m_text.clear();
    m_text_first.assign(1, 0);
    for (const Vrst_place& p : places) {
      m_kind.push_back(p.kind);
      m_world.push_back(mesh.to_world(p.pos));
      m_cos_lat.push_back(std::cos(p.pos.lat_e7 * 1e-7 * kPi / 180));
      const std::vector<char32_t> t = decode_utf8(d.names().get(p.name_id));
      m_text.insert(m_text.end(), t.begin(), t.end());
      m_text_first.push_back(Vrst_uint32(m_text.size()));
    }
    m_width.assign(m_kind.size(), 0);
  }

  /// Every character the labels use, for Vrst_glyph_atlas::build().
  /// @return Code points, unsorted, with duplicates.
  const std::vector<char32_t>& codepoints() const { return m_text; }

  /// Measures every label with the atlas. Call once after the atlas is built.
  /// @param atlas Glyph atlas built from codepoints().
  void measure(const Vrst_glyph_atlas& atlas) {
    for (Vrst_size i = 0; i < m_kind.size(); ++i)
      m_width[i] = atlas.measure(kLabelStyles[m_kind[i]].font, text(i));
  }

  /// Chooses the visible labels for a camera and builds their glyph quads.
  /// @param v Camera.
  /// @param fb_w Framebuffer width in px.
  /// @param fb_h Framebuffer height in px.
  /// @param px_per_point Framebuffer pixels per screen point (Retina factor).
  /// @param atlas Glyph atlas used by measure().
  /// @return 6 vertices (two triangles) per glyph, valid until the next layout().
  const std::vector<Vrst_text_vertex>& layout(const Vrst_view& v, int fb_w, int fb_h, Vrst_float64 px_per_point,
                                              const Vrst_glyph_atlas& atlas) {
    m_verts.clear();
    m_placed.clear();
    m_count = 0;
    const Vrst_float64 pad = 4 * px_per_point;
    for (Vrst_size i = 0; i < m_kind.size(); ++i) {
      const Vrst_label_style& st = kLabelStyles[m_kind[i]];
      if (px_per_point * m_cos_lat[i] / v.scale > st.max_metres_per_point) continue;
      const Vrst_vec2d p = world_to_screen(v, fb_w, fb_h, m_world[i]);
      const Vrst_float64 size_px = st.size_pt * px_per_point;
      const Vrst_float64 s = size_px / Vrst_glyph_atlas::kBasePx;
      const Vrst_float64 half_w = m_width[i] * s / 2, half_h = size_px * 0.6;
      const Vrst_rect r{p.x - half_w - pad, p.y - half_h - pad, p.x + half_w + pad, p.y + half_h + pad};
      if (r.x1 < 0 || r.y1 < 0 || r.x0 > fb_w || r.y0 > fb_h) continue;
      if (std::any_of(m_placed.begin(), m_placed.end(), [&](const Vrst_rect& o) { return overlaps(r, o); })) continue;
      m_placed.push_back(r);
      ++m_count;

      // Baseline a bit below the anchor so the capitals are centered on the place.
      const Vrst_float64 baseline = p.y + size_px * 0.36;
      Vrst_float64 pen = p.x - half_w;
      for (const char32_t c : text(i)) {
        const Vrst_glyph* g = atlas.find(st.font, c);
        if (!g) continue;
        if (g->w > 0) {
          const auto x0 = Vrst_float32(pen + g->left * s), y0 = Vrst_float32(baseline - g->top * s);
          const auto x1 = Vrst_float32(x0 + g->w * s), y1 = Vrst_float32(y0 + g->h * s);
          m_verts.insert(m_verts.end(), {{x0, y0, g->u0, g->v0}, {x1, y0, g->u1, g->v0}, {x0, y1, g->u0, g->v1},
                                         {x1, y0, g->u1, g->v0}, {x1, y1, g->u1, g->v1}, {x0, y1, g->u0, g->v1}});
        }
        pen += g->advance * s;
      }
    }
    return m_verts;
  }

  /// Number of places with a label.
  /// @return Count.
  Vrst_size size() const { return m_kind.size(); }
  /// @copydoc m_count
  /// @return Count.
  Vrst_size placed() const { return m_count; }

 private:
  /// Screen rectangle of a placed label, framebuffer px.
  struct Vrst_rect {
    Vrst_float64 x0;  ///< Left.
    Vrst_float64 y0;  ///< Top.
    Vrst_float64 x1;  ///< Right.
    Vrst_float64 y1;  ///< Bottom.
  };

  /// Rectangle intersection test.
  /// @param a First rectangle.
  /// @param b Second rectangle.
  /// @return true if they share any area.
  static bool overlaps(const Vrst_rect& a, const Vrst_rect& b) {
    return a.x0 < b.x1 && b.x0 < a.x1 && a.y0 < b.y1 && b.y0 < a.y1;
  }

  /// Name of label i.
  /// @param i Label index.
  /// @return Code points.
  std::span<const char32_t> text(Vrst_size i) const {
    return {m_text.data() + m_text_first[i], m_text_first[i + 1] - m_text_first[i]};
  }

  /// PlaceKind per label, in priority order (as in names.dat).
  std::vector<Vrst_uint8> m_kind;
  /// Anchor per label, Mercator metres relative to the map center.
  std::vector<Vrst_vec2d> m_world;
  /// cos(latitude) per label: converts the camera scale to ground metres at the place.
  std::vector<Vrst_float64> m_cos_lat;
  /// All names back to back as code points; decoded once, not every frame.
  std::vector<char32_t> m_text;
  /// Start of label i in m_text, size()+1 entries.
  std::vector<Vrst_uint32> m_text_first;
  /// Text width per label at Vrst_glyph_atlas::kBasePx (measure()).
  std::vector<Vrst_float32> m_width;
  /// Output of the last layout().
  std::vector<Vrst_text_vertex> m_verts;
  /// Rectangles placed by the last layout(), for the collision test.
  std::vector<Vrst_rect> m_placed;
  /// Labels placed by the last layout().
  Vrst_size m_count = 0;
};

}  // namespace nav
