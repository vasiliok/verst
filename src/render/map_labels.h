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
#include <array>
#include <cmath>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

#include "render/glyph_atlas.h"
#include "provider/map_provider.h"
#include "provider/map_scene.h"

namespace nav {

/// Font id of regular text in the label atlas (index into kLabelFontFiles).
inline constexpr int kFontRegular = 0;
/// Font id of bold text in the label atlas.
inline constexpr int kFontBold = 1;
/// Font files of the label atlas, by font id; looked up in the front end's font dir
/// (CMake downloads them into `<build>/fonts`, the iOS app bundles them).
inline constexpr std::array<std::string_view, 2> kLabelFontFiles = {"DejaVuSans.ttf", "DejaVuSans-Bold.ttf"};

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
  /// @param places Places from Vrst_map_provider::places(), most important first.
  /// @param frame Map frame, for the world coordinates.
  void build(const std::vector<Vrst_map_place>& places, const Vrst_map_frame& frame);

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
                                              const Vrst_glyph_atlas& atlas);

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
