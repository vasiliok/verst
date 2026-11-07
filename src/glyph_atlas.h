/// @file
/// Signed-distance-field glyph atlas rendered with FreeType, shared by navview and
/// the iOS app.
///
/// Glyphs are rendered once at kBasePx with FreeType's SDF renderer and packed into
/// one 8-bit texture. A distance field instead of plain coverage lets one atlas draw
/// every label size sharply and gives the dark halo around the text for free in the
/// shader. Nothing here calls a graphics API; Vrst_map_renderer uploads pixels().
#pragma once

#include <algorithm>
#include <map>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "vrst_types.h"

namespace nav {

/// Decodes UTF-8 into code points. Invalid bytes become U+FFFD, so a broken OSM
/// name still draws (with a replacement box) instead of stopping the import.
/// @param s UTF-8 text.
/// @return Code points.
std::vector<char32_t> decode_utf8(std::string_view s);

/// One glyph in the atlas. All metrics are in pixels at Vrst_glyph_atlas::kBasePx;
/// a label scales them by size_px / kBasePx.
struct Vrst_glyph {
  /// Texture coordinates of the glyph bitmap (0..1), top-left corner.
  Vrst_float32 u0;
  Vrst_float32 v0;  ///< Top texture coordinate; see u0.
  Vrst_float32 u1;  ///< Right texture coordinate; see u0.
  Vrst_float32 v1;  ///< Bottom texture coordinate; see u0.
  /// Bitmap size including the SDF spread border. 0 for blank glyphs like space.
  Vrst_float32 w;
  Vrst_float32 h;  ///< Bitmap height; see w.
  /// Offset from the pen position to the bitmap's left edge.
  Vrst_float32 left;
  /// Offset from the baseline up to the bitmap's top edge.
  Vrst_float32 top;
  /// Pen advance to the next glyph.
  Vrst_float32 advance;
};

/// Packed SDF bitmaps of the glyphs a map needs, for one or more fonts.
///
/// Built once after the map is loaded, from exactly the code points the labels use
/// (plus ASCII), so the atlas stays small whatever the language of the region. Keeps
/// FreeType out of the per-frame path: layout only reads the glyph table.
class Vrst_glyph_atlas {
 public:
  /// Render size of the glyphs in pixels. Large enough that the distance field keeps
  /// sharp corners for big city labels; smaller labels are scaled down.
  static constexpr int kBasePx = 40;
  /// Distance field range in pixels at kBasePx, on each side of the outline. Limits
  /// the halo width: a halo needs distance values that far outside the glyph.
  static constexpr int kSpread = 8;
  /// Atlas width in pixels; the height grows with the glyph count.
  static constexpr int kWidth = 1024;

  /// Renders the glyphs and packs them. Code points missing in a font are skipped
  /// (find() returns nullptr and layout leaves them out).
  /// @param font_paths TrueType/OpenType files; the index is the font id used by find().
  /// @param codepoints Characters to render, in any order, duplicates allowed.
  /// @throws std::runtime_error when FreeType fails or a font cannot be opened.
  void build(const std::vector<std::string>& font_paths, std::vector<char32_t> codepoints);

  /// Glyph of a character.
  /// @param font Font id (index into build()'s font_paths).
  /// @param cp Code point.
  /// @return Copy of the glyph, or nothing when the font has no such character.
  std::optional<Vrst_glyph> find(int font, char32_t cp) const;

  /// Width of a text line at kBasePx.
  /// @param font Font id.
  /// @param text Code points.
  /// @return Sum of the advances in pixels.
  Vrst_float32 measure(int font, std::span<const char32_t> text) const;

  /// Atlas width in pixels.
  /// @return kWidth.
  int width() const { return kWidth; }
  /// @copydoc m_height
  /// @return Pixels.
  int height() const { return m_height; }
  /// @copydoc m_pixels
  /// @return width() * height() bytes, row 0 first.
  const std::vector<Vrst_uint8>& pixels() const { return m_pixels; }
  /// Number of glyphs over all fonts.
  /// @return Count.
  Vrst_size size() const { return m_glyphs.size(); }

 private:
  /// Map key of a glyph.
  /// @param font Font id.
  /// @param cp Code point (fits in 21 bits).
  /// @return Combined key.
  static Vrst_uint64 key(int font, char32_t cp) { return Vrst_uint64(font) << 32 | cp; }

  /// Glyph table by key(). A map, not an array by code point: the code points of a
  /// region are few but spread over the whole Unicode range.
  std::map<Vrst_uint64, Vrst_glyph> m_glyphs;
  /// One byte per pixel, the distance field: 128 = on the outline, more = inside.
  std::vector<Vrst_uint8> m_pixels;
  /// Atlas height in pixels, a power of two that holds all shelves.
  int m_height = 0;
};

}  // namespace nav
