/// @file
/// Signed-distance-field glyph atlas rendered with FreeType, shared by navview and
/// the iOS app.
///
/// Glyphs are rendered once at kBasePx with FreeType's SDF renderer and packed into
/// one 8-bit texture. A distance field instead of plain coverage lets one atlas draw
/// every label size sharply and gives the dark halo around the text for free in the
/// shader. Nothing here calls a graphics API; Vrst_map_renderer uploads pixels().
#pragma once

#include <ft2build.h>
#include FT_FREETYPE_H
#include FT_MODULE_H

#include <algorithm>
#include <map>
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
inline std::vector<char32_t> decode_utf8(std::string_view s) {
  std::vector<char32_t> out;
  for (Vrst_size i = 0; i < s.size();) {
    const auto b = static_cast<unsigned char>(s[i]);
    const int n = b < 0x80 ? 1 : (b >> 5) == 0x6 ? 2 : (b >> 4) == 0xE ? 3 : (b >> 3) == 0x1E ? 4 : 0;
    if (n == 0 || i + n > s.size()) {
      out.push_back(0xFFFD);
      ++i;
      continue;
    }
    char32_t cp = n == 1 ? b : b & (0x7F >> n);
    bool ok = true;
    for (int k = 1; k < n; ++k) {
      const auto c = static_cast<unsigned char>(s[i + k]);
      ok &= (c >> 6) == 0x2;
      cp = cp << 6 | (c & 0x3F);
    }
    out.push_back(ok ? cp : 0xFFFD);
    i += ok ? n : 1;
  }
  return out;
}

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
  void build(const std::vector<std::string>& font_paths, std::vector<char32_t> codepoints) {
    for (char32_t c = 32; c < 127; ++c) codepoints.push_back(c);
    std::sort(codepoints.begin(), codepoints.end());
    codepoints.erase(std::unique(codepoints.begin(), codepoints.end()), codepoints.end());

    FT_Library lib = nullptr;
    if (FT_Init_FreeType(&lib)) throw std::runtime_error("FT_Init_FreeType failed");
    FT_Int spread = kSpread;
    FT_Property_Set(lib, "sdf", "spread", &spread);
    m_glyphs.clear();
    m_pixels.assign(Vrst_size(kWidth) * 64, 0);
    m_height = 64;
    int x = 0, y = 0, row_h = 0;
    try {
      for (Vrst_size f = 0; f < font_paths.size(); ++f) {
        FT_Face face = nullptr;
        if (FT_New_Face(lib, font_paths[f].c_str(), 0, &face))
          throw std::runtime_error(font_paths[f] + ": cannot open font");
        FT_Set_Pixel_Sizes(face, 0, kBasePx);
        for (const char32_t cp : codepoints) {
          const FT_UInt index = FT_Get_Char_Index(face, cp);
          if (index == 0 || FT_Load_Glyph(face, index, FT_LOAD_DEFAULT)) continue;
          const FT_GlyphSlot g = face->glyph;
          Vrst_glyph out{0, 0, 0, 0, 0, 0, 0, 0, Vrst_float32(g->advance.x) / 64};
          if (g->outline.n_points > 0 && FT_Render_Glyph(g, FT_RENDER_MODE_SDF) == 0) {
            const FT_Bitmap& bm = g->bitmap;
            const int w = int(bm.width), h = int(bm.rows);
            if (x + w + 1 > kWidth) {  // next shelf
              x = 0;
              y += row_h + 1;
              row_h = 0;
            }
            while (y + h + 1 > m_height) {
              m_height *= 2;
              m_pixels.resize(Vrst_size(kWidth) * m_height, 0);
            }
            for (int r = 0; r < h; ++r)
              std::copy_n(bm.buffer + r * bm.pitch, w, m_pixels.begin() + Vrst_size(y + r) * kWidth + x);
            out.u0 = Vrst_float32(x);
            out.v0 = Vrst_float32(y);
            out.u1 = Vrst_float32(x + w);
            out.v1 = Vrst_float32(y + h);
            out.w = Vrst_float32(w);
            out.h = Vrst_float32(h);
            out.left = Vrst_float32(g->bitmap_left);
            out.top = Vrst_float32(g->bitmap_top);
            x += w + 1;
            row_h = std::max(row_h, h);
          }
          m_glyphs[key(int(f), cp)] = out;
        }
        FT_Done_Face(face);
      }
    } catch (...) {
      FT_Done_FreeType(lib);
      throw;
    }
    FT_Done_FreeType(lib);
    // Pixel coordinates -> 0..1 now that the final height is known.
    for (auto& [k, g] : m_glyphs) {
      g.u0 /= kWidth;
      g.u1 /= kWidth;
      g.v0 /= Vrst_float32(m_height);
      g.v1 /= Vrst_float32(m_height);
    }
  }

  /// Glyph of a character.
  /// @param font Font id (index into build()'s font_paths).
  /// @param cp Code point.
  /// @return Glyph, or nullptr when the font has no such character.
  const Vrst_glyph* find(int font, char32_t cp) const {
    const auto it = m_glyphs.find(key(font, cp));
    return it == m_glyphs.end() ? nullptr : &it->second;
  }

  /// Width of a text line at kBasePx.
  /// @param font Font id.
  /// @param text Code points.
  /// @return Sum of the advances in pixels.
  Vrst_float32 measure(int font, std::span<const char32_t> text) const {
    Vrst_float32 w = 0;
    for (const char32_t c : text)
      if (const Vrst_glyph* g = find(font, c)) w += g->advance;
    return w;
  }

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
