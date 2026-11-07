/// @file
/// Vrst_glyph_atlas: rendering the SDF glyph atlas with FreeType; UTF-8 decoding.
/// Declarations and documentation are in glyph_atlas.h.
#include "glyph_atlas.h"

#include <ft2build.h>
#include FT_FREETYPE_H
#include FT_MODULE_H

namespace nav {

void Vrst_glyph_atlas::build(const std::vector<std::string>& font_paths, std::vector<char32_t> codepoints) {
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

std::optional<Vrst_glyph> Vrst_glyph_atlas::find(int font, char32_t cp) const {
  const auto it = m_glyphs.find(key(font, cp));
  if (it == m_glyphs.end()) return std::nullopt;
  return it->second;
}

Vrst_float32 Vrst_glyph_atlas::measure(int font, std::span<const char32_t> text) const {
  Vrst_float32 w = 0;
  for (const char32_t c : text)
    if (const std::optional<Vrst_glyph> g = find(font, c)) w += g->advance;
  return w;
}

std::vector<char32_t> decode_utf8(std::string_view s) {
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

}  // namespace nav
