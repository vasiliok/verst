/// @file
/// Tiled map, GPU side: uploads Vrst_tile_data, keeps a byte-limited cache of tiles
/// and draws the visible ones in layer order. OpenGL 3.3 core / OpenGL ES 3.0, like
/// map_renderer.h.
///
/// Lines (roads, rivers, railways, building outlines) are drawn as screen-width quads
/// with instancing: one instance per line piece, the vertex shader turns it into a
/// quad of the style's width in pixels. Core GL and GLES only guarantee 1-pixel
/// GL_LINES, and real road widths are what makes the map readable.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "gl_api.h"
#include "map_renderer.h"
#include "map_tiles.h"

namespace nav {

namespace shaders {

/// Map lines, vertex shader. One instance per piece (a_p0 -> a_p1, tile coordinates);
/// gl_VertexID 0..3 picks the quad corner. The quad is built in framebuffer pixels:
/// both ends extended by half the width (square caps close the joints) and offset
/// sideways by half the width.
inline constexpr const char* kTileLineVs = R"(
layout(location = 0) in vec2 a_p0;
layout(location = 1) in vec2 a_p1;
layout(location = 2) in float a_d0;
layout(location = 3) in uint a_attr;
uniform mat3 u_view;
uniform vec2 u_half_fb;
uniform float u_px_per_m;
uniform float u_width[24];
uniform float u_extra_px;
uniform uint u_class_mask;
flat out uint v_attr;
out float v_along;
void main() {
  uint style = a_attr & 0xFFu;
  vec2 s0 = (u_view * vec3(a_p0, 1.0)).xy * u_half_fb;
  vec2 s1 = (u_view * vec3(a_p1, 1.0)).xy * u_half_fb;
  vec2 d = s1 - s0;
  float len = length(d);
  vec2 dir = len > 1e-4 ? d / len : vec2(1.0, 0.0);
  vec2 n = vec2(-dir.y, dir.x);
  float hw = 0.5 * max(u_width[style] + u_extra_px, 1.0);
  float end = float(gl_VertexID & 1);
  float side = (gl_VertexID & 2) != 0 ? 1.0 : -1.0;
  vec2 p = mix(s0 - dir * hw, s1 + dir * hw, end) + n * side * hw;
  bool hidden = style < 16u && ((u_class_mask >> style) & 1u) == 0u;
  gl_Position = hidden ? vec4(2.0, 2.0, 2.0, 1.0) : vec4(p / u_half_fb, 0.0, 1.0);
  v_attr = a_attr;
  v_along = a_d0 * u_px_per_m + mix(-hw, len + hw, end);
}
)";

/// Map lines, fragment shader. u_pass: 0 = style colour, 1 = casing colour (bridges),
/// 2 = dashes in u_pass_color (railways), 3 = tunnel (style colour faded, dashed).
inline constexpr const char* kTileLineFs = R"(
flat in uint v_attr;
in float v_along;
uniform vec3 u_colors[24];
uniform int u_pass;
uniform vec3 u_pass_color;
uniform float u_dash_px;
uniform bool u_oneway;
uniform vec3 u_background;
out vec4 o_color;
void main() {
  uint style = v_attr & 0xFFu;
  vec3 c = u_colors[style];
  if (u_oneway && style < 16u) c = (v_attr & 0x100u) != 0u ? vec3(1.0, 0.25, 0.9) : c * 0.3;
  if (u_pass == 1) c = u_pass_color;
  if (u_pass == 2) {
    if (mod(v_along, 2.0 * u_dash_px) > u_dash_px) discard;
    c = u_pass_color;
  }
  if (u_pass == 3) {
    if (mod(v_along, 2.0 * u_dash_px) > u_dash_px) discard;
    c = mix(u_background, c, 0.45);
  }
  o_color = vec4(c, 1.0);
}
)";

/// Area fills, vertex shader: tile coordinates, a_attr = AreaKind.
inline constexpr const char* kFillVs = R"(
layout(location = 0) in vec2 a_pos;
layout(location = 1) in uint a_attr;
uniform mat3 u_view;
flat out uint v_kind;
void main() {
  vec3 p = u_view * vec3(a_pos, 1.0);
  gl_Position = vec4(p.xy, 0.0, 1.0);
  v_kind = a_attr;
}
)";

/// Area fills, fragment shader: one colour per AreaKind.
inline constexpr const char* kFillFs = R"(
flat in uint v_kind;
uniform vec3 u_colors[8];
out vec4 o_color;
void main() { o_color = vec4(u_colors[v_kind], 1.0); }
)";

}  // namespace shaders

/// Fill colour (RGB) per AreaKind: faint land use, clear water and forest, buildings
/// a little lighter than the background so blocks of houses read as blocks.
inline constexpr Vrst_float32 kAreaColors[kAreaKindCount][3] = {
    {0.125f, 0.140f, 0.150f},  // farmland
    {0.115f, 0.155f, 0.140f},  // meadow
    {0.150f, 0.152f, 0.175f},  // residential
    {0.165f, 0.145f, 0.165f},  // industrial
    {0.115f, 0.190f, 0.150f},  // park
    {0.085f, 0.200f, 0.135f},  // forest
    {0.110f, 0.230f, 0.380f},  // water
    {0.300f, 0.310f, 0.360f},  // building
};

/// Line colour (RGB) per map.dat LineKind and for the extra line styles.
inline constexpr Vrst_float32 kLineKindColors[kLineKindCount][3] = {
    {0.20f, 0.38f, 0.62f},  // stream
    {0.20f, 0.38f, 0.62f},  // river
    {0.62f, 0.62f, 0.68f},  // rail
    {0.50f, 0.50f, 0.56f},  // minor rail
};

/// Bridge casings are drawn while the map shows fewer ground metres per point than this.
inline constexpr Vrst_float64 kBridgeCasingMaxMpp = 6;

/// Per-frame switches of the tiled map.
struct Vrst_tile_draw_options {
  /// Bit per RoadClass; a 0 bit hides that class.
  Vrst_uint32 class_mask = kAllClasses;
  /// Highlight oneway roads and dim the rest.
  bool oneway = false;
  /// Draw the graph vertex dots.
  bool show_vertices = false;
  /// Vertex dot size in framebuffer px.
  Vrst_float32 point_px = 2.5f;
  /// Framebuffer px per screen point; line widths are given in points.
  Vrst_float64 px_ratio = 1;
  /// Draw land use, forest and water areas.
  bool areas = true;
  /// Draw buildings.
  bool buildings = true;
  /// Draw rivers and streams.
  bool water_lines = true;
  /// Draw railways.
  bool railways = true;
  /// Draw the tile borders (debug, key T).
  bool borders = false;
};

/// GPU tiles: upload, cache with a byte budget, draw.
/// Needs a current GL context for every call except the getters; the destructor does
/// not touch GL.
class Vrst_tile_renderer {
 public:
  /// Compiles the programs. Call once with the context current.
  /// @throws std::runtime_error on a shader or GL error.
  void init() {
    m_line_prog = Vrst_map_renderer::link_program(shaders::kTileLineVs, shaders::kTileLineFs);
    m_fill_prog = Vrst_map_renderer::link_program(shaders::kFillVs, shaders::kFillFs);
    m_point_prog = Vrst_map_renderer::link_program(shaders::kPointVs, shaders::kPointFs);
    glUseProgram(m_fill_prog);
    glUniform3fv(glGetUniformLocation(m_fill_prog, "u_colors"), kAreaKindCount, &kAreaColors[0][0]);
    Vrst_float32 colors[kLineStyleCount][3] = {};
    for (int c = 0; c < kRoadClassCount; ++c) std::copy_n(kClassColors[c], 3, colors[c]);
    for (int k = 0; k < kLineKindCount; ++k) std::copy_n(kLineKindColors[k], 3, colors[kStyleLineKind0 + k]);
    const Vrst_float32 edge[3] = {0.16f, 0.17f, 0.20f}, border[3] = {1.0f, 0.2f, 0.9f};
    std::copy_n(edge, 3, colors[kStyleBuildingEdge]);
    std::copy_n(border, 3, colors[kStyleTileBorder]);
    glUseProgram(m_line_prog);
    glUniform3fv(glGetUniformLocation(m_line_prog, "u_colors"), kLineStyleCount, &colors[0][0]);
    glUniform3fv(glGetUniformLocation(m_line_prog, "u_background"), 1, kBackground);
    for (const char* n : {"u_view", "u_half_fb", "u_px_per_m", "u_width", "u_extra_px", "u_class_mask", "u_pass",
                          "u_pass_color", "u_dash_px", "u_oneway"})
      m_u[n] = glGetUniformLocation(m_line_prog, n);
    m_u_fill_view = glGetUniformLocation(m_fill_prog, "u_view");
    m_u_point_view = glGetUniformLocation(m_point_prog, "u_view");
    m_u_point_px = glGetUniformLocation(m_point_prog, "u_point_px");
    glGenVertexArrays(1, &m_border_vao);
    glGenBuffers(1, &m_border_vbo);
    setup_line_vao(m_border_vao, m_border_vbo);
#if !VRST_GLES
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_MULTISAMPLE);
#endif
    Vrst_map_renderer::check_error("tile renderer init");
  }

  /// Uploads a built tile. Replaces a cached tile with the same id.
  /// @param t Tile from Vrst_tile_builder; may be freed afterwards.
  /// @param frame Current frame number, for the LRU order.
  void add(const Vrst_tile_data& t, Vrst_uint64 frame) {
    const Vrst_uint64 key = tile_key(t.id);
    if (auto it = m_tiles.find(key); it != m_tiles.end()) {
      release(it->second);
      m_tiles.erase(it);
    }
    Vrst_gpu_tile g{};
    g.id = t.id;
    g.origin = t.origin;
    g.last_used = frame;
    std::copy_n(t.fill_ranges, kAreaKindCount, g.fill_ranges);
    for (int l = 0; l < kTileLayerCount; ++l) std::copy_n(t.line_ranges[l], kLineStyleCount, g.line_ranges[l]);
    if (!t.fills.empty()) g.fill_vao = point_vao(t.fills, &g.fill_vbo);
    if (!t.points.empty()) g.point_vao = point_vao(t.points, &g.point_vbo);
    g.point_count = GLsizei(t.points.size());
    if (!t.lines.empty()) {
      glGenVertexArrays(1, &g.line_vao);
      glGenBuffers(1, &g.line_vbo);
      glBindBuffer(GL_ARRAY_BUFFER, g.line_vbo);
      glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(t.lines.size() * sizeof(Vrst_line_instance)), t.lines.data(), GL_STATIC_DRAW);
      setup_line_vao(g.line_vao, g.line_vbo);
    }
    glBindVertexArray(0);
    g.bytes = t.fills.size() * sizeof(Vrst_gpu_point) + t.points.size() * sizeof(Vrst_gpu_point) +
              t.lines.size() * sizeof(Vrst_line_instance);
    m_bytes += g.bytes;
    m_tiles.emplace(key, g);
  }

  /// Whether a tile is cached; marks it as used this frame.
  /// @param t Tile.
  /// @param frame Current frame number.
  /// @return true if it can be drawn.
  bool use(Vrst_tile_id t, Vrst_uint64 frame) {
    const auto it = m_tiles.find(tile_key(t));
    if (it == m_tiles.end()) return false;
    it->second.last_used = frame;
    return true;
  }

  /// Frees least recently used tiles until the cache fits its budget. Tiles used in
  /// the current frame are never freed.
  /// @param budget Byte budget of the vertex data.
  /// @param frame Current frame number.
  void evict(Vrst_size budget, Vrst_uint64 frame) {
    if (m_bytes <= budget) return;
    std::vector<std::pair<Vrst_uint64, Vrst_uint64>> lru;  // last used, key
    for (const auto& [k, g] : m_tiles)
      if (g.last_used < frame) lru.push_back({g.last_used, k});
    std::sort(lru.begin(), lru.end());
    for (const auto& [used, k] : lru) {
      if (m_bytes <= budget) break;
      release(m_tiles.at(k));
      m_tiles.erase(k);
    }
  }

  /// Clears the framebuffer and draws tiles in layer order: area fills, tunnels,
  /// building outlines, water, railways, roads, bridges, vertex dots.
  /// @param v Camera.
  /// @param fb_w Framebuffer width in px.
  /// @param fb_h Framebuffer height in px.
  /// @param keys Tiles to draw (tile_key()); all must be cached.
  /// @param ground_mpp Ground metres per screen point at the screen center; sets the line widths.
  /// @param opt Display switches.
  void draw(const Vrst_view& v, int fb_w, int fb_h, const std::vector<Vrst_uint64>& keys, Vrst_float64 ground_mpp,
            const Vrst_tile_draw_options& opt) {
    glViewport(0, 0, fb_w, fb_h);
    glClearColor(kBackground[0], kBackground[1], kBackground[2], 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    m_draw_calls = 0;
    std::vector<const Vrst_gpu_tile*> tiles;
    for (const Vrst_uint64 k : keys) tiles.push_back(&m_tiles.at(k));
    std::vector<std::array<Vrst_float32, 9>> mats(tiles.size());
    for (Vrst_size i = 0; i < tiles.size(); ++i) {
      Vrst_view tv = v;
      tv.center = {v.center.x - tiles[i]->origin.x, v.center.y - tiles[i]->origin.y};
      view_matrix(tv, fb_w, fb_h, mats[i].data());
    }

    // Area fills, kind by kind across all tiles (sorted bottom to top).
    glUseProgram(m_fill_prog);
    for (int k = 0; k < kAreaKindCount; ++k) {
      const bool building = k == int(AreaKind::Building);
      if (building ? !opt.buildings : !opt.areas) continue;
      for (Vrst_size i = 0; i < tiles.size(); ++i) {
        const Vrst_tile_range r = tiles[i]->fill_ranges[k];
        if (r.count == 0) continue;
        glUniformMatrix3fv(m_u_fill_view, 1, GL_FALSE, mats[i].data());
        glBindVertexArray(tiles[i]->fill_vao);
        glDrawArrays(GL_TRIANGLES, GLint(r.first), GLsizei(r.count));
        ++m_draw_calls;
      }
    }

    // Lines.
    glUseProgram(m_line_prog);
    Vrst_float32 width[kLineStyleCount] = {};
    line_widths(ground_mpp, opt.px_ratio, width);
    glUniform1fv(m_u.at("u_width"), kLineStyleCount, width);
    glUniform2f(m_u.at("u_half_fb"), fb_w * 0.5f, fb_h * 0.5f);
    glUniform1f(m_u.at("u_px_per_m"), Vrst_float32(v.scale));
    glUniform1ui(m_u.at("u_class_mask"), opt.class_mask);
    glUniform1i(m_u.at("u_oneway"), opt.oneway);
    const Vrst_float32 dash = Vrst_float32(5 * opt.px_ratio);
    glUniform1f(m_u.at("u_dash_px"), dash);
    auto shown = [&](int style) {
      if (style == kStyleBuildingEdge) return opt.buildings;
      if (style == kStyleLineKind0 + int(LineKind::Stream) || style == kStyleLineKind0 + int(LineKind::River))
        return opt.water_lines;
      if (style == kStyleLineKind0 + int(LineKind::Rail) || style == kStyleLineKind0 + int(LineKind::MinorRail))
        return opt.railways;
      return true;
    };
    auto rail = [](int style) {
      return style == kStyleLineKind0 + int(LineKind::Rail) || style == kStyleLineKind0 + int(LineKind::MinorRail);
    };
    const Vrst_float32 casing_light[3] = {0.75f, 0.75f, 0.78f}, casing_dark[3] = {0.02f, 0.02f, 0.03f};
    auto pass = [&](int layer, int style, int mode, Vrst_float32 extra, const Vrst_float32* color) {
      glUniform1i(m_u.at("u_pass"), mode);
      glUniform1f(m_u.at("u_extra_px"), extra);
      if (color) glUniform3fv(m_u.at("u_pass_color"), 1, color);
      for (Vrst_size i = 0; i < tiles.size(); ++i) draw_lines(*tiles[i], mats[i].data(), tiles[i]->line_ranges[layer][style]);
    };
    for (const int style : kLineStyleOrder)
      if (shown(style)) pass(kLayerTunnel, style, 3, 0, nullptr);
    for (const int style : kLineStyleOrder) {
      if (!shown(style)) continue;
      pass(kLayerGround, style, 0, 0, nullptr);
      if (rail(style)) pass(kLayerGround, style, 2, Vrst_float32(-1.4 * opt.px_ratio), kBackground);
    }
    // Bridge casings only from street zoom in: zoomed out, every small bridge over a
    // stream would become a box on the road.
    const Vrst_float32 casing = Vrst_float32(2 * opt.px_ratio);
    const bool casings = ground_mpp < kBridgeCasingMaxMpp;
    for (const int style : kLineStyleOrder) {
      if (!shown(style)) continue;
      if (casings) {
        pass(kLayerBridge, style, 1, 2 * casing, casing_light);
        pass(kLayerBridge, style, 1, casing, casing_dark);
      }
      pass(kLayerBridge, style, 0, 0, nullptr);
      if (rail(style)) pass(kLayerBridge, style, 2, Vrst_float32(-1.4 * opt.px_ratio), kBackground);
    }

    if (opt.show_vertices) {
      glUseProgram(m_point_prog);
      glUniform1f(m_u_point_px, opt.point_px);
      for (Vrst_size i = 0; i < tiles.size(); ++i) {
        if (!tiles[i]->point_count) continue;
        glUniformMatrix3fv(m_u_point_view, 1, GL_FALSE, mats[i].data());
        glBindVertexArray(tiles[i]->point_vao);
        glDrawArrays(GL_POINTS, 0, tiles[i]->point_count);
        ++m_draw_calls;
      }
    }
    if (opt.borders) draw_borders(v, fb_w, fb_h, tiles, opt.px_ratio);
    glBindVertexArray(0);
  }

  /// Deletes all GL objects. Call with the context current.
  void destroy() {
    for (auto& [k, g] : m_tiles) release(g);
    m_tiles.clear();
    m_bytes = 0;
    glDeleteBuffers(1, &m_border_vbo);
    glDeleteVertexArrays(1, &m_border_vao);
    glDeleteProgram(m_line_prog);
    glDeleteProgram(m_fill_prog);
    glDeleteProgram(m_point_prog);
    m_border_vbo = m_border_vao = m_line_prog = m_fill_prog = m_point_prog = 0;
  }

  /// Number of cached tiles.
  /// @return Count.
  Vrst_size size() const { return m_tiles.size(); }
  /// @copydoc m_bytes
  /// @return Bytes.
  Vrst_size bytes() const { return m_bytes; }
  /// @copydoc m_draw_calls
  /// @return Count.
  int draw_calls() const { return m_draw_calls; }

  /// Line width per style in framebuffer px for a zoom. Roads grow when zooming in
  /// (a street is wider than a pixel at 1 m/pt) and shrink when zooming out, but
  /// never below a hairline.
  /// @param ground_mpp Ground metres per screen point.
  /// @param px_ratio Framebuffer px per point.
  /// @param[out] out kLineStyleCount widths.
  static void line_widths(Vrst_float64 ground_mpp, Vrst_float64 px_ratio, Vrst_float32 out[kLineStyleCount]) {
    // Widths in points at 3 m/pt, by RoadClass (links narrower than their road).
    static constexpr Vrst_float32 kRoadPt[kRoadClassCount] = {5.0f, 3.2f, 4.6f, 3.0f, 4.2f, 2.8f, 3.8f, 2.6f,
                                                              3.2f, 2.2f, 2.4f, 2.0f, 1.6f, 1.2f, 1.6f};
    const Vrst_float64 f = std::clamp(std::sqrt(3.0 / std::max(ground_mpp, 1e-3)), 0.35, 2.6);
    for (int c = 0; c < kRoadClassCount; ++c) out[c] = Vrst_float32(kRoadPt[c] * f * px_ratio);
    out[kStyleLineKind0 + int(LineKind::Stream)] = Vrst_float32(1.2 * std::min(f, 1.4) * px_ratio);
    out[kStyleLineKind0 + int(LineKind::River)] = Vrst_float32(3.5 * f * px_ratio);
    out[kStyleLineKind0 + int(LineKind::Rail)] = Vrst_float32(2.8 * std::max(f, 0.7) * px_ratio);
    out[kStyleLineKind0 + int(LineKind::MinorRail)] = Vrst_float32(1.8 * std::max(f, 0.7) * px_ratio);
    out[kStyleBuildingEdge] = Vrst_float32(1.0 * px_ratio);
    out[kStyleTileBorder] = Vrst_float32(1.5 * px_ratio);
  }

 private:
  /// One uploaded tile.
  struct Vrst_gpu_tile {
    Vrst_tile_id id;                                              ///< The tile.
    Vrst_vec2d origin;                                            ///< World origin of the tile coordinates.
    GLuint fill_vao, fill_vbo;                                    ///< Area triangles.
    GLuint line_vao, line_vbo;                                    ///< Line instances.
    GLuint point_vao, point_vbo;                                  ///< Vertex dots.
    GLsizei point_count;                                          ///< Number of vertex dots.
    Vrst_tile_range fill_ranges[kAreaKindCount];                  ///< Fill range per AreaKind.
    Vrst_tile_range line_ranges[kTileLayerCount][kLineStyleCount];  ///< Line range per layer and style.
    Vrst_size bytes;                                              ///< Vertex data size.
    Vrst_uint64 last_used;                                        ///< Frame of the last use (LRU).
  };

  /// Sets up a line VAO: four per-instance attributes from a Vrst_line_instance buffer.
  /// @param vao Vertex array.
  /// @param vbo Instance buffer, bound to the attributes at offset 0.
  static void setup_line_vao(GLuint vao, GLuint vbo) {
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    for (GLuint a = 0; a < 4; ++a) {
      glEnableVertexAttribArray(a);
      glVertexAttribDivisor(a, 1);
    }
    point_line_attribs(0);
  }

  /// Points the line attributes at one instance of the bound buffer. GL 3.3 and GLES
  /// 3.0 have no base-instance draw, so a range starts by moving the pointers.
  /// @param first First instance of the range.
  static void point_line_attribs(Vrst_uint32 first) {
    const auto base = static_cast<std::uintptr_t>(first) * sizeof(Vrst_line_instance);
    const GLsizei stride = sizeof(Vrst_line_instance);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(base));
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(base + 8));
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(base + 16));
    glVertexAttribIPointer(3, 1, GL_UNSIGNED_INT, stride, reinterpret_cast<void*>(base + 20));
  }

  /// Draws one range of a tile's line instances.
  /// @param g Tile.
  /// @param mat Tile view matrix.
  /// @param r Instance range.
  void draw_lines(const Vrst_gpu_tile& g, const Vrst_float32* mat, Vrst_tile_range r) {
    if (r.count == 0) return;
    glUniformMatrix3fv(m_u.at("u_view"), 1, GL_FALSE, mat);
    glBindVertexArray(g.line_vao);
    glBindBuffer(GL_ARRAY_BUFFER, g.line_vbo);
    point_line_attribs(r.first);
    glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, GLsizei(r.count));
    ++m_draw_calls;
  }

  /// Draws the outline of every drawn tile (debug).
  /// @param v Camera.
  /// @param fb_w Framebuffer width in px.
  /// @param fb_h Framebuffer height in px.
  /// @param tiles Drawn tiles.
  /// @param px_ratio Framebuffer px per point.
  void draw_borders(const Vrst_view& v, int fb_w, int fb_h, const std::vector<const Vrst_gpu_tile*>& tiles,
                    Vrst_float64 px_ratio) {
    std::vector<Vrst_line_instance> edges;
    for (const Vrst_gpu_tile* t : tiles) {
      const auto s = Vrst_float32(tile_side(t->id.z));
      const auto ox = Vrst_float32(t->origin.x), oy = Vrst_float32(t->origin.y);
      const Vrst_float32 c[5][2] = {{0, 0}, {s, 0}, {s, s}, {0, s}, {0, 0}};
      for (int i = 0; i < 4; ++i)
        edges.push_back({ox + c[i][0], oy + c[i][1], ox + c[i + 1][0], oy + c[i + 1][1], 0, kStyleTileBorder});
    }
    // World coordinates near the map center: plain float is exact enough for a debug overlay.
    Vrst_float32 m[9];
    view_matrix(v, fb_w, fb_h, m);
    glBindBuffer(GL_ARRAY_BUFFER, m_border_vbo);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(edges.size() * sizeof(Vrst_line_instance)), edges.data(), GL_STREAM_DRAW);
    glUniform1f(m_u.at("u_extra_px"), 0);
    glUniform1f(m_u.at("u_dash_px"), Vrst_float32(6 * px_ratio));
    glUniform1i(m_u.at("u_pass"), 2);
    const Vrst_float32 magenta[3] = {1.0f, 0.2f, 0.9f};
    glUniform3fv(m_u.at("u_pass_color"), 1, magenta);
    Vrst_gpu_tile g{};
    g.line_vao = m_border_vao;
    g.line_vbo = m_border_vbo;
    draw_lines(g, m, {0, Vrst_uint32(edges.size())});
  }

  /// Uploads Vrst_gpu_point vertices into a new VAO (position + attr).
  /// @param verts Vertices.
  /// @param[out] vbo Receives the buffer.
  /// @return Vertex array.
  static GLuint point_vao(const std::vector<Vrst_gpu_point>& verts, GLuint* vbo) {
    GLuint vao = 0;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);
    glGenBuffers(1, vbo);
    glBindBuffer(GL_ARRAY_BUFFER, *vbo);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(verts.size() * sizeof(Vrst_gpu_point)), verts.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vrst_gpu_point), reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribIPointer(1, 1, GL_UNSIGNED_INT, sizeof(Vrst_gpu_point), reinterpret_cast<void*>(8));
    return vao;
  }

  /// Deletes the GL objects of one tile and subtracts its bytes.
  /// @param g Tile.
  void release(Vrst_gpu_tile& g) {
    const GLuint bufs[3] = {g.fill_vbo, g.line_vbo, g.point_vbo}, vaos[3] = {g.fill_vao, g.line_vao, g.point_vao};
    glDeleteBuffers(3, bufs);
    glDeleteVertexArrays(3, vaos);
    m_bytes -= g.bytes;
  }

  /// Cached tiles by tile_key().
  std::unordered_map<Vrst_uint64, Vrst_gpu_tile> m_tiles;
  /// Vertex data bytes of all cached tiles.
  Vrst_size m_bytes = 0;
  /// Draw calls of the last draw().
  int m_draw_calls = 0;
  /// Line program (instanced quads).
  GLuint m_line_prog = 0;
  /// Area fill program.
  GLuint m_fill_prog = 0;
  /// Vertex dot program (shared shaders with map_renderer.h).
  GLuint m_point_prog = 0;
  /// Uniform locations of the line program by name.
  std::unordered_map<std::string, GLint> m_u;
  /// u_view of the fill program.
  GLint m_u_fill_view = -1;
  /// u_view of the point program.
  GLint m_u_point_view = -1;
  /// u_point_px of the point program.
  GLint m_u_point_px = -1;
  /// Debug tile border instances, rewritten per draw.
  GLuint m_border_vao = 0;
  /// Buffer of m_border_vao.
  GLuint m_border_vbo = 0;
};

/// What select_tiles() did in one frame, for status displays.
struct Vrst_tile_frame {
  /// Level drawn (index into kTileLevels).
  int level = 0;
  /// Tiles with data on screen.
  Vrst_size visible = 0;
  /// Visible tiles not built yet; the loader was asked for them.
  Vrst_size missing = 0;
};

/// Per-frame tile work shared by the front ends: uploads finished tiles, picks the
/// level for the zoom, lists the cached tiles to draw, asks the loader for the
/// missing ones and trims the cache. A missing tile is covered by a cached coarser
/// tile until it is ready, so the map never shows holes.
/// @param r Tile renderer (GPU cache).
/// @param loader Background loader.
/// @param index Tile index.
/// @param v Camera.
/// @param fb_w Framebuffer width in px.
/// @param fb_h Framebuffer height in px.
/// @param map_center Mercator map center (world origin).
/// @param frame Frame number, increasing.
/// @param budget GPU byte budget of the cache.
/// @param[out] stats Receives level and counts; may be nullptr.
/// @return Tile keys for Vrst_tile_renderer::draw(), coarse fallbacks first.
inline std::vector<Vrst_uint64> select_tiles(Vrst_tile_renderer& r, Vrst_tile_loader& loader,
                                             const Vrst_tile_index& index, const Vrst_view& v, int fb_w, int fb_h,
                                             Vrst_vec2d map_center, Vrst_uint64 frame, Vrst_size budget,
                                             Vrst_tile_frame* stats) {
  for (std::unique_ptr<Vrst_tile_data>& t : loader.take(8)) r.add(*t, frame);
  Vrst_tile_frame f;
  f.level = level_for(1.0 / v.scale);
  std::vector<Vrst_tile_id> missing;
  std::vector<Vrst_uint64> fine, coarse;
  std::unordered_set<Vrst_uint64> drawn;
  for (const Vrst_tile_id& t : visible_tiles(v, fb_w, fb_h, map_center, kTileLevels[f.level].z)) {
    if (!index.find(t)) continue;  // nothing there
    ++f.visible;
    if (r.use(t, frame)) {
      if (drawn.insert(tile_key(t)).second) fine.push_back(tile_key(t));
      continue;
    }
    missing.push_back(t);
    for (int l = f.level + 1; l < kTileLevelCount; ++l) {
      const int shift = t.z - kTileLevels[l].z;
      const Vrst_tile_id p{kTileLevels[l].z, t.x >> shift, t.y >> shift};
      if (r.use(p, frame)) {
        if (drawn.insert(tile_key(p)).second) coarse.push_back(tile_key(p));
        break;
      }
    }
  }
  f.missing = missing.size();
  loader.request(missing);
  r.evict(budget, frame);
  if (stats) *stats = f;
  coarse.insert(coarse.end(), fine.begin(), fine.end());  // fallbacks under the fine tiles
  return coarse;
}

/// Ground resolution at the screen center, for line widths and label rules.
/// @param v Camera.
/// @param map_center Mercator map center (world origin).
/// @param px_per_point Framebuffer px per screen point.
/// @return Ground metres per screen point.
inline Vrst_float64 ground_metres_per_point(const Vrst_view& v, Vrst_vec2d map_center, Vrst_float64 px_per_point) {
  const Vrst_float64 lat = inverse_mercator({v.center.x + map_center.x, v.center.y + map_center.y}).x;
  return px_per_point * std::cos(lat * kPi / 180) / v.scale;
}

}  // namespace nav
