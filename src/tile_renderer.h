/// @file
/// Tiled map, GPU side: uploads the Vrst_map_piece pieces a Vrst_map_provider
/// delivers, keeps a byte-limited cache of them and draws the visible ones in layer
/// order. Knows pieces only by their opaque keys, never how the map is stored. OpenGL 3.3 core / OpenGL ES 3.0, like
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
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "gl_api.h"
#include "map_piece.h"
#include "map_provider.h"
#include "map_renderer.h"
#include "map_scene.h"

namespace nav {

namespace shaders {

/// Map lines, vertex shader. One instance per piece (a_p0 -> a_p1, tile coordinates);
/// gl_VertexID 0..3 picks the quad corner. The quad is built in framebuffer pixels:
/// both ends extended by half the width (square caps close the joints) and offset
/// sideways by half the width.
inline constexpr std::string_view kTileLineVs = R"(
layout(location = 0) in vec2 a_p0;
layout(location = 1) in vec2 a_p1;
layout(location = 2) in float a_d0;
layout(location = 3) in uint a_attr;
uniform mat3 u_view;
uniform vec2 u_half_fb;
uniform float u_px_per_m;
uniform float u_width[28];
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
  bool hidden = style < 20u && ((u_class_mask >> style) & 1u) == 0u;
  gl_Position = hidden ? vec4(2.0, 2.0, 2.0, 1.0) : vec4(p / u_half_fb, 0.0, 1.0);
  v_attr = a_attr;
  v_along = a_d0 * u_px_per_m + mix(-hw, len + hw, end);
}
)";

/// Map lines, fragment shader. u_pass: 0 = style colour, 1 = casing colour (bridges),
/// 2 = dashes in u_pass_color (railways), 3 = tunnel (style colour faded, dashed).
inline constexpr std::string_view kTileLineFs = R"(
flat in uint v_attr;
in float v_along;
uniform vec3 u_colors[28];
uniform int u_pass;
uniform vec3 u_pass_color;
uniform float u_dash_px;
uniform bool u_oneway;
uniform vec3 u_background;
out vec4 o_color;
void main() {
  uint style = v_attr & 0xFFu;
  vec3 c = u_colors[style];
  if (u_oneway && style < 20u) c = (v_attr & 0x100u) != 0u ? vec3(1.0, 0.25, 0.9) : c * 0.3;
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
inline constexpr std::string_view kFillVs = R"(
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
inline constexpr std::string_view kFillFs = R"(
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
  Vrst_tile_renderer() = default;                                     ///< Empty; init() creates the GL objects.
  ~Vrst_tile_renderer() = default;                                    ///< Does not touch GL; call destroy() first.
  Vrst_tile_renderer(const Vrst_tile_renderer&) = delete;             ///< Owns GL objects: not copyable.
  Vrst_tile_renderer& operator=(const Vrst_tile_renderer&) = delete;  ///< Owns GL objects: not copyable.
  Vrst_tile_renderer(Vrst_tile_renderer&&) = delete;                  ///< Owns GL objects: not movable (destroy() is explicit).
  Vrst_tile_renderer& operator=(Vrst_tile_renderer&&) = delete;       ///< Owns GL objects: not movable (destroy() is explicit).

  /// Compiles the programs. Call once with the context current.
  /// @throws std::runtime_error on a shader or GL error.
  void init();

  /// Uploads a piece. Replaces a cached piece with the same key.
  /// @param p Piece from Vrst_map_provider::take_ready(); may be freed afterwards.
  /// @param world_origin Absolute Mercator world origin (Vrst_map_frame::center()).
  /// @param frame Current frame number, for the LRU order.
  void add(const Vrst_map_piece& p, Vrst_vec2d world_origin, Vrst_uint64 frame);

  /// Whether a piece is cached.
  /// @param key Piece key.
  /// @return true if it can be drawn.
  bool has(Vrst_uint64 key) const { return m_tiles.contains(key); }

  /// Whether a piece is cached; marks it as used this frame.
  /// @param key Piece key.
  /// @param frame Current frame number.
  /// @return true if it can be drawn.
  bool use(Vrst_uint64 key, Vrst_uint64 frame);

  /// Frees least recently used tiles until the cache fits its budget. Tiles used in
  /// the current frame are never freed.
  /// @param budget Byte budget of the vertex data.
  /// @param frame Current frame number.
  void evict(Vrst_size budget, Vrst_uint64 frame);

  /// Clears the framebuffer and draws tiles in layer order: area fills, tunnels,
  /// building outlines, water, railways, roads, bridges, vertex dots.
  /// @param v Camera.
  /// @param fb_w Framebuffer width in px.
  /// @param fb_h Framebuffer height in px.
  /// @param keys Pieces to draw; all must be cached.
  /// @param ground_mpp Ground metres per screen point at the screen center; sets the line widths.
  /// @param opt Display switches.
  void draw(const Vrst_view& v, int fb_w, int fb_h, const std::vector<Vrst_uint64>& keys, Vrst_float64 ground_mpp,
            const Vrst_tile_draw_options& opt);

  /// Deletes every tile on the GPU but keeps the programs: the next map (another
  /// region) starts with an empty cache, and no tile of the old one is ever drawn.
  /// Needs the GL context current.
  void clear();

  /// Deletes all GL objects. Call with the context current.
  void destroy();

  /// Number of cached tiles.
  /// @return Count.
  Vrst_size size() const { return m_tiles.size(); }
  /// @copydoc m_bytes
  /// @return Bytes.
  Vrst_size bytes() const { return m_bytes; }
  /// @copydoc m_draw_calls
  /// @return Count.
  int draw_calls() const { return m_draw_calls; }
  /// @copydoc m_uploaded
  /// @return Bytes.
  Vrst_uint64 uploaded() const { return m_uploaded; }

  /// Line width per style in framebuffer px for a zoom. Roads grow when zooming in
  /// (a street is wider than a pixel at 1 m/pt) and shrink when zooming out, but
  /// never below a hairline.
  /// @param ground_mpp Ground metres per screen point.
  /// @param px_ratio Framebuffer px per point.
  /// @return Width per line style.
  static std::array<Vrst_float32, kLineStyleCount> line_widths(Vrst_float64 ground_mpp, Vrst_float64 px_ratio);

 private:
  /// One uploaded tile.
  struct Vrst_gpu_tile {
    Vrst_vec2d origin;                                            ///< World position of the piece origin.
    Vrst_vec2d size;                                              ///< Width and height of the piece, metres.
    GLuint fill_vao;                                              ///< Area triangles VAO.
    GLuint fill_vbo;                                              ///< Area triangles buffer.
    GLuint line_vao;                                              ///< Line instances VAO.
    GLuint line_vbo;                                              ///< Line instances buffer.
    GLuint point_vao;                                             ///< Vertex dots VAO.
    GLuint point_vbo;                                             ///< Vertex dots buffer.
    GLsizei point_count;                                          ///< Number of vertex dots.
    Vrst_tile_range fill_ranges[kAreaKindCount];                  ///< Fill range per AreaKind.
    Vrst_tile_range line_ranges[kTileLayerCount][kLineStyleCount];  ///< Line range per layer and style.
    Vrst_size bytes;                                              ///< Vertex data size.
    Vrst_uint64 last_used;                                        ///< Frame of the last use (LRU).
  };

  /// Sets up a line VAO: four per-instance attributes from a Vrst_line_instance buffer.
  /// @param vao Vertex array.
  /// @param vbo Instance buffer, bound to the attributes at offset 0.
  static void setup_line_vao(GLuint vao, GLuint vbo);

  /// Points the line attributes at one instance of the bound buffer. GL 3.3 and GLES
  /// 3.0 have no base-instance draw, so a range starts by moving the pointers.
  /// @param first First instance of the range.
  static void point_line_attribs(Vrst_uint32 first);

  /// Draws one range of a tile's line instances.
  /// @param g Tile.
  /// @param mat Tile view matrix.
  /// @param r Instance range.
  void draw_lines(const Vrst_gpu_tile& g, const std::array<Vrst_float32, 9>& mat, Vrst_tile_range r);

  /// Draws the outline of every drawn tile (debug).
  /// @param v Camera.
  /// @param fb_w Framebuffer width in px.
  /// @param fb_h Framebuffer height in px.
  /// @param tiles Drawn tiles.
  /// @param px_ratio Framebuffer px per point.
  void draw_borders(const Vrst_view& v, int fb_w, int fb_h,
                    const std::vector<std::reference_wrapper<const Vrst_gpu_tile>>& tiles, Vrst_float64 px_ratio);

  /// Uploads Vrst_gpu_point vertices into a new VAO (position + attr).
  /// @param verts Vertices.
  /// @param[out] vbo Receives the buffer.
  /// @return Vertex array.
  static GLuint point_vao(const std::vector<Vrst_gpu_point>& verts, GLuint& vbo);

  /// Deletes the GL objects of one tile and subtracts its bytes.
  /// @param g Tile.
  void release(Vrst_gpu_tile& g);

  /// Cached pieces by key.
  std::unordered_map<Vrst_uint64, Vrst_gpu_tile> m_tiles;
  /// Vertex data bytes of all cached tiles.
  Vrst_size m_bytes = 0;
  /// Bytes uploaded by add() since init(); the difference between two frames is the
  /// upload cost of a frame.
  Vrst_uint64 m_uploaded = 0;
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

/// One piece uploaded by select_tiles(), for the session log.
struct Vrst_tile_upload {
  Vrst_uint64 key = 0;         ///< The piece.
  Vrst_size bytes = 0;         ///< Vertex data uploaded.
  Vrst_float64 build_ms = 0;   ///< Time on the worker (Vrst_map_piece::build_ms).
  int worker = 0;              ///< Worker that built it.
  Vrst_float64 upload_ms = 0;  ///< Time of Vrst_tile_renderer::add() on this thread.
};

/// What select_tiles() did in one frame, for status displays and the session log.
struct Vrst_tile_frame {
  /// Pieces with data on screen.
  Vrst_size visible = 0;
  /// Visible pieces not cached yet; the provider is making them.
  Vrst_size missing = 0;
  /// Cached coarser pieces drawn in place of missing ones.
  Vrst_size fallback = 0;
  /// Tiles uploaded to the GPU in this frame, in upload order.
  std::vector<Vrst_tile_upload> uploads;
  /// Tiles deleted from the GPU cache in this frame.
  Vrst_size evicted = 0;
  /// Time spent deleting them, in ms.
  Vrst_float64 evict_ms = 0;
};

/// Per-frame map work shared by the front ends: uploads ready pieces, tells the
/// provider what the screen shows, lists the cached pieces to draw and trims the
/// cache. A missing piece is covered by a cached coarser one until it is ready, so
/// the map never shows holes.
/// @param r Tile renderer (GPU cache).
/// @param map Map provider.
/// @param f Map frame (world origin).
/// @param v Camera.
/// @param fb_w Framebuffer width in px.
/// @param fb_h Framebuffer height in px.
/// @param frame Frame number, increasing.
/// @param budget GPU byte budget of the cache.
/// @param[out] stats Receives the counts and uploads.
/// @return Piece keys for Vrst_tile_renderer::draw(), coarse fallbacks first.
std::vector<Vrst_uint64> select_tiles(Vrst_tile_renderer& r, Vrst_map_provider& map, const Vrst_map_frame& f,
                                      const Vrst_view& v, int fb_w, int fb_h, Vrst_uint64 frame, Vrst_size budget,
                                      Vrst_tile_frame& stats);

}  // namespace nav
