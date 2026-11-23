/// @file
/// OpenGL map renderer shared by the desktop viewer (OpenGL 3.3 core) and the iOS
/// app (OpenGL ES 3.0).
///
/// Draws what goes over the tiled map (Vrst_tile_renderer): place name labels (glyph
/// quads from Vrst_map_labels), an optional GPS track with a position marker and the
/// GPS position. Uses only calls both APIs have, and shares its program linking with
/// Vrst_tile_renderer.
#pragma once

#include <algorithm>
#include <array>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <string_view>

#include "render/gl_api.h"
#include "render/glyph_atlas.h"
#include "render/map_labels.h"
#include "provider/map_scene.h"
#include "geometry/region_outline.h"
#include "render/shaders.h"

namespace nav {

/// GPU side of the overlays: shader programs, buffers and draw calls. Needs a current
/// GL context for init_overlays(), the draw calls and destroy(); the destructor does
/// not touch GL, because the context may already be gone.
class Vrst_map_renderer {
 public:
  Vrst_map_renderer() = default;                                    ///< Empty; init_overlays() creates the GL objects.
  ~Vrst_map_renderer() = default;                                   ///< Does not touch GL; call destroy() first.
  Vrst_map_renderer(const Vrst_map_renderer&) = delete;             ///< Owns GL objects: not copyable.
  Vrst_map_renderer& operator=(const Vrst_map_renderer&) = delete;  ///< Owns GL objects: not copyable.
  Vrst_map_renderer(Vrst_map_renderer&&) = delete;                  ///< Owns GL objects: not movable (destroy() is explicit).
  Vrst_map_renderer& operator=(Vrst_map_renderer&&) = delete;       ///< Owns GL objects: not movable (destroy() is explicit).

  /// Compiles the overlay program (GPS track, markers). Call once with the context
  /// current; set_label_atlas() sets up the labels.
  /// @throws std::runtime_error on a shader compile/link error.
  void init_overlays();

  /// Uploads a GPS track to draw with draw_track(). Replaces an earlier track.
  /// @param points Track points in world coordinates (Vrst_map_frame::to_world()).
  /// @throws std::runtime_error on a GL error.
  void set_track(const std::vector<Vrst_vec2d>& points);

  /// Draws the track uploaded by set_track() over the map: the part not yet driven
  /// dim, the driven part bright, and a round marker at the current position. Call
  /// after the map, with the same camera.
  /// @param v Camera.
  /// @param fb_w Framebuffer width in px.
  /// @param fb_h Framebuffer height in px.
  /// @param driven Number of track points already passed (the bright part, from the start).
  /// @param marker Current position, world coordinates.
  /// @param marker_px Marker diameter in framebuffer pixels.
  void draw_track(const Vrst_view& v, int fb_w, int fb_h, Vrst_size driven, Vrst_vec2d marker,
                  Vrst_float32 marker_px) const;

  /// Draws the current GPS position: a blue dot with a white ring, the same size at
  /// every zoom. Call after the map, before the labels.
  /// @param v Camera.
  /// @param fb_w Framebuffer width in px.
  /// @param fb_h Framebuffer height in px.
  /// @param pos Position, world coordinates (Vrst_map_frame::to_world()).
  /// @param px Dot diameter in framebuffer pixels.
  void draw_position(const Vrst_view& v, int fb_w, int fb_h, Vrst_vec2d pos, Vrst_float32 px);

  /// Draws a pin (search result, dropped pin, route start): a colored dot in a white
  /// disc, the same size at every zoom. Call after the map, the route and the track,
  /// before the GPS position and the labels.
  /// @param v Camera.
  /// @param fb_w Framebuffer width in px.
  /// @param fb_h Framebuffer height in px.
  /// @param pos Position, world coordinates (Vrst_map_frame::to_world()).
  /// @param px Diameter of the white disc in framebuffer pixels.
  /// @param r Red of the dot.
  /// @param g Green of the dot.
  /// @param b Blue of the dot.
  void draw_pin(const Vrst_view& v, int fb_w, int fb_h, Vrst_vec2d pos, Vrst_float32 px, Vrst_float32 r = 0.90f,
                Vrst_float32 g = 0.12f, Vrst_float32 b = 0.10f);

  /// Sets the route to draw with draw_route(); empty = none.
  /// @param points Route, world coordinates (Vrst_map_frame::to_world()).
  void set_route(std::vector<Vrst_vec2d> points) { m_route = std::move(points); }

  /// Draws the route as a blue line with a dark casing, the same width at every zoom.
  /// Rebuilt every frame from the route points (a few thousand at most). Call after
  /// the map tiles and the region, before the track, pins, position and labels.
  /// @param v Camera.
  /// @param fb_w Framebuffer width in px.
  /// @param fb_h Framebuffer height in px.
  /// @param px Line width in framebuffer pixels.
  void draw_route(const Vrst_view& v, int fb_w, int fb_h, Vrst_float32 px);

  /// Draws one round marker with the overlay program, which must be in use with its
  /// view matrix set. The marker moves every frame: one point, rewritten in place.
  /// @param pos World position.
  /// @param px Diameter in framebuffer pixels.
  /// @param r Red.
  /// @param g Green.
  /// @param b Blue.
  void draw_point(Vrst_vec2d pos, Vrst_float32 px, Vrst_float32 r, Vrst_float32 g, Vrst_float32 b) const;

  /// Shows a region outline: draw_region() dims the screen outside it with a grid and
  /// draws its border. Replaces an earlier region.
  /// @param shape Triangles and border strips, absolute Web Mercator.
  /// @param world_origin Absolute Mercator point of world (0, 0) (Vrst_map_frame::center()).
  void set_region(const Vrst_region_shape& shape, Vrst_vec2d world_origin);
  /// No map is open: draw_region() covers the whole screen with the grid.
  void set_region_everywhere() { m_region_mode = Vrst_region_mode::Everywhere; }
  /// The open map has no outline: draw_region() draws nothing.
  void clear_region() { m_region_mode = Vrst_region_mode::Off; }

  /// Draws the outside of the region (dim veil and a grid about 120 px apart, at 1, 2 or
  /// 5 x 10^n ground metres, anchored to the world so it moves with the map) and the
  /// border. Call after the map tiles, before the track, position and labels. Needs a
  /// stencil buffer.
  /// @param v Camera.
  /// @param fb_w Framebuffer width in px.
  /// @param fb_h Framebuffer height in px.
  /// @param world_origin Absolute Mercator point of world (0, 0).
  /// @param cos_lat Cosine of the latitude at the screen center (ground metres per
  ///        Mercator metre), for the grid spacing.
  void draw_region(const Vrst_view& v, int fb_w, int fb_h, Vrst_vec2d world_origin, Vrst_float64 cos_lat) const;

  /// Uploads the label glyph atlas and compiles the text program. Replaces an earlier atlas.
  /// @param atlas Built atlas; its pixels are copied and it may be freed.
  /// @throws std::runtime_error on a shader or GL error.
  void set_label_atlas(const Vrst_glyph_atlas& atlas);

  /// Draws label glyph quads over the map with alpha blending. Call after the map
  /// (and after draw_track(), so names stay readable over the track).
  /// @param verts Quads from Vrst_map_labels::layout(), framebuffer px.
  /// @param fb_w Framebuffer width in px.
  /// @param fb_h Framebuffer height in px.
  /// @param halo_px Halo width in framebuffer px.
  void draw_labels(const std::vector<Vrst_text_vertex>& verts, int fb_w, int fb_h, Vrst_float32 halo_px) const;

  /// Deletes all GL objects. Call with the context current, before it is destroyed.
  void destroy();

  /// Throws on the first pending OpenGL error.
  /// @param where Label for the message, e.g. "setup".
  /// @throws std::runtime_error with the GL error code in hex.
  static void check_error(std::string_view where);

  /// Compiles and links a program with kGlslHeader; shared with Vrst_tile_renderer.
  /// @param vs Vertex shader body.
  /// @param fs Fragment shader body.
  /// @return Program object.
  /// @throws std::runtime_error with the compile or link log.
  static GLuint link_program(std::string_view vs, std::string_view fs) { return link(vs, fs); }

  /// GLSL version line (plus precision qualifiers on ES) prepended to every shader.
  static constexpr std::string_view kGlslHeader =
#if VRST_GLES
      "#version 300 es\nprecision highp float;\nprecision highp int;\n";
#else
      "#version 330 core\n";
#endif

 private:
  /// What draw_region() draws.
  enum class Vrst_region_mode {
    Off,         ///< Nothing (no outline known).
    Everywhere,  ///< Veil and grid on the whole screen (no map).
    Outside,     ///< Veil and grid outside the region, and its border.
  };

  /// Compiles one shader with kGlslHeader prepended.
  /// @param type GL_VERTEX_SHADER or GL_FRAGMENT_SHADER.
  /// @param body GLSL source without a version line.
  /// @return Shader object.
  /// @throws std::runtime_error with the compile log.
  static GLuint compile(GLenum type, std::string_view body);

  /// Compiles and links a program; the shader objects are deleted after linking.
  /// @param vs Vertex shader body.
  /// @param fs Fragment shader body.
  /// @return Program object.
  /// @throws std::runtime_error with the compile or link log.
  static GLuint link(std::string_view vs, std::string_view fs);

  /// Uploads vertices and creates a VAO: attribute 0 = position (2 floats),
  /// attribute 1 = attr (uint).
  /// @param verts Vertices to upload.
  /// @param[out] vbo Receives the vertex buffer.
  /// @return Vertex array object, left bound.
  static GLuint make_vao(const std::vector<Vrst_gpu_point>& verts, GLuint& vbo);

  /// Track line and marker program: one uniform color, optional round point.
  GLuint m_overlay_prog = 0;
  /// GPS track points (set_track()), drawn as a line strip.
  GLuint m_track_vbo = 0;
  /// Track VAO: m_track_vbo layout.
  GLuint m_track_vao = 0;
  /// The route, world coordinates (set_route()).
  std::vector<Vrst_vec2d> m_route;
  /// Route triangles, rewritten every draw_route().
  GLuint m_route_vbo = 0;
  /// Route VAO: m_route_vbo layout.
  GLuint m_route_vao = 0;
  /// One marker point, rewritten every draw_track().
  GLuint m_marker_vbo = 0;
  /// Marker VAO: m_marker_vbo layout.
  GLuint m_marker_vao = 0;
  /// Number of track points; 0 = no track, draw_track() does nothing.
  GLsizei m_track_count = 0;
  /// u_view of the overlay program.
  GLint m_u_overlay_view = -1;
  /// u_point_px of the overlay program (marker size).
  GLint m_u_overlay_px = -1;
  /// u_color of the overlay program.
  GLint m_u_overlay_color = -1;
  /// u_round of the overlay program: round marker vs plain line.
  GLint m_u_overlay_round = -1;
  /// Label program (SDF text with halo); 0 until set_label_atlas().
  GLuint m_text_prog = 0;
  /// Glyph atlas, one 8-bit distance channel.
  GLuint m_atlas_tex = 0;
  /// Label quads, rewritten every frame.
  GLuint m_text_vbo = 0;
  /// Text VAO: m_text_vbo layout (position, uv).
  GLuint m_text_vao = 0;
  /// u_fb_size of the text program.
  GLint m_u_text_fb = -1;
  /// u_halo_px of the text program.
  GLint m_u_text_halo = -1;
  /// What draw_region() draws.
  Vrst_region_mode m_region_mode = Vrst_region_mode::Off;
  /// Veil-and-grid program.
  GLuint m_grid_prog = 0;
  /// Full-screen quad (two triangles in NDC).
  GLuint m_quad_vbo = 0;
  /// Quad VAO.
  GLuint m_quad_vao = 0;
  /// u_origin of the grid program.
  GLint m_u_grid_origin = -1;
  /// u_axis_x of the grid program.
  GLint m_u_grid_axis_x = -1;
  /// u_axis_y of the grid program.
  GLint m_u_grid_axis_y = -1;
  /// u_veil of the grid program.
  GLint m_u_grid_veil = -1;
  /// u_line of the grid program.
  GLint m_u_grid_line = -1;
  /// Region triangles relative to the world origin, then the border strips.
  GLuint m_region_vbo = 0;
  /// Region VAO (overlay layout).
  GLuint m_region_vao = 0;
  /// Vertices of the triangles at the start of m_region_vbo.
  GLsizei m_region_triangles = 0;
  /// First vertex of each border strip in m_region_vbo.
  std::vector<GLint> m_border_first;
  /// Vertices of each border strip.
  std::vector<GLsizei> m_border_count;
};

}  // namespace nav
