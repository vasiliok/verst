/// @file
/// OpenGL map renderer shared by the desktop viewer (OpenGL 3.3 core) and the iOS
/// app (OpenGL ES 3.0).
///
/// Uploads a Vrst_map_mesh once and draws roads (one indexed line list) and graph
/// vertices (one point list) for a camera, plus place name labels (glyph quads from
/// Vrst_map_labels) and an optional GPS track overlay with a position marker. Uses only calls both APIs have, so the
/// front ends only create the context, handle input and call draw().
#pragma once

#include <algorithm>
#include <array>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <string_view>

#include "gl_api.h"
#include "glyph_atlas.h"
#include "map_labels.h"
#include "map_scene.h"
#include "shaders.h"

namespace nav {

/// Per-frame display switches for Vrst_map_renderer::draw().
struct Vrst_draw_options {
  /// Bit per RoadClass; a 0 bit hides that class (see bit(), kClassGroups).
  Vrst_uint32 class_mask = kAllClasses;
  /// Highlight oneway segments and dim the rest.
  bool oneway = false;
  /// Draw the graph vertex dots.
  bool show_vertices = true;
  /// Vertex dot size in framebuffer pixels; dead ends are drawn twice as large.
  Vrst_float32 point_px = 2.5f;
};

/// GPU side of the map: shader programs, vertex and index buffers, and the draw
/// calls. Needs a current GL context for init(), draw() and destroy(); the
/// destructor does not touch GL, because the context may already be gone.
class Vrst_map_renderer {
 public:
  Vrst_map_renderer() = default;                                    ///< Empty; init() creates the GL objects.
  ~Vrst_map_renderer() = default;                                   ///< Does not touch GL; call destroy() first.
  Vrst_map_renderer(const Vrst_map_renderer&) = delete;             ///< Owns GL objects: not copyable.
  Vrst_map_renderer& operator=(const Vrst_map_renderer&) = delete;  ///< Owns GL objects: not copyable.
  Vrst_map_renderer(Vrst_map_renderer&&) = delete;                  ///< Owns GL objects: not movable (destroy() is explicit).
  Vrst_map_renderer& operator=(Vrst_map_renderer&&) = delete;       ///< Owns GL objects: not movable (destroy() is explicit).

  /// Compiles the shaders and uploads the mesh. Call once with the context current.
  /// @param mesh Built mesh; its arrays are copied to GPU buffers and may be freed.
  /// @throws std::runtime_error on a shader compile/link error or a GL error.
  void init(const Vrst_map_mesh& mesh);

  /// Compiles only the overlay program (GPS track, marker), for front ends that draw
  /// the map itself with Vrst_tile_renderer and use this class for the track and the
  /// labels. init() calls it too.
  /// @throws std::runtime_error on a shader compile/link error.
  void init_overlays();

  /// Clears the bound framebuffer and draws the map.
  /// @param v Camera.
  /// @param fb_w Framebuffer width in px.
  /// @param fb_h Framebuffer height in px.
  /// @param opt Display switches.
  void draw(const Vrst_view& v, int fb_w, int fb_h, const Vrst_draw_options& opt) const;

  /// Uploads a GPS track to draw with draw_track(). Replaces an earlier track.
  /// @param points Track points in world coordinates (Vrst_map_mesh::to_world()).
  /// @throws std::runtime_error on a GL error.
  void set_track(const std::vector<Vrst_vec2d>& points);

  /// Draws the track uploaded by set_track() over the map: the part not yet driven
  /// dim, the driven part bright, and a round marker at the current position. Call
  /// after draw(), with the same camera.
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
  /// @param pos Position, world coordinates (Vrst_map_mesh::to_world()).
  /// @param px Dot diameter in framebuffer pixels.
  void draw_position(const Vrst_view& v, int fb_w, int fb_h, Vrst_vec2d pos, Vrst_float32 px);

  /// Draws one round marker with the overlay program, which must be in use with its
  /// view matrix set. The marker moves every frame: one point, rewritten in place.
  /// @param pos World position.
  /// @param px Diameter in framebuffer pixels.
  /// @param r Red.
  /// @param g Green.
  /// @param b Blue.
  void draw_point(Vrst_vec2d pos, Vrst_float32 px, Vrst_float32 r, Vrst_float32 g, Vrst_float32 b) const;

  /// Uploads the label glyph atlas and compiles the text program. Replaces an earlier atlas.
  /// @param atlas Built atlas; its pixels are copied and it may be freed.
  /// @throws std::runtime_error on a shader or GL error.
  void set_label_atlas(const Vrst_glyph_atlas& atlas);

  /// Draws label glyph quads over the map with alpha blending. Call after draw()
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

  /// Road program: class color, class mask and oneway highlight.
  GLuint m_line_prog = 0;
  /// Vertex dot program: round dots, dead ends larger and red.
  GLuint m_point_prog = 0;
  /// Shape point vertices; one per point of geom.dat.
  GLuint m_line_vbo = 0;
  /// Graph vertex dots.
  GLuint m_point_vbo = 0;
  /// Line index pairs, minor classes first.
  GLuint m_ibo = 0;
  /// Line VAO: m_line_vbo layout plus m_ibo.
  GLuint m_line_vao = 0;
  /// Point VAO: m_point_vbo layout.
  GLuint m_point_vao = 0;
  /// Number of line indices (2 per drawn piece).
  GLsizei m_index_count = 0;
  /// Number of vertex dots.
  GLsizei m_point_count = 0;
  /// u_view of the line program.
  GLint m_u_line_view = -1;
  /// u_class_mask of the line program.
  GLint m_u_mask = -1;
  /// u_oneway of the line program.
  GLint m_u_oneway = -1;
  /// u_view of the point program.
  GLint m_u_point_view = -1;
  /// u_point_px of the point program.
  GLint m_u_point_px = -1;
  /// Track line and marker program: one uniform color, optional round point.
  GLuint m_overlay_prog = 0;
  /// GPS track points (set_track()), drawn as a line strip.
  GLuint m_track_vbo = 0;
  /// Track VAO: m_track_vbo layout.
  GLuint m_track_vao = 0;
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
};

}  // namespace nav
