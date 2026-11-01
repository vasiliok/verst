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
#include <cstdio>
#include <stdexcept>
#include <string>

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
  /// Compiles the shaders and uploads the mesh. Call once with the context current.
  /// @param mesh Built mesh; its arrays are copied to GPU buffers and may be freed.
  /// @throws std::runtime_error on a shader compile/link error or a GL error.
  void init(const Vrst_map_mesh& mesh) {
    init_overlays();
    m_line_prog = link(shaders::kLineVs, shaders::kLineFs);
    m_point_prog = link(shaders::kPointVs, shaders::kPointFs);
    m_u_line_view = glGetUniformLocation(m_line_prog, "u_view");
    m_u_mask = glGetUniformLocation(m_line_prog, "u_class_mask");
    m_u_oneway = glGetUniformLocation(m_line_prog, "u_oneway");
    m_u_point_view = glGetUniformLocation(m_point_prog, "u_view");
    m_u_point_px = glGetUniformLocation(m_point_prog, "u_point_px");
    glUseProgram(m_line_prog);
    glUniform3fv(glGetUniformLocation(m_line_prog, "u_colors"), kRoadClassCount, &kClassColors[0][0]);

    m_line_vao = make_vao(mesh.line_verts(), &m_line_vbo);
    glGenBuffers(1, &m_ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ibo);  // recorded in the line VAO
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(mesh.line_indices().size() * sizeof(Vrst_uint32)),
                 mesh.line_indices().data(), GL_STATIC_DRAW);
    m_point_vao = make_vao(mesh.point_verts(), &m_point_vbo);
    glBindVertexArray(0);
    m_index_count = GLsizei(mesh.line_indices().size());
    m_point_count = GLsizei(mesh.point_verts().size());
#if !VRST_GLES
    glEnable(GL_PROGRAM_POINT_SIZE);  // always on in OpenGL ES
    glEnable(GL_MULTISAMPLE);         // ES multisampling is a property of the framebuffer
#endif
    check_error("renderer init");
  }

  /// Compiles only the overlay program (GPS track, marker), for front ends that draw
  /// the map itself with Vrst_tile_renderer and use this class for the track and the
  /// labels. init() calls it too.
  /// @throws std::runtime_error on a shader compile/link error.
  void init_overlays() {
    m_overlay_prog = link(shaders::kOverlayVs, shaders::kOverlayFs);
    m_u_overlay_view = glGetUniformLocation(m_overlay_prog, "u_view");
    m_u_overlay_px = glGetUniformLocation(m_overlay_prog, "u_point_px");
    m_u_overlay_color = glGetUniformLocation(m_overlay_prog, "u_color");
    m_u_overlay_round = glGetUniformLocation(m_overlay_prog, "u_round");
  }

  /// Clears the bound framebuffer and draws the map.
  /// @param v Camera.
  /// @param fb_w Framebuffer width in px.
  /// @param fb_h Framebuffer height in px.
  /// @param opt Display switches.
  void draw(const Vrst_view& v, int fb_w, int fb_h, const Vrst_draw_options& opt) const {
    glViewport(0, 0, fb_w, fb_h);
    glClearColor(kBackground[0], kBackground[1], kBackground[2], 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    Vrst_float32 m[9];
    view_matrix(v, fb_w, fb_h, m);

    glUseProgram(m_line_prog);
    glUniformMatrix3fv(m_u_line_view, 1, GL_FALSE, m);
    glUniform1ui(m_u_mask, opt.class_mask);
    glUniform1i(m_u_oneway, opt.oneway);
    glBindVertexArray(m_line_vao);
    glDrawElements(GL_LINES, m_index_count, GL_UNSIGNED_INT, nullptr);

    if (opt.show_vertices) {
      glUseProgram(m_point_prog);
      glUniformMatrix3fv(m_u_point_view, 1, GL_FALSE, m);
      glUniform1f(m_u_point_px, opt.point_px);
      glBindVertexArray(m_point_vao);
      glDrawArrays(GL_POINTS, 0, m_point_count);
    }
    glBindVertexArray(0);
  }

  /// Uploads a GPS track to draw with draw_track(). Replaces an earlier track.
  /// @param points Track points in world coordinates (Vrst_map_mesh::to_world()).
  /// @throws std::runtime_error on a GL error.
  void set_track(const std::vector<Vrst_vec2d>& points) {
    std::vector<Vrst_gpu_point> verts(points.size());
    for (Vrst_size i = 0; i < points.size(); ++i)
      verts[i] = {Vrst_float32(points[i].x), Vrst_float32(points[i].y), 0};
    if (!m_track_vao) m_track_vao = make_vao({}, &m_track_vbo);
    glBindVertexArray(m_track_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_track_vbo);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(verts.size() * sizeof(Vrst_gpu_point)), verts.data(), GL_STATIC_DRAW);
    if (!m_marker_vao) m_marker_vao = make_vao({{0, 0, 0}}, &m_marker_vbo);
    glBindVertexArray(0);
    m_track_count = GLsizei(verts.size());
    check_error("set_track");
  }

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
                  Vrst_float32 marker_px) const {
    if (m_track_count == 0) return;
    Vrst_float32 m[9];
    view_matrix(v, fb_w, fb_h, m);
    glUseProgram(m_overlay_prog);
    glUniformMatrix3fv(m_u_overlay_view, 1, GL_FALSE, m);
    glUniform1i(m_u_overlay_round, 0);
    glBindVertexArray(m_track_vao);
    glUniform4f(m_u_overlay_color, 0.2f, 0.55f, 1.0f, 1.0f);
    glDrawArrays(GL_LINE_STRIP, 0, m_track_count);
    const GLsizei n = GLsizei(std::min<Vrst_size>(driven, Vrst_size(m_track_count)));
    if (n >= 2) {
      glUniform4f(m_u_overlay_color, 1.0f, 0.85f, 0.1f, 1.0f);
      glDrawArrays(GL_LINE_STRIP, 0, n);
    }
    draw_point(marker, marker_px, 0.95f, 0.15f, 0.1f);
  }

  /// Draws the current GPS position: a blue dot with a white ring, the same size at
  /// every zoom. Call after the map, before the labels.
  /// @param v Camera.
  /// @param fb_w Framebuffer width in px.
  /// @param fb_h Framebuffer height in px.
  /// @param pos Position, world coordinates (Vrst_map_mesh::to_world()).
  /// @param px Dot diameter in framebuffer pixels.
  void draw_position(const Vrst_view& v, int fb_w, int fb_h, Vrst_vec2d pos, Vrst_float32 px) {
    if (!m_marker_vao) {
      m_marker_vao = make_vao({{0, 0, 0}}, &m_marker_vbo);
      glBindVertexArray(0);
    }
    Vrst_float32 m[9];
    view_matrix(v, fb_w, fb_h, m);
    glUseProgram(m_overlay_prog);
    glUniformMatrix3fv(m_u_overlay_view, 1, GL_FALSE, m);
    draw_point(pos, px, 0.10f, 0.50f, 1.0f);
  }

  /// Draws one round marker with the overlay program, which must be in use with its
  /// view matrix set. The marker moves every frame: one point, rewritten in place.
  /// @param pos World position.
  /// @param px Diameter in framebuffer pixels.
  /// @param r Red.
  /// @param g Green.
  /// @param b Blue.
  void draw_point(Vrst_vec2d pos, Vrst_float32 px, Vrst_float32 r, Vrst_float32 g, Vrst_float32 b) const {
    const Vrst_gpu_point p{Vrst_float32(pos.x), Vrst_float32(pos.y), 0};
    glBindVertexArray(m_marker_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_marker_vbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof p, &p);
    glUniform1i(m_u_overlay_round, 1);
    glUniform1f(m_u_overlay_px, px);
    glUniform4f(m_u_overlay_color, r, g, b, 1.0f);
    glDrawArrays(GL_POINTS, 0, 1);
    glBindVertexArray(0);
  }

  /// Uploads the label glyph atlas and compiles the text program. Replaces an earlier atlas.
  /// @param atlas Built atlas; its pixels are copied and it may be freed.
  /// @throws std::runtime_error on a shader or GL error.
  void set_label_atlas(const Vrst_glyph_atlas& atlas) {
    if (!m_text_prog) {
      m_text_prog = link(shaders::kTextVs, shaders::kTextFs);
      m_u_text_fb = glGetUniformLocation(m_text_prog, "u_fb_size");
      glUseProgram(m_text_prog);
      glUniform1i(glGetUniformLocation(m_text_prog, "u_atlas"), 0);
      glUniform3f(glGetUniformLocation(m_text_prog, "u_text_color"), 0.96f, 0.96f, 0.93f);
      glUniform3f(glGetUniformLocation(m_text_prog, "u_halo_color"), 0.07f, 0.08f, 0.10f);
      m_u_text_halo = glGetUniformLocation(m_text_prog, "u_halo_px");
      glGenTextures(1, &m_atlas_tex);
      glGenVertexArrays(1, &m_text_vao);
      glGenBuffers(1, &m_text_vbo);
      glBindVertexArray(m_text_vao);
      glBindBuffer(GL_ARRAY_BUFFER, m_text_vbo);
      glEnableVertexAttribArray(0);
      glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vrst_text_vertex), reinterpret_cast<void*>(0));
      glEnableVertexAttribArray(1);
      glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(Vrst_text_vertex), reinterpret_cast<void*>(8));
      glBindVertexArray(0);
    }
    glBindTexture(GL_TEXTURE_2D, m_atlas_tex);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    // R8 is filterable in both GL 3.3 and GLES 3.0; linear filtering is what makes the
    // distance field scale smoothly.
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, atlas.width(), atlas.height(), 0, GL_RED, GL_UNSIGNED_BYTE,
                 atlas.pixels().data());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);
    check_error("set_label_atlas");
  }

  /// Draws label glyph quads over the map with alpha blending. Call after draw()
  /// (and after draw_track(), so names stay readable over the track).
  /// @param verts Quads from Vrst_map_labels::layout(), framebuffer px.
  /// @param fb_w Framebuffer width in px.
  /// @param fb_h Framebuffer height in px.
  /// @param halo_px Halo width in framebuffer px.
  void draw_labels(const std::vector<Vrst_text_vertex>& verts, int fb_w, int fb_h, Vrst_float32 halo_px) const {
    if (!m_text_prog || verts.empty()) return;
    glBindBuffer(GL_ARRAY_BUFFER, m_text_vbo);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(verts.size() * sizeof(Vrst_text_vertex)), verts.data(), GL_STREAM_DRAW);
    glUseProgram(m_text_prog);
    glUniform2f(m_u_text_fb, Vrst_float32(fb_w), Vrst_float32(fb_h));
    glUniform1f(m_u_text_halo, halo_px);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_atlas_tex);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBindVertexArray(m_text_vao);
    glDrawArrays(GL_TRIANGLES, 0, GLsizei(verts.size()));
    glBindVertexArray(0);
    glDisable(GL_BLEND);
    glBindTexture(GL_TEXTURE_2D, 0);
  }

  /// Deletes all GL objects. Call with the context current, before it is destroyed.
  void destroy() {
    glDeleteBuffers(1, &m_line_vbo);
    glDeleteBuffers(1, &m_point_vbo);
    glDeleteBuffers(1, &m_ibo);
    glDeleteVertexArrays(1, &m_line_vao);
    glDeleteVertexArrays(1, &m_point_vao);
    glDeleteProgram(m_line_prog);
    glDeleteProgram(m_point_prog);
    glDeleteBuffers(1, &m_track_vbo);
    glDeleteBuffers(1, &m_marker_vbo);
    glDeleteVertexArrays(1, &m_track_vao);
    glDeleteVertexArrays(1, &m_marker_vao);
    glDeleteProgram(m_overlay_prog);
    m_line_vbo = m_point_vbo = m_ibo = m_line_vao = m_point_vao = m_line_prog = m_point_prog = 0;
    m_track_vbo = m_marker_vbo = m_track_vao = m_marker_vao = m_overlay_prog = 0;
    m_track_count = 0;
    glDeleteBuffers(1, &m_text_vbo);
    glDeleteVertexArrays(1, &m_text_vao);
    glDeleteTextures(1, &m_atlas_tex);
    glDeleteProgram(m_text_prog);
    m_text_vbo = m_text_vao = m_atlas_tex = m_text_prog = 0;
  }

  /// Throws on the first pending OpenGL error.
  /// @param where Label for the message, e.g. "setup".
  /// @throws std::runtime_error with the GL error code in hex.
  static void check_error(const char* where) {
    if (const GLenum e = glGetError(); e != GL_NO_ERROR) {
      char msg[96];
      std::snprintf(msg, sizeof msg, "GL error 0x%04x at %s", e, where);
      throw std::runtime_error(msg);
    }
  }

  /// Compiles and links a program with kGlslHeader; shared with Vrst_tile_renderer.
  /// @param vs Vertex shader body.
  /// @param fs Fragment shader body.
  /// @return Program object.
  /// @throws std::runtime_error with the compile or link log.
  static GLuint link_program(const char* vs, const char* fs) { return link(vs, fs); }

  /// GLSL version line (plus precision qualifiers on ES) prepended to every shader.
  static constexpr const char* kGlslHeader =
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
  static GLuint compile(GLenum type, const char* body) {
    const GLuint s = glCreateShader(type);
    const char* parts[2] = {kGlslHeader, body};
    glShaderSource(s, 2, parts, nullptr);
    glCompileShader(s);
    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
      char log[2048];
      glGetShaderInfoLog(s, sizeof log, nullptr, log);
      throw std::runtime_error(std::string("shader compile: ") + log);
    }
    return s;
  }

  /// Compiles and links a program; the shader objects are deleted after linking.
  /// @param vs Vertex shader body.
  /// @param fs Fragment shader body.
  /// @return Program object.
  /// @throws std::runtime_error with the compile or link log.
  static GLuint link(const char* vs, const char* fs) {
    const GLuint p = glCreateProgram();
    const GLuint v = compile(GL_VERTEX_SHADER, vs), f = compile(GL_FRAGMENT_SHADER, fs);
    glAttachShader(p, v);
    glAttachShader(p, f);
    glLinkProgram(p);
    glDeleteShader(v);
    glDeleteShader(f);
    GLint ok = 0;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
      char log[2048];
      glGetProgramInfoLog(p, sizeof log, nullptr, log);
      throw std::runtime_error(std::string("program link: ") + log);
    }
    return p;
  }

  /// Uploads vertices and creates a VAO: attribute 0 = position (2 floats),
  /// attribute 1 = attr (uint).
  /// @param verts Vertices to upload.
  /// @param[out] vbo Receives the vertex buffer.
  /// @return Vertex array object, left bound.
  static GLuint make_vao(const std::vector<Vrst_gpu_point>& verts, GLuint* vbo) {
    GLuint vao = 0;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);
    glGenBuffers(1, vbo);
    glBindBuffer(GL_ARRAY_BUFFER, *vbo);
    // Small buffers (the marker) are rewritten every frame; DYNAMIC hints that.
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(verts.size() * sizeof(Vrst_gpu_point)), verts.data(),
                 verts.size() <= 1 ? GL_DYNAMIC_DRAW : GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vrst_gpu_point), reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribIPointer(1, 1, GL_UNSIGNED_INT, sizeof(Vrst_gpu_point), reinterpret_cast<void*>(8));
    return vao;
  }

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
