/// @file
/// OpenGL map renderer shared by the desktop viewer (OpenGL 3.3 core) and the iOS
/// app (OpenGL ES 3.0).
///
/// Uploads a Vrst_map_mesh once and draws roads (one indexed line list) and graph
/// vertices (one point list) for a camera. Uses only calls both APIs have, so the
/// front ends only create the context, handle input and call draw().
#pragma once

#include <cstdio>
#include <stdexcept>
#include <string>

#include "gl_api.h"
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

  /// Deletes all GL objects. Call with the context current, before it is destroyed.
  void destroy() {
    glDeleteBuffers(1, &m_line_vbo);
    glDeleteBuffers(1, &m_point_vbo);
    glDeleteBuffers(1, &m_ibo);
    glDeleteVertexArrays(1, &m_line_vao);
    glDeleteVertexArrays(1, &m_point_vao);
    glDeleteProgram(m_line_prog);
    glDeleteProgram(m_point_prog);
    m_line_vbo = m_point_vbo = m_ibo = m_line_vao = m_point_vao = m_line_prog = m_point_prog = 0;
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
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(verts.size() * sizeof(Vrst_gpu_point)), verts.data(), GL_STATIC_DRAW);
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
};

}  // namespace nav
