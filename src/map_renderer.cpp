/// @file
/// Vrst_map_renderer: GL setup, drawing of roads, overlays and labels.
/// Declarations and documentation are in map_renderer.h.
#include "map_renderer.h"

namespace nav {

void Vrst_map_renderer::init(const Vrst_map_mesh& mesh) {
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

  m_line_vao = make_vao(mesh.line_verts(), m_line_vbo);
  glGenBuffers(1, &m_ibo);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ibo);  // recorded in the line VAO
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(mesh.line_indices().size() * sizeof(Vrst_uint32)),
               mesh.line_indices().data(), GL_STATIC_DRAW);
  m_point_vao = make_vao(mesh.point_verts(), m_point_vbo);
  glBindVertexArray(0);
  m_index_count = static_cast<GLsizei>(mesh.line_indices().size());
  m_point_count = static_cast<GLsizei>(mesh.point_verts().size());
#if !VRST_GLES
  glEnable(GL_PROGRAM_POINT_SIZE);  // always on in OpenGL ES
  glEnable(GL_MULTISAMPLE);         // ES multisampling is a property of the framebuffer
#endif
  check_error("renderer init");
}

void Vrst_map_renderer::init_overlays() {
  m_overlay_prog = link(shaders::kOverlayVs, shaders::kOverlayFs);
  m_u_overlay_view = glGetUniformLocation(m_overlay_prog, "u_view");
  m_u_overlay_px = glGetUniformLocation(m_overlay_prog, "u_point_px");
  m_u_overlay_color = glGetUniformLocation(m_overlay_prog, "u_color");
  m_u_overlay_round = glGetUniformLocation(m_overlay_prog, "u_round");
}

void Vrst_map_renderer::draw(const Vrst_view& v, int fb_w, int fb_h, const Vrst_draw_options& opt) const {
  glViewport(0, 0, fb_w, fb_h);
  glClearColor(kBackground[0], kBackground[1], kBackground[2], 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);
  const std::array<Vrst_float32, 9> m = view_matrix(v, fb_w, fb_h);

  glUseProgram(m_line_prog);
  glUniformMatrix3fv(m_u_line_view, 1, GL_FALSE, m.data());
  glUniform1ui(m_u_mask, opt.class_mask);
  glUniform1i(m_u_oneway, opt.oneway);
  glBindVertexArray(m_line_vao);
  glDrawElements(GL_LINES, m_index_count, GL_UNSIGNED_INT, nullptr);

  if (opt.show_vertices) {
    glUseProgram(m_point_prog);
    glUniformMatrix3fv(m_u_point_view, 1, GL_FALSE, m.data());
    glUniform1f(m_u_point_px, opt.point_px);
    glBindVertexArray(m_point_vao);
    glDrawArrays(GL_POINTS, 0, m_point_count);
  }
  glBindVertexArray(0);
}

void Vrst_map_renderer::set_track(const std::vector<Vrst_vec2d>& points) {
  std::vector<Vrst_gpu_point> verts(points.size());
  for (Vrst_size i = 0; i < points.size(); ++i)
    verts[i] = {static_cast<Vrst_float32>(points[i].x), static_cast<Vrst_float32>(points[i].y), 0};
  if (!m_track_vao) m_track_vao = make_vao({}, m_track_vbo);
  glBindVertexArray(m_track_vao);
  glBindBuffer(GL_ARRAY_BUFFER, m_track_vbo);
  glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(verts.size() * sizeof(Vrst_gpu_point)), verts.data(),
               GL_STATIC_DRAW);
  if (!m_marker_vao) m_marker_vao = make_vao({{0, 0, 0}}, m_marker_vbo);
  glBindVertexArray(0);
  m_track_count = static_cast<GLsizei>(verts.size());
  check_error("set_track");
}

void Vrst_map_renderer::draw_track(const Vrst_view& v, int fb_w, int fb_h, Vrst_size driven, Vrst_vec2d marker,
                                   Vrst_float32 marker_px) const {
  if (m_track_count == 0) return;
  const std::array<Vrst_float32, 9> m = view_matrix(v, fb_w, fb_h);
  glUseProgram(m_overlay_prog);
  glUniformMatrix3fv(m_u_overlay_view, 1, GL_FALSE, m.data());
  glUniform1i(m_u_overlay_round, 0);
  glBindVertexArray(m_track_vao);
  glUniform4f(m_u_overlay_color, 0.2f, 0.55f, 1.0f, 1.0f);
  glDrawArrays(GL_LINE_STRIP, 0, m_track_count);
  const GLsizei n = static_cast<GLsizei>(std::min<Vrst_size>(driven, static_cast<Vrst_size>(m_track_count)));
  if (n >= 2) {
    glUniform4f(m_u_overlay_color, 1.0f, 0.85f, 0.1f, 1.0f);
    glDrawArrays(GL_LINE_STRIP, 0, n);
  }
  draw_point(marker, marker_px, 0.95f, 0.15f, 0.1f);
}

void Vrst_map_renderer::draw_position(const Vrst_view& v, int fb_w, int fb_h, Vrst_vec2d pos, Vrst_float32 px) {
  if (!m_marker_vao) {
    m_marker_vao = make_vao({{0, 0, 0}}, m_marker_vbo);
    glBindVertexArray(0);
  }
  const std::array<Vrst_float32, 9> m = view_matrix(v, fb_w, fb_h);
  glUseProgram(m_overlay_prog);
  glUniformMatrix3fv(m_u_overlay_view, 1, GL_FALSE, m.data());
  draw_point(pos, px, 0.10f, 0.50f, 1.0f);
}

void Vrst_map_renderer::draw_point(Vrst_vec2d pos, Vrst_float32 px, Vrst_float32 r, Vrst_float32 g,
                                   Vrst_float32 b) const {
  const Vrst_gpu_point p{static_cast<Vrst_float32>(pos.x), static_cast<Vrst_float32>(pos.y), 0};
  glBindVertexArray(m_marker_vao);
  glBindBuffer(GL_ARRAY_BUFFER, m_marker_vbo);
  glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof p, &p);
  glUniform1i(m_u_overlay_round, 1);
  glUniform1f(m_u_overlay_px, px);
  glUniform4f(m_u_overlay_color, r, g, b, 1.0f);
  glDrawArrays(GL_POINTS, 0, 1);
  glBindVertexArray(0);
}

void Vrst_map_renderer::set_label_atlas(const Vrst_glyph_atlas& atlas) {
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

void Vrst_map_renderer::draw_labels(const std::vector<Vrst_text_vertex>& verts, int fb_w, int fb_h,
                                    Vrst_float32 halo_px) const {
  if (!m_text_prog || verts.empty()) return;
  glBindBuffer(GL_ARRAY_BUFFER, m_text_vbo);
  glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(verts.size() * sizeof(Vrst_text_vertex)), verts.data(),
               GL_STREAM_DRAW);
  glUseProgram(m_text_prog);
  glUniform2f(m_u_text_fb, static_cast<Vrst_float32>(fb_w), static_cast<Vrst_float32>(fb_h));
  glUniform1f(m_u_text_halo, halo_px);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, m_atlas_tex);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glBindVertexArray(m_text_vao);
  glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(verts.size()));
  glBindVertexArray(0);
  glDisable(GL_BLEND);
  glBindTexture(GL_TEXTURE_2D, 0);
}

void Vrst_map_renderer::destroy() {
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

void Vrst_map_renderer::check_error(std::string_view where) {
  if (const GLenum e = glGetError(); e != GL_NO_ERROR) {
    std::array<char, 16> hex{};
    std::snprintf(hex.data(), hex.size(), "0x%04x", e);
    throw std::runtime_error("GL error " + std::string(hex.data()) + " at " + std::string(where));
  }
}

GLuint Vrst_map_renderer::compile(GLenum type, std::string_view body) {
  const GLuint s = glCreateShader(type);
  // The GL API takes C strings; the lengths make NUL terminators unnecessary.
  const std::array<const GLchar*, 2> parts = {kGlslHeader.data(), body.data()};
  const std::array<GLint, 2> lengths = {static_cast<GLint>(kGlslHeader.size()), static_cast<GLint>(body.size())};
  glShaderSource(s, 2, parts.data(), lengths.data());
  glCompileShader(s);
  GLint ok = 0;
  glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
  if (!ok) {
    std::array<char, 2048> log{};
    glGetShaderInfoLog(s, static_cast<GLsizei>(log.size()), nullptr, log.data());
    throw std::runtime_error("shader compile: " + std::string(log.data()));
  }
  return s;
}

GLuint Vrst_map_renderer::link(std::string_view vs, std::string_view fs) {
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
    std::array<char, 2048> log{};
    glGetProgramInfoLog(p, static_cast<GLsizei>(log.size()), nullptr, log.data());
    throw std::runtime_error("program link: " + std::string(log.data()));
  }
  return p;
}

GLuint Vrst_map_renderer::make_vao(const std::vector<Vrst_gpu_point>& verts, GLuint& vbo) {
  GLuint vao = 0;
  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);
  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  // Small buffers (the marker) are rewritten every frame; DYNAMIC hints that.
  glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(verts.size() * sizeof(Vrst_gpu_point)), verts.data(),
               verts.size() <= 1 ? GL_DYNAMIC_DRAW : GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vrst_gpu_point), reinterpret_cast<void*>(0));
  glEnableVertexAttribArray(1);
  glVertexAttribIPointer(1, 1, GL_UNSIGNED_INT, sizeof(Vrst_gpu_point), reinterpret_cast<void*>(8));
  return vao;
}

}  // namespace nav
