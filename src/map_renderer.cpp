/// @file
/// Vrst_map_renderer: GL setup, overlays (GPS track, position) and labels.
/// Declarations and documentation are in map_renderer.h.
#include "map_renderer.h"

#include <cmath>

namespace nav {

void Vrst_map_renderer::init_overlays() {
  m_overlay_prog = link(shaders::kOverlayVs, shaders::kOverlayFs);
  m_u_overlay_view = glGetUniformLocation(m_overlay_prog, "u_view");
  m_u_overlay_px = glGetUniformLocation(m_overlay_prog, "u_point_px");
  m_u_overlay_color = glGetUniformLocation(m_overlay_prog, "u_color");
  m_u_overlay_round = glGetUniformLocation(m_overlay_prog, "u_round");
  m_grid_prog = link(shaders::kGridVs, shaders::kGridFs);
  m_u_grid_origin = glGetUniformLocation(m_grid_prog, "u_origin");
  m_u_grid_axis_x = glGetUniformLocation(m_grid_prog, "u_axis_x");
  m_u_grid_axis_y = glGetUniformLocation(m_grid_prog, "u_axis_y");
  m_u_grid_veil = glGetUniformLocation(m_grid_prog, "u_veil");
  m_u_grid_line = glGetUniformLocation(m_grid_prog, "u_line");
  m_quad_vao = make_vao({{-1, -1, 0}, {1, -1, 0}, {1, 1, 0}, {-1, -1, 0}, {1, 1, 0}, {-1, 1, 0}}, m_quad_vbo);
  glBindVertexArray(0);
}

void Vrst_map_renderer::set_region(const Vrst_region_shape& shape, Vrst_vec2d world_origin) {
  std::vector<Vrst_gpu_point> verts;
  const auto add = [&](Vrst_vec2d p) {
    verts.push_back({static_cast<Vrst_float32>(p.x - world_origin.x), static_cast<Vrst_float32>(p.y - world_origin.y), 0});
  };
  for (const Vrst_vec2d& p : shape.triangles) add(p);
  m_region_triangles = static_cast<GLsizei>(verts.size());
  m_border_first.clear();
  m_border_count.clear();
  for (const std::vector<Vrst_vec2d>& strip : shape.outlines) {
    m_border_first.push_back(static_cast<GLint>(verts.size()));
    for (const Vrst_vec2d& p : strip) add(p);
    m_border_count.push_back(static_cast<GLsizei>(strip.size()));
  }
  if (!m_region_vao) m_region_vao = make_vao({}, m_region_vbo);
  glBindVertexArray(m_region_vao);
  glBindBuffer(GL_ARRAY_BUFFER, m_region_vbo);
  glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(verts.size() * sizeof(Vrst_gpu_point)), verts.data(),
               GL_STATIC_DRAW);
  glBindVertexArray(0);
  m_region_mode = Vrst_region_mode::Outside;
  check_error("set_region");
}

void Vrst_map_renderer::draw_region(const Vrst_view& v, int fb_w, int fb_h, Vrst_vec2d world_origin,
                                    Vrst_float64 cos_lat) const {
  if (m_region_mode == Vrst_region_mode::Off || !m_grid_prog) return;
  const std::array<Vrst_float32, 9> m = view_matrix(v, fb_w, fb_h);
  if (m_region_mode == Vrst_region_mode::Outside) {
    // The region into the stencil only; the veil then goes where the stencil stayed 0.
    glEnable(GL_STENCIL_TEST);
    glStencilFunc(GL_ALWAYS, 1, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glUseProgram(m_overlay_prog);
    glUniformMatrix3fv(m_u_overlay_view, 1, GL_FALSE, m.data());
    glUniform1i(m_u_overlay_round, 0);
    glBindVertexArray(m_region_vao);
    glDrawArrays(GL_TRIANGLES, 0, m_region_triangles);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glStencilFunc(GL_EQUAL, 0, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
  }
  // Grid spacing: 1, 2 or 5 x 10^n ground metres, the first at least 120 px apart.
  const Vrst_float64 ground_per_px = cos_lat / v.scale;
  Vrst_float64 step = std::pow(10.0, std::floor(std::log10(120 * ground_per_px)));
  for (const Vrst_float64 f : {1.0, 2.0, 5.0, 10.0})
    if (f * step >= 120 * ground_per_px) {
      step *= f;
      break;
    }
  const Vrst_float64 spacing = step / cos_lat;  // Mercator metres
  // NDC -> world: p = center + R(-angle) * (ndc * fb / 2 / scale), in grid cells.
  const Vrst_float64 c = std::cos(v.angle), s = std::sin(v.angle);
  const Vrst_float64 hx = fb_w / 2.0 / v.scale / spacing, hy = fb_h / 2.0 / v.scale / spacing;
  const Vrst_vec2d abs_center{world_origin.x + v.center.x, world_origin.y + v.center.y};
  glUseProgram(m_grid_prog);
  glUniform2f(m_u_grid_origin, static_cast<Vrst_float32>(std::fmod(abs_center.x, spacing) / spacing),
              static_cast<Vrst_float32>(std::fmod(abs_center.y, spacing) / spacing));
  glUniform2f(m_u_grid_axis_x, static_cast<Vrst_float32>(c * hx), static_cast<Vrst_float32>(-s * hx));
  glUniform2f(m_u_grid_axis_y, static_cast<Vrst_float32>(s * hy), static_cast<Vrst_float32>(c * hy));
  // Bluish veil and clear lines: the outside must read as "no map here" on the dark map.
  glUniform4f(m_u_grid_veil, 0.10f, 0.12f, 0.18f, 0.75f);
  glUniform4f(m_u_grid_line, 0.45f, 0.52f, 0.65f, 0.9f);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glBindVertexArray(m_quad_vao);
  glDrawArrays(GL_TRIANGLES, 0, 6);
  glDisable(GL_BLEND);
  glDisable(GL_STENCIL_TEST);
  if (m_region_mode == Vrst_region_mode::Outside) {
    glUseProgram(m_overlay_prog);
    glUniformMatrix3fv(m_u_overlay_view, 1, GL_FALSE, m.data());
    glUniform4f(m_u_overlay_color, 1.0f, 0.55f, 0.15f, 1.0f);
    glBindVertexArray(m_region_vao);
    for (Vrst_size i = 0; i < m_border_first.size(); ++i) glDrawArrays(GL_LINE_STRIP, m_border_first[i], m_border_count[i]);
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

void Vrst_map_renderer::draw_route(const Vrst_view& v, int fb_w, int fb_h, Vrst_float32 px) {
  if (m_route.size() < 2) return;
  if (!m_route_vao) m_route_vao = make_vao({}, m_route_vbo);
  const std::array<Vrst_float32, 9> m = view_matrix(v, fb_w, fb_h);
  glUseProgram(m_overlay_prog);
  glUniformMatrix3fv(m_u_overlay_view, 1, GL_FALSE, m.data());
  glUniform1i(m_u_overlay_round, 0);
  glBindVertexArray(m_route_vao);
  glBindBuffer(GL_ARRAY_BUFFER, m_route_vbo);
  std::vector<Vrst_gpu_point> verts;
  verts.reserve(m_route.size() * 6);
  // Two passes: the casing, then the line on top. Each piece is a quad lengthened by
  // half the width at both ends, so the corners have no gaps.
  constexpr Vrst_float32 kCasingPx = 3;
  for (const auto& [width, r, g, b] : {std::array<Vrst_float32, 4>{px + kCasingPx, 0.05f, 0.18f, 0.45f},
                                       std::array<Vrst_float32, 4>{px, 0.20f, 0.55f, 1.0f}}) {
    const Vrst_float64 h = width / 2.0 / v.scale;  // world units
    verts.clear();
    for (Vrst_size i = 0; i + 1 < m_route.size(); ++i) {
      const Vrst_vec2d p0 = m_route[i], p1 = m_route[i + 1];
      const Vrst_float64 len = std::hypot(p1.x - p0.x, p1.y - p0.y);
      if (len <= 0) continue;
      const Vrst_vec2d d{(p1.x - p0.x) / len * h, (p1.y - p0.y) / len * h}, n{-d.y, d.x};
      const auto at = [](Vrst_float64 x, Vrst_float64 y) {
        return Vrst_gpu_point{static_cast<Vrst_float32>(x), static_cast<Vrst_float32>(y), 0};
      };
      const Vrst_gpu_point a0 = at(p0.x - d.x + n.x, p0.y - d.y + n.y), a1 = at(p0.x - d.x - n.x, p0.y - d.y - n.y);
      const Vrst_gpu_point b0 = at(p1.x + d.x + n.x, p1.y + d.y + n.y), b1 = at(p1.x + d.x - n.x, p1.y + d.y - n.y);
      verts.insert(verts.end(), {a0, a1, b0, a1, b1, b0});
    }
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(verts.size() * sizeof(Vrst_gpu_point)), verts.data(),
                 GL_STREAM_DRAW);
    glUniform4f(m_u_overlay_color, r, g, b, 1.0f);
    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(verts.size()));
  }
  glBindVertexArray(0);
  check_error("draw_route");
}

void Vrst_map_renderer::draw_pin(const Vrst_view& v, int fb_w, int fb_h, Vrst_vec2d pos, Vrst_float32 px,
                                 Vrst_float32 r, Vrst_float32 g, Vrst_float32 b) {
  constexpr Vrst_float32 kDot = 0.62f;  // red dot diameter, of the white disc
  if (!m_marker_vao) {
    m_marker_vao = make_vao({{0, 0, 0}}, m_marker_vbo);
    glBindVertexArray(0);
  }
  const std::array<Vrst_float32, 9> m = view_matrix(v, fb_w, fb_h);
  glUseProgram(m_overlay_prog);
  glUniformMatrix3fv(m_u_overlay_view, 1, GL_FALSE, m.data());
  draw_point(pos, px, 1.0f, 1.0f, 1.0f);
  draw_point(pos, kDot * px, r, g, b);
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
  glDeleteBuffers(1, &m_track_vbo);
  glDeleteBuffers(1, &m_marker_vbo);
  glDeleteVertexArrays(1, &m_track_vao);
  glDeleteVertexArrays(1, &m_marker_vao);
  glDeleteProgram(m_overlay_prog);
  m_track_vbo = m_marker_vbo = m_track_vao = m_marker_vao = m_overlay_prog = 0;
  m_track_count = 0;
  glDeleteBuffers(1, &m_route_vbo);
  glDeleteVertexArrays(1, &m_route_vao);
  m_route_vbo = m_route_vao = 0;
  glDeleteBuffers(1, &m_text_vbo);
  glDeleteVertexArrays(1, &m_text_vao);
  glDeleteTextures(1, &m_atlas_tex);
  glDeleteProgram(m_text_prog);
  m_text_vbo = m_text_vao = m_atlas_tex = m_text_prog = 0;
  glDeleteBuffers(1, &m_quad_vbo);
  glDeleteBuffers(1, &m_region_vbo);
  glDeleteVertexArrays(1, &m_quad_vao);
  glDeleteVertexArrays(1, &m_region_vao);
  glDeleteProgram(m_grid_prog);
  m_quad_vbo = m_region_vbo = m_quad_vao = m_region_vao = m_grid_prog = 0;
  m_region_mode = Vrst_region_mode::Off;
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
