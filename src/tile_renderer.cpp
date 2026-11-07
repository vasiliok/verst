/// @file
/// Vrst_tile_renderer: tile upload, cache and drawing; select_tiles().
/// Declarations and documentation are in tile_renderer.h.
#include "tile_renderer.h"

namespace nav {

void Vrst_tile_renderer::init() {
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
  for (const std::string n : {"u_view", "u_half_fb", "u_px_per_m", "u_width", "u_extra_px", "u_class_mask", "u_pass",
                              "u_pass_color", "u_dash_px", "u_oneway"})
    m_u[n] = glGetUniformLocation(m_line_prog, n.c_str());
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

void Vrst_tile_renderer::add(const Vrst_tile_data& t, Vrst_uint64 frame) {
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
  if (!t.fills.empty()) g.fill_vao = point_vao(t.fills, g.fill_vbo);
  if (!t.points.empty()) g.point_vao = point_vao(t.points, g.point_vbo);
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

bool Vrst_tile_renderer::use(Vrst_tile_id t, Vrst_uint64 frame) {
  const auto it = m_tiles.find(tile_key(t));
  if (it == m_tiles.end()) return false;
  it->second.last_used = frame;
  return true;
}

void Vrst_tile_renderer::evict(Vrst_size budget, Vrst_uint64 frame) {
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

void Vrst_tile_renderer::draw(const Vrst_view& v, int fb_w, int fb_h, const std::vector<Vrst_uint64>& keys,
                              Vrst_float64 ground_mpp, const Vrst_tile_draw_options& opt) {
  glViewport(0, 0, fb_w, fb_h);
  glClearColor(kBackground[0], kBackground[1], kBackground[2], 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);
  m_draw_calls = 0;
  std::vector<std::reference_wrapper<const Vrst_gpu_tile>> tiles;
  for (const Vrst_uint64 k : keys) tiles.push_back(std::cref(m_tiles.at(k)));
  std::vector<std::array<Vrst_float32, 9>> mats(tiles.size());
  for (Vrst_size i = 0; i < tiles.size(); ++i) {
    const Vrst_gpu_tile& t = tiles[i];
    Vrst_view tv = v;
    tv.center = {v.center.x - t.origin.x, v.center.y - t.origin.y};
    mats[i] = view_matrix(tv, fb_w, fb_h);
  }

  // Area fills, kind by kind across all tiles (sorted bottom to top).
  glUseProgram(m_fill_prog);
  for (int k = 0; k < kAreaKindCount; ++k) {
    const bool building = k == int(AreaKind::Building);
    if (building ? !opt.buildings : !opt.areas) continue;
    for (Vrst_size i = 0; i < tiles.size(); ++i) {
      const Vrst_gpu_tile& t = tiles[i];
      const Vrst_tile_range r = t.fill_ranges[k];
      if (r.count == 0) continue;
      glUniformMatrix3fv(m_u_fill_view, 1, GL_FALSE, mats[i].data());
      glBindVertexArray(t.fill_vao);
      glDrawArrays(GL_TRIANGLES, GLint(r.first), GLsizei(r.count));
      ++m_draw_calls;
    }
  }

  // Lines.
  glUseProgram(m_line_prog);
  const std::array<Vrst_float32, kLineStyleCount> width = line_widths(ground_mpp, opt.px_ratio);
  glUniform1fv(m_u.at("u_width"), kLineStyleCount, width.data());
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
  const std::array<Vrst_float32, 3> casing_light = {0.75f, 0.75f, 0.78f}, casing_dark = {0.02f, 0.02f, 0.03f};
  // color: the pass colour, or empty to keep the style colours.
  auto pass = [&](int layer, int style, int mode, Vrst_float32 extra, std::span<const Vrst_float32> color) {
    glUniform1i(m_u.at("u_pass"), mode);
    glUniform1f(m_u.at("u_extra_px"), extra);
    if (!color.empty()) glUniform3fv(m_u.at("u_pass_color"), 1, color.data());
    for (Vrst_size i = 0; i < tiles.size(); ++i) {
      const Vrst_gpu_tile& t = tiles[i];
      draw_lines(t, mats[i], t.line_ranges[layer][style]);
    }
  };
  for (const int style : kLineStyleOrder)
    if (shown(style)) pass(kLayerTunnel, style, 3, 0, {});
  for (const int style : kLineStyleOrder) {
    if (!shown(style)) continue;
    pass(kLayerGround, style, 0, 0, {});
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
    pass(kLayerBridge, style, 0, 0, {});
    if (rail(style)) pass(kLayerBridge, style, 2, Vrst_float32(-1.4 * opt.px_ratio), kBackground);
  }

  if (opt.show_vertices) {
    glUseProgram(m_point_prog);
    glUniform1f(m_u_point_px, opt.point_px);
    for (Vrst_size i = 0; i < tiles.size(); ++i) {
      const Vrst_gpu_tile& t = tiles[i];
      if (!t.point_count) continue;
      glUniformMatrix3fv(m_u_point_view, 1, GL_FALSE, mats[i].data());
      glBindVertexArray(t.point_vao);
      glDrawArrays(GL_POINTS, 0, t.point_count);
      ++m_draw_calls;
    }
  }
  if (opt.borders) draw_borders(v, fb_w, fb_h, tiles, opt.px_ratio);
  glBindVertexArray(0);
}

void Vrst_tile_renderer::destroy() {
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

std::array<Vrst_float32, kLineStyleCount> Vrst_tile_renderer::line_widths(Vrst_float64 ground_mpp,
                                                                          Vrst_float64 px_ratio) {
  std::array<Vrst_float32, kLineStyleCount> out{};
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
  return out;
}

void Vrst_tile_renderer::setup_line_vao(GLuint vao, GLuint vbo) {
  glBindVertexArray(vao);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  for (GLuint a = 0; a < 4; ++a) {
    glEnableVertexAttribArray(a);
    glVertexAttribDivisor(a, 1);
  }
  point_line_attribs(0);
}

void Vrst_tile_renderer::point_line_attribs(Vrst_uint32 first) {
  const auto base = static_cast<std::uintptr_t>(first) * sizeof(Vrst_line_instance);
  const GLsizei stride = sizeof(Vrst_line_instance);
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(base));
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(base + 8));
  glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(base + 16));
  glVertexAttribIPointer(3, 1, GL_UNSIGNED_INT, stride, reinterpret_cast<void*>(base + 20));
}

void Vrst_tile_renderer::draw_lines(const Vrst_gpu_tile& g, const std::array<Vrst_float32, 9>& mat, Vrst_tile_range r) {
  if (r.count == 0) return;
  glUniformMatrix3fv(m_u.at("u_view"), 1, GL_FALSE, mat.data());
  glBindVertexArray(g.line_vao);
  glBindBuffer(GL_ARRAY_BUFFER, g.line_vbo);
  point_line_attribs(r.first);
  glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, GLsizei(r.count));
  ++m_draw_calls;
}

void Vrst_tile_renderer::draw_borders(const Vrst_view& v, int fb_w, int fb_h,
                                      const std::vector<std::reference_wrapper<const Vrst_gpu_tile>>& tiles,
                                      Vrst_float64 px_ratio) {
  std::vector<Vrst_line_instance> edges;
  for (const Vrst_gpu_tile& t : tiles) {
    const auto s = Vrst_float32(tile_side(t.id.z));
    const auto ox = Vrst_float32(t.origin.x), oy = Vrst_float32(t.origin.y);
    const Vrst_float32 c[5][2] = {{0, 0}, {s, 0}, {s, s}, {0, s}, {0, 0}};
    for (int i = 0; i < 4; ++i)
      edges.push_back({ox + c[i][0], oy + c[i][1], ox + c[i + 1][0], oy + c[i + 1][1], 0, kStyleTileBorder});
  }
  // World coordinates near the map center: plain float is exact enough for a debug overlay.
  const std::array<Vrst_float32, 9> m = view_matrix(v, fb_w, fb_h);
  glBindBuffer(GL_ARRAY_BUFFER, m_border_vbo);
  glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(edges.size() * sizeof(Vrst_line_instance)), edges.data(), GL_STREAM_DRAW);
  glUniform1f(m_u.at("u_extra_px"), 0);
  glUniform1f(m_u.at("u_dash_px"), Vrst_float32(6 * px_ratio));
  glUniform1i(m_u.at("u_pass"), 2);
  const std::array<Vrst_float32, 3> magenta = {1.0f, 0.2f, 0.9f};
  glUniform3fv(m_u.at("u_pass_color"), 1, magenta.data());
  Vrst_gpu_tile g{};
  g.line_vao = m_border_vao;
  g.line_vbo = m_border_vbo;
  draw_lines(g, m, {0, Vrst_uint32(edges.size())});
}

GLuint Vrst_tile_renderer::point_vao(const std::vector<Vrst_gpu_point>& verts, GLuint& vbo) {
  GLuint vao = 0;
  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);
  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(verts.size() * sizeof(Vrst_gpu_point)), verts.data(), GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vrst_gpu_point), reinterpret_cast<void*>(0));
  glEnableVertexAttribArray(1);
  glVertexAttribIPointer(1, 1, GL_UNSIGNED_INT, sizeof(Vrst_gpu_point), reinterpret_cast<void*>(8));
  return vao;
}

void Vrst_tile_renderer::release(Vrst_gpu_tile& g) {
  const GLuint bufs[3] = {g.fill_vbo, g.line_vbo, g.point_vbo}, vaos[3] = {g.fill_vao, g.line_vao, g.point_vao};
  glDeleteBuffers(3, bufs);
  glDeleteVertexArrays(3, vaos);
  m_bytes -= g.bytes;
}

std::vector<Vrst_uint64> select_tiles(Vrst_tile_renderer& r, Vrst_tile_loader& loader, const Vrst_tile_index& index,
                                      const Vrst_view& v, int fb_w, int fb_h, Vrst_vec2d map_center, Vrst_uint64 frame,
                                      Vrst_size budget, Vrst_tile_frame& stats) {
  for (std::unique_ptr<Vrst_tile_data>& t : loader.take(8)) r.add(*t, frame);
  Vrst_tile_frame f;
  f.level = level_for(1.0 / v.scale);
  std::vector<Vrst_tile_id> missing;
  std::vector<Vrst_uint64> fine, coarse;
  std::unordered_set<Vrst_uint64> drawn;
  for (const Vrst_tile_id& t : visible_tiles(v, fb_w, fb_h, map_center, kTileLevels[f.level].z)) {
    if (!index.contains(t)) continue;  // nothing there
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
  stats = f;
  coarse.insert(coarse.end(), fine.begin(), fine.end());  // fallbacks under the fine tiles
  return coarse;
}

}  // namespace nav
