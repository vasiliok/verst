// navview <out_dir> [--screenshot <file.bmp>]
//
// Draws the road network from the nav files. Web Mercator, relative to the
// map center in double, stored as float.
//
// Controls: drag = pan, wheel = zoom at cursor, Q/E = rotate, 1-9 = toggle
// road classes, O = oneway highlight, V = vertices, R = reset, Esc = quit.
// The same settings are in the Dear ImGui panel.

#include <glad/glad.h>
// glad must come first
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "nav_files.h"
#include "shaders.h"

using namespace nav;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadius = 6378137.0;

struct Vec2d {
  double x, y;
};

Vec2d mercator(Coord c) {
  const double lon = c.lon_e7 * 1e-7 * kPi / 180.0;
  const double lat = c.lat_e7 * 1e-7 * kPi / 180.0;
  return {kEarthRadius * lon, kEarthRadius * std::log(std::tan(kPi / 4 + lat / 2))};
}

Vec2d inverse_mercator(Vec2d m) {  // -> degrees (lat, lon)
  return {(2 * std::atan(std::exp(m.y / kEarthRadius)) - kPi / 2) * 180.0 / kPi, m.x / kEarthRadius * 180.0 / kPi};
}

// Key 1..9 -> classes toggled together.
constexpr uint32_t bit(RoadClass c) { return 1u << static_cast<int>(c); }
constexpr uint32_t kClassGroups[9] = {
    bit(RoadClass::Motorway) | bit(RoadClass::MotorwayLink),
    bit(RoadClass::Trunk) | bit(RoadClass::TrunkLink),
    bit(RoadClass::Primary) | bit(RoadClass::PrimaryLink),
    bit(RoadClass::Secondary) | bit(RoadClass::SecondaryLink),
    bit(RoadClass::Tertiary) | bit(RoadClass::TertiaryLink),
    bit(RoadClass::Unclassified),
    bit(RoadClass::Residential) | bit(RoadClass::LivingStreet),
    bit(RoadClass::Service),
    bit(RoadClass::Ferry),
};
constexpr const char* kGroupNames[9] = {"motorway", "trunk",       "primary", "secondary", "tertiary",
                                        "unclassified", "residential", "service", "ferry"};

// Indexed by RoadClass.
constexpr float kClassColors[kRoadClassCount][3] = {
    {0.91f, 0.36f, 0.42f}, {0.91f, 0.50f, 0.55f},  // motorway
    {0.98f, 0.58f, 0.36f}, {0.98f, 0.68f, 0.50f},  // trunk
    {0.99f, 0.80f, 0.40f}, {0.99f, 0.85f, 0.55f},  // primary
    {0.93f, 0.93f, 0.55f}, {0.93f, 0.93f, 0.65f},  // secondary
    {0.88f, 0.88f, 0.88f}, {0.80f, 0.80f, 0.80f},  // tertiary
    {0.68f, 0.68f, 0.72f},                         // unclassified
    {0.56f, 0.59f, 0.64f},                         // residential
    {0.45f, 0.62f, 0.52f},                         // living_street
    {0.40f, 0.42f, 0.47f},                         // service
    {0.35f, 0.60f, 0.95f},                         // ferry
};

struct GpuPoint {
  float x, y;
  uint32_t attr;
};
static_assert(sizeof(GpuPoint) == 12);

struct View {
  Vec2d center{0, 0};  // world metres relative to map center
  double scale = 1;    // framebuffer pixels per metre
  double angle = 0;    // radians, counter-clockwise
};

struct App {
  NavData data;
  Vec2d map_center{};    // mercator metres, subtracted from every point
  Vec2d half_extent{};   // of the points, metres
  View view;
  int fb_w = 1, fb_h = 1;
  double px_ratio = 1;   // framebuffer px per window coordinate
  uint32_t class_mask = (1u << kRoadClassCount) - 1;
  bool oneway = false;
  bool show_vertices = true;
  float point_px = 2.5f;  // vertex dot size, window points
  bool dragging = false;
  double last_x = 0, last_y = 0;  // cursor, framebuffer px
  double initial_scale = 1;
  uint32_t dead_ends = 0;
};

// World -> NDC as a column-major mat3, computed in double.
void view_matrix(const App& app, float out[9]) {
  const View& v = app.view;
  const double c = std::cos(v.angle) * v.scale, s = std::sin(v.angle) * v.scale;
  const double sx = 2.0 / app.fb_w, sy = 2.0 / app.fb_h;
  // p' = D * S*R * (p - center)
  const double tx = -(c * v.center.x - s * v.center.y), ty = -(s * v.center.x + c * v.center.y);
  const double m[9] = {c * sx, s * sy, 0, -s * sx, c * sy, 0, tx * sx, ty * sy, 1};
  for (int i = 0; i < 9; ++i) out[i] = static_cast<float>(m[i]);
}

// Cursor in framebuffer px (origin top-left) -> pixel offset from screen center, y up.
Vec2d screen_offset(const App& app, double x, double y) { return {x - app.fb_w / 2.0, app.fb_h / 2.0 - y}; }

// Screen offset (px) -> world delta (metres).
Vec2d unproject_delta(const View& v, Vec2d d) {
  const double c = std::cos(-v.angle), s = std::sin(-v.angle);
  return {(c * d.x - s * d.y) / v.scale, (s * d.x + c * d.y) / v.scale};
}

Vec2d cursor_world(const App& app, double x, double y) {
  const Vec2d d = unproject_delta(app.view, screen_offset(app, x, y));
  return {app.view.center.x + d.x, app.view.center.y + d.y};
}

void reset_view(App& app) {
  app.view.center = {0, 0};
  app.view.angle = 0;
  const double sx = app.fb_w / (2 * std::max(app.half_extent.x, 1.0));
  const double sy = app.fb_h / (2 * std::max(app.half_extent.y, 1.0));
  app.view.scale = 0.95 * std::min(sx, sy);
  app.initial_scale = app.view.scale;
}

App* app_of(GLFWwindow* w) { return static_cast<App*>(glfwGetWindowUserPointer(w)); }

void update_size(GLFWwindow* win, App& app) {
  int ww, wh;
  glfwGetWindowSize(win, &ww, &wh);
  glfwGetFramebufferSize(win, &app.fb_w, &app.fb_h);
  app.fb_w = std::max(app.fb_w, 1);
  app.fb_h = std::max(app.fb_h, 1);
  app.px_ratio = ww > 0 ? double(app.fb_w) / ww : 1.0;
}

void on_key(GLFWwindow* win, int key, int, int action, int) {
  if (action == GLFW_RELEASE || ImGui::GetIO().WantCaptureKeyboard) return;
  App& app = *app_of(win);
  if (key >= GLFW_KEY_1 && key <= GLFW_KEY_9 && action == GLFW_PRESS) {
    const int g = key - GLFW_KEY_1;
    app.class_mask ^= kClassGroups[g];
    std::printf("%-12s %s\n", kGroupNames[g], (app.class_mask & kClassGroups[g]) ? "on" : "off");
    return;
  }
  switch (key) {
    case GLFW_KEY_ESCAPE: glfwSetWindowShouldClose(win, GLFW_TRUE); break;
    case GLFW_KEY_Q: app.view.angle += 5 * kPi / 180; break;
    case GLFW_KEY_E: app.view.angle -= 5 * kPi / 180; break;
    case GLFW_KEY_O: if (action == GLFW_PRESS) app.oneway = !app.oneway; break;
    case GLFW_KEY_V: if (action == GLFW_PRESS) app.show_vertices = !app.show_vertices; break;
    case GLFW_KEY_R: reset_view(app); break;
    default: break;
  }
}

void on_mouse_button(GLFWwindow* win, int button, int action, int) {
  if (button != GLFW_MOUSE_BUTTON_LEFT) return;
  // Drags that start on the panel belong to ImGui; releases always end a map drag.
  app_of(win)->dragging = action == GLFW_PRESS && !ImGui::GetIO().WantCaptureMouse;
}

void on_cursor(GLFWwindow* win, double x, double y) {
  App& app = *app_of(win);
  x *= app.px_ratio;
  y *= app.px_ratio;
  if (app.dragging) {
    const Vec2d d = unproject_delta(app.view, {x - app.last_x, -(y - app.last_y)});
    app.view.center.x -= d.x;
    app.view.center.y -= d.y;
  }
  app.last_x = x;
  app.last_y = y;
}

void on_scroll(GLFWwindow* win, double, double yoff) {
  if (ImGui::GetIO().WantCaptureMouse) return;
  App& app = *app_of(win);
  const Vec2d before = cursor_world(app, app.last_x, app.last_y);
  app.view.scale = std::clamp(app.view.scale * std::pow(1.25, yoff), app.initial_scale * 0.1, 2000.0);
  const Vec2d d = unproject_delta(app.view, screen_offset(app, app.last_x, app.last_y));
  app.view.center = {before.x - d.x, before.y - d.y};
}

void on_framebuffer_size(GLFWwindow* win, int, int) {
  App& app = *app_of(win);
  update_size(win, app);
}

GLuint compile(GLenum type, const char* src) {
  GLuint s = glCreateShader(type);
  glShaderSource(s, 1, &src, nullptr);
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

GLuint link(const char* vs, const char* fs) {
  GLuint p = glCreateProgram();
  GLuint v = compile(GL_VERTEX_SHADER, vs), f = compile(GL_FRAGMENT_SHADER, fs);
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

void check_gl(const char* where) {
  for (GLenum e; (e = glGetError()) != GL_NO_ERROR;) throw std::runtime_error(std::string("GL error 0x") +
                                                                             std::to_string(e) + " at " + where);
}

GLuint make_vao(const std::vector<GpuPoint>& verts, GLuint* vbo) {
  GLuint vao;
  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);
  glGenBuffers(1, vbo);
  glBindBuffer(GL_ARRAY_BUFFER, *vbo);
  glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(verts.size() * sizeof(GpuPoint)), verts.data(), GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(GpuPoint), reinterpret_cast<void*>(0));
  glEnableVertexAttribArray(1);
  glVertexAttribIPointer(1, 1, GL_UNSIGNED_INT, sizeof(GpuPoint), reinterpret_cast<void*>(8));
  return vao;
}

Vec2d cursor_latlon(const App& app) {
  const Vec2d w = cursor_world(app, app.last_x, app.last_y);
  return inverse_mercator({w.x + app.map_center.x, w.y + app.map_center.y});
}

void draw_panel(App& app) {
  const Graph& g = app.data.graph;
  ImGui::SetNextWindowPos({10, 10}, ImGuiCond_FirstUseEver);
  ImGui::Begin("navview", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

  ImGui::Text("V %u   S %u   A %u   P %zu", g.num_vertices(), app.data.edges.num_segments(), g.num_arcs(),
              app.data.geom.points.size());
  ImGui::Text("%.0f fps", ImGui::GetIO().Framerate);
  const Vec2d ll = cursor_latlon(app);
  ImGui::Text("cursor  %.6f, %.6f", ll.x, ll.y);
  const double center_lat = inverse_mercator(app.map_center).x * kPi / 180;
  ImGui::Text("zoom    %.2fx   %.2f m/px", app.view.scale / app.initial_scale,
              app.px_ratio * std::cos(center_lat) / app.view.scale);

  ImGui::SeparatorText("Road classes");
  for (int i = 0; i < 9; ++i) {
    const int cls = std::countr_zero(kClassGroups[i]);
    const float* c = kClassColors[cls];
    ImGui::ColorButton("##swatch", ImVec4(c[0], c[1], c[2], 1), ImGuiColorEditFlags_NoTooltip, ImVec2(14, 14));
    ImGui::SameLine();
    bool on = (app.class_mask & kClassGroups[i]) != 0;
    char label[32];
    std::snprintf(label, sizeof label, "%d  %s", i + 1, kGroupNames[i]);
    if (ImGui::Checkbox(label, &on)) app.class_mask ^= kClassGroups[i];
  }

  ImGui::SeparatorText("Display");
  ImGui::Checkbox("Oneway highlight (O)", &app.oneway);
  ImGui::Checkbox("Vertices (V)", &app.show_vertices);
  if (app.show_vertices) {
    ImGui::SliderFloat("Dot size", &app.point_px, 1.0f, 8.0f, "%.1f");
    ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1), "dead ends: %u", app.dead_ends);
  }
  float deg = static_cast<float>(app.view.angle * 180 / kPi);
  deg = std::remainder(deg, 360.0f);
  if (ImGui::SliderFloat("Rotation", &deg, -180.0f, 180.0f, "%.0f deg")) app.view.angle = deg * kPi / 180;
  if (ImGui::Button("Reset view (R)")) reset_view(app);

  ImGui::End();
}

void save_bmp(const std::string& path, int w, int h) {
  std::vector<uint8_t> px(size_t(w) * h * 3);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glReadBuffer(GL_BACK);
  glReadPixels(0, 0, w, h, GL_BGR, GL_UNSIGNED_BYTE, px.data());
  const uint32_t row = (uint32_t(w) * 3 + 3) & ~3u, img = row * uint32_t(h);
  uint8_t hdr[54] = {'B', 'M'};
  auto put32 = [&](int at, uint32_t v) { std::memcpy(hdr + at, &v, 4); };
  put32(2, 54 + img);
  put32(10, 54);
  put32(14, 40);
  put32(18, uint32_t(w));
  put32(22, uint32_t(h));  // bottom-up, like glReadPixels
  hdr[26] = 1;
  hdr[28] = 24;
  put32(34, img);
  std::ofstream f(path, std::ios::binary);
  f.write(reinterpret_cast<char*>(hdr), 54);
  const char pad[3] = {};
  for (int y = 0; y < h; ++y) {
    f.write(reinterpret_cast<char*>(px.data() + size_t(y) * w * 3), w * 3);
    f.write(pad, row - uint32_t(w) * 3);
  }
}

int run(const std::string& dir, const std::string& screenshot) {
  App app;
  app.data.open(dir);
  const Graph& g = app.data.graph;
  const Edges& e = app.data.edges;
  const Geom& geo = app.data.geom;
  const uint32_t V = g.num_vertices(), S = e.num_segments();
  const size_t P = geo.points.size();

  // --- projection: double, relative to map center --------------------------
  Vec2d lo{1e300, 1e300}, hi{-1e300, -1e300};
  std::vector<Vec2d> merc(P);
  for (size_t i = 0; i < P; ++i) {
    merc[i] = mercator(geo.points[i]);
    lo = {std::min(lo.x, merc[i].x), std::min(lo.y, merc[i].y)};
    hi = {std::max(hi.x, merc[i].x), std::max(hi.y, merc[i].y)};
  }
  if (P == 0) lo = hi = {0, 0};
  app.map_center = {(lo.x + hi.x) / 2, (lo.y + hi.y) / 2};
  app.half_extent = {(hi.x - lo.x) / 2, (hi.y - lo.y) / 2};

  std::vector<GpuPoint> line_verts(P);
  for (uint32_t s = 0; s < S; ++s) {
    const uint32_t attr = e.attr[s].road_class | uint32_t(e.attr[s].flags) << 16;
    for (uint32_t i = geo.seg_first_point[s]; i < geo.seg_first_point[s + 1]; ++i)
      line_verts[i] = {float(merc[i].x - app.map_center.x), float(merc[i].y - app.map_center.y), attr};
  }
  std::vector<Vec2d>().swap(merc);

  // Line pairs only inside a segment; minor classes first so major roads draw on top.
  std::vector<uint32_t> indices;
  indices.reserve((P - S) * 2);
  for (int cls = kRoadClassCount - 1; cls >= 0; --cls)
    for (uint32_t s = 0; s < S; ++s) {
      if (e.attr[s].road_class != cls) continue;
      for (uint32_t i = geo.seg_first_point[s]; i + 1 < geo.seg_first_point[s + 1]; ++i) {
        indices.push_back(i);
        indices.push_back(i + 1);
      }
    }

  std::vector<uint32_t> degree(V, 0);
  for (const SegVertices& sv : e.vertices) ++degree[sv.a], ++degree[sv.b];
  std::vector<GpuPoint> vertex_verts;
  vertex_verts.reserve(V);
  for (int pass = 0; pass < 2; ++pass)  // dead ends last, on top
    for (uint32_t v = 0; v < V; ++v) {
      const uint32_t dead = degree[v] == 1;
      if (dead != uint32_t(pass)) continue;
      app.dead_ends += dead;
      const Vec2d m = mercator(g.vertex_coord[v]);
      vertex_verts.push_back({float(m.x - app.map_center.x), float(m.y - app.map_center.y), dead});
    }

  // --- window --------------------------------------------------------------
  if (!glfwInit()) throw std::runtime_error("glfwInit failed");
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
  glfwWindowHint(GLFW_SAMPLES, 4);
  GLFWwindow* win = glfwCreateWindow(1280, 900, "navview", nullptr, nullptr);
  if (!win) {
    glfwTerminate();
    throw std::runtime_error("cannot create OpenGL 3.3 core window");
  }
  glfwMakeContextCurrent(win);
  if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)))
    throw std::runtime_error("gladLoadGL failed");
  glfwSwapInterval(1);
  glfwSetWindowUserPointer(win, &app);
  glfwSetKeyCallback(win, on_key);
  glfwSetMouseButtonCallback(win, on_mouse_button);
  glfwSetCursorPosCallback(win, on_cursor);
  glfwSetScrollCallback(win, on_scroll);
  glfwSetFramebufferSizeCallback(win, on_framebuffer_size);
  update_size(win, app);
  reset_view(app);
  std::printf("GL %s | %s\n", glGetString(GL_VERSION), glGetString(GL_RENDERER));

  // ImGui after our callbacks: its GLFW backend chains to them.
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::GetIO().IniFilename = nullptr;
  ImGui::StyleColorsDark();
  ImGui_ImplGlfw_InitForOpenGL(win, true);
  ImGui_ImplOpenGL3_Init("#version 330 core");

  // --- GPU buffers ---------------------------------------------------------
  GLuint line_vbo, point_vbo, ibo;
  const GLuint line_vao = make_vao(line_verts, &line_vbo);
  glGenBuffers(1, &ibo);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(indices.size() * 4), indices.data(), GL_STATIC_DRAW);
  const GLuint point_vao = make_vao(vertex_verts, &point_vbo);
  glBindVertexArray(0);

  const GLuint line_prog = link(shaders::kLineVs, shaders::kLineFs);
  const GLuint point_prog = link(shaders::kPointVs, shaders::kPointFs);
  const GLint u_line_view = glGetUniformLocation(line_prog, "u_view");
  const GLint u_colors = glGetUniformLocation(line_prog, "u_colors");
  const GLint u_mask = glGetUniformLocation(line_prog, "u_class_mask");
  const GLint u_oneway = glGetUniformLocation(line_prog, "u_oneway");
  const GLint u_point_view = glGetUniformLocation(point_prog, "u_view");
  const GLint u_point_px = glGetUniformLocation(point_prog, "u_point_px");
  glUseProgram(line_prog);
  glUniform3fv(u_colors, kRoadClassCount, &kClassColors[0][0]);
  glEnable(GL_PROGRAM_POINT_SIZE);
  glEnable(GL_MULTISAMPLE);
  check_gl("setup");

  // --- loop ----------------------------------------------------------------
  double fps_t0 = glfwGetTime(), fps = 0;
  int frames = 0, shot_frame = 0;
  while (!glfwWindowShouldClose(win)) {
    glViewport(0, 0, app.fb_w, app.fb_h);
    glClearColor(0.105f, 0.115f, 0.135f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    float m[9];
    view_matrix(app, m);

    glUseProgram(line_prog);
    glUniformMatrix3fv(u_line_view, 1, GL_FALSE, m);
    glUniform1ui(u_mask, app.class_mask);
    glUniform1i(u_oneway, app.oneway);
    glBindVertexArray(line_vao);
    glDrawElements(GL_LINES, GLsizei(indices.size()), GL_UNSIGNED_INT, nullptr);

    if (app.show_vertices) {
      glUseProgram(point_prog);
      glUniformMatrix3fv(u_point_view, 1, GL_FALSE, m);
      glUniform1f(u_point_px, float(app.point_px * app.px_ratio));
      glBindVertexArray(point_vao);
      glDrawArrays(GL_POINTS, 0, GLsizei(vertex_verts.size()));
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    draw_panel(app);
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    if (!screenshot.empty() && ++shot_frame == 3) {  // ImGui sizes auto-resize windows on frame 1
      check_gl("screenshot");
      save_bmp(screenshot, app.fb_w, app.fb_h);
      std::printf("saved %s (%dx%d)\n", screenshot.c_str(), app.fb_w, app.fb_h);
      break;
    }
    glfwSwapBuffers(win);
    glfwPollEvents();

    ++frames;
    const double now = glfwGetTime();
    if (now - fps_t0 >= 0.25) {
      fps = frames / (now - fps_t0);
      frames = 0;
      fps_t0 = now;
      const Vec2d ll = cursor_latlon(app);
      char title[256];
      std::snprintf(title, sizeof title, "navview  V %u  S %u  P %zu  |  %.0f fps  |  %.6f, %.6f  |  zoom %.2fx%s",
                    V, S, P, fps, ll.x, ll.y, app.view.scale / app.initial_scale, app.oneway ? "  | oneway" : "");
      glfwSetWindowTitle(win, title);
    }
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glDeleteBuffers(1, &line_vbo);
  glDeleteBuffers(1, &point_vbo);
  glDeleteBuffers(1, &ibo);
  glDeleteVertexArrays(1, &line_vao);
  glDeleteVertexArrays(1, &point_vao);
  glDeleteProgram(line_prog);
  glDeleteProgram(point_prog);
  glfwDestroyWindow(win);
  glfwTerminate();
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2 && !(argc == 4 && std::string(argv[2]) == "--screenshot")) {
    std::fprintf(stderr, "usage: %s <out_dir> [--screenshot <file.bmp>]\n", argv[0]);
    return 2;
  }
  try {
    return run(argv[1], argc == 4 ? argv[3] : "");
  } catch (const std::exception& e) {
    std::fprintf(stderr, "navview: %s\n", e.what());
    return 1;
  }
}
