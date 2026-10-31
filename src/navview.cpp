/// @file
/// `navview <out_dir> [--center <lat> <lon>] [--mpp <metres>] [--gpx <file.gpx> [--speed <x>] [--track-time <s>]] [--screenshot <file.bmp>]`
///
/// Draws the road network from the nav files. Web Mercator, relative to the
/// map center in double, stored as float.
///
/// With --gpx it replays a recorded GPS track: a marker moves along the track at the
/// recorded times, multiplied by --speed, and the camera follows it.
///
/// Controls: drag = pan, wheel = zoom at cursor, Q/E = rotate, 1-9 = toggle
/// road classes, O = oneway highlight, V = vertices, R = start view,
/// F = fit map, L = place names, Space = play/pause track, C = follow marker, Esc = quit.
/// The same settings are in the Dear ImGui panel.

#include "gl_api.h"  // glad must come before GLFW
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "glyph_atlas.h"
#include "gpx_track.h"
#include "map_labels.h"
#include "map_renderer.h"
#include "map_scene.h"
#include "nav_files.h"

/// Everything of the navview tool: projection, camera, input, GPU buffers and the
/// render loop. main() only parses arguments and calls run().
namespace nav::viewer {

#ifndef VRST_FONT_DIR
#define VRST_FONT_DIR "fonts"  ///< Label font directory; CMake sets it to `<build>/fonts`.
#endif

/// Command-line options of navview, filled by main().
struct Vrst_options {
  /// Directory with the four nav files.
  std::string dir;
  /// BMP path for --screenshot; empty for the normal interactive window.
  std::string screenshot;
  /// Start view center (--center), also used by R. Lets a session open directly on
  /// the road being examined instead of panning there by hand.
  Vrst_coord center = kDefaultPoint;
  /// Start zoom in ground metres per window point (--mpp); smaller is closer.
  Vrst_float64 metres_per_point = kDefaultMetresPerPoint;
  /// Whether --center was given; without it a --gpx track starts at its first point.
  bool has_center = false;
  /// GPX file to replay (--gpx); empty = no track.
  std::string gpx;
  /// Playback speed factor (--speed); 2 = twice the recorded speed.
  Vrst_float64 speed = 1;
  /// Playback start, seconds since the first track point (--track-time).
  Vrst_float64 track_time = 0;
};

/// All viewer state: loaded data, camera, screen size, toggles and input state.
/// One object reached from the GLFW callbacks through the window user pointer, so
/// the viewer needs no globals and callbacks, panel and render loop share one state.
struct Vrst_app {
  /// Mapped nav files; kept open for name lookups and the cursor position.
  Vrst_nav_data data;
  /// GPU-ready geometry, map center and extent; shared with the iOS app.
  Vrst_map_mesh mesh;
  /// Current pan / zoom / rotation.
  Vrst_view view;
  /// Framebuffer size in pixels. Differs from the window size on HiDPI; the viewport
  /// and the NDC scale need the framebuffer one.
  int fb_w = 1;
  int fb_h = 1;  ///< Framebuffer height in pixels; see fb_w.
  /// Framebuffer pixels per window coordinate (2 on Retina). GLFW reports the cursor
  /// in window coordinates; without this zoom-at-cursor misses by a factor of 2.
  Vrst_float64 px_ratio = 1;
  /// Bit per RoadClass; a 0 bit discards that class in the fragment shader. One
  /// uniform instead of rebuilding the index buffer on every toggle.
  Vrst_uint32 class_mask = kAllClasses;
  /// Highlight oneway segments (O).
  bool oneway = false;
  /// Draw the vertex dots (V).
  bool show_vertices = true;
  /// Draw place names (L).
  bool show_labels = true;
  /// City, town, village and hamlet labels; layout() runs every frame.
  Vrst_map_labels labels;
  /// SDF glyphs of every character the labels use.
  Vrst_glyph_atlas atlas;
  /// Vertex dot size in window points; scaled by px_ratio so it looks the same on
  /// every screen.
  Vrst_float32 point_px = 2.5f;
  /// Left button held on the map, not on the ImGui panel; pans on cursor move.
  bool dragging = false;
  /// Last cursor position, framebuffer px. Pan needs the delta, and scroll events
  /// carry no position, so zoom-at-cursor needs the last known one.
  Vrst_float64 last_x = 0;
  Vrst_float64 last_y = 0;  ///< Last cursor y, framebuffer px; see last_x.
  /// Scale that fits the whole map: zoom 1x in the UI and the zoom-out limit.
  Vrst_float64 initial_scale = 1;
  /// Point the start view and R center on: kDefaultPoint or --center.
  Vrst_coord start_center = kDefaultPoint;
  /// Zoom of the start view and R, ground metres per window point.
  Vrst_float64 start_metres_per_point = kDefaultMetresPerPoint;
  /// Recorded GPS track (--gpx); empty when none was given.
  Vrst_gpx_track track;
  /// Playback position, seconds since the first track point.
  Vrst_float64 track_time = 0;
  /// Playback speed factor; 2 = twice the recorded speed.
  Vrst_float64 track_speed = 1;
  /// Playback running (Space). Stops by itself at the end of the track.
  bool playing = true;
  /// Keep the marker at the screen center (C). A map drag turns it off, so the user
  /// can look around without fighting the camera.
  bool follow = true;
};

/// Centers the map, zooms to fit it and resets rotation (F).
/// @param app Viewer state; its view is replaced.
void fit_view(Vrst_app& app) {
  app.view = {{0, 0}, app.mesh.fit_scale(app.fb_w, app.fb_h), 0};
}

/// Moves to the start view (R): start_center at start_metres_per_point, or fits the
/// map when the point is outside the loaded data.
/// @param app Viewer state; its view is replaced.
void reset_view(Vrst_app& app) {
  app.view = app.mesh.start_view(app.data.graph().bbox(), app.start_center, app.start_metres_per_point,
                                 app.px_ratio, app.fb_w, app.fb_h);
}

/// Viewer state of a window, for the GLFW callbacks.
/// @param w Window.
/// @return State stored as the window user pointer.
Vrst_app* app_of(GLFWwindow* w) { return static_cast<Vrst_app*>(glfwGetWindowUserPointer(w)); }

/// Reads framebuffer size and pixel ratio after a resize or a move to another screen.
/// @param win Window.
/// @param[out] app Receives fb_w, fb_h and px_ratio.
void update_size(GLFWwindow* win, Vrst_app& app) {
  int ww, wh;
  glfwGetWindowSize(win, &ww, &wh);
  glfwGetFramebufferSize(win, &app.fb_w, &app.fb_h);
  app.fb_w = std::max(app.fb_w, 1);
  app.fb_h = std::max(app.fb_h, 1);
  app.px_ratio = ww > 0 ? Vrst_float64(app.fb_w) / ww : 1.0;
}

/// GLFW key callback: 1-9 toggle class groups, Q/E rotate, O/V toggle highlights,
/// R/F set the view, Esc quits. Ignored while ImGui has keyboard focus.
/// @param win Window.
/// @param key GLFW key code.
/// @param scancode Platform scancode; unused.
/// @param action GLFW_PRESS or GLFW_REPEAT (holding Q/E keeps rotating); releases are ignored.
/// @param mods Modifier bits; unused.
void on_key(GLFWwindow* win, int key, [[maybe_unused]] int scancode, int action, [[maybe_unused]] int mods) {
  if (action == GLFW_RELEASE || ImGui::GetIO().WantCaptureKeyboard) return;
  Vrst_app& app = *app_of(win);
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
    case GLFW_KEY_L: if (action == GLFW_PRESS) app.show_labels = !app.show_labels; break;
    case GLFW_KEY_R: reset_view(app); break;
    case GLFW_KEY_F: fit_view(app); break;
    case GLFW_KEY_SPACE:
      if (action == GLFW_PRESS && !app.track.empty()) {
        if (!app.playing && app.track_time >= app.track.duration()) app.track_time = 0;  // replay from the start
        app.playing = !app.playing;
      }
      break;
    case GLFW_KEY_C: if (action == GLFW_PRESS) app.follow = !app.follow; break;
    default: break;
  }
}

/// GLFW mouse button callback: the left button starts and ends a map drag. Drags
/// that start on the ImGui panel belong to ImGui.
/// @param win Window.
/// @param button GLFW mouse button; only the left one is used.
/// @param action GLFW_PRESS or GLFW_RELEASE.
/// @param mods Modifier bits; unused.
void on_mouse_button(GLFWwindow* win, int button, int action, [[maybe_unused]] int mods) {
  if (button != GLFW_MOUSE_BUTTON_LEFT) return;
  // Drags that start on the panel belong to ImGui; releases always end a map drag.
  app_of(win)->dragging = action == GLFW_PRESS && !ImGui::GetIO().WantCaptureMouse;
}

/// GLFW cursor callback: pans while dragging and remembers the cursor for
/// zoom-at-cursor.
/// @param win Window.
/// @param x Cursor x in window coordinates; converted to framebuffer px.
/// @param y Cursor y in window coordinates, from the top.
void on_cursor(GLFWwindow* win, Vrst_float64 x, Vrst_float64 y) {
  Vrst_app& app = *app_of(win);
  x *= app.px_ratio;
  y *= app.px_ratio;
  if (app.dragging) {
    pan_by(app.view, x - app.last_x, y - app.last_y);
    app.follow = false;
  }
  app.last_x = x;
  app.last_y = y;
}

/// GLFW scroll callback: zooms by 1.25 per wheel step and keeps the point under the
/// cursor fixed. Scale is limited to 0.1x the fit scale and 2000 px per metre.
/// @param win Window.
/// @param xoff Horizontal scroll; unused.
/// @param yoff Wheel steps; positive zooms in.
void on_scroll(GLFWwindow* win, [[maybe_unused]] Vrst_float64 xoff, Vrst_float64 yoff) {
  if (ImGui::GetIO().WantCaptureMouse) return;
  Vrst_app& app = *app_of(win);
  zoom_at(app.view, app.fb_w, app.fb_h, app.last_x, app.last_y, std::pow(1.25, yoff), app.initial_scale * 0.1, 2000.0);
}

/// GLFW framebuffer size callback.
/// @param win Window.
/// @param width New width; unused, update_size() reads both sizes itself.
/// @param height New height; unused.
void on_framebuffer_size(GLFWwindow* win, [[maybe_unused]] int width, [[maybe_unused]] int height) {
  Vrst_app& app = *app_of(win);
  update_size(win, app);
}

/// Geographic position under the cursor.
/// @param app Viewer state.
/// @return Degrees: x = latitude, y = longitude.
Vrst_vec2d cursor_latlon(const Vrst_app& app) {
  return app.mesh.to_latlon(screen_to_world(app.view, app.fb_w, app.fb_h, app.last_x, app.last_y));
}

/// Builds the ImGui panel: counts, fps, cursor, zoom, class toggles and display
/// options. Changes go straight into the viewer state.
/// @param app Viewer state, read and changed by the widgets.
void draw_panel(Vrst_app& app) {
  const Vrst_graph& g = app.data.graph();
  ImGui::SetNextWindowPos({10, 10}, ImGuiCond_FirstUseEver);
  ImGui::Begin("navview", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

  ImGui::Text("V %u   S %u   A %u   P %zu", g.num_vertices(), app.data.edges().num_segments(), g.num_arcs(),
              app.data.geom().points().size());
  ImGui::Text("%.0f fps", ImGui::GetIO().Framerate);
  const Vrst_vec2d ll = cursor_latlon(app);
  ImGui::Text("cursor  %.6f, %.6f", ll.x, ll.y);
  const Vrst_float64 center_lat = inverse_mercator(app.mesh.center()).x * kPi / 180;
  ImGui::Text("zoom    %.2fx   %.2f m/px", app.view.scale / app.initial_scale,
              app.px_ratio * std::cos(center_lat) / app.view.scale);

  ImGui::SeparatorText("Road classes");
  for (int i = 0; i < kClassGroupCount; ++i) {
    const int cls = std::countr_zero(kClassGroups[i]);
    const Vrst_float32* c = kClassColors[cls];
    ImGui::ColorButton("##swatch", ImVec4(c[0], c[1], c[2], 1), ImGuiColorEditFlags_NoTooltip, ImVec2(14, 14));
    ImGui::SameLine();
    bool on = (app.class_mask & kClassGroups[i]) != 0;
    char label[32];
    std::snprintf(label, sizeof label, "%d  %s", i + 1, kGroupNames[i]);
    if (ImGui::Checkbox(label, &on)) app.class_mask ^= kClassGroups[i];
  }

  ImGui::SeparatorText("Display");
  ImGui::Checkbox("Oneway highlight (O)", &app.oneway);
  ImGui::Checkbox("Place names (L)", &app.show_labels);
  if (app.show_labels) {
    ImGui::SameLine();
    ImGui::TextDisabled("%zu / %zu", app.labels.placed(), app.labels.size());
  }
  ImGui::Checkbox("Vertices (V)", &app.show_vertices);
  if (app.show_vertices) {
    ImGui::SliderFloat("Dot size", &app.point_px, 1.0f, 8.0f, "%.1f");
    ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1), "dead ends: %u", app.mesh.dead_ends());
  }
  Vrst_float32 deg = static_cast<Vrst_float32>(app.view.angle * 180 / kPi);
  deg = std::remainder(deg, 360.0f);
  if (ImGui::SliderFloat("Rotation", &deg, -180.0f, 180.0f, "%.0f deg")) app.view.angle = deg * kPi / 180;
  if (ImGui::Button("Start view (R)")) reset_view(app);
  ImGui::SameLine();
  if (ImGui::Button("Fit map (F)")) fit_view(app);

  if (!app.track.empty()) {
    ImGui::SeparatorText("GPS track");
    const Vrst_track_sample ts = app.track.sample(app.track_time);
    const int t = int(app.track_time), d = int(app.track.duration());
    ImGui::Text("time   %02d:%02d / %02d:%02d   point %zu / %zu", t / 60, t % 60, d / 60, d % 60, ts.index + 1,
                app.track.points().size());
    ImGui::Text("speed  %.0f km/h", ts.speed_kmh);
    Vrst_float32 tf = Vrst_float32(app.track_time);
    if (ImGui::SliderFloat("Time", &tf, 0.0f, Vrst_float32(app.track.duration()), "%.0f s")) app.track_time = tf;
    Vrst_float32 sp = Vrst_float32(app.track_speed);
    if (ImGui::SliderFloat("Speed", &sp, 0.25f, 32.0f, "x%.2g", ImGuiSliderFlags_Logarithmic)) app.track_speed = sp;
    if (ImGui::Button(app.playing ? "Pause (Space)" : "Play (Space)")) {
      if (!app.playing && app.track_time >= app.track.duration()) app.track_time = 0;
      app.playing = !app.playing;
    }
    ImGui::SameLine();
    if (ImGui::Button("Restart")) app.track_time = 0;
    ImGui::SameLine();
    ImGui::Checkbox("Follow (C)", &app.follow);
  }

  ImGui::End();
}

/// Saves the back buffer as a 24-bit BMP (--screenshot).
/// @param path Output file.
/// @param w Framebuffer width in px.
/// @param h Framebuffer height in px.
void save_bmp(const std::string& path, int w, int h) {
  std::vector<Vrst_uint8> px(Vrst_size(w) * h * 3);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glReadBuffer(GL_BACK);
  glReadPixels(0, 0, w, h, GL_BGR, GL_UNSIGNED_BYTE, px.data());
  const Vrst_uint32 row = (Vrst_uint32(w) * 3 + 3) & ~3u, img = row * Vrst_uint32(h);
  Vrst_uint8 hdr[54] = {'B', 'M'};
  auto put32 = [&](int at, Vrst_uint32 v) { std::memcpy(hdr + at, &v, 4); };
  put32(2, 54 + img);
  put32(10, 54);
  put32(14, 40);
  put32(18, Vrst_uint32(w));
  put32(22, Vrst_uint32(h));  // bottom-up, like glReadPixels
  hdr[26] = 1;
  hdr[28] = 24;
  put32(34, img);
  std::ofstream f(path, std::ios::binary);
  f.write(reinterpret_cast<char*>(hdr), 54);
  const char pad[3] = {};
  for (int y = 0; y < h; ++y) {
    f.write(reinterpret_cast<char*>(px.data() + Vrst_size(y) * w * 3), w * 3);
    f.write(pad, row - Vrst_uint32(w) * 3);
  }
}

/// Loads the data, builds the GPU buffers and runs the render loop until the window
/// closes.
/// @param opt Command-line options. With a screenshot path it renders 3 frames,
///        saves the last and returns.
/// @return 0.
/// @throws std::runtime_error on a load, window or OpenGL failure.
int run(const Vrst_options& opt) {
  const std::string& screenshot = opt.screenshot;
  Vrst_app app;
  app.start_center = opt.center;
  app.start_metres_per_point = opt.metres_per_point;
  app.data.open(opt.dir);
  app.mesh.build(app.data);
  app.labels.build(app.data, app.mesh);
  {
    std::vector<std::string> fonts;
    for (const char* f : kLabelFontFiles) fonts.push_back(std::string(VRST_FONT_DIR) + "/" + f);
    app.atlas.build(fonts, app.labels.codepoints());
    app.labels.measure(app.atlas);
    std::printf("labels: %zu places, %zu glyphs, atlas %dx%d\n", app.labels.size(), app.atlas.size(),
                app.atlas.width(), app.atlas.height());
  }
  if (!opt.gpx.empty()) {
    app.track.load(opt.gpx);
    app.track_speed = opt.speed;
    app.track_time = opt.track_time;
    if (!opt.has_center) app.start_center = app.track.sample(app.track_time).pos;
    std::printf("track %s: %zu points, %.1f km, %.0f s, playing at x%g\n", opt.gpx.c_str(),
                app.track.points().size(), app.track.length_m() / 1000, app.track.duration(), app.track_speed);
  }
  const Vrst_uint32 V = app.data.graph().num_vertices(), S = app.data.edges().num_segments();
  const Vrst_size P = app.data.geom().points().size();

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
  app.initial_scale = app.mesh.fit_scale(app.fb_w, app.fb_h);  // zoom 1x = whole map
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
  Vrst_map_renderer renderer;
  renderer.init(app.mesh);
  renderer.set_label_atlas(app.atlas);
  if (!app.track.empty()) {
    std::vector<Vrst_vec2d> world;
    world.reserve(app.track.points().size());
    for (const Vrst_coord& c : app.track.points()) world.push_back(app.mesh.to_world(c));
    renderer.set_track(world);
  }

  // --- loop ----------------------------------------------------------------
  Vrst_float64 fps_t0 = glfwGetTime(), fps = 0;
  int frames = 0, shot_frame = 0;
  Vrst_float64 last_frame = glfwGetTime();
  while (!glfwWindowShouldClose(win)) {
    const Vrst_float64 frame_t = glfwGetTime();
    if (!app.track.empty() && app.playing && screenshot.empty()) {
      app.track_time += (frame_t - last_frame) * app.track_speed;
      if (app.track_time >= app.track.duration()) {
        app.track_time = app.track.duration();
        app.playing = false;
      }
    }
    last_frame = frame_t;
    Vrst_track_sample ts{};
    if (!app.track.empty()) {
      ts = app.track.sample(app.track_time);
      if (app.follow) app.view.center = app.mesh.to_world(ts.pos);
    }

    renderer.draw(app.view, app.fb_w, app.fb_h,
                  {app.class_mask, app.oneway, app.show_vertices, Vrst_float32(app.point_px * app.px_ratio)});
    if (!app.track.empty())
      renderer.draw_track(app.view, app.fb_w, app.fb_h, ts.index + 1, app.mesh.to_world(ts.pos),
                          Vrst_float32(14 * app.px_ratio));
    if (app.show_labels)
      renderer.draw_labels(app.labels.layout(app.view, app.fb_w, app.fb_h, app.px_ratio, app.atlas), app.fb_w,
                           app.fb_h, Vrst_float32(2.0 * app.px_ratio));

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    draw_panel(app);
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    if (!screenshot.empty() && ++shot_frame == 3) {  // ImGui sizes auto-resize windows on frame 1
      Vrst_map_renderer::check_error("screenshot");
      save_bmp(screenshot, app.fb_w, app.fb_h);
      std::printf("saved %s (%dx%d)\n", screenshot.c_str(), app.fb_w, app.fb_h);
      break;
    }
    glfwSwapBuffers(win);
    glfwPollEvents();

    ++frames;
    const Vrst_float64 now = glfwGetTime();
    if (now - fps_t0 >= 0.25) {
      fps = frames / (now - fps_t0);
      frames = 0;
      fps_t0 = now;
      const Vrst_vec2d ll = cursor_latlon(app);
      char title[256];
      std::snprintf(title, sizeof title, "navview  V %u  S %u  P %zu  |  %.0f fps  |  %.6f, %.6f  |  zoom %.2fx%s",
                    V, S, P, fps, ll.x, ll.y, app.view.scale / app.initial_scale, app.oneway ? "  | oneway" : "");
      glfwSetWindowTitle(win, title);
    }
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  renderer.destroy();
  glfwDestroyWindow(win);
  glfwTerminate();
  return 0;
}

}  // namespace nav::viewer

/// navview entry point.
/// @param argc Argument count.
/// @param argv `<out_dir> [--center <lat> <lon>] [--mpp <metres>] [--gpx <file.gpx> [--speed <x>] [--track-time <s>]] [--screenshot <file.bmp>]`.
/// @return 0 on success, 1 on error, 2 on wrong usage.
int main(int argc, char** argv) {
  nav::viewer::Vrst_options opt;
  bool ok = argc >= 2;
  if (ok) opt.dir = argv[1];
  auto number = [&](const char* s, nav::Vrst_float64* out) {
    char* end = nullptr;
    *out = std::strtod(s, &end);
    return end != s && *end == '\0';
  };
  for (int i = 2; ok && i < argc; ++i) {
    const std::string a = argv[i];
    nav::Vrst_float64 lat = 0, lon = 0;
    if (a == "--screenshot" && i + 1 < argc) {
      opt.screenshot = argv[++i];
    } else if (a == "--center" && i + 2 < argc && number(argv[i + 1], &lat) && number(argv[i + 2], &lon) &&
               std::abs(lat) <= 90 && std::abs(lon) <= 180) {
      opt.center = {static_cast<nav::Vrst_int32>(std::lround(lat * 1e7)), static_cast<nav::Vrst_int32>(std::lround(lon * 1e7))};
      i += 2;
      opt.has_center = true;
    } else if (a == "--mpp" && i + 1 < argc && number(argv[i + 1], &opt.metres_per_point) && opt.metres_per_point > 0) {
      ++i;
    } else if (a == "--gpx" && i + 1 < argc) {
      opt.gpx = argv[++i];
    } else if (a == "--speed" && i + 1 < argc && number(argv[i + 1], &opt.speed) && opt.speed > 0) {
      ++i;
    } else if (a == "--track-time" && i + 1 < argc && number(argv[i + 1], &opt.track_time) && opt.track_time >= 0) {
      ++i;
    } else {
      ok = false;
    }
  }
  if (!ok) {
    std::fprintf(stderr, "usage: %s <out_dir> [--center <lat> <lon>] [--mpp <metres>] [--gpx <file.gpx> [--speed <x>] [--track-time <s>]] [--screenshot <file.bmp>]\n", argv[0]);
    return 2;
  }
  try {
    return nav::viewer::run(opt);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "navview: %s\n", e.what());
    return 1;
  }
}
