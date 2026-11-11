/// @file
/// Test scenarios: scripted map motion (scroll, fling, zoom, rotate, jump) that the app
/// plays by itself, so a performance run is the same every time and needs no hands.
///
/// A scenario is plain text, one step per line, `#` starts a comment:
///
///     wait <s>                      pause
///     wait_tiles <max s>            pause until no tile is missing (or max s)
///     pan <dx> <dy> <s>             scroll by dx, dy screen points, smooth start and stop
///     fling <dx> <dy> <s>           scroll fast at first, then slower (released finger)
///     zoom <factor> <s>             zoom around the screen center (pinch); 0 s = at once
///     zoom_in / zoom_out            2x / 0.5x at once (double tap / two-finger tap)
///     rotate <degrees> <s>          rotate around the screen center, counter-clockwise
///     goto <lat> <lon> <m/pt>       jump to a place and zoom (the front end applies it)
///     view <s> <lat> <lon> <m/pt> <deg>  move straight to this camera in s seconds: screen
///                                   center, ground metres per point, rotation (counter-
///                                   clockwise); 0 s = at once. Vrst_activity_recorder
///                                   writes these.
///
/// Motion follows the clock, not the frame count: a slow phone shows fewer frames of the
/// same motion, which is exactly what a before/after comparison needs.
#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "map_scene.h"

namespace nav {

/// What a scenario step does.
enum class Vrst_step_kind { Wait, WaitTiles, Pan, Fling, Zoom, Rotate, Goto, View };

/// One parsed scenario line.
struct Vrst_scenario_step {
  Vrst_step_kind kind = Vrst_step_kind::Wait;  ///< What it does.
  Vrst_float64 seconds = 0;                    ///< Duration; for WaitTiles the longest wait.
  Vrst_vec2d pan{};                            ///< Pan, Fling: total move in screen points.
  Vrst_float64 factor = 1;                     ///< Zoom: total scale factor.
  Vrst_float64 degrees = 0;                    ///< Rotate: total angle; View: rotation; counter-clockwise.
  Vrst_latlon at{};                             ///< Goto, View: place (View: screen center).
  Vrst_float64 mpp = 0;                        ///< Goto, View: ground metres per screen point.
  std::string text;                            ///< Source line, for the overlay and the log.
};

/// Screen and limits a scenario needs to move the camera in one frame.
struct Vrst_scenario_frame {
  int fb_w = 1;                     ///< Framebuffer width in px.
  int fb_h = 1;                     ///< Framebuffer height in px.
  Vrst_float64 px_per_point = 1;    ///< Framebuffer px per screen point.
  Vrst_float64 min_scale = 0;       ///< Zoom-out limit (Vrst_view::scale).
  Vrst_float64 max_scale = 1e9;     ///< Zoom-in limit (Vrst_view::scale).
  bool tiles_ready = true;          ///< No visible tile is missing.
  Vrst_vec2d world_origin{};        ///< Absolute Mercator point of world (0, 0) (Vrst_map_frame::center()).
};

/// Result of one Vrst_scenario::advance().
struct Vrst_scenario_tick {
  bool moved = false;                ///< The camera changed: redraw.
  bool step_changed = false;         ///< A new step began in this call.
  bool finished = false;             ///< The last step ended in this call.
  std::optional<Vrst_scenario_step> jump;  ///< A Goto step to apply now.
};

/// The built-in scenario: about 40 s of scrolling, flinging, zooming and rotating.
inline constexpr std::string_view kStandardScenario = R"(# verst standard test
wait_tiles 20
wait 1
pan 0 -300 2
pan 0 300 0.5
fling -600 0 1.2
wait 0.5
fling 600 200 1.2
zoom 4 2
wait_tiles 10
zoom 0.0625 3
wait_tiles 10
zoom 4 1.5
rotate 45 1.5
rotate -45 1.5
zoom_in
wait 0.5
zoom_out
wait 0.5
pan 200 0 0.4
pan -200 0 0.4
pan 200 0 0.4
pan -200 0 0.4
wait_tiles 10
)";

/// A playable scenario; see the file comment.
class Vrst_scenario {
 public:
  /// Parses a scenario text.
  /// @param name Name for the log and the overlay.
  /// @param text Scenario lines.
  /// @return The scenario, not started.
  /// @throws std::runtime_error naming the line number of the first bad line.
  static Vrst_scenario parse(std::string name, std::string_view text);

  /// Starts at the first step.
  /// @param now Clock in seconds (any origin, the same as in advance()).
  void start(Vrst_float64 now);
  /// Stops playing (a touch of the user, or the Stop menu item).
  void stop() { m_running = false; }

  /// Plays the steps up to `now` and moves the camera by the motion since the last call.
  /// @param now Clock in seconds.
  /// @param v Camera, changed.
  /// @param f Screen size, zoom limits and tile state.
  /// @return What happened in this call.
  Vrst_scenario_tick advance(Vrst_float64 now, Vrst_view& v, const Vrst_scenario_frame& f);

  /// @return true between start() and the end of the last step (or stop()).
  bool running() const { return m_running; }
  /// @return Name given to parse().
  const std::string& name() const { return m_name; }
  /// @return All steps.
  const std::vector<Vrst_scenario_step>& steps() const { return m_steps; }
  /// @return Index of the step playing now; steps().size() when done.
  Vrst_size step_index() const { return m_index; }
  /// @return Total length in seconds, counting WaitTiles at their longest.
  Vrst_float64 max_seconds() const;

 private:
  /// Moves the camera by the part of a step between two progress values.
  /// @param s The step.
  /// @param from Progress already applied, 0..1.
  /// @param to Progress to reach, 0..1.
  /// @param v Camera, changed.
  /// @param f Screen and limits.
  static void apply(const Vrst_scenario_step& s, Vrst_float64 from, Vrst_float64 to, Vrst_view& v,
                    const Vrst_scenario_frame& f);
  /// Moves the camera part of the way to a View step's camera.
  /// @param s The View step.
  /// @param k Part of the remaining way, 0..1; 1 = exactly there.
  /// @param v Camera, changed.
  /// @param f Screen and world origin.
  static void move_toward(const Vrst_scenario_step& s, Vrst_float64 k, Vrst_view& v, const Vrst_scenario_frame& f);
  /// Eased progress of a step: smooth start and stop, fast start for a fling, linear
  /// for View (recorded motion already has its own speed).
  /// @param s The step.
  /// @param x Linear time progress, 0..1.
  /// @return Motion progress, 0..1.
  static Vrst_float64 ease(const Vrst_scenario_step& s, Vrst_float64 x);

  /// Name for the log and the overlay.
  std::string m_name;
  /// Parsed steps.
  std::vector<Vrst_scenario_step> m_steps;
  /// Step playing now.
  Vrst_size m_index = 0;
  /// Clock when the current step began.
  Vrst_float64 m_step_start = 0;
  /// Motion progress of the current step already applied, 0..1.
  Vrst_float64 m_done = 0;
  /// Playing.
  bool m_running = false;
};

}  // namespace nav
