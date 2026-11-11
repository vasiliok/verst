/// @file
/// Records what the user does with the map (pan, zoom, rotate, jumps by buttons) as a
/// test scenario (scenario.h) that plays it back the same way.
///
/// The recorder watches the camera, not the touches: whatever moved the map (a gesture,
/// Fit, Start, the location button), the scenario repeats the camera path with its
/// timing. Every change becomes a `view <s> <lat> <lon> <m/pt> <deg>` line (at most
/// kMaxPerSecond per second), and a pause becomes a `wait <s>` line. Positions are in
/// degrees and ground metres per screen point, so a recording plays on any phone and
/// with any map that covers the place.
#pragma once

#include <string>

#include "map_scene.h"
#include "vrst_types.h"

namespace nav {

/// Camera recorder; see the file comment.
class Vrst_activity_recorder {
 public:
  /// Camera lines per second, at most; playback moves straight between them.
  static constexpr Vrst_float64 kMaxPerSecond = 30;
  /// A camera standing still longer than this (s) is written as a `wait`.
  static constexpr Vrst_float64 kPauseSeconds = 0.25;

  /// Starts a recording at the current camera (written as a `view 0 ...` line).
  /// @param now Clock in seconds (any origin, the same as in sample() and finish()).
  /// @param v Camera now.
  /// @param world_origin Absolute Mercator point of world (0, 0) (Vrst_map_frame::center()).
  /// @param px_per_point Framebuffer px per screen point.
  void start(Vrst_float64 now, const Vrst_view& v, Vrst_vec2d world_origin, Vrst_float64 px_per_point);

  /// Writes the camera when it changed since the last line. Call often (every display
  /// tick); calls closer than 1 / kMaxPerSecond to the last line are skipped.
  /// @param now Clock in seconds.
  /// @param v Camera now.
  void sample(Vrst_float64 now, const Vrst_view& v);

  /// Ends the recording.
  /// @param now Clock in seconds.
  /// @return The scenario text: a comment line with the length, then the steps.
  std::string finish(Vrst_float64 now);

  /// @return true between start() and finish().
  bool recording() const { return m_recording; }
  /// @param now Clock in seconds.
  /// @return Seconds since start(), 0 when not recording.
  Vrst_float64 seconds(Vrst_float64 now) const { return m_recording ? now - m_start : 0; }

 private:
  /// Appends a `view` line.
  /// @param seconds Time to move there.
  /// @param v Camera.
  void write_view(Vrst_float64 seconds, const Vrst_view& v);

  /// Steps written so far.
  std::string m_steps;
  /// Camera of the last `view` line.
  Vrst_view m_last;
  /// Clock of the last `view` line.
  Vrst_float64 m_last_t = 0;
  /// Clock of start().
  Vrst_float64 m_start = 0;
  /// Absolute Mercator point of world (0, 0).
  Vrst_vec2d m_origin{};
  /// Framebuffer px per screen point.
  Vrst_float64 m_px_per_point = 1;
  /// `view` lines written.
  Vrst_size m_views = 0;
  /// Between start() and finish().
  bool m_recording = false;
};

}  // namespace nav
