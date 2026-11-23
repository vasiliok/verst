/// @file
/// Vrst_activity_recorder: the camera path as scenario lines.
/// Declarations and documentation are in activity_recorder.h.
#include "session/activity_recorder.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>

namespace nav {

void Vrst_activity_recorder::start(Vrst_float64 now, const Vrst_view& v, Vrst_vec2d world_origin,
                                   Vrst_float64 px_per_point) {
  m_steps.clear();
  m_origin = world_origin;
  m_px_per_point = px_per_point;
  m_start = now;
  m_views = 0;
  m_recording = true;
  write_view(0, v);
  m_last_t = now;
}

void Vrst_activity_recorder::sample(Vrst_float64 now, const Vrst_view& v) {
  if (!m_recording || now - m_last_t < 1 / kMaxPerSecond) return;
  if (v.center.x == m_last.center.x && v.center.y == m_last.center.y && v.scale == m_last.scale &&
      v.angle == m_last.angle)
    return;
  Vrst_float64 move = now - m_last_t;
  if (move > kPauseSeconds) {
    // It stood still, then moved within about one sample: wait, then a short move.
    std::array<char, 32> buf{};
    const int n = std::snprintf(buf.data(), buf.size(), "wait %.3f\n", move - 1 / kMaxPerSecond);
    m_steps.append(buf.data(), static_cast<Vrst_size>(std::clamp(n, 0, static_cast<int>(buf.size()) - 1)));
    move = 1 / kMaxPerSecond;
  }
  write_view(move, v);
  m_last_t = now;
}

std::string Vrst_activity_recorder::finish(Vrst_float64 now) {
  m_recording = false;
  std::array<char, 96> buf{};
  const int n = std::snprintf(buf.data(), buf.size(), "# recorded camera path: %.1f s, %zu views\n",
                              now - m_start, m_views);
  std::string text(buf.data(), static_cast<Vrst_size>(std::clamp(n, 0, static_cast<int>(buf.size()) - 1)));
  text += m_steps;
  m_steps.clear();
  return text;
}

void Vrst_activity_recorder::write_view(Vrst_float64 seconds, const Vrst_view& v) {
  const Vrst_latlon p = inverse_mercator({m_origin.x + v.center.x, m_origin.y + v.center.y});
  // Inverse of Vrst_map_frame::view_at: ground metres per point from px per Mercator metre.
  const Vrst_float64 mpp = m_px_per_point * std::cos(p.lat * kPi / 180) / v.scale;
  std::array<char, 128> buf{};
  const int n = std::snprintf(buf.data(), buf.size(), "view %.3f %.7f %.7f %.5g %.2f\n", seconds, p.lat, p.lon, mpp,
                              v.angle * 180 / kPi);
  m_steps.append(buf.data(), static_cast<Vrst_size>(std::clamp(n, 0, static_cast<int>(buf.size()) - 1)));
  m_last = v;
  ++m_views;
}

}  // namespace nav
