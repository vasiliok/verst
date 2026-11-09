/// @file
/// Platform-independent map drawing data shared by navview (OpenGL) and the iOS app
/// (OpenGL ES): Web Mercator projection, road class tables, the GPU vertex format, the
/// 2D camera and the mesh built from the nav files.
///
/// Nothing here calls a graphics API. Each front end uploads Vrst_map_mesh's arrays
/// and feeds view_matrix() to its own shaders, so both draw the same map.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <string_view>
#include <vector>

#include "nav_files.h"

namespace nav {

constexpr Vrst_float64 kPi = std::numbers::pi;  ///< Pi in double, for degree <-> radian conversion.
/// WGS84 equatorial radius in metres, the sphere radius of Web Mercator (EPSG:3857).
constexpr Vrst_float64 kEarthRadius = 6378137.0;

/// 2D point or vector in double precision: Web Mercator metres, screen pixels or
/// (lat, lon) degrees depending on use.
/// All view math is done with it; only center-relative values are converted to
/// float for the GPU.
struct Vrst_vec2d {
  /// Double on purpose: Web Mercator metres reach 2e7, where float has ~2 m steps.
  /// Everything is computed in double and only center-relative values go to float.
  Vrst_float64 x;
  Vrst_float64 y;  ///< Second component; same units and precision as x.
};

/// Web Mercator (EPSG:3857) projection.
/// @param c Geographic coordinate.
/// @return Mercator metres (x east, y north) in double.
inline Vrst_vec2d mercator(Vrst_coord c) {
  const Vrst_float64 lon = c.lon_e7 * 1e-7 * kPi / 180.0;
  const Vrst_float64 lat = c.lat_e7 * 1e-7 * kPi / 180.0;
  return {kEarthRadius * lon, kEarthRadius * std::log(std::tan(kPi / 4 + lat / 2))};
}

/// Inverse of mercator().
/// @param m Web Mercator metres.
/// @return Degrees: x = latitude, y = longitude.
inline Vrst_vec2d inverse_mercator(Vrst_vec2d m) {
  return {(2 * std::atan(std::exp(m.y / kEarthRadius)) - kPi / 2) * 180.0 / kPi, m.x / kEarthRadius * 180.0 / kPi};
}

/// Bit of a road class in a class mask.
/// @param c Road class.
/// @return Mask with only that class's bit set.
constexpr Vrst_uint32 bit(RoadClass c) { return 1u << static_cast<int>(c); }

/// Number of road class groups the UIs toggle (keys 1..9 in navview).
inline constexpr int kClassGroupCount = 9;
/// Classes toggled together: a road class and its link ramps, residential with
/// living_street.
constexpr Vrst_uint32 kClassGroups[kClassGroupCount] = {
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
/// Label of each kClassGroups entry, for panels, menus and the console.
constexpr std::array<std::string_view, kClassGroupCount> kGroupNames = {
    "motorway", "trunk", "primary", "secondary", "tertiary", "unclassified", "residential", "service", "ferry"};
/// Mask with every road class visible.
inline constexpr Vrst_uint32 kAllClasses = (1u << kRoadClassCount) - 1;

/// Line color (RGB) per road class, indexed by RoadClass.
constexpr Vrst_float32 kClassColors[kRoadClassCount][3] = {
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
/// Default start view when none is given: this point (47.322, 5.041) at
/// street zoom. start_view() falls back to the whole map when it is outside the data.
constexpr Vrst_coord kDefaultPoint{473220000, 50410000};
constexpr Vrst_float64 kDefaultMetresPerPoint = 5.0;  ///< Start zoom: ground metres per screen point.

/// Background color (RGB) behind the map.
constexpr Vrst_float32 kBackground[3] = {0.105f, 0.115f, 0.135f};

/// Vertex as uploaded to the GPU: float position relative to the map center plus one
/// packed attribute.
/// Kept to 12 bytes so the buffer stays small. All roads are one vertex buffer drawn
/// with one line-list call; the shader decides color and visibility from attr.
struct Vrst_gpu_point {
  /// Mercator metres relative to the map center. Float is enough only because of the
  /// subtraction: at 25 km from the center the step is ~2 mm, so zoom does not shake.
  Vrst_float32 x;
  Vrst_float32 y;  ///< Second component; same units as x.
  /// Lines: `road_class | flags << 16`. Points: 1 = dead end. One integer attribute
  /// lets one draw call color, filter and highlight everything in the shader.
  Vrst_uint32 attr;
};
static_assert(sizeof(Vrst_gpu_point) == 12);

/// Camera of the 2D map: center, scale and rotation.
/// Everything the view matrix is built from; independent of window system and
/// graphics API.
struct Vrst_view {
  /// Point under the screen center, Mercator metres relative to the map center. Kept
  /// in double so panning at deep zoom moves smoothly.
  Vrst_vec2d center{0, 0};
  /// Framebuffer pixels per Mercator metre. Framebuffer, not window, units so the
  /// matrix is right on HiDPI and Retina screens.
  Vrst_float64 scale = 1;
  /// Map rotation, radians counter-clockwise. Separate from scale so zoom at a point
  /// and pan stay correct while rotated.
  Vrst_float64 angle = 0;
};

/// World -> NDC as a column-major mat3, computed in double and stored as float.
/// Same layout for GLSL mat3 and for three Metal float3 columns.
/// @param v Camera.
/// @param fb_w Framebuffer width in px.
/// @param fb_h Framebuffer height in px.
/// @return 9 floats, column-major.
inline std::array<Vrst_float32, 9> view_matrix(const Vrst_view& v, int fb_w, int fb_h) {
  const Vrst_float64 c = std::cos(v.angle) * v.scale, s = std::sin(v.angle) * v.scale;
  const Vrst_float64 sx = 2.0 / fb_w, sy = 2.0 / fb_h;
  // p' = D * S*R * (p - center)
  const Vrst_float64 tx = -(c * v.center.x - s * v.center.y), ty = -(s * v.center.x + c * v.center.y);
  const Vrst_float64 m[9] = {c * sx, s * sy, 0, -s * sx, c * sy, 0, tx * sx, ty * sy, 1};
  std::array<Vrst_float32, 9> out{};
  for (int i = 0; i < 9; ++i) out[i] = static_cast<Vrst_float32>(m[i]);
  return out;
}

/// Screen point -> pixel offset from the screen center, y up.
/// @param fb_w Framebuffer width in px.
/// @param fb_h Framebuffer height in px.
/// @param x Screen x in framebuffer px.
/// @param y Screen y in framebuffer px, from the top.
/// @return Offset in framebuffer px, y up.
inline Vrst_vec2d screen_offset(int fb_w, int fb_h, Vrst_float64 x, Vrst_float64 y) {
  return {x - fb_w / 2.0, fb_h / 2.0 - y};
}

/// Screen offset -> world delta, undoing scale and rotation.
/// @param v Camera.
/// @param d Offset in framebuffer px, y up.
/// @return Same offset in Mercator metres.
inline Vrst_vec2d unproject_delta(const Vrst_view& v, Vrst_vec2d d) {
  const Vrst_float64 c = std::cos(-v.angle), s = std::sin(-v.angle);
  return {(c * d.x - s * d.y) / v.scale, (s * d.x + c * d.y) / v.scale};
}

/// World position under a screen point.
/// @param v Camera.
/// @param fb_w Framebuffer width in px.
/// @param fb_h Framebuffer height in px.
/// @param x Screen x in framebuffer px.
/// @param y Screen y in framebuffer px, from the top.
/// @return Mercator metres relative to the map center.
inline Vrst_vec2d screen_to_world(const Vrst_view& v, int fb_w, int fb_h, Vrst_float64 x, Vrst_float64 y) {
  const Vrst_vec2d d = unproject_delta(v, screen_offset(fb_w, fb_h, x, y));
  return {v.center.x + d.x, v.center.y + d.y};
}

/// Screen position of a world point; the inverse of screen_to_world().
/// @param v Camera.
/// @param fb_w Framebuffer width in px.
/// @param fb_h Framebuffer height in px.
/// @param w Mercator metres relative to the map center.
/// @return Framebuffer px, y from the top.
inline Vrst_vec2d world_to_screen(const Vrst_view& v, int fb_w, int fb_h, Vrst_vec2d w) {
  const Vrst_float64 c = std::cos(v.angle) * v.scale, s = std::sin(v.angle) * v.scale;
  const Vrst_float64 dx = w.x - v.center.x, dy = w.y - v.center.y;
  return {fb_w / 2.0 + c * dx - s * dy, fb_h / 2.0 - (s * dx + c * dy)};
}

/// Moves the camera so the world point under (x, y) moves with a drag of (dx, dy).
/// @param v Camera, changed.
/// @param dx Drag x in framebuffer px.
/// @param dy Drag y in framebuffer px, down positive (screen convention).
inline void pan_by(Vrst_view& v, Vrst_float64 dx, Vrst_float64 dy) {
  const Vrst_vec2d d = unproject_delta(v, {dx, -dy});
  v.center.x -= d.x;
  v.center.y -= d.y;
}

/// Zooms by a factor while keeping the world point under a screen point fixed.
/// @param v Camera, changed.
/// @param fb_w Framebuffer width in px.
/// @param fb_h Framebuffer height in px.
/// @param x Fixed screen x in framebuffer px.
/// @param y Fixed screen y in framebuffer px, from the top.
/// @param factor New scale / old scale, before clamping.
/// @param min_scale Lowest allowed scale (zoomed out).
/// @param max_scale Highest allowed scale (zoomed in).
inline void zoom_at(Vrst_view& v, int fb_w, int fb_h, Vrst_float64 x, Vrst_float64 y, Vrst_float64 factor,
                    Vrst_float64 min_scale, Vrst_float64 max_scale) {
  const Vrst_vec2d before = screen_to_world(v, fb_w, fb_h, x, y);
  v.scale = std::clamp(v.scale * factor, min_scale, max_scale);
  const Vrst_vec2d d = unproject_delta(v, screen_offset(fb_w, fb_h, x, y));
  v.center = {before.x - d.x, before.y - d.y};
}

/// Rotates the camera around a screen point, which stays fixed.
/// @param v Camera, changed.
/// @param fb_w Framebuffer width in px.
/// @param fb_h Framebuffer height in px.
/// @param x Fixed screen x in framebuffer px.
/// @param y Fixed screen y in framebuffer px, from the top.
/// @param radians Counter-clockwise rotation to add.
inline void rotate_at(Vrst_view& v, int fb_w, int fb_h, Vrst_float64 x, Vrst_float64 y, Vrst_float64 radians) {
  const Vrst_vec2d before = screen_to_world(v, fb_w, fb_h, x, y);
  v.angle += radians;
  const Vrst_vec2d d = unproject_delta(v, screen_offset(fb_w, fb_h, x, y));
  v.center = {before.x - d.x, before.y - d.y};
}

/// Map geometry ready for the GPU, built once from the nav files.
/// Projects every shape point to Mercator relative to the map center, packs road
/// class and flags into each vertex, orders the line index list so minor classes
/// draw first (major roads end up on top) and lists the graph vertices with dead
/// ends last (drawn on top).
class Vrst_map_mesh {
 public:
  /// Builds all arrays: projects the road shape points, computes the map center and
  /// extent, and (with gpu_arrays) the line vertices, the line index list sorted by
  /// road class and the vertex dots with dead ends last. The steps are explained at
  /// the top of the function body.
  /// @param d Opened nav files; only read during the call.
  /// @param gpu_arrays false computes only the map center and extent, for front ends
  ///        that draw tiles (map_tiles.h) instead of one buffer.
  void build(const Vrst_nav_data& d, bool gpu_arrays = true);

  /// Scale that shows the whole map with a 5% margin.
  /// @param fb_w Framebuffer width in px.
  /// @param fb_h Framebuffer height in px.
  /// @return Framebuffer px per Mercator metre.
  Vrst_float64 fit_scale(int fb_w, int fb_h) const;

  /// Camera centered on a geographic point at a given ground resolution.
  /// @param p Point to center on.
  /// @param metres_per_point Ground metres per screen point (smaller = closer).
  /// @param px_per_point Framebuffer pixels per screen point (Retina factor).
  /// @return Camera with no rotation.
  Vrst_view view_at(Vrst_coord p, Vrst_float64 metres_per_point, Vrst_float64 px_per_point) const;

  /// Start view used by every front end: centered on a point at a ground resolution,
  /// or the whole map when the point is outside the data.
  /// @param bbox Extent of the data (Vrst_graph::bbox()).
  /// @param p Point to center on.
  /// @param metres_per_point Ground metres per screen point (smaller = closer).
  /// @param px_per_point Framebuffer pixels per screen point (Retina factor).
  /// @param fb_w Framebuffer width in px, for the fit-map fallback.
  /// @param fb_h Framebuffer height in px, for the fit-map fallback.
  /// @return Camera with no rotation.
  Vrst_view start_view(const Vrst_bbox& bbox, Vrst_coord p, Vrst_float64 metres_per_point,
                       Vrst_float64 px_per_point, int fb_w, int fb_h) const;

  /// World position of a geographic point, for overlays such as a GPS track.
  /// @param p Geographic coordinate.
  /// @return Mercator metres relative to the map center, like the mesh vertices.
  Vrst_vec2d to_world(Vrst_coord p) const {
    const Vrst_vec2d m = mercator(p);
    return {m.x - m_center.x, m.y - m_center.y};
  }

  /// Geographic position of a world point.
  /// @param w Mercator metres relative to the map center.
  /// @return Degrees: x = latitude, y = longitude.
  Vrst_vec2d to_latlon(Vrst_vec2d w) const { return inverse_mercator({w.x + m_center.x, w.y + m_center.y}); }

  /// @copydoc m_center
  /// @return Mercator metres.
  Vrst_vec2d center() const { return m_center; }
  /// @copydoc m_half_extent
  /// @return Metres.
  Vrst_vec2d half_extent() const { return m_half_extent; }
  /// @copydoc m_line_verts
  /// @return Vertex array.
  const std::vector<Vrst_gpu_point>& line_verts() const { return m_line_verts; }
  /// @copydoc m_line_indices
  /// @return Index array.
  const std::vector<Vrst_uint32>& line_indices() const { return m_line_indices; }
  /// @copydoc m_point_verts
  /// @return Vertex array.
  const std::vector<Vrst_gpu_point>& point_verts() const { return m_point_verts; }
  /// @copydoc m_dead_ends
  /// @return Count.
  Vrst_uint32 dead_ends() const { return m_dead_ends; }

 private:
  /// Mercator position subtracted from every point before the float conversion.
  /// Without it vertices are stored as ~1e6..2e7 m floats and lines jitter at zoom.
  Vrst_vec2d m_center{};
  /// Half size of the point extent in metres; used to fit the whole map.
  Vrst_vec2d m_half_extent{};
  /// One vertex per shape point, same index as in geom.dat.
  std::vector<Vrst_gpu_point> m_line_verts;
  /// Index pairs for a line list, minor road classes first.
  std::vector<Vrst_uint32> m_line_indices;
  /// One vertex per graph vertex, dead ends last.
  std::vector<Vrst_gpu_point> m_point_verts;
  /// Count of degree-1 vertices.
  Vrst_uint32 m_dead_ends = 0;
};

}  // namespace nav
