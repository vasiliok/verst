/// @file
/// Plane geometry shared by osm2nav (local metres) and the tile builder (Web Mercator
/// metres): line simplification, ring area, the smallest enclosing rectangle, clipping a
/// ring to a box and removing repeated points. Header-only templates over any point
/// type with double `x` and `y` that can be built from `{x, y}` (Vrst_xy_point).
#pragma once

#include <algorithm>
#include <cmath>
#include <concepts>
#include <utility>
#include <vector>

#include "geo.h"
#include "vrst_types.h"

namespace nav {

/// A 2D point with double coordinates: Vrst_vec2d, or osm2nav's local metres.
template <typename P>
concept Vrst_xy_point = requires(P p) {
  { p.x } -> std::convertible_to<Vrst_float64>;
  { p.y } -> std::convertible_to<Vrst_float64>;
  P{Vrst_float64{}, Vrst_float64{}};
};

/// Douglas-Peucker simplification of a polyline or a closed ring (first point repeated
/// at the end), in place: drops every point that lies within tol of the simplified line.
///
/// Iterative, with an explicit stack instead of recursion, so an outline with 100 000
/// points cannot overflow the call stack:
///  - keep[] marks the points that survive; the two ends always do.
///  - Each stack entry is a span [i, j] whose two ends are kept. The point between them
///    farthest from the straight line i-j is found: the perpendicular distance, or the
///    plain distance to i when i and j are the same point (a closed ring).
///  - Farther than tol: the line i-j would cut off a real detail of the shape, so that
///    point is kept and both halves [i, at] and [at, j] are checked. Otherwise every point
///    between i and j lies within tol of the line and is dropped.
///
/// Cost: O(n log n) on usual shapes, O(n^2) at worst (a spiral). The result keeps the
/// shape within tol but not its topology: a simplified ring can cross itself.
/// @param pts Points; replaced by the kept points, in the original order.
/// @param tol Tolerance in the units of the points; 0 or less leaves them unchanged.
template <Vrst_xy_point P>
void simplify_line(std::vector<P>& pts, Vrst_float64 tol) {
  if (tol <= 0 || pts.size() <= 2) return;
  std::vector<Vrst_uint8> keep(pts.size(), 0);
  keep.front() = keep.back() = 1;
  std::vector<std::pair<Vrst_size, Vrst_size>> stack{{0, pts.size() - 1}};
  while (!stack.empty()) {
    const auto [i, j] = stack.back();
    stack.pop_back();
    const P a = pts[i], b = pts[j];
    // (dx, dy): the line a->b that would replace the points between. A point's distance
    // to it is |cross(p - a, b - a)| / len; len == 0 when a and b coincide (closed ring).
    const Vrst_float64 dx = b.x - a.x, dy = b.y - a.y, len = std::sqrt(dx * dx + dy * dy);
    Vrst_float64 best = -1;
    Vrst_size at = i;
    for (Vrst_size k = i + 1; k < j; ++k) {
      const Vrst_float64 px = pts[k].x - a.x, py = pts[k].y - a.y;
      const Vrst_float64 d = len > 0 ? std::abs(px * dy - py * dx) / len : std::hypot(px, py);
      if (d > best) best = d, at = k;
    }
    if (best > tol) {
      keep[at] = 1;
      stack.emplace_back(i, at);
      stack.emplace_back(at, j);
    }
  }
  Vrst_size n = 0;
  for (Vrst_size k = 0; k < pts.size(); ++k)
    if (keep[k]) pts[n++] = pts[k];
  pts.resize(n);
}

/// Signed area of a ring (shoelace).
/// @param r Ring without the closing point.
/// @return Area in squared units of the points, positive when counter-clockwise.
template <Vrst_xy_point P>
Vrst_float64 ring_area(const std::vector<P>& r) {
  Vrst_float64 s = 0;
  for (Vrst_size i = 0, j = r.size() - 1; i < r.size(); j = i++) s += r[j].x * r[i].y - r[i].x * r[j].y;
  return s / 2;
}

/// Smallest rectangle, at any rotation, that contains a ring. osm2nav uses it for
/// buildings: when the rectangle is barely bigger than the outline, the building is
/// stored as these 4 corners instead of its full outline.
///
/// Method: the convex hull of the points (Andrew's monotone chain), then one candidate
/// rectangle per hull edge, aligned with that edge; the smallest wins. This is enough
/// because the minimum-area enclosing rectangle always has a side on a hull edge
/// (Freeman and Shapira, 1975). Each candidate measures the whole hull, so the cost is
/// O(n log n) for the hull plus O(h^2) for h hull points (rotating calipers would make
/// the second part O(h); building hulls are small, so this simpler loop is kept).
/// @param r Ring without the closing point, at least 3 points; taken by value because
///        it is sorted in place.
/// @return 4 corners, counter-clockwise; all zero when every point is the same (no hull
///         edge has a length).
template <Vrst_xy_point P>
std::vector<P> min_area_rectangle(std::vector<P> r) {
  // 1. Convex hull: sort the points left to right, then walk them twice (lower hull left
  //    to right, upper hull back) keeping only left turns.
  std::ranges::sort(r, [](const P& a, const P& b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
  // cross > 0: o -> a -> b turns left (counter-clockwise); <= 0: right turn or straight.
  const auto cross = [](const P& o, const P& a, const P& b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
  };
  std::vector<P> h(2 * r.size(), P{0, 0});  // hull under construction, k points used
  Vrst_size k = 0;
  for (const P& p : r) {  // lower hull: drop the last point while it makes a right turn
    while (k >= 2 && cross(h[k - 2], h[k - 1], p) <= 0) --k;
    h[k++] = p;
  }
  for (Vrst_size i = r.size() - 1, t = k + 1; i-- > 0;) {  // upper hull; t protects the lower one
    while (k >= t && cross(h[k - 2], h[k - 1], r[i]) <= 0) --k;
    h[k++] = r[i];
  }
  h.resize(k > 1 ? k - 1 : k);  // the last point repeats the first
  // 2. One rectangle per hull edge, its sides along and across the edge; keep the smallest.
  Vrst_float64 best = 1e300;
  std::vector<P> out(4, P{0, 0});
  for (Vrst_size i = 0; i < h.size(); ++i) {
    const P a = h[i], b = h[(i + 1) % h.size()];
    const Vrst_float64 len = std::hypot(b.x - a.x, b.y - a.y);
    if (len <= 0) continue;
    const Vrst_float64 ux = (b.x - a.x) / len, uy = (b.y - a.y) / len;  // edge direction
    // Extent of the hull in the edge's frame: u along the edge, v across it.
    Vrst_float64 u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300;
    for (const P& p : h) {
      const Vrst_float64 u = p.x * ux + p.y * uy, v = -p.x * uy + p.y * ux;
      u0 = std::min(u0, u), u1 = std::max(u1, u), v0 = std::min(v0, v), v1 = std::max(v1, v);
    }
    const Vrst_float64 area = (u1 - u0) * (v1 - v0);
    if (area < best) {
      best = area;
      // Back from (u, v) to x, y: rotate by the edge direction.
      const auto pt = [&](Vrst_float64 u, Vrst_float64 v) { return P{u * ux - v * uy, u * uy + v * ux}; };
      out = {pt(u0, v0), pt(u1, v0), pt(u1, v1), pt(u0, v1)};
    }
  }
  return out;
}

/// Clips a ring to an axis-aligned box (Sutherland-Hodgman: one pass per box side,
/// keeping the inside part and adding the crossing points). The result is the part of
/// the polygon inside the box; parts of the outline that run outside come back as
/// pieces of the box border. A concave ring that leaves and re-enters the box can give
/// a ring with zero-width bridges along the border, which a triangulation fills with
/// nothing, so the drawn area is still right.
/// @param ring Ring without the closing point.
/// @param box Box.
/// @return Clipped ring without the closing point; fewer than 3 points when nothing of
///         the ring is inside.
template <Vrst_xy_point P>
std::vector<P> clip_ring(const std::vector<P>& ring, const Vrst_box2d& box) {
  std::vector<P> in = ring, out;
  // One side per pass: axis 0 = x, 1 = y; keep values >= lo (sign +1) or <= hi (sign -1).
  const auto clip = [&](int axis, Vrst_float64 limit, Vrst_float64 sign) {
    out.clear();
    const auto coord = [axis](const P& p) { return axis == 0 ? p.x : p.y; };
    const auto inside = [&](const P& p) { return (coord(p) - limit) * sign >= 0; };
    for (Vrst_size i = 0; i < in.size(); ++i) {
      const P& a = in[i];
      const P& b = in[(i + 1) % in.size()];
      const bool ia = inside(a), ib = inside(b);
      if (ia) out.push_back(a);
      if (ia != ib) {  // the edge a-b crosses the side: add the crossing point
        const Vrst_float64 t = (limit - coord(a)) / (coord(b) - coord(a));
        out.push_back(P{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t});
      }
    }
    std::swap(in, out);
  };
  clip(0, box.lo.x, 1);
  if (!in.empty()) clip(0, box.hi.x, -1);
  if (!in.empty()) clip(1, box.lo.y, 1);
  if (!in.empty()) clip(1, box.hi.y, -1);
  return in;
}

/// Removes repeated points from a ring: a point closer than eps to the one before it,
/// and a last point that repeats the first. Clipping and simplification leave such
/// points, and a triangulation may spend long on them.
/// @param ring Ring; changed in place.
/// @param eps Distance below which two points are the same, units of the points.
template <Vrst_xy_point P>
void remove_repeated_points(std::vector<P>& ring, Vrst_float64 eps) {
  const auto same = [eps](const P& a, const P& b) { return std::abs(a.x - b.x) <= eps && std::abs(a.y - b.y) <= eps; };
  Vrst_size n = 0;
  for (Vrst_size i = 0; i < ring.size(); ++i)
    if (n == 0 || !same(ring[i], ring[n - 1])) ring[n++] = ring[i];
  while (n > 1 && same(ring[n - 1], ring[0])) --n;
  ring.resize(n);
}

}  // namespace nav
