/// @file
/// Tile index, tile builder, background loader and visible_tiles().
/// Declarations and documentation are in map_tiles.h.
#include "map_tiles.h"

#include <algorithm>
#include <exception>
#include <utility>

#include <mapbox/earcut.hpp>

namespace nav {

void Vrst_tile_index::build(const Vrst_nav_data& d) {
  for (auto& m : m_tiles) m.clear();
  const auto add = [&](int level, const Vrst_box2d& b, auto member, Vrst_uint32 id) {
    const int z = kTileLevels[level].z;
    const Vrst_tile_id a = tile_at(z, {b.lo.x, b.hi.y}), c = tile_at(z, {b.hi.x, b.lo.y});
    for (Vrst_int32 y = a.y; y <= c.y; ++y)
      for (Vrst_int32 x = a.x; x <= c.x; ++x) (m_tiles[level][tile_key({z, x, y})].*member).push_back(id);
  };
  const Vrst_geom& geo = d.geom();
  const Vrst_edges& e = d.edges();
  for (Vrst_uint32 s = 0; s < e.num_segments(); ++s) {
    const Vrst_box2d b = box_of(geo.points().subspan(geo.seg_first_point()[s], geo.seg_first_point()[s + 1] - geo.seg_first_point()[s]));
    const Vrst_uint32 cls = bit(static_cast<RoadClass>(e.attr()[s].road_class));
    for (int l = 0; l < kTileLevelCount; ++l)
      if (kTileLevels[l].class_mask & cls) add(l, b, &Vrst_tile_items::segs, s);
  }
  if (d.has_map()) {
    const Vrst_map_layers& m = d.map();
    for (Vrst_uint32 a = 0; a < m.num_areas(); ++a) {
      const Vrst_box2d b = box_of(m.area_outer(a));
      const bool building = m.area_kind(a) == AreaKind::Building;
      const Vrst_float64 size = std::max(b.hi.x - b.lo.x, b.hi.y - b.lo.y);
      for (int l = 0; l < kTileLevelCount; ++l)
        if (building ? kTileLevels[l].buildings : size >= kTileLevels[l].min_area_m) add(l, b, &Vrst_tile_items::areas, a);
    }
    for (Vrst_uint32 i = 0; i < m.num_lines(); ++i) {
      const Vrst_box2d b = box_of(m.line_points(i));
      const bool stream = m.line_attr()[i].kind == static_cast<Vrst_uint8>(LineKind::Stream);
      for (int l = 0; l < kTileLevelCount; ++l)
        if (!stream || kTileLevels[l].streams) add(l, b, &Vrst_tile_items::lines, i);
    }
  }
  const Vrst_graph& g = d.graph();
  std::vector<Vrst_uint8> degree(g.num_vertices(), 0);
  for (const Vrst_seg_vertices& sv : e.vertices()) {
    degree[sv.a] = static_cast<Vrst_uint8>(std::min(degree[sv.a] + 1, 2));
    degree[sv.b] = static_cast<Vrst_uint8>(std::min(degree[sv.b] + 1, 2));
  }
  m_dead_end.assign(g.num_vertices(), 0);
  for (Vrst_uint32 v = 0; v < g.num_vertices(); ++v) {
    m_dead_end[v] = degree[v] == 1;
    const Vrst_vec2d p = mercator(g.vertex_coord()[v]);
    for (int l = 0; l < kTileLevelCount; ++l)
      if (kTileLevels[l].vertices) add(l, {p, p}, &Vrst_tile_items::vertices, v);
  }
}

const Vrst_tile_items& Vrst_tile_index::find(Vrst_tile_id t) const {
  static const Vrst_tile_items kNone{};
  const int l = level_of_zoom(t.z);
  if (l < 0) return kNone;
  const auto it = m_tiles[l].find(tile_key(t));
  return it == m_tiles[l].end() ? kNone : it->second;
}

std::vector<Vrst_tile_id> Vrst_tile_index::tile_ids(int level) const {
  std::vector<Vrst_tile_id> out;
  out.reserve(m_tiles[level].size());
  for (const auto& [k, items] : m_tiles[level]) out.push_back(tile_of_key(k));
  return out;
}

Vrst_box2d Vrst_tile_index::box_of(std::span<const Vrst_coord> pts) {
  if (pts.empty()) return {{0, 0}, {0, 0}};
  Vrst_coord lo = pts[0], hi = pts[0];
  for (const Vrst_coord& c : pts) {
    lo = {std::min(lo.lat_e7, c.lat_e7), std::min(lo.lon_e7, c.lon_e7)};
    hi = {std::max(hi.lat_e7, c.lat_e7), std::max(hi.lon_e7, c.lon_e7)};
  }
  return {mercator(lo), mercator(hi)};
}

std::unique_ptr<Vrst_tile_data> Vrst_tile_builder::build(Vrst_tile_id id) const {
  auto out = std::make_unique<Vrst_tile_data>();
  out->id = id;
  const Vrst_box2d tb = tile_bounds(id);
  out->origin = {tb.lo.x - m_center.x, tb.lo.y - m_center.y};
  const Vrst_tile_items& items = m_index.find(id);
  const Vrst_tile_level& level = kTileLevels[level_of_zoom(id.z)];
  // Pieces a little outside the tile are kept, so wide lines meet across the border.
  const Vrst_float64 margin = tile_side(id.z) * 0.02;
  const Vrst_box2d keep{{tb.lo.x - margin, tb.lo.y - margin}, {tb.hi.x + margin, tb.hi.y + margin}};
  std::array<Vrst_style_buckets, kTileLayerCount> buckets;
  Vrst_area_buckets fills;
  {
    std::vector<Vrst_vec2d> pts;
    // Roads.
    const Vrst_geom& geo = m_data.geom();
    for (const Vrst_uint32 s : items.segs) {
      const Vrst_seg_attr& a = m_data.edges().attr()[s];
      project(geo.points().subspan(geo.seg_first_point()[s], geo.seg_first_point()[s + 1] - geo.seg_first_point()[s]), pts);
      simplify(pts, level.tolerance_m);
      const int layer = a.flags & kSegTunnel ? kLayerTunnel : a.flags & kSegBridge ? kLayerBridge : kLayerGround;
      const Vrst_uint32 attr = a.road_class | (a.flags & kSegOneway ? kTileLineOneway : 0);
      add_pieces(pts, tb.lo, keep, attr, buckets[layer][a.road_class]);
    }
    if (m_data.has_map()) {
      const Vrst_map_layers& m = m_data.map();
      for (const Vrst_uint32 l : items.lines) {
        const Vrst_line_attr& la = m.line_attr()[l];
        project(m.line_points(l), pts);
        simplify(pts, level.tolerance_m);
        const int layer = la.flags & kLineTunnel ? kLayerTunnel : la.flags & kLineBridge ? kLayerBridge : kLayerGround;
        const int style = kStyleLineKind0 + la.kind;
        add_pieces(pts, tb.lo, keep, static_cast<Vrst_uint32>(style), buckets[layer][style]);
      }
      for (const Vrst_uint32 ar : items.areas) add_area(ar, level, tb.lo, keep, fills, buckets[kLayerGround]);
    }
    // Graph vertex dots, dead ends last so they are drawn on top.
    for (int pass = 0; pass < 2; ++pass)
      for (const Vrst_uint32 v : items.vertices) {
        const bool dead = m_index.dead_end(v);
        if (dead != (pass == 1)) continue;
        const Vrst_vec2d p = mercator(m_data.graph().vertex_coord()[v]);
        out->points.push_back({static_cast<Vrst_float32>(p.x - tb.lo.x), static_cast<Vrst_float32>(p.y - tb.lo.y),
                               static_cast<Vrst_uint32>(dead)});
      }
  }
  for (int k = 0; k < kAreaKindCount; ++k) {
    out->fill_ranges[k] = {static_cast<Vrst_uint32>(out->fills.size()), static_cast<Vrst_uint32>(fills[k].size())};
    out->fills.insert(out->fills.end(), fills[k].begin(), fills[k].end());
  }
  for (int layer = 0; layer < kTileLayerCount; ++layer)
    for (int style = 0; style < kLineStyleCount; ++style) {
      const std::vector<Vrst_line_instance>& b = buckets[layer][style];
      out->line_ranges[layer][style] = {static_cast<Vrst_uint32>(out->lines.size()),
                                        static_cast<Vrst_uint32>(b.size())};
      out->lines.insert(out->lines.end(), b.begin(), b.end());
    }
  return out;
}

void Vrst_tile_builder::simplify(std::vector<Vrst_vec2d>& pts, Vrst_float64 tol) {
  if (tol <= 0 || pts.size() <= 2) return;
  const std::vector<Vrst_vec2d>& p = pts;
  std::vector<Vrst_uint8> keep(p.size(), 0);
  keep.front() = keep.back() = 1;
  std::vector<std::pair<Vrst_size, Vrst_size>> stack{{0, p.size() - 1}};
  while (!stack.empty()) {
    const auto [i, j] = stack.back();
    stack.pop_back();
    const Vrst_float64 dx = p[j].x - p[i].x, dy = p[j].y - p[i].y, len = std::hypot(dx, dy);
    Vrst_float64 best = -1;
    Vrst_size at = i;
    for (Vrst_size k = i + 1; k < j; ++k) {
      const Vrst_float64 px = p[k].x - p[i].x, py = p[k].y - p[i].y;
      const Vrst_float64 dist = len > 0 ? std::abs(px * dy - py * dx) / len : std::hypot(px, py);
      if (dist > best) best = dist, at = k;
    }
    if (best > tol) {
      keep[at] = 1;
      stack.emplace_back(i, at);
      stack.emplace_back(at, j);
    }
  }
  Vrst_size n = 0;
  for (Vrst_size k = 0; k < p.size(); ++k)
    if (keep[k]) pts[n++] = p[k];
  pts.resize(n);
}

void Vrst_tile_builder::add_pieces(const std::vector<Vrst_vec2d>& pts, Vrst_vec2d origin, const Vrst_box2d& keep,
                                   Vrst_uint32 attr, std::vector<Vrst_line_instance>& out) {
  Vrst_float64 dist = 0;
  for (Vrst_size i = 0; i + 1 < pts.size(); ++i) {
    const Vrst_vec2d a = pts[i], b = pts[i + 1];
    if (touches(a, b, keep))
      out.push_back({static_cast<Vrst_float32>(a.x - origin.x), static_cast<Vrst_float32>(a.y - origin.y),
                     static_cast<Vrst_float32>(b.x - origin.x), static_cast<Vrst_float32>(b.y - origin.y),
                     static_cast<Vrst_float32>(dist), attr});
    dist += std::hypot(b.x - a.x, b.y - a.y);
  }
}

void Vrst_tile_builder::add_area(Vrst_uint32 a, const Vrst_tile_level& level, Vrst_vec2d origin, const Vrst_box2d& keep,
                                 Vrst_area_buckets& fills, Vrst_style_buckets& lines) const {
  const Vrst_map_layers& m = m_data.map();
  const AreaKind kind = m.area_kind(a);
  std::vector<Vrst_vec2d> pts;
  std::vector<Vrst_uint32> tris;
  if (level.tolerance_m <= 0) {
    project(m.area_points(a), pts);
    const std::span<const Vrst_uint32> idx = m.area_indices(a);
    tris.assign(idx.begin(), idx.end());
  } else {
    project(m.area_outer(a), pts);
    pts.push_back(pts.front());  // closed, so simplification keeps both ends of the seam
    simplify(pts, level.tolerance_m);
    pts.pop_back();
    if (pts.size() < 3) return;
    std::vector<std::vector<std::array<Vrst_float64, 2>>> poly(1);
    for (const Vrst_vec2d& p : pts) poly[0].push_back({p.x, p.y});
    tris = mapbox::earcut<Vrst_uint32>(poly);
  }
  std::vector<Vrst_gpu_point>& out = fills[static_cast<int>(kind)];
  for (Vrst_size t = 0; t + 2 < tris.size(); t += 3) {
    const Vrst_vec2d p0 = pts[tris[t]], p1 = pts[tris[t + 1]], p2 = pts[tris[t + 2]];
    const Vrst_box2d tb{{std::min({p0.x, p1.x, p2.x}), std::min({p0.y, p1.y, p2.y})},
                        {std::max({p0.x, p1.x, p2.x}), std::max({p0.y, p1.y, p2.y})}};
    if (!touches(tb.lo, tb.hi, keep)) continue;
    for (const Vrst_vec2d& p : {p0, p1, p2})
      out.push_back({static_cast<Vrst_float32>(p.x - origin.x), static_cast<Vrst_float32>(p.y - origin.y),
                     static_cast<Vrst_uint32>(kind)});
  }
  if (kind == AreaKind::Building) {
    std::vector<Vrst_vec2d> ring(pts.begin(), pts.begin() + m.area_outer(a).size());
    ring.push_back(ring.front());
    add_pieces(ring, origin, keep, kStyleBuildingEdge, lines[kStyleBuildingEdge]);
  }
}

Vrst_tile_loader::Vrst_tile_loader(Vrst_build_fn build, int threads) : m_build(std::move(build)) {
  for (int i = 0; i < std::max(threads, 1); ++i) m_workers.emplace_back([this] { work(); });
}

Vrst_tile_loader::~Vrst_tile_loader() {
  {
    const std::scoped_lock lock(m_mutex);
    m_stop = true;
  }
  m_wake.notify_all();
  for (std::thread& t : m_workers) t.join();
}

void Vrst_tile_loader::request(const std::vector<Vrst_tile_id>& wanted) {
  {
    const std::scoped_lock lock(m_mutex);
    m_queue.clear();
    for (const Vrst_tile_id& t : wanted)
      if (!m_busy.contains(tile_key(t))) m_queue.push_back(t);
  }
  m_wake.notify_all();
}

std::vector<std::unique_ptr<Vrst_tile_data>> Vrst_tile_loader::take(Vrst_size max) {
  const std::scoped_lock lock(m_mutex);
  std::vector<std::unique_ptr<Vrst_tile_data>> out;
  while (!m_done.empty() && out.size() < max) {
    m_busy.erase(tile_key(m_done.front()->id));
    out.push_back(std::move(m_done.front()));
    m_done.pop_front();
  }
  return out;
}

void Vrst_tile_loader::work() {
  for (;;) {
    Vrst_tile_id id{};
    {
      std::unique_lock<std::mutex> lock(m_mutex);
      m_wake.wait(lock, [this] { return m_stop || !m_queue.empty(); });
      if (m_stop) return;
      id = m_queue.front();
      m_queue.pop_front();
      m_busy.insert(tile_key(id));
    }
    std::unique_ptr<Vrst_tile_data> t;
    try {
      t = m_build(id);
    } catch (const std::exception&) {
      // a broken tile must not end the thread (std::terminate) or stay busy forever
    }
    const std::scoped_lock lock(m_mutex);
    if (t) {
      m_done.push_back(std::move(t));
    } else {
      m_busy.erase(tile_key(id));
      ++m_failed;
    }
  }
}

std::vector<Vrst_tile_id> visible_tiles(const Vrst_view& v, int fb_w, int fb_h, Vrst_vec2d map_center, int z) {
  Vrst_vec2d lo{1e300, 1e300}, hi{-1e300, -1e300};
  for (const auto& [sx, sy] : {std::pair{0, 0}, {fb_w, 0}, {0, fb_h}, {fb_w, fb_h}}) {
    const Vrst_vec2d w = screen_to_world(v, fb_w, fb_h, sx, sy);
    lo = {std::min(lo.x, w.x + map_center.x), std::min(lo.y, w.y + map_center.y)};
    hi = {std::max(hi.x, w.x + map_center.x), std::max(hi.y, w.y + map_center.y)};
  }
  const Vrst_tile_id a = tile_at(z, {lo.x, hi.y}), c = tile_at(z, {hi.x, lo.y});
  std::vector<Vrst_tile_id> out;
  if (static_cast<Vrst_int64>(c.x - a.x + 1) * (c.y - a.y + 1) > 4096) return out;
  for (Vrst_int32 y = a.y; y <= c.y; ++y)
    for (Vrst_int32 x = a.x; x <= c.x; ++x) out.push_back({z, x, y});
  const Vrst_vec2d mid{v.center.x + map_center.x, v.center.y + map_center.y};
  const auto dist = [&](const Vrst_tile_id& t) {
    const Vrst_box2d b = tile_bounds(t);
    return std::hypot((b.lo.x + b.hi.x) / 2 - mid.x, (b.lo.y + b.hi.y) / 2 - mid.y);
  };
  std::ranges::sort(out, [&](const Vrst_tile_id& p, const Vrst_tile_id& q) { return dist(p) < dist(q); });
  return out;
}

}  // namespace nav
