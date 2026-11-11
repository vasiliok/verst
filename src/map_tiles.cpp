/// @file
/// Tile index, tile builder, background loader and visible_tiles().
/// Declarations and documentation are in map_tiles.h.
#include "map_tiles.h"

#include <algorithm>
#include <chrono>
#include <exception>
#include <utility>

#include <mapbox/earcut.hpp>

#include "geometry.h"

namespace nav {

namespace {

/// Two outline points closer than this part of the level tolerance are the same point
/// (1e-3: 25 cm at zoom 10); clipping leaves such pairs on the tile border.
constexpr Vrst_float64 kSamePointPart = 1e-3;

/// Whether every point lies in a box.
/// @param pts Points.
/// @param box Box.
/// @return true when no point is outside.
bool inside(const std::vector<Vrst_vec2d>& pts, const Vrst_box2d& box) {
  return std::ranges::all_of(pts, [&box](const Vrst_vec2d& p) {
    return p.x >= box.lo.x && p.x <= box.hi.x && p.y >= box.lo.y && p.y <= box.hi.y;
  });
}

/// Draw layer of a road or a line from its tunnel and bridge flags.
/// @param tunnel tunnel=* is set.
/// @param bridge bridge=* is set.
/// @return kLayerTunnel, kLayerBridge or kLayerGround; tunnel wins when both are set.
int layer_of(bool tunnel, bool bridge) {
  if (tunnel) return kLayerTunnel;
  return bridge ? kLayerBridge : kLayerGround;
}

}  // namespace

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

void Vrst_build_progress::stage(Vrst_build_stage stage, Vrst_size total) {
  m_done.store(0, std::memory_order_relaxed);
  m_total.store(total, std::memory_order_relaxed);
  m_step.store(static_cast<int>(Vrst_area_step::None), std::memory_order_relaxed);
  m_stage.store(static_cast<int>(stage), std::memory_order_relaxed);
}

void Vrst_build_progress::area(Vrst_uint32 id, int kind, Vrst_size raw_points) {
  m_area.store(id, std::memory_order_relaxed);
  m_kind.store(kind, std::memory_order_relaxed);
  m_raw.store(raw_points, std::memory_order_relaxed);
  m_simplified.store(0, std::memory_order_relaxed);
  m_clipped.store(0, std::memory_order_relaxed);
  m_step.store(static_cast<int>(Vrst_area_step::None), std::memory_order_relaxed);
  m_area_start_ns.store(now_ns(), std::memory_order_relaxed);
}

void Vrst_build_progress::area_step(Vrst_area_step step, Vrst_size points) {
  // The points the area has when a step starts are the result of the step before.
  if (step == Vrst_area_step::Clip) m_simplified.store(points, std::memory_order_relaxed);
  if (step == Vrst_area_step::Triangulate) m_clipped.store(points, std::memory_order_relaxed);
  m_step.store(static_cast<int>(step), std::memory_order_relaxed);
}

void Vrst_build_progress::triangulating(const std::vector<Vrst_vec2d>& ring) {
  const std::scoped_lock lock(m_mutex);
  m_ring = std::cref(ring);
}

void Vrst_build_progress::triangulated() {
  const std::scoped_lock lock(m_mutex);
  m_ring.reset();
}

void Vrst_build_progress::area_done() {
  const Vrst_float64 ms = static_cast<Vrst_float64>(now_ns() - m_area_start_ns.load(std::memory_order_relaxed)) / 1e6;
  const std::scoped_lock lock(m_mutex);
  if (ms <= m_slowest.ms) return;
  m_slowest = {m_area.load(std::memory_order_relaxed), m_kind.load(std::memory_order_relaxed),
               m_raw.load(std::memory_order_relaxed),  m_simplified.load(std::memory_order_relaxed),
               m_clipped.load(std::memory_order_relaxed), ms};
}

void Vrst_build_progress::reset() {
  stage(Vrst_build_stage::Idle, 0);
  const std::scoped_lock lock(m_mutex);
  m_ring.reset();
  m_slowest = {};
}

Vrst_build_snapshot Vrst_build_progress::snapshot(bool with_ring) const {
  Vrst_build_snapshot s;
  s.stage = static_cast<Vrst_build_stage>(m_stage.load(std::memory_order_relaxed));
  s.done = m_done.load(std::memory_order_relaxed);
  s.total = m_total.load(std::memory_order_relaxed);
  s.step = static_cast<Vrst_area_step>(m_step.load(std::memory_order_relaxed));
  s.area = {m_area.load(std::memory_order_relaxed), m_kind.load(std::memory_order_relaxed),
            m_raw.load(std::memory_order_relaxed),  m_simplified.load(std::memory_order_relaxed),
            m_clipped.load(std::memory_order_relaxed),
            static_cast<Vrst_float64>(now_ns() - m_area_start_ns.load(std::memory_order_relaxed)) / 1e6};
  const std::scoped_lock lock(m_mutex);
  s.slowest = m_slowest;
  // The worker holds no lock while earcut reads the ring, and earcut does not change
  // it; triangulated() waits for this copy before the ring can change.
  if (with_ring && m_ring) s.ring = m_ring->get();
  return s;
}

std::unique_ptr<Vrst_map_piece> Vrst_tile_builder::build(Vrst_tile_id id, Vrst_build_progress& progress) const {
  auto out = std::make_unique<Vrst_map_piece>();
  out->key = tile_key(id);
  const Vrst_box2d tb = tile_bounds(id);
  out->bounds = tb;
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
    progress.stage(Vrst_build_stage::Roads, items.segs.size());
    for (const Vrst_uint32 s : items.segs) {
      const Vrst_seg_attr& a = m_data.edges().attr()[s];
      project(geo.points().subspan(geo.seg_first_point()[s], geo.seg_first_point()[s + 1] - geo.seg_first_point()[s]), pts);
      simplify_line(pts, level.tolerance_m);
      const int layer = layer_of((a.flags & kSegTunnel) != 0, (a.flags & kSegBridge) != 0);
      const Vrst_uint32 attr = a.road_class | (a.flags & kSegOneway ? kTileLineOneway : 0);
      add_pieces(pts, tb.lo, keep, attr, buckets[layer][a.road_class]);
      progress.item_done();
    }
    if (m_data.has_map()) {
      const Vrst_map_layers& m = m_data.map();
      progress.stage(Vrst_build_stage::Lines, items.lines.size());
      for (const Vrst_uint32 l : items.lines) {
        const Vrst_line_attr& la = m.line_attr()[l];
        project(m.line_points(l), pts);
        simplify_line(pts, level.tolerance_m);
        const int layer = layer_of((la.flags & kLineTunnel) != 0, (la.flags & kLineBridge) != 0);
        const int style = kStyleLineKind0 + la.kind;
        add_pieces(pts, tb.lo, keep, static_cast<Vrst_uint32>(style), buckets[layer][style]);
        progress.item_done();
      }
      progress.stage(Vrst_build_stage::Areas, items.areas.size());
      for (const Vrst_uint32 ar : items.areas) {
        add_area(ar, level, tb.lo, keep, fills, buckets[kLayerGround], progress);
        progress.area_done();
        progress.item_done();
      }
    }
    progress.stage(Vrst_build_stage::Points, items.vertices.size());
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
  progress.stage(Vrst_build_stage::Finish, 0);  // packing the arrays of the piece
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
                                 Vrst_area_buckets& fills, Vrst_style_buckets& lines,
                                 Vrst_build_progress& progress) const {
  const Vrst_map_layers& m = m_data.map();
  const AreaKind kind = m.area_kind(a);
  progress.area(a, static_cast<int>(kind), m.area_outer(a).size());
  std::vector<Vrst_vec2d> pts;
  std::vector<Vrst_uint32> tris;
  if (level.tolerance_m <= 0) {
    progress.area_step(Vrst_area_step::Copy, m.area_outer(a).size());
    project(m.area_points(a), pts);
    const std::span<const Vrst_uint32> idx = m.area_indices(a);
    tris.assign(idx.begin(), idx.end());
  } else {
    project(m.area_outer(a), pts);
    progress.area_step(Vrst_area_step::Simplify, pts.size());
    pts.push_back(pts.front());  // closed, so simplification keeps both ends of the seam
    simplify_line(pts, level.tolerance_m);
    pts.pop_back();
    progress.area_step(Vrst_area_step::Clip, pts.size());
    // Only the part inside the tile is triangulated: a big forest outline is cut to the
    // stretch that crosses this tile, so earcut never sees the whole (possibly
    // self-crossing) simplified outline. Clipped after simplifying, on the few points
    // left, and only when the outline leaves the tile. Buildings are small and are not
    // clipped, so their outlines show no fake edges on the tile border.
    if (kind != AreaKind::Building && !inside(pts, keep)) pts = clip_ring(pts, keep);
    remove_repeated_points(pts, level.tolerance_m * kSamePointPart);
    if (pts.size() < 3) return;
    std::vector<std::vector<std::array<Vrst_float64, 2>>> poly(1);
    for (const Vrst_vec2d& p : pts) poly[0].push_back({p.x, p.y});
    progress.area_step(Vrst_area_step::Triangulate, pts.size());
    progress.triangulating(pts);
    tris = mapbox::earcut<Vrst_uint32>(poly);
    progress.triangulated();
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
    // Base level: pts holds the outer ring first, then the holes. Coarser levels: pts is
    // the simplified outer ring alone, shorter than area_outer().
    std::vector<Vrst_vec2d> ring = pts;
    if (level.tolerance_m <= 0) ring.resize(m.area_outer(a).size());
    ring.push_back(ring.front());
    add_pieces(ring, origin, keep, kStyleBuildingEdge, lines[kStyleBuildingEdge]);
  }
}

Vrst_tile_loader::Vrst_tile_loader(Vrst_build_fn build, int threads) : m_build(std::move(build)) {
  const auto n = static_cast<Vrst_size>(std::max(threads, 1));
  m_worker_tile.assign(n, std::nullopt);
  m_worker_since.assign(n, Vrst_clock::now());
  m_worker_built.assign(n, 0);
  for (Vrst_size i = 0; i < n; ++i) m_progress.push_back(std::make_unique<Vrst_build_progress>());
  for (int i = 0; i < static_cast<int>(n); ++i) m_workers.emplace_back([this, i] { work(i); });
}

Vrst_tile_loader::~Vrst_tile_loader() {
  {
    const std::scoped_lock lock(m_mutex);
    m_stop = true;
  }
  m_wake.notify_all();
  // std::jthread would join by itself, but Apple's libc++ does not ship it yet.
  for (std::thread& t : m_workers) t.join();
}

void Vrst_tile_loader::request(const std::vector<Vrst_tile_id>& wanted) {
  {
    const std::scoped_lock lock(m_mutex);
    std::unordered_set<Vrst_uint64> before;
    for (const Vrst_tile_id& t : m_queue) before.insert(tile_key(t));
    m_queue.clear();
    for (const Vrst_tile_id& t : wanted) {
      if (m_busy.contains(tile_key(t))) continue;
      m_queue.push_back(t);
      if (before.erase(tile_key(t)) == 0) ++m_requested;  // new in the queue
    }
    m_dropped += before.size();  // were queued, no longer wanted
  }
  m_wake.notify_all();
}

std::vector<std::unique_ptr<Vrst_map_piece>> Vrst_tile_loader::take(Vrst_size max) {
  const std::scoped_lock lock(m_mutex);
  std::vector<std::unique_ptr<Vrst_map_piece>> out;
  while (!m_done.empty() && out.size() < max) {
    m_busy.erase(m_done.front()->key);
    out.push_back(std::move(m_done.front()));
    m_done.pop_front();
  }
  return out;
}

void Vrst_tile_loader::work(int worker) {
  // A broken tile must neither end the thread (std::terminate) nor stay busy forever:
  // a failed build counts as null.
  Vrst_build_progress& progress = *m_progress[static_cast<Vrst_size>(worker)];
  const auto try_build = [this, &progress](Vrst_tile_id tile) -> std::unique_ptr<Vrst_map_piece> {
    try {
      return m_build(tile, progress);
    } catch (const std::exception&) {
      return nullptr;
    }
  };
  for (;;) {
    Vrst_tile_id id{};
    {
      std::unique_lock<std::mutex> lock(m_mutex);
      m_wake.wait(lock, [this] { return m_stop || !m_queue.empty(); });
      if (m_stop) return;
      id = m_queue.front();
      m_queue.pop_front();
      m_busy.insert(tile_key(id));
      const auto w = static_cast<Vrst_size>(worker);
      m_worker_tile[w] = id;
      m_worker_since[w] = Vrst_clock::now();
      note({Vrst_loader_event_kind::Start, id, worker, 0, 0, {}});
    }
    const auto start = std::chrono::steady_clock::now();
    progress.reset();
    std::unique_ptr<Vrst_map_piece> t = try_build(id);
    const Vrst_area_probe slowest = progress.slowest();
    progress.reset();
    if (t) {
      t->build_ms = std::chrono::duration<Vrst_float64, std::milli>(std::chrono::steady_clock::now() - start).count();
      t->worker = worker;
    }
    const std::scoped_lock lock(m_mutex);
    const auto w = static_cast<Vrst_size>(worker);
    m_worker_tile[w] = std::nullopt;
    const Vrst_float64 ms = std::chrono::duration<Vrst_float64, std::milli>(Vrst_clock::now() - start).count();
    if (t) {
      ++m_worker_built[w];
      note({Vrst_loader_event_kind::Done, id, worker, ms, 0, slowest});
      m_done.push_back(std::move(t));
    } else {
      note({Vrst_loader_event_kind::Failed, id, worker, ms, 0, slowest});
      m_busy.erase(tile_key(id));
      ++m_failed;
    }
  }
}

Vrst_loader_status Vrst_tile_loader::status() const {
  const std::scoped_lock lock(m_mutex);
  Vrst_loader_status s;
  s.queued = m_queue.size();
  s.done = m_done.size();
  s.building = m_busy.size() - m_done.size();
  s.requested = m_requested;
  s.dropped = m_dropped;
  s.failed = m_failed;
  const Vrst_clock::time_point now = Vrst_clock::now();
  for (Vrst_size w = 0; w < m_worker_tile.size(); ++w) {
    Vrst_worker_state ws;
    ws.built = m_worker_built[w];
    s.built += ws.built;
    if (m_worker_tile[w]) {
      ws.busy = true;
      ws.tile = *m_worker_tile[w];
      ws.busy_ms = std::chrono::duration<Vrst_float64, std::milli>(now - m_worker_since[w]).count();
      ws.work = m_progress[w]->snapshot(false);
    }
    s.workers.push_back(ws);
  }
  return s;
}

Vrst_build_snapshot Vrst_tile_loader::work_of(int worker) const {
  if (worker < 0 || static_cast<Vrst_size>(worker) >= m_progress.size()) return {};
  return m_progress[static_cast<Vrst_size>(worker)]->snapshot(true);
}

void Vrst_tile_loader::record_events(bool on) {
  const std::scoped_lock lock(m_mutex);
  m_record = on;
  if (!on) m_events.clear();
}

void Vrst_tile_loader::note(const Vrst_loader_event& e) {
  if (m_record && m_events.size() < kMaxEvents) m_events.push_back({e, Vrst_clock::now()});
}

std::vector<Vrst_loader_event> Vrst_tile_loader::take_events() {
  std::vector<Vrst_timed_event> taken;
  {
    const std::scoped_lock lock(m_mutex);
    taken.swap(m_events);
  }
  const Vrst_clock::time_point now = Vrst_clock::now();
  std::vector<Vrst_loader_event> out;
  out.reserve(taken.size());
  for (const Vrst_timed_event& te : taken) {
    Vrst_loader_event e = te.event;
    e.age_ms = std::chrono::duration<Vrst_float64, std::milli>(now - te.at).count();
    out.push_back(e);
  }
  return out;
}

std::vector<Vrst_tile_id> visible_tiles(const Vrst_map_area& area, int z) {
  Vrst_vec2d lo{1e300, 1e300}, hi{-1e300, -1e300};
  for (const Vrst_vec2d& w : area.corners) {
    lo = {std::min(lo.x, w.x), std::min(lo.y, w.y)};
    hi = {std::max(hi.x, w.x), std::max(hi.y, w.y)};
  }
  const Vrst_tile_id a = tile_at(z, {lo.x, hi.y}), c = tile_at(z, {hi.x, lo.y});
  std::vector<Vrst_tile_id> out;
  if (static_cast<Vrst_int64>(c.x - a.x + 1) * (c.y - a.y + 1) > 4096) return out;
  for (Vrst_int32 y = a.y; y <= c.y; ++y)
    for (Vrst_int32 x = a.x; x <= c.x; ++x) out.push_back({z, x, y});
  const Vrst_vec2d mid = area.center;
  const auto dist = [&](const Vrst_tile_id& t) {
    const Vrst_box2d b = tile_bounds(t);
    return std::hypot((b.lo.x + b.hi.x) / 2 - mid.x, (b.lo.y + b.hi.y) / 2 - mid.y);
  };
  std::ranges::sort(out, [&](const Vrst_tile_id& p, const Vrst_tile_id& q) { return dist(p) < dist(q); });
  return out;
}

}  // namespace nav
