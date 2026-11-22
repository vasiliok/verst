/// @file
/// Tiled map, data side (internal to the nav map provider): the Web Mercator z/x/y
/// grid, the zoom levels, which roads, areas and lines each tile holds, the tile
/// builder and a background loader. Clients never include this file; they see the
/// tiles only as opaque Vrst_map_piece keys through map_provider.h.
///
/// The map is cut into square tiles so that a frame costs what is on screen, not
/// what the region holds. Zoomed out, coarser levels with fewer road classes and
/// simplified outlines keep the cost down. The grid is the one OSM slippy tiles,
/// MBTiles and MapLibre use.
#pragma once

#include <algorithm>
#include <atomic>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "map_piece.h"
#include "map_provider.h"
#include "map_scene.h"
#include "nav_files.h"

namespace nav {

/// Geographic point of a stored coordinate.
/// @param c Coordinate in 1e-7 degrees.
/// @return Degrees.
inline Vrst_latlon to_latlon(Vrst_coord c) { return {c.lat_e7 * 1e-7, c.lon_e7 * 1e-7}; }

/// Web Mercator projection of a stored coordinate.
/// @param c Coordinate in 1e-7 degrees.
/// @return Mercator metres (x east, y north) in double.
inline Vrst_vec2d mercator(Vrst_coord c) { return mercator(to_latlon(c)); }

/// Half the width of the Web Mercator square in metres: x and y run from -kMercatorHalf
/// to +kMercatorHalf.
inline constexpr Vrst_float64 kMercatorHalf = kPi * kEarthRadius;

/// Tile address in the slippy-map scheme: zoom z, column x from the west, row y from
/// the north.
struct Vrst_tile_id {
  /// Zoom level; the grid has 2^z x 2^z tiles.
  int z;
  /// Column, 0 at 180 degW.
  Vrst_int32 x;
  /// Row, 0 at the northern edge of the Mercator square.
  Vrst_int32 y;
};

/// One 64-bit number per tile, for hash maps.
/// @param t Tile.
/// @return Key; unique for z < 256 and x, y < 2^28.
inline Vrst_uint64 tile_key(Vrst_tile_id t) {
  return static_cast<Vrst_uint64>(t.z) << 56 | static_cast<Vrst_uint64>(static_cast<Vrst_uint32>(t.x)) << 28 |
         static_cast<Vrst_uint32>(t.y);
}

/// Tile of a key.
/// @param k Key from tile_key().
/// @return Tile.
inline Vrst_tile_id tile_of_key(Vrst_uint64 k) {
  return {static_cast<int>(k >> 56), static_cast<Vrst_int32>((k >> 28) & 0xFFFFFFF),
          static_cast<Vrst_int32>(k & 0xFFFFFFF)};
}

/// Side length of a tile.
/// @param z Zoom level.
/// @return Mercator metres.
inline Vrst_float64 tile_side(int z) { return 2 * kMercatorHalf / static_cast<Vrst_float64>(Vrst_uint64{1} << z); }

/// Bounds of a tile.
/// @param t Tile.
/// @return Box in absolute Mercator metres.
inline Vrst_box2d tile_bounds(Vrst_tile_id t) {
  const Vrst_float64 s = tile_side(t.z), x0 = -kMercatorHalf + t.x * s, y1 = kMercatorHalf - t.y * s;
  return {{x0, y1 - s}, {x0 + s, y1}};
}

/// Tile that contains a point.
/// @param z Zoom level.
/// @param m Absolute Mercator metres.
/// @return Tile, clamped to the grid.
inline Vrst_tile_id tile_at(int z, Vrst_vec2d m) {
  const Vrst_float64 s = tile_side(z);
  const Vrst_int32 n = static_cast<Vrst_int32>(static_cast<Vrst_uint64>(1) << z);
  const auto clamp = [n](Vrst_float64 v) {
    return std::clamp(static_cast<Vrst_int32>(std::floor(v)), Vrst_int32{0}, n - 1);
  };
  return {z, clamp((m.x + kMercatorHalf) / s), clamp((kMercatorHalf - m.y) / s)};
}

/// One zoom level of the tile pyramid: when it is used and what its tiles hold.
struct Vrst_tile_level {
  /// Grid zoom.
  int z;
  /// Used while the camera shows at most this many Mercator metres per framebuffer
  /// pixel; the next level takes over above it.
  Vrst_float64 max_mpp;
  /// Road classes the tiles hold (bit per RoadClass).
  Vrst_uint32 class_mask;
  /// Douglas-Peucker tolerance for lines and outlines, Mercator metres; 0 = full detail.
  Vrst_float64 tolerance_m;
  /// Areas whose bounding box is smaller than this (Mercator metres) are left out.
  Vrst_float64 min_area_m;
  /// Holds streams and ditches.
  bool streams;
  /// Holds buildings.
  bool buildings;
  /// Holds the graph vertex dots.
  bool vertices;
};

/// Mask of the road classes from motorway down to the given class, plus ferries.
/// @param last Lowest class kept (its link class too).
/// @return Class mask.
constexpr Vrst_uint32 classes_down_to(RoadClass last) {
  return ((2u << static_cast<int>(last)) - 1) | bit(RoadClass::Ferry);
}

/// Ways only walkers and cyclists use: drawn at the finest level only.
inline constexpr Vrst_uint32 kPathClasses =
    bit(RoadClass::Pedestrian) | bit(RoadClass::Footway) | bit(RoadClass::Cycleway) | bit(RoadClass::Track);

/// The tile pyramid, finest first. z14 holds everything at full detail; each
/// coarser level drops minor roads and small areas and simplifies the rest, so a
/// zoomed-out view draws about as much as a zoomed-in one.
inline constexpr Vrst_tile_level kTileLevels[] = {
    {14, 6, kAllClasses, 0, 0, true, true, true},
    {12, 24, kAllClasses & ~kPathClasses & ~(bit(RoadClass::Service) | bit(RoadClass::LivingStreet)), 6, 60, true,
     false, false},
    {10, 96, classes_down_to(RoadClass::TertiaryLink), 24, 250, false, false, false},
    {8, 1e30, classes_down_to(RoadClass::SecondaryLink), 96, 1000, false, false, false},
};
/// Number of tile levels.
inline constexpr int kTileLevelCount = static_cast<int>(std::size(kTileLevels));

/// Level for a camera zoom.
/// @param merc_m_per_px Mercator metres per framebuffer pixel (1 / Vrst_view::scale).
/// @return Index into kTileLevels.
inline int level_for(Vrst_float64 merc_m_per_px) {
  for (int i = 0; i < kTileLevelCount; ++i)
    if (merc_m_per_px <= kTileLevels[i].max_mpp) return i;
  return kTileLevelCount - 1;
}

/// Index into kTileLevels of a zoom.
/// @param z Grid zoom.
/// @return Level index, or -1 when no level uses that zoom.
inline int level_of_zoom(int z) {
  for (int i = 0; i < kTileLevelCount; ++i)
    if (kTileLevels[i].z == z) return i;
  return -1;
}

/// Content of one tile: ids of the objects that touch it.
struct Vrst_tile_items {
  std::vector<Vrst_uint32> segs;      ///< Road segment ids.
  std::vector<Vrst_uint32> areas;     ///< map.dat area ids.
  std::vector<Vrst_uint32> lines;     ///< map.dat line ids.
  std::vector<Vrst_uint32> vertices;  ///< Graph vertex ids (levels with vertices only).
};

/// Which objects touch which tile, per level. Built once after loading, so building
/// a tile reads only its own objects instead of scanning the region.
class Vrst_tile_index {
 public:
  /// Sorts every road segment, area, line and vertex into the tiles of each level it
  /// belongs to, by bounding box.
  /// @param d Opened nav files; map.dat is used when present.
  void build(const Vrst_nav_data& d);

  /// Objects of one tile.
  /// @param t Tile; its zoom must be one of kTileLevels.
  /// @return Items; empty lists when nothing touches the tile.
  const Vrst_tile_items& find(Vrst_tile_id t) const;

  /// Whether anything touches a tile.
  /// @param t Tile; its zoom must be one of kTileLevels.
  /// @return true when the index has items for it.
  bool contains(Vrst_tile_id t) const {
    const int l = level_of_zoom(t.z);
    return l >= 0 && m_tiles[l].contains(tile_key(t));
  }

  /// Whether a graph vertex is a dead end (one segment), for the red dots.
  /// @param v Vertex id.
  /// @return true for degree 1.
  bool dead_end(Vrst_uint32 v) const { return m_dead_end[v] != 0; }

  /// Number of dead-end vertices.
  /// @return Count.
  Vrst_size dead_ends() const { return static_cast<Vrst_size>(std::count(m_dead_end.begin(), m_dead_end.end(), 1)); }

  /// All non-empty tiles of a level, in no particular order.
  /// @param level Index into kTileLevels.
  /// @return Tiles.
  std::vector<Vrst_tile_id> tile_ids(int level) const;

  /// Number of non-empty tiles of a level.
  /// @param level Index into kTileLevels.
  /// @return Count.
  Vrst_size size(int level) const { return m_tiles[level].size(); }

 private:
  /// Mercator bounding box of geographic points. Mercator is monotonic in latitude
  /// and longitude, so only the two extreme corners are projected.
  /// @param pts Points.
  /// @return Box; empty input gives a box around 0,0.
  static Vrst_box2d box_of(std::span<const Vrst_coord> pts);

  /// Items per tile key, one map per level.
  std::unordered_map<Vrst_uint64, Vrst_tile_items> m_tiles[kTileLevelCount];
  /// 1 per dead-end vertex.
  std::vector<Vrst_uint8> m_dead_end;
};

/// Stage of a tile build, in build order.
enum class Vrst_build_stage : int { Idle, Roads, Lines, Areas, Points, Finish };
/// Step of the area being added (zoomed-out levels; the base level only copies triangles).
enum class Vrst_area_step : int { None, Simplify, Clip, Triangulate, Copy };

/// One area of a build, for debugging: which, how big at each step, how long.
struct Vrst_area_probe {
  Vrst_uint32 id = 0;          ///< map.dat area id.
  int kind = 0;                ///< AreaKind as int.
  Vrst_size raw_points = 0;    ///< Outer ring as stored.
  Vrst_size simplified = 0;    ///< After simplify_line().
  Vrst_size clipped = 0;       ///< After clip_ring() and remove_repeated_points().
  Vrst_float64 ms = 0;         ///< Time on this area (so far, when it is the current one).
};

/// What a tile build is doing now, read while it runs (Vrst_build_progress::snapshot()).
struct Vrst_build_snapshot {
  Vrst_build_stage stage = Vrst_build_stage::Idle;  ///< Stage.
  Vrst_size done = 0;                               ///< Items of the stage done.
  Vrst_size total = 0;                              ///< Items of the stage.
  Vrst_area_step step = Vrst_area_step::None;       ///< Areas stage: step of the current area.
  Vrst_area_probe area;                             ///< Areas stage: the current area.
  Vrst_area_probe slowest;                          ///< Slowest area of this build so far.
  std::vector<Vrst_vec2d> ring;  ///< Triangulate step: the exact earcut input (absolute Mercator), when asked for.
};

/// Progress of one tile build, written by the worker that builds it and read by any
/// thread (the loader's status()), so a build that never ends shows where it is: stage,
/// item, the area and its sizes, and on request the exact triangulation input. Counters
/// are relaxed atomics (a snapshot may mix two items, which is fine for a debug view);
/// the ring is shared under a mutex only while earcut runs on it.
class Vrst_build_progress {
 public:
  /// Starts a stage.
  /// @param stage Stage.
  /// @param total Items in it.
  void stage(Vrst_build_stage stage, Vrst_size total);
  /// One more item of the stage is done.
  void item_done() { m_done.fetch_add(1, std::memory_order_relaxed); }
  /// Starts an area.
  /// @param id map.dat area id.
  /// @param kind AreaKind as int.
  /// @param raw_points Points of its outer ring.
  void area(Vrst_uint32 id, int kind, Vrst_size raw_points);
  /// The current area moves to a step.
  /// @param step Step.
  /// @param points Points it has now (after the step before).
  void area_step(Vrst_area_step step, Vrst_size points);
  /// earcut is about to run on this ring; snapshot(true) may copy it until triangulated().
  /// @param ring Earcut input, absolute Mercator; must stay unchanged until triangulated().
  void triangulating(const std::vector<Vrst_vec2d>& ring);
  /// earcut returned.
  void triangulated();
  /// The current area is done: updates the slowest one.
  void area_done();
  /// Back to idle (the build ended).
  void reset();

  /// What the build is doing now.
  /// @param with_ring Also copy the earcut input when the build is triangulating.
  /// @return Snapshot.
  Vrst_build_snapshot snapshot(bool with_ring) const;

  /// The slowest area of the build so far (for the Done event).
  /// @return Probe.
  Vrst_area_probe slowest() const {
    const std::scoped_lock lock(m_mutex);
    return m_slowest;
  }

 private:
  /// Clock of the area timing.
  using Vrst_clock = std::chrono::steady_clock;
  /// Nanoseconds now on Vrst_clock, for the atomics.
  /// @return Nanoseconds.
  static Vrst_int64 now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(Vrst_clock::now().time_since_epoch()).count();
  }

  std::atomic<int> m_stage{0};                  ///< Vrst_build_stage.
  std::atomic<Vrst_size> m_done{0};             ///< Items of the stage done.
  std::atomic<Vrst_size> m_total{0};            ///< Items of the stage.
  std::atomic<int> m_step{0};                   ///< Vrst_area_step.
  std::atomic<Vrst_uint32> m_area{0};           ///< Current area id.
  std::atomic<int> m_kind{0};                   ///< Current area kind.
  std::atomic<Vrst_size> m_raw{0};              ///< Current area: stored points.
  std::atomic<Vrst_size> m_simplified{0};       ///< Current area: after simplify.
  std::atomic<Vrst_size> m_clipped{0};          ///< Current area: after clip.
  std::atomic<Vrst_int64> m_area_start_ns{0};   ///< Current area: start time.
  /// Guards m_ring and m_slowest.
  mutable std::mutex m_mutex;
  /// Earcut input while earcut runs; empty otherwise.
  std::optional<std::reference_wrapper<const std::vector<Vrst_vec2d>>> m_ring;
  /// Slowest area of the build so far.
  Vrst_area_probe m_slowest;
};

/// Builds the draw arrays of one tile from the nav files. Stateless after
/// construction and read-only on the mapped files, so several threads may call
/// build() at once.
class Vrst_tile_builder {
 public:
  /// Binds the builder to its data.
  /// @param d Opened nav files; must outlive the builder.
  /// @param index Built tile index; must outlive the builder.
  Vrst_tile_builder(const Vrst_nav_data& d, const Vrst_tile_index& index) : m_data(d), m_index(index) {}

  /// Builds one tile.
  /// @param id Tile; its zoom must be one of kTileLevels.
  /// @return Draw arrays; empty arrays when nothing touches the tile.
  std::unique_ptr<Vrst_map_piece> build(Vrst_tile_id id) const {
    Vrst_build_progress unused;
    return build(id, unused);
  }

  /// Builds one tile and reports how far it got while it runs.
  /// @param id Tile; its zoom must be one of kTileLevels.
  /// @param progress Updated during the build (the caller resets it).
  /// @return Draw arrays; empty arrays when nothing touches the tile.
  std::unique_ptr<Vrst_map_piece> build(Vrst_tile_id id, Vrst_build_progress& progress) const;

 private:
  /// Line instances of one layer, one list per line style.
  using Vrst_style_buckets = std::array<std::vector<Vrst_line_instance>, kLineStyleCount>;
  /// Fill triangle vertices, one list per AreaKind.
  using Vrst_area_buckets = std::array<std::vector<Vrst_gpu_point>, kAreaKindCount>;

  /// Projects geographic points to absolute Mercator.
  /// @param in Points.
  /// @param[out] out Receives the projected points (replaced).
  static void project(std::span<const Vrst_coord> in, std::vector<Vrst_vec2d>& out) {
    out.resize(in.size());
    for (Vrst_size i = 0; i < in.size(); ++i) out[i] = mercator(in[i]);
  }

  /// Whether a segment's bounding box touches a box.
  /// @param a Segment start.
  /// @param b Segment end.
  /// @param box Box.
  /// @return true if they overlap.
  static bool touches(Vrst_vec2d a, Vrst_vec2d b, const Vrst_box2d& box) {
    return std::max(a.x, b.x) >= box.lo.x && std::min(a.x, b.x) <= box.hi.x && std::max(a.y, b.y) >= box.lo.y &&
           std::min(a.y, b.y) <= box.hi.y;
  }

  /// Adds the pieces of a polyline that touch the tile.
  /// @param pts Polyline, absolute Mercator.
  /// @param origin Tile origin, absolute Mercator.
  /// @param keep Tile box with margin.
  /// @param attr Style and flags for every piece.
  /// @param[out] out Receives the instances.
  static void add_pieces(const std::vector<Vrst_vec2d>& pts, Vrst_vec2d origin, const Vrst_box2d& keep, Vrst_uint32 attr,
                         std::vector<Vrst_line_instance>& out);

  /// Adds one map.dat area: its triangles that touch the tile and, for buildings, the
  /// outline. Base tiles use the stored triangles. Coarser levels take the outer ring
  /// (holes are dropped), simplify it, clip it to the tile (keep box) unless it is a
  /// building or already inside, and triangulate it again: a big forest outline is cut
  /// to the part inside one tile, so earcut never sees the whole outline.
  /// @param a Area id.
  /// @param level Tile level.
  /// @param origin Tile origin, absolute Mercator.
  /// @param keep Tile box with margin.
  /// @param[out] fills Triangle vertices per AreaKind.
  /// @param[out] lines Line buckets of the ground layer, for building outlines.
  /// @param progress Area, sizes and step for debugging.
  void add_area(Vrst_uint32 a, const Vrst_tile_level& level, Vrst_vec2d origin, const Vrst_box2d& keep,
                Vrst_area_buckets& fills, Vrst_style_buckets& lines, Vrst_build_progress& progress) const;

  /// Nav files the tiles are built from.
  const Vrst_nav_data& m_data;
  /// Which objects touch which tile.
  const Vrst_tile_index& m_index;
};

/// What a loader worker did, for the session log (Vrst_tile_loader::take_events()).
enum class Vrst_loader_event_kind { Start, Done, Failed };

/// One worker event of the tile loader.
struct Vrst_loader_event {
  Vrst_loader_event_kind kind = Vrst_loader_event_kind::Start;  ///< What happened.
  Vrst_tile_id tile{};                                          ///< The tile.
  int worker = 0;                                               ///< Worker index.
  Vrst_float64 build_ms = 0;                                    ///< Done / Failed: build time.
  Vrst_float64 age_ms = 0;  ///< How long before take_events() it happened.
  Vrst_area_probe slowest;  ///< Done: the slowest area of the build.
};

/// What one loader worker is doing now.
struct Vrst_worker_state {
  bool busy = false;         ///< Building a tile.
  Vrst_tile_id tile{};       ///< The tile, when busy.
  Vrst_float64 busy_ms = 0;  ///< Time spent on it so far, when busy.
  Vrst_size built = 0;       ///< Tiles this worker finished since start.
  Vrst_build_snapshot work;  ///< When busy: where the build is (without the ring).
};

/// Snapshot of the tile loader queue, for the session log (Vrst_tile_loader::status()).
struct Vrst_loader_status {
  Vrst_size queued = 0;     ///< Tiles waiting for a worker.
  Vrst_size building = 0;   ///< Tiles being built now.
  Vrst_size done = 0;       ///< Tiles finished, not taken yet.
  Vrst_size requested = 0;  ///< Tiles added to the queue since start.
  Vrst_size dropped = 0;    ///< Queued tiles removed by request() before a worker took them.
  Vrst_size built = 0;      ///< Tiles finished since start.
  Vrst_size failed = 0;     ///< Builds that failed since start.
  std::vector<Vrst_worker_state> workers;  ///< One entry per worker thread.
};

/// Builds tiles on background threads, so panning into a new area never stalls a
/// frame. The front end asks for the tiles it misses every frame (request()) and
/// collects finished ones (take()); requests that are no longer wanted are dropped.
class Vrst_tile_loader {
 public:
  /// Builds one tile. When it throws or returns null, the loader counts the tile in failed().
  using Vrst_build_fn = std::function<std::unique_ptr<Vrst_map_piece>(Vrst_tile_id, Vrst_build_progress&)>;

  /// Starts the worker threads.
  /// @param builder Tile builder; must outlive the loader.
  /// @param threads Number of worker threads, at least 1.
  Vrst_tile_loader(const Vrst_tile_builder& builder, int threads)
      : Vrst_tile_loader([&builder](Vrst_tile_id id, Vrst_build_progress& p) { return builder.build(id, p); },
                         threads) {}
  /// Starts the worker threads with any build function (tests use a failing one).
  /// @param build Builds one tile; called from the worker threads at the same time.
  /// @param threads Number of worker threads, at least 1.
  Vrst_tile_loader(Vrst_build_fn build, int threads);
  /// Stops and joins the workers; tiles being built are finished first.
  ~Vrst_tile_loader();
  Vrst_tile_loader(const Vrst_tile_loader&) = delete;             ///< Owns threads: not copyable.
  Vrst_tile_loader& operator=(const Vrst_tile_loader&) = delete;  ///< Owns threads: not copyable.
  Vrst_tile_loader(Vrst_tile_loader&&) = delete;                  ///< Workers point at this: not movable.
  Vrst_tile_loader& operator=(Vrst_tile_loader&&) = delete;       ///< Workers point at this: not movable.

  /// Replaces the queue with the tiles wanted now, most important first. Tiles being
  /// built or already finished are not queued again.
  /// @param wanted Missing tiles, nearest to the screen center first.
  void request(const std::vector<Vrst_tile_id>& wanted);

  /// Takes finished tiles.
  /// @param max At most this many; uploading is the per-frame cost.
  /// @return Finished tiles, oldest first.
  std::vector<std::unique_ptr<Vrst_map_piece>> take(Vrst_size max);

  /// Tiles queued or being built.
  /// @return Count.
  Vrst_size pending() const {
    const std::scoped_lock lock(m_mutex);
    return m_queue.size() + m_busy.size() - m_done.size();
  }

  /// Tiles whose build threw or returned null. They are dropped, so a later request()
  /// tries them again.
  /// @return Count since the loader started.
  Vrst_size failed() const {
    const std::scoped_lock lock(m_mutex);
    return m_failed;
  }

  /// Snapshot of the queue and of every worker, for the session log.
  /// @return Status now.
  Vrst_loader_status status() const;

  /// Where one worker's build is now, with the exact earcut input when it is
  /// triangulating (to reproduce a build that never ends).
  /// @param worker Worker index.
  /// @return Snapshot; Idle for a bad index or an idle worker.
  Vrst_build_snapshot work_of(int worker) const;

  /// Turns recording of worker events on or off (off by default, so a front end that
  /// never calls take_events() does not collect them).
  /// @param on Record events.
  void record_events(bool on);
  /// Takes the worker events recorded since the last call.
  /// @return Events, oldest first, with their age.
  std::vector<Vrst_loader_event> take_events();

 private:
  /// Clock of the worker timing.
  using Vrst_clock = std::chrono::steady_clock;
  /// A worker event with its time, before take_events() turns it into an age.
  struct Vrst_timed_event {
    Vrst_loader_event event;     ///< The event.
    Vrst_clock::time_point at;   ///< When it happened.
  };
  /// Records a worker event when recording is on; the caller holds m_mutex.
  /// @param e Event.
  void note(const Vrst_loader_event& e);

  /// Worker loop: take the first queued tile, build it, hand it over.
  /// @param worker Index of this worker thread, stored in Vrst_map_piece::worker.
  void work(int worker);

  /// Build function shared by the workers.
  Vrst_build_fn m_build;
  /// Guards the queues.
  mutable std::mutex m_mutex;
  /// Wakes workers when tiles are queued or on stop.
  std::condition_variable m_wake;
  /// Tiles to build, most important first.
  std::deque<Vrst_tile_id> m_queue;
  /// Keys of tiles being built or finished but not taken; never queued twice.
  std::unordered_set<Vrst_uint64> m_busy;
  /// Finished tiles waiting for take().
  std::deque<std::unique_ptr<Vrst_map_piece>> m_done;
  /// Set by the destructor.
  bool m_stop = false;
  /// Builds that failed; see failed().
  Vrst_size m_failed = 0;
  /// Tiles added to the queue since start; see Vrst_loader_status::requested.
  Vrst_size m_requested = 0;
  /// Queued tiles dropped by request(); see Vrst_loader_status::dropped.
  Vrst_size m_dropped = 0;
  /// Per worker: tile being built, or nullopt when idle.
  std::vector<std::optional<Vrst_tile_id>> m_worker_tile;
  /// Per worker: when the current build began.
  std::vector<Vrst_clock::time_point> m_worker_since;
  /// Per worker: tiles finished.
  std::vector<Vrst_size> m_worker_built;
  /// Per worker: progress of the build it runs (not movable, so held by pointer owner).
  std::vector<std::unique_ptr<Vrst_build_progress>> m_progress;
  /// Record worker events (record_events()).
  bool m_record = false;
  /// Recorded worker events not taken yet; capped at kMaxEvents.
  std::vector<Vrst_timed_event> m_events;
  /// Most events kept between two take_events() calls.
  static constexpr Vrst_size kMaxEvents = 4096;
  /// Worker threads.
  std::vector<std::thread> m_workers;
};

/// Tiles of a level that cover the screen, nearest to the screen center first.
/// Works for any rotation: the box is the one around all four screen corners.
/// @param area What the screen shows (absolute Mercator).
/// @param z Grid zoom.
/// @return Tiles; empty when the view covers more than 4096 tiles (never with a
///         level from level_for()).
std::vector<Vrst_tile_id> visible_tiles(const Vrst_map_area& area, int z);

}  // namespace nav
