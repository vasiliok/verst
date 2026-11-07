/// @file
/// Tiled map, CPU side: the Web Mercator z/x/y grid, the zoom levels, which roads,
/// areas and lines each tile holds, the tile builder and a background loader.
/// No graphics API calls: Vrst_tile_renderer (tile_renderer.h) uploads and draws
/// the Vrst_tile_data built here.
///
/// The map is cut into square tiles so that a frame costs what is on screen, not
/// what the region holds. Zoomed out, coarser levels with fewer road classes and
/// simplified outlines keep the cost down. The grid is the one OSM slippy tiles,
/// MBTiles and MapLibre use.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "map_scene.h"
#include "nav_files.h"

namespace nav {

/// Half the width of the Web Mercator square in metres: x and y run from -kMercatorHalf
/// to +kMercatorHalf.
inline constexpr Vrst_float64 kMercatorHalf = kPi * kEarthRadius;

/// Tile address in the slippy-map scheme: zoom z, column x from the west, row y from
/// the north.
struct Vrst_tile_id {
  /// Zoom level; the grid has 2^z × 2^z tiles.
  int z;
  /// Column, 0 at 180°W.
  Vrst_int32 x;
  /// Row, 0 at the northern edge of the Mercator square.
  Vrst_int32 y;
};

/// Axis-aligned box in Mercator metres (y north).
struct Vrst_box2d {
  /// South-west corner.
  Vrst_vec2d lo;
  /// North-east corner.
  Vrst_vec2d hi;
};

/// One 64-bit number per tile, for hash maps.
/// @param t Tile.
/// @return Key; unique for z < 256 and x, y < 2^28.
inline Vrst_uint64 tile_key(Vrst_tile_id t) {
  return Vrst_uint64(t.z) << 56 | Vrst_uint64(Vrst_uint32(t.x)) << 28 | Vrst_uint32(t.y);
}

/// Tile of a key.
/// @param k Key from tile_key().
/// @return Tile.
inline Vrst_tile_id tile_of_key(Vrst_uint64 k) {
  return {int(k >> 56), Vrst_int32((k >> 28) & 0xFFFFFFF), Vrst_int32(k & 0xFFFFFFF)};
}

/// Side length of a tile.
/// @param z Zoom level.
/// @return Mercator metres.
inline Vrst_float64 tile_side(int z) { return 2 * kMercatorHalf / Vrst_float64(Vrst_uint64(1) << z); }

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
  const Vrst_int32 n = Vrst_int32(Vrst_uint64(1) << z);
  const auto clamp = [n](Vrst_float64 v) { return std::clamp(Vrst_int32(std::floor(v)), Vrst_int32(0), n - 1); };
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
  /// Douglas–Peucker tolerance for lines and outlines, Mercator metres; 0 = full detail.
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

/// The tile pyramid, finest first. z14 holds everything at full detail; each
/// coarser level drops minor roads and small areas and simplifies the rest, so a
/// zoomed-out view draws about as much as a zoomed-in one.
inline constexpr Vrst_tile_level kTileLevels[] = {
    {14, 6, kAllClasses, 0, 0, true, true, true},
    {12, 24, kAllClasses & ~(bit(RoadClass::Service) | bit(RoadClass::LivingStreet)), 6, 60, true, false, false},
    {10, 96, classes_down_to(RoadClass::TertiaryLink), 24, 250, false, false, false},
    {8, 1e30, classes_down_to(RoadClass::SecondaryLink), 96, 1000, false, false, false},
};
/// Number of tile levels.
inline constexpr int kTileLevelCount = int(std::size(kTileLevels));

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

// --- line styles --------------------------------------------------------------

/// Number of line styles: the per-style width and colour tables have this size.
/// Styles 0..14 are the RoadClass values.
inline constexpr int kLineStyleCount = 24;
/// Style of a map.dat line: kStyleLineKind0 + LineKind.
inline constexpr int kStyleLineKind0 = 16;
/// Style of building outlines.
inline constexpr int kStyleBuildingEdge = 20;
/// Style of the debug tile borders (navview key T).
inline constexpr int kStyleTileBorder = 21;
/// Line styles bottom to top within a layer: building outlines, water, railways,
/// then roads from minor to major so major roads stay on top at junctions and at
/// tile borders.
inline constexpr Vrst_uint8 kLineStyleOrder[] = {
    kStyleBuildingEdge, kStyleLineKind0 + int(LineKind::Stream), kStyleLineKind0 + int(LineKind::River),
    kStyleLineKind0 + int(LineKind::MinorRail), kStyleLineKind0 + int(LineKind::Rail), 14, 13, 12, 11, 10, 9, 8, 7,
    6, 5, 4, 3, 2, 1, 0};

/// Draw layers of lines: tunnels below everything, bridges above everything.
enum TileLayer : int {
  kLayerTunnel = 0,  ///< tunnel=*: drawn first, faded
  kLayerGround = 1,  ///< everything else
  kLayerBridge = 2,  ///< bridge=*: drawn last, with a casing
  kTileLayerCount = 3
};
/// Per-instance flag bits, stored in Vrst_line_instance::attr above the style byte.
enum TileLineFlags : Vrst_uint32 {
  kTileLineOneway = 1u << 8,  ///< oneway road (oneway highlight)
};

/// One straight piece of a line, drawn as a screen-width quad (one GL instance).
/// Coordinates are relative to the tile origin, so floats stay exact at any map size.
/// Size: 24 B (checked by static_assert).
struct Vrst_line_instance {
  Vrst_float32 x0;  ///< Start x, metres from the tile origin.
  Vrst_float32 y0;  ///< Start y.
  Vrst_float32 x1;  ///< End x.
  Vrst_float32 y1;  ///< End y.
  /// Distance along the whole line at the start, metres; drives the dash pattern so
  /// dashes run on across pieces.
  Vrst_float32 d0;
  /// Line style (low byte) | TileLineFlags.
  Vrst_uint32 attr;
};
static_assert(sizeof(Vrst_line_instance) == 24);

/// First element and count of one draw range inside a tile buffer.
struct Vrst_tile_range {
  Vrst_uint32 first;  ///< First element.
  Vrst_uint32 count;  ///< Number of elements.
};

/// Everything one tile draws, built on the CPU (any thread) and uploaded by
/// Vrst_tile_renderer. Arrays are sorted into draw ranges.
struct Vrst_tile_data {
  /// The tile.
  Vrst_tile_id id;
  /// South-west tile corner in world coordinates (Mercator relative to the map
  /// center, like Vrst_view::center). All tile coordinates are relative to it.
  Vrst_vec2d origin;
  /// Area triangles, 3 vertices each; attr = AreaKind.
  std::vector<Vrst_gpu_point> fills;
  /// Range of fills per AreaKind.
  Vrst_tile_range fill_ranges[kAreaKindCount];
  /// Line pieces (roads, rivers, railways, building outlines).
  std::vector<Vrst_line_instance> lines;
  /// Range of lines per layer and style.
  Vrst_tile_range line_ranges[kTileLayerCount][kLineStyleCount];
  /// Graph vertex dots; attr 1 = dead end.
  std::vector<Vrst_gpu_point> points;
};

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
  Vrst_size dead_ends() const { return Vrst_size(std::count(m_dead_end.begin(), m_dead_end.end(), 1)); }

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

/// Builds the draw arrays of one tile from the nav files. Stateless after
/// construction and read-only on the mapped files, so several threads may call
/// build() at once.
class Vrst_tile_builder {
 public:
  /// Binds the builder to its data.
  /// @param d Opened nav files; must outlive the builder.
  /// @param index Built tile index; must outlive the builder.
  /// @param map_center Mercator map center (Vrst_map_mesh::center()); the world origin.
  Vrst_tile_builder(const Vrst_nav_data& d, const Vrst_tile_index& index, Vrst_vec2d map_center)
      : m_data(d), m_index(index), m_center(map_center) {}

  /// Builds one tile.
  /// @param id Tile; its zoom must be one of kTileLevels.
  /// @return Draw arrays; empty arrays when nothing touches the tile.
  std::unique_ptr<Vrst_tile_data> build(Vrst_tile_id id) const;

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

  /// Douglas–Peucker simplification in place; the end points are always kept.
  /// @param pts Polyline, replaced by the kept points.
  /// @param tol Tolerance in Mercator metres; 0 leaves the line unchanged.
  static void simplify(std::vector<Vrst_vec2d>& pts, Vrst_float64 tol);

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
  /// outline. Base tiles use the stored triangles; coarser levels simplify the outer
  /// ring and triangulate it again, dropping holes too small to see.
  /// @param a Area id.
  /// @param level Tile level.
  /// @param origin Tile origin, absolute Mercator.
  /// @param keep Tile box with margin.
  /// @param[out] fills Triangle vertices per AreaKind.
  /// @param[out] lines Line buckets of the ground layer, for building outlines.
  void add_area(Vrst_uint32 a, const Vrst_tile_level& level, Vrst_vec2d origin, const Vrst_box2d& keep,
                Vrst_area_buckets& fills, Vrst_style_buckets& lines) const;

  /// Nav files the tiles are built from.
  const Vrst_nav_data& m_data;
  /// Which objects touch which tile.
  const Vrst_tile_index& m_index;
  /// World origin (Mercator map center).
  Vrst_vec2d m_center;
};

/// Builds tiles on background threads, so panning into a new area never stalls a
/// frame. The front end asks for the tiles it misses every frame (request()) and
/// collects finished ones (take()); requests that are no longer wanted are dropped.
class Vrst_tile_loader {
 public:
  /// Builds one tile. When it throws or returns null, the loader counts the tile in failed().
  using Vrst_build_fn = std::function<std::unique_ptr<Vrst_tile_data>(Vrst_tile_id)>;

  /// Starts the worker threads.
  /// @param builder Tile builder; must outlive the loader.
  /// @param threads Number of worker threads, at least 1.
  Vrst_tile_loader(const Vrst_tile_builder& builder, int threads)
      : Vrst_tile_loader([&builder](Vrst_tile_id id) { return builder.build(id); }, threads) {}
  /// Starts the worker threads with any build function (tests use a failing one).
  /// @param build Builds one tile; called from the worker threads at the same time.
  /// @param threads Number of worker threads, at least 1.
  Vrst_tile_loader(Vrst_build_fn build, int threads);
  /// Stops and joins the workers; tiles being built are finished first.
  ~Vrst_tile_loader();
  Vrst_tile_loader(const Vrst_tile_loader&) = delete;             ///< Owns threads: not copyable.
  Vrst_tile_loader& operator=(const Vrst_tile_loader&) = delete;  ///< Owns threads: not copyable.

  /// Replaces the queue with the tiles wanted now, most important first. Tiles being
  /// built or already finished are not queued again.
  /// @param wanted Missing tiles, nearest to the screen center first.
  void request(const std::vector<Vrst_tile_id>& wanted);

  /// Takes finished tiles.
  /// @param max At most this many; uploading is the per-frame cost.
  /// @return Finished tiles, oldest first.
  std::vector<std::unique_ptr<Vrst_tile_data>> take(Vrst_size max);

  /// Tiles queued or being built.
  /// @return Count.
  Vrst_size pending() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_queue.size() + m_busy.size() - m_done.size();
  }

  /// Tiles whose build threw or returned null. They are dropped, so a later request()
  /// tries them again.
  /// @return Count since the loader started.
  Vrst_size failed() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_failed;
  }

 private:
  /// Worker loop: take the first queued tile, build it, hand it over.
  void work();

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
  std::deque<std::unique_ptr<Vrst_tile_data>> m_done;
  /// Set by the destructor.
  bool m_stop = false;
  /// Builds that failed; see failed().
  Vrst_size m_failed = 0;
  /// Worker threads.
  std::vector<std::thread> m_workers;
};

/// Tiles of a level that cover the screen, nearest to the screen center first.
/// Works for any rotation: the box is the one around all four screen corners.
/// @param v Camera.
/// @param fb_w Framebuffer width in px.
/// @param fb_h Framebuffer height in px.
/// @param map_center Mercator map center (world origin).
/// @param z Grid zoom.
/// @return Tiles; empty when the view covers more than 4096 tiles (never with a
///         level from level_for()).
std::vector<Vrst_tile_id> visible_tiles(const Vrst_view& v, int fb_w, int fb_h, Vrst_vec2d map_center, int z);

}  // namespace nav
