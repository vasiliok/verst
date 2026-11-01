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

#include <mapbox/earcut.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <condition_variable>
#include <deque>
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
  void build(const Vrst_nav_data& d) {
    for (auto& m : m_tiles) m.clear();
    auto add = [&](int level, const Vrst_box2d& b, auto member, Vrst_uint32 id) {
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
        const bool stream = m.line_attr()[i].kind == Vrst_uint8(LineKind::Stream);
        for (int l = 0; l < kTileLevelCount; ++l)
          if (!stream || kTileLevels[l].streams) add(l, b, &Vrst_tile_items::lines, i);
      }
    }
    const Vrst_graph& g = d.graph();
    std::vector<Vrst_uint8> degree(g.num_vertices(), 0);
    for (const Vrst_seg_vertices& sv : e.vertices()) {
      degree[sv.a] = Vrst_uint8(std::min(degree[sv.a] + 1, 2));
      degree[sv.b] = Vrst_uint8(std::min(degree[sv.b] + 1, 2));
    }
    m_dead_end.assign(g.num_vertices(), 0);
    for (Vrst_uint32 v = 0; v < g.num_vertices(); ++v) {
      m_dead_end[v] = degree[v] == 1;
      const Vrst_vec2d p = mercator(g.vertex_coord()[v]);
      for (int l = 0; l < kTileLevelCount; ++l)
        if (kTileLevels[l].vertices) add(l, {p, p}, &Vrst_tile_items::vertices, v);
    }
  }

  /// Objects of one tile.
  /// @param t Tile; its zoom must be one of kTileLevels.
  /// @return Items, or nullptr when nothing touches the tile.
  const Vrst_tile_items* find(Vrst_tile_id t) const {
    const int l = level_of_zoom(t.z);
    if (l < 0) return nullptr;
    const auto it = m_tiles[l].find(tile_key(t));
    return it == m_tiles[l].end() ? nullptr : &it->second;
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
  std::vector<Vrst_tile_id> tile_ids(int level) const {
    std::vector<Vrst_tile_id> out;
    out.reserve(m_tiles[level].size());
    for (const auto& [k, items] : m_tiles[level]) out.push_back(tile_of_key(k));
    return out;
  }

  /// Number of non-empty tiles of a level.
  /// @param level Index into kTileLevels.
  /// @return Count.
  Vrst_size size(int level) const { return m_tiles[level].size(); }

 private:
  /// Mercator bounding box of geographic points. Mercator is monotonic in latitude
  /// and longitude, so only the two extreme corners are projected.
  /// @param pts Points.
  /// @return Box; empty input gives a box around 0,0.
  static Vrst_box2d box_of(std::span<const Vrst_coord> pts) {
    if (pts.empty()) return {{0, 0}, {0, 0}};
    Vrst_coord lo = pts[0], hi = pts[0];
    for (const Vrst_coord& c : pts) {
      lo = {std::min(lo.lat_e7, c.lat_e7), std::min(lo.lon_e7, c.lon_e7)};
      hi = {std::max(hi.lat_e7, c.lat_e7), std::max(hi.lon_e7, c.lon_e7)};
    }
    return {mercator(lo), mercator(hi)};
  }

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
  std::unique_ptr<Vrst_tile_data> build(Vrst_tile_id id) const {
    auto out = std::make_unique<Vrst_tile_data>();
    out->id = id;
    const Vrst_box2d tb = tile_bounds(id);
    out->origin = {tb.lo.x - m_center.x, tb.lo.y - m_center.y};
    const Vrst_tile_items* items = m_index.find(id);
    const Vrst_tile_level& level = kTileLevels[level_of_zoom(id.z)];
    // Pieces a little outside the tile are kept, so wide lines meet across the border.
    const Vrst_float64 margin = tile_side(id.z) * 0.02;
    const Vrst_box2d keep{{tb.lo.x - margin, tb.lo.y - margin}, {tb.hi.x + margin, tb.hi.y + margin}};
    std::vector<Vrst_line_instance> buckets[kTileLayerCount][kLineStyleCount];
    std::vector<Vrst_gpu_point> fills[kAreaKindCount];
    if (items) {
      std::vector<Vrst_vec2d> pts;
      // Roads.
      const Vrst_geom& geo = m_data.geom();
      for (const Vrst_uint32 s : items->segs) {
        const Vrst_seg_attr& a = m_data.edges().attr()[s];
        project(geo.points().subspan(geo.seg_first_point()[s], geo.seg_first_point()[s + 1] - geo.seg_first_point()[s]), &pts);
        simplify(&pts, level.tolerance_m);
        const int layer = a.flags & kSegTunnel ? kLayerTunnel : a.flags & kSegBridge ? kLayerBridge : kLayerGround;
        const Vrst_uint32 attr = a.road_class | (a.flags & kSegOneway ? kTileLineOneway : 0);
        add_pieces(pts, tb.lo, keep, attr, &buckets[layer][a.road_class]);
      }
      if (m_data.has_map()) {
        const Vrst_map_layers& m = m_data.map();
        for (const Vrst_uint32 l : items->lines) {
          const Vrst_line_attr& la = m.line_attr()[l];
          project(m.line_points(l), &pts);
          simplify(&pts, level.tolerance_m);
          const int layer = la.flags & kLineTunnel ? kLayerTunnel : la.flags & kLineBridge ? kLayerBridge : kLayerGround;
          const int style = kStyleLineKind0 + la.kind;
          add_pieces(pts, tb.lo, keep, Vrst_uint32(style), &buckets[layer][style]);
        }
        for (const Vrst_uint32 ar : items->areas) add_area(ar, level, tb.lo, keep, fills, buckets[kLayerGround]);
      }
      // Graph vertex dots, dead ends last so they are drawn on top.
      for (int pass = 0; pass < 2; ++pass)
        for (const Vrst_uint32 v : items->vertices) {
          const bool dead = m_index.dead_end(v);
          if (dead != (pass == 1)) continue;
          const Vrst_vec2d p = mercator(m_data.graph().vertex_coord()[v]);
          out->points.push_back({Vrst_float32(p.x - tb.lo.x), Vrst_float32(p.y - tb.lo.y), Vrst_uint32(dead)});
        }
    }
    for (int k = 0; k < kAreaKindCount; ++k) {
      out->fill_ranges[k] = {Vrst_uint32(out->fills.size()), Vrst_uint32(fills[k].size())};
      out->fills.insert(out->fills.end(), fills[k].begin(), fills[k].end());
    }
    for (int layer = 0; layer < kTileLayerCount; ++layer)
      for (int style = 0; style < kLineStyleCount; ++style) {
        const std::vector<Vrst_line_instance>& b = buckets[layer][style];
        out->line_ranges[layer][style] = {Vrst_uint32(out->lines.size()), Vrst_uint32(b.size())};
        out->lines.insert(out->lines.end(), b.begin(), b.end());
      }
    return out;
  }

 private:
  /// Projects geographic points to absolute Mercator.
  /// @param in Points.
  /// @param[out] out Receives the projected points (replaced).
  static void project(std::span<const Vrst_coord> in, std::vector<Vrst_vec2d>* out) {
    out->resize(in.size());
    for (Vrst_size i = 0; i < in.size(); ++i) (*out)[i] = mercator(in[i]);
  }

  /// Douglas–Peucker simplification in place; the end points are always kept.
  /// @param pts Polyline, replaced by the kept points.
  /// @param tol Tolerance in Mercator metres; 0 leaves the line unchanged.
  static void simplify(std::vector<Vrst_vec2d>* pts, Vrst_float64 tol) {
    if (tol <= 0 || pts->size() <= 2) return;
    const std::vector<Vrst_vec2d>& p = *pts;
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
        stack.push_back({i, at});
        stack.push_back({at, j});
      }
    }
    Vrst_size n = 0;
    for (Vrst_size k = 0; k < p.size(); ++k)
      if (keep[k]) (*pts)[n++] = p[k];
    pts->resize(n);
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
                         std::vector<Vrst_line_instance>* out) {
    Vrst_float64 dist = 0;
    for (Vrst_size i = 0; i + 1 < pts.size(); ++i) {
      const Vrst_vec2d a = pts[i], b = pts[i + 1];
      if (touches(a, b, keep))
        out->push_back({Vrst_float32(a.x - origin.x), Vrst_float32(a.y - origin.y), Vrst_float32(b.x - origin.x),
                        Vrst_float32(b.y - origin.y), Vrst_float32(dist), attr});
      dist += std::hypot(b.x - a.x, b.y - a.y);
    }
  }

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
                std::vector<Vrst_gpu_point>* fills, std::vector<Vrst_line_instance>* lines) const {
    const Vrst_map_layers& m = m_data.map();
    const AreaKind kind = m.area_kind(a);
    std::vector<Vrst_vec2d> pts;
    std::vector<Vrst_uint32> tris;
    if (level.tolerance_m <= 0) {
      project(m.area_points(a), &pts);
      const std::span<const Vrst_uint32> idx = m.area_indices(a);
      tris.assign(idx.begin(), idx.end());
    } else {
      project(m.area_outer(a), &pts);
      pts.push_back(pts.front());  // closed, so simplification keeps both ends of the seam
      simplify(&pts, level.tolerance_m);
      pts.pop_back();
      if (pts.size() < 3) return;
      std::vector<std::vector<std::array<Vrst_float64, 2>>> poly(1);
      for (const Vrst_vec2d& p : pts) poly[0].push_back({p.x, p.y});
      tris = mapbox::earcut<Vrst_uint32>(poly);
    }
    std::vector<Vrst_gpu_point>& out = fills[int(kind)];
    for (Vrst_size t = 0; t + 2 < tris.size(); t += 3) {
      const Vrst_vec2d p0 = pts[tris[t]], p1 = pts[tris[t + 1]], p2 = pts[tris[t + 2]];
      const Vrst_box2d tb{{std::min({p0.x, p1.x, p2.x}), std::min({p0.y, p1.y, p2.y})},
                          {std::max({p0.x, p1.x, p2.x}), std::max({p0.y, p1.y, p2.y})}};
      if (!touches(tb.lo, tb.hi, keep)) continue;
      for (const Vrst_vec2d& p : {p0, p1, p2})
        out.push_back({Vrst_float32(p.x - origin.x), Vrst_float32(p.y - origin.y), Vrst_uint32(kind)});
    }
    if (kind == AreaKind::Building) {
      std::vector<Vrst_vec2d> ring(pts.begin(), pts.begin() + m.area_outer(a).size());
      ring.push_back(ring.front());
      add_pieces(ring, origin, keep, kStyleBuildingEdge, &lines[kStyleBuildingEdge]);
    }
  }

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
  /// Starts the worker threads.
  /// @param builder Tile builder; must outlive the loader.
  /// @param threads Number of worker threads, at least 1.
  Vrst_tile_loader(const Vrst_tile_builder& builder, int threads) : m_builder(builder) {
    for (int i = 0; i < std::max(threads, 1); ++i) m_workers.emplace_back([this] { work(); });
  }
  /// Stops and joins the workers; tiles being built are finished first.
  ~Vrst_tile_loader() {
    {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_stop = true;
    }
    m_wake.notify_all();
    for (std::thread& t : m_workers) t.join();
  }
  Vrst_tile_loader(const Vrst_tile_loader&) = delete;             ///< Owns threads: not copyable.
  Vrst_tile_loader& operator=(const Vrst_tile_loader&) = delete;  ///< Owns threads: not copyable.

  /// Replaces the queue with the tiles wanted now, most important first. Tiles being
  /// built or already finished are not queued again.
  /// @param wanted Missing tiles, nearest to the screen center first.
  void request(const std::vector<Vrst_tile_id>& wanted) {
    {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_queue.clear();
      for (const Vrst_tile_id& t : wanted)
        if (!m_busy.count(tile_key(t))) m_queue.push_back(t);
    }
    m_wake.notify_all();
  }

  /// Takes finished tiles.
  /// @param max At most this many; uploading is the per-frame cost.
  /// @return Finished tiles, oldest first.
  std::vector<std::unique_ptr<Vrst_tile_data>> take(Vrst_size max) {
    std::lock_guard<std::mutex> lock(m_mutex);
    std::vector<std::unique_ptr<Vrst_tile_data>> out;
    while (!m_done.empty() && out.size() < max) {
      m_busy.erase(tile_key(m_done.front()->id));
      out.push_back(std::move(m_done.front()));
      m_done.pop_front();
    }
    return out;
  }

  /// Tiles queued or being built.
  /// @return Count.
  Vrst_size pending() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_queue.size() + m_busy.size() - m_done.size();
  }

 private:
  /// Worker loop: take the first queued tile, build it, hand it over.
  void work() {
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
      std::unique_ptr<Vrst_tile_data> t = m_builder.build(id);
      std::lock_guard<std::mutex> lock(m_mutex);
      m_done.push_back(std::move(t));
    }
  }

  /// Builder shared by the workers.
  const Vrst_tile_builder& m_builder;
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
inline std::vector<Vrst_tile_id> visible_tiles(const Vrst_view& v, int fb_w, int fb_h, Vrst_vec2d map_center, int z) {
  Vrst_vec2d lo{1e300, 1e300}, hi{-1e300, -1e300};
  for (const auto& [sx, sy] : {std::pair{0, 0}, {fb_w, 0}, {0, fb_h}, {fb_w, fb_h}}) {
    const Vrst_vec2d w = screen_to_world(v, fb_w, fb_h, sx, sy);
    lo = {std::min(lo.x, w.x + map_center.x), std::min(lo.y, w.y + map_center.y)};
    hi = {std::max(hi.x, w.x + map_center.x), std::max(hi.y, w.y + map_center.y)};
  }
  const Vrst_tile_id a = tile_at(z, {lo.x, hi.y}), c = tile_at(z, {hi.x, lo.y});
  std::vector<Vrst_tile_id> out;
  if (Vrst_int64(c.x - a.x + 1) * (c.y - a.y + 1) > 4096) return out;
  for (Vrst_int32 y = a.y; y <= c.y; ++y)
    for (Vrst_int32 x = a.x; x <= c.x; ++x) out.push_back({z, x, y});
  const Vrst_vec2d mid{v.center.x + map_center.x, v.center.y + map_center.y};
  const auto dist = [&](const Vrst_tile_id& t) {
    const Vrst_box2d b = tile_bounds(t);
    return std::hypot((b.lo.x + b.hi.x) / 2 - mid.x, (b.lo.y + b.hi.y) / 2 - mid.y);
  };
  std::sort(out.begin(), out.end(), [&](const Vrst_tile_id& p, const Vrst_tile_id& q) { return dist(p) < dist(q); });
  return out;
}

}  // namespace nav
