/// @file
/// Ready-to-draw content of one map piece, as a Vrst_map_provider delivers it and
/// Vrst_tile_renderer uploads it: area triangles, line pieces and vertex dots, sorted
/// into draw ranges. CPU only, no graphics API calls and no storage terms: how the
/// provider made the piece (built from nav files, read from a tile file) is hidden.
#pragma once

#include <array>
#include <vector>

#include "geo.h"
#include "map_kinds.h"
#include "vrst_types.h"

namespace nav {

/// Vertex as uploaded to the GPU: float position relative to the piece origin plus
/// one packed attribute. Kept to 12 bytes so the buffers stay small; the shader
/// decides colour and visibility from attr.
struct Vrst_gpu_point {
  /// Mercator metres relative to the piece origin. Float is enough only because of the
  /// subtraction: a piece is at most a few km wide, where the step is below 1 mm.
  Vrst_float32 x;
  Vrst_float32 y;  ///< Second component; same units as x.
  /// Fills: AreaKind. Points: 1 = dead end. One integer attribute lets one draw call
  /// colour, filter and highlight everything in the shader.
  Vrst_uint32 attr;
};
static_assert(sizeof(Vrst_gpu_point) == 12);

// --- line styles --------------------------------------------------------------

/// Number of line styles: the per-style width and colour tables have this size.
/// Styles 0..14 are the RoadClass values.
inline constexpr int kLineStyleCount = 24;
/// Style of a line kind (water, rail): kStyleLineKind0 + LineKind.
inline constexpr int kStyleLineKind0 = 16;
/// Line style of a line kind.
/// @param k Line kind.
/// @return kStyleLineKind0 + k.
constexpr int line_style(LineKind k) { return kStyleLineKind0 + static_cast<int>(k); }
/// Style of building outlines.
inline constexpr int kStyleBuildingEdge = 20;
/// Style of the debug piece borders (navview key T).
inline constexpr int kStyleTileBorder = 21;
/// Line styles bottom to top within a layer: building outlines, water, railways,
/// then roads from minor to major so major roads stay on top at junctions and at
/// piece borders.
inline constexpr Vrst_uint8 kLineStyleOrder[] = {
    kStyleBuildingEdge, line_style(LineKind::Stream), line_style(LineKind::River), line_style(LineKind::MinorRail),
    line_style(LineKind::Rail), 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0};

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
/// Coordinates are relative to the piece origin, so floats stay exact at any map size.
/// Size: 24 B (checked by static_assert).
struct Vrst_line_instance {
  Vrst_float32 x0;  ///< Start x, metres from the piece origin.
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

/// First element and count of one draw range inside a piece buffer.
struct Vrst_tile_range {
  Vrst_uint32 first = 0;  ///< First element.
  Vrst_uint32 count = 0;  ///< Number of elements.
};

/// Everything one map piece draws, made on the CPU (any thread) by a
/// Vrst_map_provider and uploaded by Vrst_tile_renderer. Arrays are sorted into draw
/// ranges.
struct Vrst_map_piece {
  /// Opaque key of the piece; the same key as in Vrst_piece_need.
  Vrst_uint64 key = 0;
  /// Ground the piece covers, absolute Mercator metres. All piece coordinates are
  /// relative to bounds.lo, the piece origin.
  Vrst_box2d bounds{};
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
  /// Time spent making the piece on its worker thread, ms (for the session log).
  Vrst_float64 build_ms = 0;
  /// Index of the worker thread that made the piece.
  int worker = 0;
};

}  // namespace nav
