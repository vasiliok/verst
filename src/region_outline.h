/// @file
/// Outline of a map region: where the downloaded data ends. Read from the Osmosis .poly
/// files Geofabrik publishes next to every extract, used to draw the region border and a
/// grid outside it (Vrst_map_renderer::set_region()).
///
/// .poly format: a name line, then rings - a ring name line (starting with '!' for a
/// hole), one "lon lat" pair per line, "END" - and a final "END".
#pragma once

#include <string_view>
#include <vector>

#include "geo.h"
#include "vrst_types.h"

namespace nav {

/// One ring of a .poly file.
struct Vrst_poly_ring {
  std::vector<Vrst_latlon> points;  ///< Points in file order; the closing point may repeat the first.
  bool hole = false;                ///< Ring name starts with '!': cut out of the ring before it.
};

/// The drawable shape of a region: filled area and border lines.
struct Vrst_region_shape {
  std::vector<Vrst_vec2d> triangles;               ///< Absolute Web Mercator, 3 per triangle.
  std::vector<std::vector<Vrst_vec2d>> outlines;   ///< One closed strip per ring (outer and holes).
};

/// Parses a .poly file.
/// @param text File content.
/// @return Rings in file order; at least one outer ring.
/// @throws std::runtime_error naming the line of the first error.
std::vector<Vrst_poly_ring> parse_poly(std::string_view text);

/// Projects rings and triangulates every outer ring with the holes that follow it.
/// Several .poly files (a merged region) give one shape: their rings are appended.
/// @param rings Rings from parse_poly(), possibly of several files.
/// @return Shape.
Vrst_region_shape region_shape(const std::vector<Vrst_poly_ring>& rings);

}  // namespace nav
