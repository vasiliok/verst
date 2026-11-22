/// @file
/// The OSM -> nav files conversion (what the osm2nav tool does) as a library, so the
/// iPhone app can run it on an extract it downloaded.
///
/// convert_osm() reads one .osm.pbf in streaming passes and writes graph.dat,
/// edges.dat, geom.dat, names.dat and map.dat, and the address search database (house
/// numbers, streets and places gathered during the same passes, see osm_addresses.h).
/// Memory is kept low for a phone: the road
/// part keeps only road nodes and is freed before the map layers start; the map layers
/// keep only the node locations that areas and lines can need, and stream area points
/// and triangles to temporary files instead of holding them. Everything is written into
/// `<out_dir>.tmp` and renamed to out_dir at the end, so a cancelled or failed run never
/// leaves half a map.
#pragma once

#include <array>
#include <filesystem>
#include <optional>
#include <string>

#include "convert_progress.h"
#include "format.h"

namespace nav {

/// What one conversion produced and how long it took; osm2nav prints it, the app logs it.
struct Vrst_convert_stats {
  Vrst_uint64 ways_read = 0;          ///< Ways in the input.
  Vrst_size ways_kept = 0;            ///< Car road ways.
  Vrst_uint64 nodes_read = 0;         ///< Nodes in the input.
  Vrst_size road_nodes = 0;           ///< Nodes of road ways.
  Vrst_uint64 missing_refs = 0;       ///< Road node refs not found in the input.
  Vrst_uint64 ways_with_missing = 0;  ///< Road ways with such refs.
  Vrst_uint64 missing_breaks = 0;     ///< Places where a missing node splits a way.
  Vrst_uint64 loops_split = 0;        ///< Closed loops split in two.
  Vrst_uint32 vertices = 0;           ///< V.
  Vrst_uint32 segments = 0;           ///< S.
  Vrst_uint64 oneway_segments = 0;    ///< Segments with one arc.
  Vrst_uint32 arcs = 0;               ///< A.
  Vrst_size points = 0;               ///< P.
  Vrst_size names = 0;                ///< Strings in names.dat.
  std::array<Vrst_size, kPlaceKindCount> places{};  ///< Places per PlaceKind.
  std::array<Vrst_size, kAreaKindCount> areas{};    ///< Areas per AreaKind.
  Vrst_size rectangles = 0;           ///< Buildings stored as rectangles.
  Vrst_size outlines = 0;             ///< Buildings that kept their outline.
  Vrst_size triangles = 0;            ///< Area triangles.
  Vrst_size lines = 0;                ///< Rivers and railways.
  Vrst_size line_points = 0;          ///< Their points.
  Vrst_size area_nodes = 0;           ///< Node locations kept for the areas.
  Vrst_float64 road_km = 0;           ///< Total road length.
  Vrst_bbox extent{};                 ///< Extent of all nodes of the input.
  std::optional<Vrst_bbox> header_bbox;  ///< Box from the PBF header, if any.
  Vrst_size search_addresses = 0;     ///< House numbers in the search database.
  Vrst_size search_streets = 0;       ///< Streets in it (per name and city).
  Vrst_size search_places = 0;        ///< Places in it.
  Vrst_uint64 build_id = 0;           ///< Id shared by the five files and the search database.
  /// Size of each file in bytes: graph, edges, geom, names, map, search database.
  std::array<Vrst_uint64, 6> file_bytes{};
};

/// Converts one extract.
/// @param pbf .osm.pbf file (one Geofabrik extract, or several merged with merge_pbf()).
/// @param out_dir Output folder; replaced at the end of a successful run.
/// @param progress Phase and fraction while it runs; cancel() stops it.
/// @return Counts and sizes.
/// @throws Vrst_convert_cancelled after cancel(); std::runtime_error (or a libosmium
///         error) on a bad input or a write error. Either way `<out_dir>.tmp` is removed
///         and out_dir is left as it was.
Vrst_convert_stats convert_osm(const std::string& pbf, const std::filesystem::path& out_dir,
                               Vrst_convert_progress& progress);

}  // namespace nav
