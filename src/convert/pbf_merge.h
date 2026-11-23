/// @file
/// Merging several OSM extracts into one, so neighbouring Geofabrik regions (e.g.
/// Bourgogne + Franche-Comte) become one map. Used by the pbf_merge tool and the app.
///
/// Inputs must be sorted like Geofabrik extracts (nodes, then ways, then relations,
/// each by id). The merge streams through all inputs at once and writes every object
/// once: objects on a shared border are in both extracts and are written once, the
/// highest version. Memory stays small whatever the input size.
#pragma once

#include <string>
#include <vector>

#include "base/convert_progress.h"
#include "vrst_types.h"

namespace nav {

/// Result of merge_pbf().
struct Vrst_merge_stats {
  Vrst_uint64 written = 0;     ///< Objects written.
  Vrst_uint64 duplicates = 0;  ///< Objects found in more than one input, written once.
};

/// Merges sorted extracts into one sorted .osm.pbf.
/// @param inputs Two or more .osm.pbf files.
/// @param output File to write; replaced. Removed again on an error or cancel.
/// @param progress Phase Merge, fraction from the first input's read position.
/// @return Counts.
/// @throws Vrst_convert_cancelled after progress.cancel(); std::runtime_error when an
///         input is not sorted; libosmium errors on bad input.
Vrst_merge_stats merge_pbf(const std::vector<std::string>& inputs, const std::string& output,
                           Vrst_convert_progress& progress);

}  // namespace nav
