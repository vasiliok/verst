/// @file
/// An address search result in the client's terms (Vrst_map_provider::search()): no
/// storage, engine or file types, so front ends can include it.
#pragma once

#include <string>

#include "geo.h"
#include "vrst_types.h"

namespace nav {

/// What a search record or result stands for.
enum class Vrst_search_kind : Vrst_uint8 {
  Place,    ///< City, town, village or hamlet.
  Street,   ///< Street (or addr:place) of a city.
  Address,  ///< House number on a street; only in results, not a street record.
  Point,    ///< A point from a map link or coordinates; only in results.
};

/// One search result, ready to show.
struct Vrst_search_result {
  Vrst_latlon pos;                                   ///< Where it is.
  Vrst_search_kind kind = Vrst_search_kind::Street;  ///< Place, street or house number.
  std::string title;                                 ///< "12 Rue de la Liberte", "Dijon".
  std::string detail;                                ///< "Dijon 21000 - 1.2 km", "village - 8 km".
  Vrst_float64 metres_per_point = 3;                 ///< Zoom that shows it well.
};

}  // namespace nav
