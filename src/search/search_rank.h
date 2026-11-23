/// @file
/// The address search on top of any engine: parse the query, ask the index for
/// candidate streets and places, rank them and turn them into results with a title, a
/// detail line, a position and a zoom. Engine-independent, so every engine behind
/// Vrst_search_index gives the same results.
#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "geometry/geo.h"
#include "search/geo_link.h"
#include "search/search_engine.h"
#include "provider/search_result.h"

namespace nav {

/// Searches a region.
///
/// Ranking: with a house number, streets that have it first; then candidates whose
/// every word matches a whole word (before ones that match only as prefixes); for a
/// one-word query places before streets, bigger places first; then nearest to `near`.
/// A street without the asked number is still listed, its detail saying so.
/// @param index The region's search database.
/// @param text What the user typed.
/// @param near Map center or GPS position, for distances and ranking; nullopt = none.
/// @param max At most this many results.
/// @return Results, best first; empty when the text has no word.
std::vector<Vrst_search_result> search_index(const Vrst_search_index& index, std::string_view text,
                                             std::optional<Vrst_latlon> near, Vrst_size max);

/// The result of a point read from a map link or coordinates (parse_geo_link()).
/// @param link The point.
/// @param near Map center, for the distance; nullopt = none.
/// @return Result titled with the link's place name, or "Point"; detail with the
///         coordinates (and "map center of the link" when the link had no pin).
Vrst_search_result link_result(const Vrst_geo_link& link, std::optional<Vrst_latlon> near);

/// Searches a place text from a map link without coordinates ("Cafe du Port, 12 Rue de
/// la Liberte, 21000 Dijon, France"): the part with
/// the postcode is the city; each other part is searched with the city, in order, and
/// the first street or house number whose words all match is the answer. Without one,
/// the city itself.
/// @param index The region's search database.
/// @param text The place text (parse_link_address()).
/// @param near Map center, for distances; nullopt = none.
/// @return At most one result, its detail saying it comes from the link's address.
std::vector<Vrst_search_result> search_link_address(const Vrst_search_index& index, std::string_view text,
                                                    std::optional<Vrst_latlon> near);

}  // namespace nav
