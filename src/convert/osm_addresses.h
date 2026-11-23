/// @file
/// Vrst_address_collector: gathers the searchable names of an extract while the
/// converter reads it (house numbers, named roads, places) and writes them through a
/// Vrst_search_index_writer, whatever engine that is.
///
/// Nothing is read twice: address nodes and places come from the road pass over the
/// nodes, named roads from the road segments, building addresses from the map-layer
/// pass that assembles every building.
#pragma once

#include <string_view>
#include <vector>

#include "storage/format.h"
#include "search/search_engine.h"
#include "base/string_pool.h"
#include "vrst_types.h"

namespace nav {

/// The addr:* tags of one OSM object; views valid during the call only.
struct Vrst_address_tags {
  std::string_view number;    ///< addr:housenumber.
  std::string_view street;    ///< addr:street.
  std::string_view place;     ///< addr:place, used when there is no street (hamlets).
  std::string_view city;      ///< addr:city; when empty the nearest city, town or village is used.
  std::string_view postcode;  ///< addr:postcode.
};

/// Counts and size of a written search database.
struct Vrst_search_stats {
  Vrst_size addresses = 0;  ///< House numbers kept.
  Vrst_size streets = 0;    ///< Streets (per name and city).
  Vrst_size places = 0;     ///< Cities, towns, villages and hamlets.
  Vrst_uint64 bytes = 0;    ///< Size of the database files.
};

/// Collects addresses, roads and places, then writes them as streets and house numbers.
///
/// About 24 bytes per address or road segment in memory (a position and four interned
/// strings); the strings themselves are kept once.
class Vrst_address_collector {
 public:
  /// Adds an address point or building. Objects without a number, or without both a
  /// street and an addr:place, are ignored.
  /// @param pos Position (the building's center for a building).
  /// @param tags Its addr:* tags.
  void add_address(Vrst_coord pos, const Vrst_address_tags& tags);
  /// Adds a named road segment, so a street without house numbers is found too.
  /// @param pos A point of the segment.
  /// @param name Road name; ignored when empty.
  void add_road(Vrst_coord pos, std::string_view name);
  /// Adds a place (also the city of addresses without addr:city).
  /// @param pos Place node.
  /// @param kind City, town, village or hamlet.
  /// @param name Name; ignored when empty.
  void add_place(Vrst_coord pos, PlaceKind kind, std::string_view name);

  /// House numbers added so far.
  /// @return Count.
  Vrst_size addresses() const { return m_numbers; }

  /// Merges everything into streets (one per name and city, at the point nearest to the
  /// middle of its addresses and roads), sorts them and writes places, streets and
  /// house numbers.
  /// @param writer Engine writer.
  /// @param build_id Id of the nav files.
  /// @return Counts and size.
  /// @throws std::runtime_error on a write error.
  Vrst_search_stats write(Vrst_search_index_writer& writer, Vrst_uint64 build_id);

 private:
  /// One address or road segment.
  struct Vrst_row {
    Vrst_coord pos;         ///< Position.
    Vrst_uint32 number;     ///< number_key() in m_strings; 0 for a road segment.
    Vrst_uint32 street;     ///< Street (or addr:place) name.
    Vrst_uint32 city;       ///< addr:city; 0 = take the nearest place.
    Vrst_uint32 postcode;   ///< addr:postcode; 0 = none.
  };
  /// One place.
  struct Vrst_place_row {
    Vrst_coord pos;     ///< Place node.
    PlaceKind kind;     ///< Kind.
    Vrst_uint32 name;   ///< Name in m_strings.
  };

  /// All strings of the rows and places.
  Vrst_string_pool m_strings;
  /// Addresses and road segments in the order they came.
  std::vector<Vrst_row> m_rows;
  /// Places.
  std::vector<Vrst_place_row> m_places;
  /// Rows with a house number.
  Vrst_size m_numbers = 0;
};

}  // namespace nav
