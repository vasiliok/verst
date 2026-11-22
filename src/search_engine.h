/// @file
/// The address search database behind an abstract level, so the engine (SQLite FTS5
/// today) can be replaced without touching the converter, the ranking or the app.
///
/// The converter hands engine-neutral records to a Vrst_search_index_writer; the query
/// side reads them back through a Vrst_search_index. Only the engine files
/// (search_sqlite.h today) know how the records are stored. Matching words, house
/// numbers and ranking are shared rules (search_text.h, search_rank.h), so every engine
/// gives the same results.
#pragma once

#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "format.h"
#include "search_result.h"
#include "vrst_types.h"

namespace nav {

/// One searchable name: a street of a city, or a place.
struct Vrst_search_street {
  Vrst_uint32 id = 0;                            ///< Dense id, given by the writer's caller.
  Vrst_search_kind kind = Vrst_search_kind::Street;  ///< Place or Street.
  PlaceKind place = PlaceKind::City;             ///< Kind of a Place; unused for a Street.
  std::string name;                              ///< Name as in OSM.
  std::string city;                              ///< City of a street; empty for a place.
  std::string postcode;                          ///< Postcode of a street, may be empty.
  Vrst_coord pos{};                              ///< Where to show it.
};

/// One house number of a street.
struct Vrst_search_address {
  Vrst_uint32 street_id = 0;  ///< Vrst_search_street::id.
  std::string number;         ///< number_key() form ("12bis").
  Vrst_coord pos{};           ///< Building or address point.
};

/// The engines there are.
enum class Vrst_search_engine : Vrst_uint8 {
  Sqlite,  ///< SQLite FTS5, file search.db.
};

/// Writes one region's search database. Streets first (ids 0, 1, 2 ... in order), then
/// addresses, then finish().
class Vrst_search_index_writer {
 public:
  Vrst_search_index_writer() = default;                                             ///< Base of the engines.
  virtual ~Vrst_search_index_writer() = default;                                    ///< Owned through the base.
  Vrst_search_index_writer(const Vrst_search_index_writer&) = delete;             ///< Owns a file: not copyable.
  Vrst_search_index_writer& operator=(const Vrst_search_index_writer&) = delete;  ///< Owns a file: not copyable.
  Vrst_search_index_writer(Vrst_search_index_writer&&) = delete;                  ///< Polymorphic: not movable.
  Vrst_search_index_writer& operator=(Vrst_search_index_writer&&) = delete;       ///< Polymorphic: not movable.

  /// Adds a street or place.
  /// @param s Record; s.id must be the number of streets added before.
  virtual void add_street(const Vrst_search_street& s) = 0;
  /// Adds a house number of a street added before.
  /// @param a Record.
  virtual void add_address(const Vrst_search_address& a) = 0;
  /// Completes the database.
  /// @param build_id Id of the nav files it belongs to.
  /// @return Size of the written files in bytes.
  /// @throws std::runtime_error on a write error.
  virtual Vrst_uint64 finish(Vrst_uint64 build_id) = 0;
};

/// Reads one region's search database.
class Vrst_search_index {
 public:
  Vrst_search_index() = default;                                      ///< Base of the engines.
  virtual ~Vrst_search_index() = default;                             ///< Owned through the base.
  Vrst_search_index(const Vrst_search_index&) = delete;             ///< Owns a file: not copyable.
  Vrst_search_index& operator=(const Vrst_search_index&) = delete;  ///< Owns a file: not copyable.
  Vrst_search_index(Vrst_search_index&&) = delete;                  ///< Polymorphic: not movable.
  Vrst_search_index& operator=(Vrst_search_index&&) = delete;       ///< Polymorphic: not movable.

  /// Id of the nav files the database was made with.
  /// @return Build id.
  virtual Vrst_uint64 build_id() const = 0;
  /// Streets and places whose name, city or postcode has a word starting with each term.
  /// @param terms Normalized words (Vrst_search_query::terms); all must match.
  /// @param max At most this many, in no particular order.
  /// @return Matching records; empty for no terms.
  virtual std::vector<Vrst_search_street> find_streets(std::span<const std::string> terms, Vrst_size max) const = 0;
  /// One house number of a street.
  /// @param street_id Vrst_search_street::id.
  /// @param number number_key() form.
  /// @return Its position, or nullopt when the street has no such number.
  virtual std::optional<Vrst_coord> find_number(Vrst_uint32 street_id, std::string_view number) const = 0;
};

/// Starts a search database in a region folder.
/// @param engine Engine to write.
/// @param dir Region folder (the conversion's .tmp folder).
/// @return Writer.
/// @throws std::runtime_error when the file cannot be created.
std::unique_ptr<Vrst_search_index_writer> make_search_writer(Vrst_search_engine engine, const std::filesystem::path& dir);

/// Opens the search database of a region folder, whichever engine wrote it.
/// @param dir Region folder.
/// @return The index, or nullptr when the folder has none.
/// @throws std::runtime_error when the file is there but cannot be read.
std::unique_ptr<Vrst_search_index> open_search_index(const std::filesystem::path& dir);

}  // namespace nav
