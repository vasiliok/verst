/// @file
/// The SQLite FTS5 search engine: search.db in the region folder. Used only through the
/// interfaces of search_engine.h; nothing else includes SQLite.
///
/// Tables: `meta(key, value)` (build_id), `street(id, kind, place, name, city, postcode,
/// lat, lon)`, `addr(street_id, number, lat, lon)` keyed by street and number, and `fts`,
/// a contentless FTS5 table over normalize(name city postcode) with rowid = street id
/// and prefix indexes for 2 and 3 letters. Coordinates are integers in 1e-7 degrees.
#pragma once

#include <filesystem>
#include <memory>

#include "search/search_engine.h"

namespace nav {

/// File name of the SQLite engine in a region folder.
inline constexpr const char* kSqliteSearchFile = "search.db";

/// Starts search.db in a folder, replacing one that is there.
/// @param dir Folder.
/// @return Writer.
/// @throws std::runtime_error when the database cannot be created.
std::unique_ptr<Vrst_search_index_writer> make_sqlite_search_writer(const std::filesystem::path& dir);

/// Opens search.db read-only.
/// @param file Path of search.db.
/// @return Index.
/// @throws std::runtime_error when it cannot be opened or has no build id.
std::unique_ptr<Vrst_search_index> open_sqlite_search_index(const std::filesystem::path& file);

}  // namespace nav
