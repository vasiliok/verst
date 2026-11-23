/// @file
/// make_search_writer() and open_search_index(): the only place that maps an engine to
/// its implementation. Documentation is in search_engine.h.
#include "search/search_engine.h"

#include "search/search_sqlite.h"

namespace nav {

std::unique_ptr<Vrst_search_index_writer> make_search_writer(Vrst_search_engine engine,
                                                             const std::filesystem::path& dir) {
  switch (engine) {
    case Vrst_search_engine::Sqlite: return make_sqlite_search_writer(dir);
  }
  return nullptr;
}

std::unique_ptr<Vrst_search_index> open_search_index(const std::filesystem::path& dir) {
  if (const std::filesystem::path f = dir / kSqliteSearchFile; std::filesystem::exists(f))
    return open_sqlite_search_index(f);
  return nullptr;
}

}  // namespace nav
