/// @file
/// `navsearch <dir> <text> [<lat> <lon>]`
///
/// Searches the address database of a converted region (search.db, written by
/// osm2nav) through the same engine-neutral code the app uses (search_engine.h,
/// search_rank.h) and prints the results with their position and the query time. The
/// text may also be a map link or coordinates (geo_link.h); short links are not opened.
/// For development, like navcheck.

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <memory>
#include <optional>

#include "search/search_rank.h"

using namespace nav;

/// navsearch entry point.
/// @param argc Argument count.
/// @param argv `<dir> <text> [<lat> <lon>]`.
/// @return 0 on success, 1 on error, 2 on wrong usage.
int main(int argc, char** argv) {
  if (argc != 3 && argc != 5) {
    std::fprintf(stderr, "usage: %s <dir> <text> [<lat> <lon>]\n", argv[0]);
    return 2;
  }
  try {
    const std::unique_ptr<Vrst_search_index> index = open_search_index(argv[1]);
    if (!index) {
      std::fprintf(stderr, "navsearch: %s has no search database\n", argv[1]);
      return 1;
    }
    std::optional<Vrst_latlon> near;
    if (argc == 5) near = Vrst_latlon{std::strtod(argv[3], nullptr), std::strtod(argv[4], nullptr)};
    constexpr Vrst_size kMaxResults = 20;
    const auto t0 = std::chrono::steady_clock::now();
    std::vector<Vrst_search_result> results;
    if (const std::optional<Vrst_geo_link> link = parse_geo_link(argv[2])) {
      results.push_back(link_result(*link, near));
    } else if (const std::optional<std::string> address = parse_link_address(argv[2])) {
      results = search_link_address(*index, *address, near);
    } else {
      results = search_index(*index, argv[2], near, kMaxResults);
    }
    const Vrst_float64 ms = std::chrono::duration<Vrst_float64, std::milli>(std::chrono::steady_clock::now() - t0).count();
    static constexpr const char* kKinds[] = {"place", "street", "address", "point"};
    for (const Vrst_search_result& r : results)
      std::printf("%-8s %-40s %-36s %.7f %.7f  %g m/pt\n", kKinds[static_cast<int>(r.kind)], r.title.c_str(),
                  r.detail.c_str(), r.pos.lat, r.pos.lon, r.metres_per_point);
    std::printf("%zu results in %.2f ms\n", results.size(), ms);
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "navsearch: %s\n", e.what());
    return 1;
  }
}
