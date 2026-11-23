/// @file
/// `navroute <dir> <lat> <lon> <lat> <lon> [car|bike|foot]`
///
/// Routes between two points of a converted region and back through the map provider,
/// the same call the app makes (Vrst_map_provider::route_pair()), and prints the length,
/// the time, the settled vertices and the search time of both, how much of the way back
/// is on the outbound roads, and the roads at both ends.
/// For development, like navsearch.

#include <cstdio>
#include <cstdlib>
#include <exception>
#include <memory>
#include <string_view>

#include "provider/map_provider.h"

using namespace nav;

/// navroute entry point.
/// @param argc Argument count.
/// @param argv `<dir> <lat> <lon> <lat> <lon> [car|bike|foot]`.
/// @return 0 when a route was found, 1 when not or on error, 2 on wrong usage.
int main(int argc, char** argv) {
  if (argc != 6 && argc != 7) {
    std::fprintf(stderr, "usage: %s <dir> <lat> <lon> <lat> <lon> [car|bike|foot]\n", argv[0]);
    return 2;
  }
  const std::string_view m = argc == 7 ? argv[6] : "car";
  const Vrst_travel_mode mode = m == "bike" ? Vrst_travel_mode::Bike : m == "foot" ? Vrst_travel_mode::Foot
                                                                                   : Vrst_travel_mode::Car;
  try {
    const std::unique_ptr<Vrst_map_provider> map = open_map(argv[1], {.threads = 1});
    if (!map->can_route(mode)) {
      std::fprintf(stderr, "navroute: this map routes cars only (made before format 4)\n");
      return 1;
    }
    const Vrst_latlon from{std::strtod(argv[2], nullptr), std::strtod(argv[3], nullptr)};
    const Vrst_latlon to{std::strtod(argv[4], nullptr), std::strtod(argv[5], nullptr)};
    for (const Vrst_latlon p : {from, to})
      if (const auto r = map->road_at(p, mode))
        std::printf("road at %.6f, %.6f: %s (%.0f m away)\n", p.lat, p.lon, r->name.empty() ? "(no name)" : r->name.c_str(),
                    r->distance_m);
    const Vrst_route_pair pair = map->route_pair(from, to, mode);
    const Vrst_route& r = pair.there;
    if (!r.found) {
      std::printf("no route: %s (%zu settled, %.1f ms)\n", r.error.c_str(), r.settled, r.ms);
      return 1;
    }
    std::printf("%s: %.2f km, %.0f min, %zu points, %zu settled, %.1f ms\n", std::string(m).c_str(), r.length_m / 1000,
                r.time_s / 60, r.points.size(), r.settled, r.ms);
    const Vrst_route& b = pair.back;
    if (!b.found) {
      std::printf("back: none (%s)\n", b.error.c_str());
    } else {
      std::printf("back: %.2f km, %.0f min, %.0f%% on the outbound roads, %zu settled, %.1f ms\n", b.length_m / 1000,
                  b.time_s / 60, b.length_m > 0 ? 100 * b.shared_m / b.length_m : 0.0, b.settled, b.ms);
    }
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "navroute: %s\n", e.what());
    return 1;
  }
}
