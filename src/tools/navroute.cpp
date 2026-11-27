/// @file
/// `navroute <dir> <lat> <lon> <lat> <lon> [car|bike|foot] [heading=<deg>] [speed=<kmh>] [gpx=<file>]`
///
/// Routes between two points of a converted region and back through the map provider,
/// the same call the app makes (Vrst_map_provider::route_pair()), and prints the length,
/// the time, the settled vertices and the search time of both, how much of the way back
/// is on the outbound roads, and the roads at both ends.
/// With heading= (and speed=, default 50 km/h) the start is a moving car, as in a reroute
/// (Vrst_map_provider::route_moving()): it prints the situation the rules read, the
/// strategy they chose and where the route turns back.
/// gpx= writes the outbound route as a GPX track driven at speed= (one point a second),
/// to play in the app as a test drive (--gpx).
/// For development, like navsearch.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include "provider/map_provider.h"
#include "route/reroute_rules.h"

using namespace nav;

namespace {

/// Great-circle distance.
/// @param a First point.
/// @param b Second point.
/// @return Metres.
double metres(Vrst_latlon a, Vrst_latlon b) {
  const double r = kPi / 180, dlat = (b.lat - a.lat) * r, dlon = (b.lon - a.lon) * r;
  const double h = std::sin(dlat / 2) * std::sin(dlat / 2) +
                   std::cos(a.lat * r) * std::cos(b.lat * r) * std::sin(dlon / 2) * std::sin(dlon / 2);
  return 2 * kEarthRadius * std::asin(std::min(1.0, std::sqrt(h)));
}

/// Direction from one point to another, degrees clockwise from north.
/// @param a From.
/// @param b To.
/// @return Degrees.
double bearing(Vrst_latlon a, Vrst_latlon b) {
  return std::atan2((b.lon - a.lon) * std::cos(a.lat * kPi / 180), b.lat - a.lat) * 180 / kPi;
}

/// Prints the situation of a moving car and the rule chosen.
/// @param r Route from route_moving().
void print_situation(const Vrst_route& r) {
  const Vrst_road_situation& s = r.situation;
  if (!s.found) {
    std::printf("situation: no road near\n");
    return;
  }
  std::printf("situation: %s, %.0f km/h, heading %.0f, road \"%s\", on the road at %.6f, %.6f\n", road_type_name(s.type),
              s.motion.speed_kmh, s.motion.heading_deg, s.road.c_str(), s.motion.pos.lat, s.motion.pos.lon);
  for (const Vrst_road_event& e : s.ahead)
    std::printf("  ahead %c %5.0f m  %s  at %.6f, %.6f\n", event_code(e.kind), e.ahead_m, e.uturn_ok ? "uturn ok" : "no uturn",
                e.pos.lat, e.pos.lon);
  std::printf("strategy: %s, %zu U-turn(s) in the route\n", strategy_name(r.strategy), r.uturns);
}

/// Prints where a route turns back on itself (a piece more than 150 degrees off the one
/// before), in the first 20 km.
/// @param pts Route points.
void print_turnbacks(const std::vector<Vrst_latlon>& pts) {
  double along = 0;
  for (Vrst_size i = 1; i + 1 < pts.size() && along < 20000; ++i) {
    along += metres(pts[i - 1], pts[i]);
    const double d = std::abs(std::fmod(bearing(pts[i], pts[i + 1]) - bearing(pts[i - 1], pts[i]) + 540.0, 360.0) - 180.0);
    if (d > 150) std::printf("  turns back %.0f m along at %.6f, %.6f\n", along, pts[i].lat, pts[i].lon);
  }
}

/// Writes a route as a GPX track driven at a constant speed, one point a second.
/// @param path File.
/// @param pts Route points.
/// @param kmh Speed.
void write_gpx(const std::string& path, const std::vector<Vrst_latlon>& pts, double kmh) {
  std::ofstream f(path);
  if (!f) throw std::runtime_error(path + ": cannot write");
  f << "<?xml version=\"1.0\"?>\n<gpx version=\"1.1\" creator=\"navroute\"><trk><trkseg>\n";
  const double step = kmh / 3.6;  // metres a second
  double t = 0, carry = 0;        // seconds; metres of the piece already used
  const auto point = [&](Vrst_latlon p) {
    const int s = static_cast<int>(t);
    char line[160];
    std::snprintf(line, sizeof line, "<trkpt lat=\"%.7f\" lon=\"%.7f\"><time>2026-01-01T%02d:%02d:%02dZ</time></trkpt>\n", p.lat,
                  p.lon, s / 3600, s / 60 % 60, s % 60);
    f << line;
    t += 1;
  };
  if (!pts.empty()) point(pts.front());
  for (Vrst_size i = 1; i < pts.size(); ++i) {
    const double len = metres(pts[i - 1], pts[i]);
    for (double at = step - carry; at <= len; at += step) {
      const double u = at / len;
      point({pts[i - 1].lat + (pts[i].lat - pts[i - 1].lat) * u, pts[i - 1].lon + (pts[i].lon - pts[i - 1].lon) * u});
      carry = len - at;
    }
    if (len < step - carry) carry += len;
  }
  f << "</trkseg></trk></gpx>\n";
}

}  // namespace

/// navroute entry point.
/// @param argc Argument count.
/// @param argv See the file comment.
/// @return 0 when a route was found, 1 when not or on error, 2 on wrong usage.
int main(int argc, char** argv) {
  if (argc < 6) {
    std::fprintf(stderr, "usage: %s <dir> <lat> <lon> <lat> <lon> [car|bike|foot] [heading=<deg>] [speed=<kmh>] [gpx=<file>]\n",
                 argv[0]);
    return 2;
  }
  std::string_view m = "car";
  std::optional<double> heading;
  double kmh = 50;
  std::string gpx;
  for (int i = 6; i < argc; ++i) {
    const std::string_view a = argv[i];
    if (a.starts_with("heading=")) heading = std::strtod(argv[i] + 8, nullptr);
    else if (a.starts_with("speed=")) kmh = std::strtod(argv[i] + 6, nullptr);
    else if (a.starts_with("gpx=")) gpx = a.substr(4);
    else m = a;
  }
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
    Vrst_route_pair pair = map->route_pair(from, to, mode);
    if (heading) {
      pair.there = map->route_moving({from, *heading, kmh}, to, mode);
      print_situation(pair.there);
    }
    const Vrst_route& r = pair.there;
    if (!r.found) {
      std::printf("no route: %s (%zu settled, %.1f ms)\n", r.error.c_str(), r.settled, r.ms);
      return 1;
    }
    std::printf("%s: %.2f km, %.0f min, %zu points, %zu settled, %.1f ms\n", std::string(m).c_str(), r.length_m / 1000,
                r.time_s / 60, r.points.size(), r.settled, r.ms);
    if (heading) print_turnbacks(r.points);
    const Vrst_route& b = pair.back;
    if (!b.found) {
      std::printf("back: none (%s)\n", b.error.c_str());
    } else {
      std::printf("back: %.2f km, %.0f min, %.0f%% on the outbound roads, %zu settled, %.1f ms\n", b.length_m / 1000,
                  b.time_s / 60, b.length_m > 0 ? 100 * b.shared_m / b.length_m : 0.0, b.settled, b.ms);
    }
    if (!gpx.empty()) {
      write_gpx(gpx, r.points, kmh);
      std::printf("gpx: %s at %.0f km/h\n", gpx.c_str(), kmh);
    }
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "navroute: %s\n", e.what());
    return 1;
  }
}
