/// @file
/// `osm2nav <input.osm.pbf> <out_dir>`
///
/// Converts OSM car roads into graph.dat / edges.dat / geom.dat / names.dat, the
/// drawing-only layers into map.dat and the addresses into search.db, with convert_osm() (osm_convert.h, the same code
/// the iPhone app runs). Prints each phase with its time and the peak memory so far,
/// then the counts.

#include <cstdio>
#include <exception>
#include <string>

#ifndef _WIN32
#include <sys/resource.h>
#endif

#include "convert/osm_convert.h"

using namespace nav;

namespace {

/// Peak resident memory of the process.
/// @return MiB; 0 on Windows.
Vrst_float64 peak_rss_mb() {
#ifdef _WIN32
  return 0;
#else
  rusage ru{};
  getrusage(RUSAGE_SELF, &ru);
#ifdef __APPLE__
  return static_cast<Vrst_float64>(ru.ru_maxrss) / (1024.0 * 1024.0);  // bytes
#else
  return static_cast<Vrst_float64>(ru.ru_maxrss) / 1024.0;  // KiB
#endif
#endif
}

/// Prints the counts and sizes of a finished conversion.
/// @param st Stats from convert_osm().
/// @param total_ms Wall time of the whole run.
void print_stats(const Vrst_convert_stats& st, Vrst_float64 total_ms) {
  const auto kib = [](Vrst_uint64 b) { return static_cast<Vrst_float64>(b) / 1024.0; };
  const auto ull = [](Vrst_uint64 v) { return static_cast<unsigned long long>(v); };
  std::printf("\n");
  std::printf("ways read            %12llu\n", ull(st.ways_read));
  std::printf("ways kept            %12zu\n", st.ways_kept);
  std::printf("nodes read           %12llu\n", ull(st.nodes_read));
  std::printf("road nodes           %12zu\n", st.road_nodes);
  std::printf("missing node refs    %12llu  (%llu ways, %llu breaks)\n", ull(st.missing_refs),
              ull(st.ways_with_missing), ull(st.missing_breaks));
  std::printf("closed loops split   %12llu\n", ull(st.loops_split));
  std::printf("vertices V           %12u\n", st.vertices);
  std::printf("segments S           %12u  (%llu oneway)\n", st.segments, ull(st.oneway_segments));
  std::printf("arcs A               %12u\n", st.arcs);
  std::printf("points P             %12zu\n", st.points);
  std::printf("names N              %12zu\n", st.names);
  Vrst_size places = 0, areas = 0;
  for (const Vrst_size n : st.places) places += n;
  for (const Vrst_size n : st.areas) areas += n;
  std::printf("places L             %12zu  (%zu city, %zu town, %zu village, %zu hamlet)\n", places, st.places[0],
              st.places[1], st.places[2], st.places[3]);
  std::printf("areas R              %12zu  (%zu water, %zu forest, %zu buildings: %zu rect, %zu outline)\n", areas,
              st.areas[static_cast<int>(AreaKind::Water)], st.areas[static_cast<int>(AreaKind::Forest)],
              st.areas[static_cast<int>(AreaKind::Building)], st.rectangles, st.outlines);
  std::printf("area triangles       %12zu\n", st.triangles);
  std::printf("area nodes kept      %12zu\n", st.area_nodes);
  std::printf("lines L              %12zu  (%zu points)\n", st.lines, st.line_points);
  std::printf("search               %12zu  addresses, %zu streets, %zu places\n", st.search_addresses,
              st.search_streets, st.search_places);
  std::printf("road length          %12.1f km\n", st.road_km);
  std::printf("node extent          lat [%.7f, %.7f] lon [%.7f, %.7f]\n", st.extent.min.lat_e7 * 1e-7,
              st.extent.max.lat_e7 * 1e-7, st.extent.min.lon_e7 * 1e-7, st.extent.max.lon_e7 * 1e-7);
  if (st.header_bbox)
    std::printf("header bbox          lat [%.7f, %.7f] lon [%.7f, %.7f]\n", st.header_bbox->min.lat_e7 * 1e-7,
                st.header_bbox->max.lat_e7 * 1e-7, st.header_bbox->min.lon_e7 * 1e-7,
                st.header_bbox->max.lon_e7 * 1e-7);
  std::printf("build_id             %016llx\n", ull(st.build_id));
  static constexpr const char* kFiles[] = {"graph.dat", "edges.dat", "geom.dat", "names.dat", "map.dat",
                                                  "search.db"};
  for (Vrst_size i = 0; i < st.file_bytes.size(); ++i)
    std::printf("%-20s %12.1f KiB\n", kFiles[i], kib(st.file_bytes[i]));
  std::printf("total time           %12.1f ms\n", total_ms);
  std::printf("peak RSS             %12.1f MiB\n", peak_rss_mb());
}

}  // namespace

/// osm2nav entry point.
/// @param argc Argument count.
/// @param argv `<input.osm.pbf> <out_dir>`.
/// @return 0 on success, 1 on error, 2 on wrong usage.
int main(int argc, char** argv) {
  if (argc != 3) {
    std::fprintf(stderr, "usage: %s <input.osm.pbf> <out_dir>\n", argv[0]);
    return 2;
  }
  try {
    Vrst_float64 total_ms = 0;
    Vrst_convert_progress progress;
    progress.on_phase_done([&total_ms](Vrst_convert_phase p, Vrst_float64 ms) {
      total_ms += ms;
      std::printf("  %-22s %8.1f ms  peak %7.1f MiB\n", phase_name(p), ms, peak_rss_mb());
    });
    const Vrst_convert_stats st = convert_osm(argv[1], argv[2], progress);
    print_stats(st, total_ms);
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "osm2nav: %s\n", e.what());
    return 1;
  }
}
