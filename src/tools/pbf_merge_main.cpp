/// @file
/// `pbf_merge <out.osm.pbf> <in1.osm.pbf> <in2.osm.pbf> ...`
///
/// Merges sorted extracts (e.g. two neighbouring Geofabrik regions) into one, with
/// merge_pbf() (pbf_merge.h, the same code the iPhone app runs).

#include <chrono>
#include <cstdio>
#include <exception>
#include <string>
#include <vector>

#include "convert/pbf_merge.h"

using namespace nav;

/// pbf_merge entry point.
/// @param argc Argument count.
/// @param argv `<out.osm.pbf> <in1.osm.pbf> <in2.osm.pbf> ...`.
/// @return 0 on success, 1 on error, 2 on wrong usage.
int main(int argc, char** argv) {
  if (argc < 4) {
    std::fprintf(stderr, "usage: %s <out.osm.pbf> <in1.osm.pbf> <in2.osm.pbf> ...\n", argv[0]);
    return 2;
  }
  try {
    const auto t0 = std::chrono::steady_clock::now();
    Vrst_convert_progress progress;
    const Vrst_merge_stats st = merge_pbf(std::vector<std::string>(argv + 2, argv + argc), argv[1], progress);
    const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("written %llu objects, %llu duplicates dropped, %.1f s\n", static_cast<unsigned long long>(st.written),
                static_cast<unsigned long long>(st.duplicates), s);
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "pbf_merge: %s\n", e.what());
    return 1;
  }
}
