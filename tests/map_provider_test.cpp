/// @file
/// Tests of the client API (map_provider.h) on today's nav-file provider: what a
/// client sees when it shows an area, collects the pieces and watches the status.
/// Needs VRST_TEST_DIR (an osm2nav output folder); skipped without it.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include "map_provider.h"
#include "map_scene.h"

namespace nav {
namespace {

/// Opens the test map, or skips the test.
/// @param options Threads and event recording.
/// @return Provider, or null when VRST_TEST_DIR is not set.
std::unique_ptr<Vrst_map_provider> open_test_map(const Vrst_map_options& options) {
  const char* dir = std::getenv("VRST_TEST_DIR");
  if (!dir || !*dir) return nullptr;
  return open_map(dir, options);
}

/// Area of a 1600 x 1200 px screen at the map center, 3 Mercator metres per pixel
/// (the finest level).
/// @param frame Map frame.
/// @return Area for show().
Vrst_map_area center_area(const Vrst_map_frame& frame) {
  return frame.area_of({{0, 0}, 1.0 / 3, 0.3}, 1600, 1200);
}

}  // namespace

/// Extent, places and piece names come back in client terms.
TEST(MapProvider, DescribesTheMap) {
  const std::unique_ptr<Vrst_map_provider> map = open_test_map({});
  if (!map) GTEST_SKIP() << "VRST_TEST_DIR not set";
  const Vrst_geo_box e = map->extent();
  EXPECT_LT(e.south_west.lat, e.north_east.lat);
  EXPECT_LT(e.south_west.lon, e.north_east.lon);
  for (const Vrst_map_place& p : map->places()) {
    EXPECT_FALSE(p.name.empty());
    EXPECT_TRUE(contains({{e.south_west.lat - 1, e.south_west.lon - 1}, {e.north_east.lat + 1, e.north_east.lon + 1}},
                         p.pos));
  }
  EXPECT_FALSE(map->describe().empty());
  EXPECT_EQ(map->status().threads, 2u);
}

/// A client that shows an area and takes what is ready ends up holding every needed
/// piece; then the provider is settled, reports nothing missing and records one
/// Settled event.
TEST(MapProvider, ShowTakeUntilSettled) {
  const std::unique_ptr<Vrst_map_provider> map = open_test_map({.threads = 2, .record_events = true});
  if (!map) GTEST_SKIP() << "VRST_TEST_DIR not set";
  const Vrst_map_frame frame(map->extent());
  const Vrst_map_area area = center_area(frame);
  std::unordered_set<Vrst_uint64> held;
  const Vrst_has_piece has = [&held](Vrst_uint64 k) { return held.contains(k); };

  std::vector<Vrst_piece_need> needs = map->show(area, has);
  ASSERT_FALSE(needs.empty());
  for (const Vrst_piece_need& n : needs) EXPECT_FALSE(n.fallbacks.empty()) << map->piece_name(n.key);
  EXPECT_FALSE(map->status().settled);
  EXPECT_EQ(map->status().missing, needs.size());

  const auto t0 = std::chrono::steady_clock::now();
  while (!map->status().settled && std::chrono::steady_clock::now() - t0 < std::chrono::seconds(60)) {
    for (const Vrst_map_piece& p : map->take_ready(8)) {
      EXPECT_TRUE(held.insert(p.key).second) << "piece delivered twice";
      EXPECT_LT(p.bounds.lo.x, p.bounds.hi.x);
    }
    needs = map->show(area, has);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  const Vrst_provider_status s = map->status();
  EXPECT_TRUE(s.settled);
  EXPECT_EQ(s.missing, 0u);
  for (const Vrst_piece_need& n : needs) EXPECT_TRUE(held.contains(n.key)) << map->piece_name(n.key);

  Vrst_size started = 0, done = 0, settled = 0;
  for (const Vrst_work_event& e : map->take_events()) {
    started += e.kind == Vrst_work_event_kind::Started;
    done += e.kind == Vrst_work_event_kind::Done;
    settled += e.kind == Vrst_work_event_kind::Settled;
  }
  EXPECT_EQ(started, done);
  EXPECT_EQ(done, held.size());
  EXPECT_EQ(settled, 1u);
}

}  // namespace nav
