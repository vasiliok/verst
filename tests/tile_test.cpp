/// @file
/// GoogleTest tests of the tiled map (src/map_tiles.h).
///
/// The math tests need no data. The region tests open an imported region from the
/// directory in the environment variable VRST_TEST_DIR (CTest sets it to the Andorra
/// import of the `convert` test) and are skipped without it.
/// VRST_TEST_EVERY=n checks only every n-th object in the hole tests (large regions).

#include <gtest/gtest.h>

#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "map_tiles.h"
#include "nav_files.h"

using namespace nav;

namespace {

/// Slippy-map tile of a point, the OSM wiki formula, independent of map_tiles.h.
/// @param z Zoom.
/// @param lat Latitude in degrees.
/// @param lon Longitude in degrees.
/// @return Tile.
Vrst_tile_id slippy(int z, Vrst_float64 lat, Vrst_float64 lon) {
  const Vrst_float64 n = std::ldexp(1.0, z), r = lat * kPi / 180;
  return {z, static_cast<Vrst_int32>(std::floor((lon + 180) / 360 * n)),
          static_cast<Vrst_int32>(std::floor((1 - std::asinh(std::tan(r)) / kPi) / 2 * n))};
}

/// Whether two boxes overlap.
/// @param lo First box low corner.
/// @param hi First box high corner.
/// @param b Second box.
/// @return true if they share any point.
bool touches(Vrst_vec2d lo, Vrst_vec2d hi, const Vrst_box2d& b) {
  return hi.x >= b.lo.x && lo.x <= b.hi.x && hi.y >= b.lo.y && lo.y <= b.hi.y;
}

/// Key of a piece or triangle quantized to centimetres, with its tile and style.
/// @param tile Tile key.
/// @param v Values.
/// @return Hash key.
std::string quant_key(Vrst_uint64 tile, std::initializer_list<Vrst_float64> v) {
  std::string k = std::to_string(tile);
  for (const Vrst_float64 x : v) k += ',' + std::to_string(std::llround(x * 100));
  return k;
}

/// A coordinate as the builder stores it: float relative to the tile origin.
/// @param v Absolute Mercator value.
/// @param origin Tile origin on the same axis.
/// @return Value after the float round trip.
Vrst_float64 rel(Vrst_float64 v, Vrst_float64 origin) {
  return static_cast<Vrst_float64>(static_cast<Vrst_float32>(v - origin));
}

}  // namespace

// --- tile math (no data) --------------------------------------------------------

/// tile_at() agrees with the slippy-map formula on random points of the world.
TEST(TileMath, TileAtMatchesSlippyFormula) {
  std::mt19937 rng(1);
  std::uniform_real_distribution<Vrst_float64> lat_d(-85, 85), lon_d(-180, 179.999);
  for (int i = 0; i < 20000; ++i) {
    const Vrst_coord c{static_cast<Vrst_int32>(std::lround(lat_d(rng) * 1e7)),
                       static_cast<Vrst_int32>(std::lround(lon_d(rng) * 1e7))};
    const Vrst_vec2d m = mercator(c);
    for (const Vrst_tile_level& l : kTileLevels) {
      const Vrst_tile_id t = tile_at(l.z, m), e = slippy(l.z, c.lat_e7 * 1e-7, c.lon_e7 * 1e-7);
      const Vrst_box2d b = tile_bounds(t);
      // A point within a millimetre of a border may round either way.
      if (std::min({m.x - b.lo.x, b.hi.x - m.x, m.y - b.lo.y, b.hi.y - m.y}) < 1e-3) continue;
      ASSERT_EQ(t.x, e.x) << "z" << l.z << " lat " << c.lat_e7 << " lon " << c.lon_e7;
      ASSERT_EQ(t.y, e.y) << "z" << l.z << " lat " << c.lat_e7 << " lon " << c.lon_e7;
    }
  }
}

/// tile_key() and tile_of_key() are inverse; tile_bounds() contains the point.
TEST(TileMath, KeysAndBounds) {
  std::mt19937 rng(3);
  std::uniform_real_distribution<Vrst_float64> u(-kMercatorHalf, kMercatorHalf);
  for (int i = 0; i < 20000; ++i) {
    const Vrst_vec2d m{u(rng), u(rng)};
    for (const int z : {0, 8, 10, 12, 14, 20}) {
      const Vrst_tile_id t = tile_at(z, m), k = tile_of_key(tile_key(t));
      ASSERT_EQ(k.z, t.z);
      ASSERT_EQ(k.x, t.x);
      ASSERT_EQ(k.y, t.y);
      const Vrst_box2d b = tile_bounds(t);
      ASSERT_TRUE(touches(m, m, {{b.lo.x - 1e-6, b.lo.y - 1e-6}, {b.hi.x + 1e-6, b.hi.y + 1e-6}}));
      ASSERT_NEAR(b.hi.x - b.lo.x, tile_side(z), 1e-6);
    }
  }
  const Vrst_box2d world = tile_bounds({0, 0, 0});
  EXPECT_NEAR(world.lo.x, -kMercatorHalf, 1e-6);
  EXPECT_NEAR(world.hi.y, kMercatorHalf, 1e-6);
}

/// Points outside the Mercator square are clamped onto the grid.
TEST(TileMath, TileAtClamps) {
  const Vrst_tile_id a = tile_at(14, {-3 * kMercatorHalf, 3 * kMercatorHalf});
  const Vrst_tile_id b = tile_at(14, {3 * kMercatorHalf, -3 * kMercatorHalf});
  EXPECT_EQ(a.x, 0);
  EXPECT_EQ(a.y, 0);
  EXPECT_EQ(b.x, (1 << 14) - 1);
  EXPECT_EQ(b.y, (1 << 14) - 1);
}

/// Levels go from fine to coarse, level_for() switches exactly at max_mpp, and a
/// coarser level never holds a road class the finer one drops.
TEST(TileLevels, OrderedWithThresholds) {
  EXPECT_EQ(level_for(0.01), 0);
  EXPECT_EQ(level_for(1e9), kTileLevelCount - 1);
  for (int i = 0; i + 1 < kTileLevelCount; ++i) {
    EXPECT_GT(kTileLevels[i].z, kTileLevels[i + 1].z);
    EXPECT_LT(kTileLevels[i].max_mpp, kTileLevels[i + 1].max_mpp);
    EXPECT_EQ(level_for(kTileLevels[i].max_mpp), i);
    EXPECT_EQ(level_for(kTileLevels[i].max_mpp * 1.001), i + 1);
    EXPECT_EQ(kTileLevels[i + 1].class_mask & ~kTileLevels[i].class_mask, 0u);
    EXPECT_EQ(level_of_zoom(kTileLevels[i].z), i);
  }
  EXPECT_EQ(level_of_zoom(13), -1);
}

/// visible_tiles() contains the tile under every screen point, at random zooms and
/// rotations, and lists each tile once.
TEST(VisibleTiles, CoverScreenAtAnyRotation) {
  const Vrst_vec2d center = mercator({473220000, 50410000});
  std::mt19937 rng(2);
  std::uniform_real_distribution<Vrst_float64> u(0, 1);
  for (int i = 0; i < 500; ++i) {
    const int fb_w = 400 + static_cast<int>(u(rng) * 2400), fb_h = 300 + static_cast<int>(u(rng) * 1600);
    const Vrst_view v{{(u(rng) - 0.5) * 40000, (u(rng) - 0.5) * 40000}, std::pow(10.0, -3 + u(rng) * 3.5), u(rng) * 2 * kPi};
    const int z = kTileLevels[level_for(1 / v.scale)].z;
    const std::vector<Vrst_tile_id> tiles = visible_tiles(v, fb_w, fb_h, center, z);
    ASSERT_FALSE(tiles.empty());
    ASSERT_LE(tiles.size(), 400u) << "level choice should keep the tile count small";
    std::unordered_set<Vrst_uint64> set;
    for (const Vrst_tile_id& t : tiles) ASSERT_TRUE(set.insert(tile_key(t)).second) << "duplicate tile";
    for (int k = 0; k < 64; ++k) {
      const Vrst_float64 x = k < 4 ? (k & 1) * fb_w : u(rng) * fb_w, y = k < 4 ? (k >> 1) * fb_h : u(rng) * fb_h;
      const Vrst_vec2d w = screen_to_world(v, fb_w, fb_h, x, y);
      ASSERT_TRUE(set.count(tile_key(tile_at(z, {w.x + center.x, w.y + center.y}))))
          << "view " << i << " screen point " << x << "," << y << " not covered";
    }
  }
}

/// visible_tiles() lists the tile nearest the screen center first.
TEST(VisibleTiles, NearestFirst) {
  const Vrst_vec2d center = mercator({473220000, 50410000});
  const Vrst_view v{{1234, -567}, 1.0 / 3, 0.3};
  const std::vector<Vrst_tile_id> tiles = visible_tiles(v, 2560, 1800, center, 14);
  ASSERT_FALSE(tiles.empty());
  const Vrst_tile_id mid = tile_at(14, {v.center.x + center.x, v.center.y + center.y});
  EXPECT_EQ(tile_key(tiles.front()), tile_key(mid));
}

/// A build that throws neither ends the worker nor blocks the tile: the other tiles still
/// arrive, the failure is counted, and asking for the tile again retries it.
TEST(TileLoader, FailedBuildIsCountedAndRetried) {
  const Vrst_tile_loader::Vrst_build_fn build = [](Vrst_tile_id id) {
    if (id.x == 1) throw std::runtime_error("broken tile");
    std::unique_ptr<Vrst_tile_data> t = std::make_unique<Vrst_tile_data>();
    t->id = id;
    return t;
  };
  Vrst_tile_loader loader(build, 2);
  const auto wait_idle = [&loader] {
    const auto t0 = std::chrono::steady_clock::now();
    while (loader.pending() > 0 && std::chrono::steady_clock::now() - t0 < std::chrono::seconds(10))
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
  };
  loader.request({{14, 0, 0}, {14, 1, 0}, {14, 2, 0}, {14, 3, 0}});
  wait_idle();
  EXPECT_EQ(loader.take(16).size(), 3u);
  EXPECT_EQ(loader.failed(), 1u);
  EXPECT_EQ(loader.pending(), 0u);

  loader.request({{14, 1, 0}});  // not stuck as busy: built (and failing) again
  wait_idle();
  EXPECT_EQ(loader.failed(), 2u);
  EXPECT_TRUE(loader.take(16).empty());
}

// --- region tests (VRST_TEST_DIR) -------------------------------------------------

namespace {

/// Environment variable as text.
/// @tparam N Length of the name literal.
/// @param name Variable name.
/// @return Its value, or "" when it is not set.
template <Vrst_size N>
std::string env(const char (&name)[N]) {
  const char* v = std::getenv(name);  // C API; copied at once
  return v ? v : "";
}

/// Opens the region once for all region tests and builds the index.
class RegionTest : public ::testing::Test {
 protected:
  /// Opens VRST_TEST_DIR, builds the mesh center and the tile index once.
  static void SetUpTestSuite() {
    const std::string dir = env("VRST_TEST_DIR");
    if (dir.empty()) return;
    s_data = std::make_unique<Vrst_nav_data>();
    s_data->open(dir);
    s_mesh = std::make_unique<Vrst_map_mesh>();
    s_mesh->build(*s_data, false);
    s_index = std::make_unique<Vrst_tile_index>();
    s_index->build(*s_data);
    s_builder = std::make_unique<Vrst_tile_builder>(*s_data, *s_index, s_mesh->center());
    const std::string every = env("VRST_TEST_EVERY");
    Vrst_uint32 n = 0;
    if (std::from_chars(every.data(), every.data() + every.size(), n).ec == std::errc{} && n > 0) s_every = n;
  }
  /// Frees the region.
  static void TearDownTestSuite() {
    s_builder.reset();
    s_index.reset();
    s_mesh.reset();
    s_data.reset();
  }
  /// Skips the test when no region is configured.
  void SetUp() override {
    if (!s_data) GTEST_SKIP() << "VRST_TEST_DIR not set";
  }

  static std::unique_ptr<Vrst_nav_data> s_data;         ///< Opened region.
  static std::unique_ptr<Vrst_map_mesh> s_mesh;         ///< Map center.
  static std::unique_ptr<Vrst_tile_index> s_index;      ///< Tile index.
  static std::unique_ptr<Vrst_tile_builder> s_builder;  ///< Tile builder.
  static Vrst_uint32 s_every;                           ///< Hole tests check every n-th object.
};
std::unique_ptr<Vrst_nav_data> RegionTest::s_data;
std::unique_ptr<Vrst_map_mesh> RegionTest::s_mesh;
std::unique_ptr<Vrst_tile_index> RegionTest::s_index;
std::unique_ptr<Vrst_tile_builder> RegionTest::s_builder;
Vrst_uint32 RegionTest::s_every = 1;

}  // namespace

/// Every tile of every level is well formed: contiguous draw ranges, kinds and
/// styles match their range, the level's class and building filters hold, and all
/// geometry touches its tile.
TEST_F(RegionTest, BuiltTilesWellFormed) {
  for (int l = 0; l < kTileLevelCount; ++l) {
    const Vrst_tile_level& level = kTileLevels[l];
    const std::vector<Vrst_tile_id> ids = s_index->tile_ids(l);
    ASSERT_FALSE(ids.empty()) << "z" << level.z;
    for (const Vrst_tile_id& id : ids) {
      SCOPED_TRACE("tile " + std::to_string(id.z) + "/" + std::to_string(id.x) + "/" + std::to_string(id.y));
      const std::unique_ptr<Vrst_tile_data> t = s_builder->build(id);
      const Vrst_box2d tb = tile_bounds(id);
      const Vrst_float64 m = tile_side(id.z) * 0.021;
      const Vrst_box2d keep{{tb.lo.x - m, tb.lo.y - m}, {tb.hi.x + m, tb.hi.y + m}};
      const Vrst_vec2d o{t->origin.x + s_mesh->center().x, t->origin.y + s_mesh->center().y};
      ASSERT_NEAR(o.x, tb.lo.x, 1e-6);
      ASSERT_NEAR(o.y, tb.lo.y, 1e-6);
      Vrst_uint32 next = 0;
      for (int k = 0; k < kAreaKindCount; ++k) {
        const Vrst_tile_range r = t->fill_ranges[k];
        ASSERT_EQ(r.first, next);
        ASSERT_EQ(r.count % 3, 0u);
        for (Vrst_uint32 i = 0; i < r.count; ++i) ASSERT_EQ(t->fills[r.first + i].attr, static_cast<Vrst_uint32>(k));
        next += r.count;
      }
      ASSERT_EQ(next, t->fills.size());
      if (!level.buildings) ASSERT_EQ(t->fill_ranges[static_cast<int>(AreaKind::Building)].count, 0u);
      if (!level.vertices) ASSERT_TRUE(t->points.empty());
      next = 0;
      for (const auto& layer_ranges : t->line_ranges)
        for (int s = 0; s < kLineStyleCount; ++s) {
          const Vrst_tile_range r = layer_ranges[s];
          ASSERT_EQ(r.first, next);
          for (Vrst_uint32 i = 0; i < r.count; ++i) {
            const Vrst_line_instance& li = t->lines[r.first + i];
            const Vrst_uint32 style = li.attr & 0xFF;
            ASSERT_EQ(style, static_cast<Vrst_uint32>(s));
            if (style < static_cast<Vrst_uint32>(kRoadClassCount))
              ASSERT_TRUE(level.class_mask & (1u << style)) << "class " << style;
            if (s == kStyleBuildingEdge) ASSERT_TRUE(level.buildings);
            ASSERT_TRUE(std::isfinite(li.d0) && li.d0 >= 0);
            ASSERT_TRUE(touches({o.x + std::min(li.x0, li.x1), o.y + std::min(li.y0, li.y1)},
                                {o.x + std::max(li.x0, li.x1), o.y + std::max(li.y0, li.y1)}, keep));
          }
          next += r.count;
        }
      ASSERT_EQ(next, t->lines.size());
      for (Vrst_size i = 0; i + 2 < t->fills.size(); i += 3) {
        Vrst_vec2d lo{1e300, 1e300}, hi{-1e300, -1e300};
        for (Vrst_size k = i; k < i + 3; ++k) {
          lo = {std::min(lo.x, o.x + t->fills[k].x), std::min(lo.y, o.y + t->fills[k].y)};
          hi = {std::max(hi.x, o.x + t->fills[k].x), std::max(hi.y, o.y + t->fills[k].y)};
        }
        ASSERT_TRUE(touches(lo, hi, keep));
      }
      for (const Vrst_gpu_point& p : t->points) ASSERT_TRUE(touches({o.x + p.x, o.y + p.y}, {o.x + p.x, o.y + p.y}, keep));
    }
  }
}

/// No holes on the base level: every road piece, river/railway piece, area triangle
/// and building outline edge is in the tile under its midpoint, at the same position.
/// Objects are grouped by expected tile first, so only those tiles are built and
/// VRST_TEST_EVERY keeps the memory small on large regions.
TEST_F(RegionTest, BaseLevelHasNoHoles) {
  const int z = kTileLevels[0].z;
  // Expected entries per tile: (key inside the tile, what it is).
  std::unordered_map<Vrst_uint64, std::vector<std::pair<std::string, std::string>>> expect;
  const auto want_piece = [&](Vrst_vec2d a, Vrst_vec2d b, int style, const std::string& what) {
    const Vrst_tile_id t = tile_at(z, {(a.x + b.x) / 2, (a.y + b.y) / 2});
    const Vrst_box2d tb = tile_bounds(t);
    expect[tile_key(t)].emplace_back(quant_key(0, {static_cast<Vrst_float64>(style), rel(a.x, tb.lo.x),
                                                   rel(a.y, tb.lo.y), rel(b.x, tb.lo.x), rel(b.y, tb.lo.y)}),
                                     what);
  };
  const Vrst_geom& geo = s_data->geom();
  for (Vrst_uint32 s = 0; s < s_data->edges().num_segments(); s += s_every)
    for (Vrst_uint32 i = geo.seg_first_point()[s]; i + 1 < geo.seg_first_point()[s + 1]; ++i)
      want_piece(mercator(geo.points()[i]), mercator(geo.points()[i + 1]), s_data->edges().attr()[s].road_class,
                 "segment " + std::to_string(s) + " point " + std::to_string(i));
  if (s_data->has_map()) {
    const Vrst_map_layers& m = s_data->map();
    for (Vrst_uint32 l = 0; l < m.num_lines(); l += s_every) {
      const std::span<const Vrst_coord> p = m.line_points(l);
      for (Vrst_size i = 0; i + 1 < p.size(); ++i)
        want_piece(mercator(p[i]), mercator(p[i + 1]), kStyleLineKind0 + m.line_attr()[l].kind, "line " + std::to_string(l));
    }
    for (Vrst_uint32 a = 0; a < m.num_areas(); a += s_every) {
      const std::span<const Vrst_coord> p = m.area_points(a);
      const std::span<const Vrst_uint32> idx = m.area_indices(a);
      for (Vrst_size i = 0; i + 2 < idx.size(); i += 3) {
        const Vrst_vec2d q[3] = {mercator(p[idx[i]]), mercator(p[idx[i + 1]]), mercator(p[idx[i + 2]])};
        const Vrst_tile_id t = tile_at(z, {(q[0].x + q[1].x + q[2].x) / 3, (q[0].y + q[1].y + q[2].y) / 3});
        const Vrst_box2d tb = tile_bounds(t);
        expect[tile_key(t)].emplace_back(
            "T" + quant_key(0, {static_cast<Vrst_float64>(m.area_kinds()[a]), rel(q[0].x, tb.lo.x),
                                rel(q[0].y, tb.lo.y), rel(q[1].x, tb.lo.x), rel(q[1].y, tb.lo.y),
                                rel(q[2].x, tb.lo.x), rel(q[2].y, tb.lo.y)}),
            "area " + std::to_string(a) + " triangle " + std::to_string(i / 3));
      }
      if (m.area_kind(a) == AreaKind::Building) {
        const std::span<const Vrst_coord> ring = m.area_outer(a);
        for (Vrst_size i = 0; i < ring.size(); ++i)
          want_piece(mercator(ring[i]), mercator(ring[(i + 1) % ring.size()]), kStyleBuildingEdge,
                     "building " + std::to_string(a) + " edge " + std::to_string(i));
      }
    }
  }
  Vrst_size checked = 0;
  for (const auto& [key, wanted] : expect) {
    const std::unique_ptr<Vrst_tile_data> t = s_builder->build(tile_of_key(key));
    std::unordered_set<std::string> have;
    for (const Vrst_line_instance& li : t->lines)
      have.insert(quant_key(0, {static_cast<Vrst_float64>(li.attr & 0xFF), li.x0, li.y0, li.x1, li.y1}));
    for (Vrst_size i = 0; i + 2 < t->fills.size(); i += 3)
      have.insert("T" + quant_key(0, {static_cast<Vrst_float64>(t->fills[i].attr), t->fills[i].x, t->fills[i].y,
                                      t->fills[i + 1].x, t->fills[i + 1].y, t->fills[i + 2].x, t->fills[i + 2].y}));
    for (const auto& [k, what] : wanted) {
      ASSERT_TRUE(have.count(k)) << what << " missing from tile " << key;
      ++checked;
    }
  }
  EXPECT_GT(checked, 0u);
  RecordProperty("checked", static_cast<int>(checked));
}

/// Overview levels really are lighter than the base level: fewer pieces, and areas
/// small on screen are left out.
TEST_F(RegionTest, OverviewLevelsAreLighter) {
  Vrst_size prev = ~static_cast<Vrst_size>(0);
  for (int l = 0; l < kTileLevelCount; ++l) {
    Vrst_size pieces = 0;
    for (const Vrst_tile_id& id : s_index->tile_ids(l)) pieces += s_builder->build(id)->lines.size();
    EXPECT_LT(pieces, prev) << "z" << kTileLevels[l].z;
    prev = pieces;
  }
}

/// The loader returns every requested tile exactly once, also when requests overlap
/// and are replaced while workers are busy.
TEST_F(RegionTest, LoaderReturnsEachTileOnce) {
  const std::vector<Vrst_tile_id> ids = s_index->tile_ids(0);
  Vrst_tile_loader loader(*s_builder, 3);
  loader.request(std::vector<Vrst_tile_id>(ids.begin(), ids.begin() + static_cast<Vrst_int64>(ids.size() / 2)));
  loader.request(ids);
  std::unordered_set<Vrst_uint64> got;
  const auto t0 = std::chrono::steady_clock::now();
  while (got.size() < ids.size() && std::chrono::steady_clock::now() - t0 < std::chrono::seconds(120)) {
    for (const std::unique_ptr<Vrst_tile_data>& t : loader.take(64))
      ASSERT_TRUE(got.insert(tile_key(t->id)).second) << "tile returned twice";
    std::vector<Vrst_tile_id> rest;
    for (const Vrst_tile_id& t : ids)
      if (!got.contains(tile_key(t))) rest.push_back(t);
    loader.request(rest);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_EQ(got.size(), ids.size());
  EXPECT_EQ(loader.pending(), 0u);
}

/// Dropping a request: tiles no longer asked for are not built.
TEST_F(RegionTest, LoaderDropsStaleRequests) {
  const std::vector<Vrst_tile_id> ids = s_index->tile_ids(0);
  ASSERT_GE(ids.size(), 8u);
  Vrst_tile_loader loader(*s_builder, 1);
  loader.request(ids);
  loader.request({});  // the user panned away
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  Vrst_size built = 0;
  for (int i = 0; i < 10; ++i) {
    built += loader.take(1000).size();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  EXPECT_LE(built, 2u) << "only the tile(s) already in progress may finish";
  EXPECT_EQ(loader.pending(), 0u);
}
