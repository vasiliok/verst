/// @file
/// GoogleTest tests of the shared plane geometry (src/geometry/geometry.h): simplification, ring
/// area, smallest rectangle, clipping to a box and repeated points, plus the zoom-10
/// regression: a huge self-crossing forest outline is cut to one tile before triangulation.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "geometry/geometry.h"

using namespace nav;

namespace {

/// Square ring, counter-clockwise, without the closing point.
/// @param lo Lower-left corner coordinate (x and y).
/// @param hi Upper-right corner coordinate (x and y).
/// @return 4 points.
std::vector<Vrst_vec2d> square(Vrst_float64 lo, Vrst_float64 hi) {
  return {{lo, lo}, {hi, lo}, {hi, hi}, {lo, hi}};
}

}  // namespace

/// The ends stay; points within the tolerance of a straight line go.
TEST(Geometry, SimplifyKeepsEndsAndDropsNearlyStraightPoints) {
  std::vector<Vrst_vec2d> line{{0, 0}, {1, 0.01}, {2, -0.01}, {3, 5}, {4, 0}};
  simplify_line(line, 0.1);
  ASSERT_EQ(line.size(), 3u);
  EXPECT_EQ(line.front().x, 0);
  EXPECT_EQ(line[1].x, 3);
  EXPECT_EQ(line.back().x, 4);
  std::vector<Vrst_vec2d> same = line;
  simplify_line(same, 0);
  EXPECT_EQ(same.size(), line.size());
}

/// Counter-clockwise is positive, clockwise negative.
TEST(Geometry, RingAreaSignAndValue) {
  std::vector<Vrst_vec2d> r = square(0, 2);
  EXPECT_DOUBLE_EQ(ring_area(r), 4);
  std::ranges::reverse(r);
  EXPECT_DOUBLE_EQ(ring_area(r), -4);
}

/// A rotated square is its own smallest rectangle.
TEST(Geometry, MinAreaRectangleOfARotatedSquare) {
  const std::vector<Vrst_vec2d> diamond{{0, -1}, {1, 0}, {0, 1}, {-1, 0}, {0.2, 0.1}};
  const std::vector<Vrst_vec2d> rect = min_area_rectangle(diamond);
  ASSERT_EQ(rect.size(), 4u);
  EXPECT_NEAR(std::abs(ring_area(rect)), 2, 1e-9);
}

/// Inside stays, outside goes, crossing is cut at the border, bigger than the box fills it.
TEST(Geometry, ClipRingToBox) {
  const Vrst_box2d box{{0, 0}, {10, 10}};
  EXPECT_NEAR(ring_area(clip_ring(square(2, 4), box)), 4, 1e-9);
  EXPECT_LT(clip_ring(square(20, 30), box).size(), 3u);
  EXPECT_NEAR(ring_area(clip_ring(square(5, 15), box)), 25, 1e-9);
  EXPECT_NEAR(ring_area(clip_ring(square(-100, 100), box)), 100, 1e-9);
}

/// Consecutive repeats and a closing point equal to the first are removed.
TEST(Geometry, RemoveRepeatedPoints) {
  std::vector<Vrst_vec2d> r{{0, 0}, {1, 0}, {1, 0.0001}, {1, 1}, {0, 1}, {0, 0}};
  remove_repeated_points(r, 0.001);
  EXPECT_EQ(r.size(), 4u);
}

/// Regression (zoom 10 hang on the iPhone): a 20 000-point wavy outline far bigger than a
/// zoom-10 tile is cut to the tile and simplified at the zoom-10 tolerance before
/// triangulation, leaving a few hundred points at most.
TEST(Geometry, HugeOutlineIsCutToOneTile) {
  constexpr Vrst_float64 kTile = 39136;  // zoom-10 tile side, Mercator metres
  std::vector<Vrst_vec2d> forest;
  for (int i = 0; i < 20000; ++i) {
    const Vrst_float64 a = 2 * kPi * i / 20000;
    const Vrst_float64 r = 3 * kTile * (1 + 0.05 * std::sin(a * 400));  // wavy, crosses itself when simplified
    forest.push_back({r * std::cos(a), r * std::sin(a)});
  }
  const Vrst_box2d tile{{0, 0}, {kTile, kTile}};
  std::vector<Vrst_vec2d> part = clip_ring(forest, tile);
  part.push_back(part.front());
  simplify_line(part, 250);
  part.pop_back();
  remove_repeated_points(part, 0.25);
  EXPECT_LT(part.size(), 300u);
  EXPECT_NEAR(std::abs(ring_area(part)), kTile * kTile, kTile * kTile * 0.01);
}
