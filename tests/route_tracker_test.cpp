/// @file
/// Tests of the off-route decision (route_tracker.h).
#include <gtest/gtest.h>

#include "route/route_tracker.h"

using namespace nav;

namespace {

/// Metres per degree of latitude (as in the tracker).
constexpr Vrst_float64 kDeg_m = kEarthRadius * kPi / 180;

/// A road east along 47 N from 5.00 E to 5.02 E (about 1.5 km).
std::vector<Vrst_latlon> east_road() {
  return {{47.0, 5.00}, {47.0, 5.01}, {47.0, 5.02}};
}

/// A point on the road's latitude, some metres north of it.
/// @param lon Longitude.
/// @param north_m Metres north of the road.
/// @return Point.
Vrst_latlon at(Vrst_float64 lon, Vrst_float64 north_m) {
  return {47.0 + north_m / kDeg_m, lon};
}

}  // namespace

/// Driving along the road never reports off route; the distance left goes down.
TEST(RouteTracker, OnRouteStaysOn) {
  Vrst_route_tracker tr(east_road(), Vrst_travel_mode::Car);
  Vrst_float64 left = 1e9;
  for (int s = 0; s < 20; ++s) {
    const Vrst_route_check c = tr.update(at(5.0 + s * 0.0005, 10), 5, s);
    EXPECT_FALSE(c.off_route);
    EXPECT_NEAR(c.off_m, 10, 0.5);
    EXPECT_LT(c.left_m, left);
    left = c.left_m;
  }
}

/// Leaving the road: off route only after kOffTime_s, and only once per departure.
TEST(RouteTracker, LeavingIsReportedAfterTheDelay) {
  Vrst_route_tracker tr(east_road(), Vrst_travel_mode::Car);
  EXPECT_FALSE(tr.update(at(5.005, 0), 5, 0).off_route);
  int reports = 0, first = -1;
  for (int s = 1; s <= 12; ++s) {
    if (tr.update(at(5.005, 100), 5, s).off_route) {
      ++reports;
      if (first < 0) first = s;
    }
  }
  EXPECT_EQ(first, 1 + static_cast<int>(Vrst_route_tracker::kOffTime_s));
  EXPECT_EQ(reports, 2);  // at 6 and 12: the timer starts again after a report
}

/// A short excursion that comes back resets the timer.
TEST(RouteTracker, ComingBackResets) {
  Vrst_route_tracker tr(east_road(), Vrst_travel_mode::Car);
  tr.update(at(5.005, 0), 5, 0);
  for (int s = 1; s <= 4; ++s) EXPECT_FALSE(tr.update(at(5.005, 100), 5, s).off_route);
  EXPECT_FALSE(tr.update(at(5.006, 0), 5, 5).off_route);
  for (int s = 6; s <= 9; ++s) EXPECT_FALSE(tr.update(at(5.006, 100), 5, s).off_route);
}

/// A poor fix widens the tolerance; a very poor one decides nothing.
TEST(RouteTracker, Accuracy) {
  EXPECT_DOUBLE_EQ(Vrst_route_tracker::tolerance_m(Vrst_travel_mode::Foot, 5), 25);
  EXPECT_DOUBLE_EQ(Vrst_route_tracker::tolerance_m(Vrst_travel_mode::Car, 40), 60);
  Vrst_route_tracker tr(east_road(), Vrst_travel_mode::Car);
  tr.update(at(5.005, 0), 5, 0);
  for (int s = 1; s <= 10; ++s) EXPECT_FALSE(tr.update(at(5.005, 500), 100, s).off_route);
}

/// Starting away from the road: waiting there is fine, moving farther away is off route.
TEST(RouteTracker, StartAwayFromTheRoad) {
  Vrst_route_tracker tr(east_road(), Vrst_travel_mode::Car);
  for (int s = 0; s <= 10; ++s) EXPECT_FALSE(tr.update(at(5.0, 120), 5, s).off_route);
  bool off = false;
  for (int s = 11; s <= 20; ++s) off = off || tr.update(at(5.0, 300), 5, s).off_route;
  EXPECT_TRUE(off);
}

/// After arrival nothing is off route.
TEST(RouteTracker, ArrivedNeverOff) {
  Vrst_route_tracker tr(east_road(), Vrst_travel_mode::Foot);
  EXPECT_TRUE(tr.update(at(5.0199, 0), 5, 0).arrived);
  for (int s = 1; s <= 10; ++s) EXPECT_FALSE(tr.update(at(5.03, 200), 5, s).off_route);
}
