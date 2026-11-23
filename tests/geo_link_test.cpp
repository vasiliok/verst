/// @file
/// Tests of points read from map links and coordinate text (geo_link.h).
#include <gtest/gtest.h>

#include "search/geo_link.h"

using namespace nav;

/// Google's place pin wins over the map center; the place name is taken.
TEST(GeoLink, GooglePlacePin) {
  const auto l = parse_geo_link(
      "https://www.google.com/maps/place/Jardin+Darcy/@47.3213,5.0300,17z/data=!3m1!4b1!4m6!3m5!1s0x0:0x0!8m2"
      "!3d47.3222!4d5.0315!16s");
  ASSERT_TRUE(l);
  EXPECT_DOUBLE_EQ(l->pos.lat, 47.3222);
  EXPECT_DOUBLE_EQ(l->pos.lon, 5.0315);
  EXPECT_EQ(l->name, "Jardin Darcy");
  EXPECT_TRUE(l->exact);
}

/// Coordinate parameters, plain and escaped, geo: and OSM links.
TEST(GeoLink, CoordinateLinks) {
  EXPECT_DOUBLE_EQ(parse_geo_link("https://maps.google.com/?q=47.322,5.041")->pos.lon, 5.041);
  EXPECT_DOUBLE_EQ(parse_geo_link("https://www.google.com/maps/search/?api=1&query=47.322%2C5.041")->pos.lat, 47.322);
  EXPECT_DOUBLE_EQ(parse_geo_link("https://maps.google.com/?q=loc:-33.85+151.21")->pos.lat, -33.85);
  EXPECT_DOUBLE_EQ(parse_geo_link("geo:47.322,5.041?z=15")->pos.lon, 5.041);
  EXPECT_DOUBLE_EQ(parse_geo_link("https://www.openstreetmap.org/#map=17/47.32200/5.04100")->pos.lat, 47.322);
  EXPECT_DOUBLE_EQ(parse_geo_link("https://www.openstreetmap.org/?mlat=47.322&mlon=5.041")->pos.lon, 5.041);
}

/// Only the map center: found, marked not exact.
TEST(GeoLink, MapCenterOnly) {
  const auto l = parse_geo_link("https://www.google.com/maps/@47.322,5.041,15z");
  ASSERT_TRUE(l);
  EXPECT_FALSE(l->exact);
}

/// Coordinates as text: decimal and degrees-minutes-seconds; typed words and numbers are not points.
TEST(GeoLink, CoordinateText) {
  EXPECT_DOUBLE_EQ(parse_geo_link(" 47.32222, 5.04167 ")->pos.lat, 47.32222);
  const auto d = parse_geo_link("47\xC2\xB0" "19'20.0\"N 5\xC2\xB0" "02'30.0\"E");
  ASSERT_TRUE(d);
  EXPECT_NEAR(d->pos.lat, 47.32222, 1e-5);
  EXPECT_NEAR(d->pos.lon, 5.04167, 1e-5);
  EXPECT_FALSE(parse_geo_link("12 3"));
  EXPECT_FALSE(parse_geo_link("12 rue de la paix"));
  EXPECT_FALSE(parse_geo_link("95.0, 5.0"));
}

/// A shared place without coordinates gives its place text; short links are recognised.
TEST(GeoLink, PlaceTextAndShortLinks) {
  const auto a = parse_link_address("https://maps.google.com?q=Cafe+du+Port,+12+Rue+de+la+Libert%C3%A9,+21000+Dijon,+France&ftid=0x1:0x2");
  ASSERT_TRUE(a);
  EXPECT_EQ(*a, "Cafe du Port, 12 Rue de la Libert\xC3\xA9, 21000 Dijon, France");
  EXPECT_FALSE(parse_geo_link("https://maps.google.com?q=Cafe+du+Port,+21000+Dijon"));
  EXPECT_FALSE(parse_link_address("rue de la paix"));
  EXPECT_TRUE(is_short_map_link("https://maps.app.goo.gl/AbCdEf?g_st=ic"));
  EXPECT_FALSE(is_short_map_link("https://www.google.com/maps/@47.3,5.0,15z"));
}
