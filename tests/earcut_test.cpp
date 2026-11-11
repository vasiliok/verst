/// @file
/// Regression test for the iPhone zoom-10 hang: earcut looped forever on this ring
/// (water area 532134 in tile 10/524/356, simplified, exactly as the phone logged it)
/// when its area() was compiled with a fused multiply-add, as arm64 compilers do by
/// default. CMakeLists.txt builds every file that includes earcut with
/// -ffp-contract=off; on an arm64 host this test hangs without that flag.

#include <gtest/gtest.h>

#include <array>
#include <vector>

#include <mapbox/earcut.hpp>

/// The logged ring triangulates (20 points: 18 triangles at most).
TEST(Earcut, PhoneHangRingTriangulates) {
  const std::vector<std::vector<std::array<double, 2>>> ring{{
      {482091.09613944829, 6084737.4121047603},
      {482313.17852358083, 6084905.6735109743},
      {482743.09439702443, 6084911.9792292435},
      {483013.15548168885, 6085012.0416405611},
      {483007.81214613089, 6084724.3031196222},
      {483017.38562233903, 6085013.0372922979},
      {483279.20906468487, 6085120.7343019946},
      {483387.63424871746, 6085258.8008541437},
      {483878.77584209747, 6085577.2584813526},
      {483921.63384605281, 6085653.7630865313},
      {484224.75681948289, 6085668.6989890737},
      {484198.48541965568, 6085547.0551152779},
      {484230.87939147657, 6085670.6904447051},
      {483917.06974693033, 6085662.7246249476},
      {483870.2042413064, 6085581.5732565951},
      {483412.23585618276, 6085286.6799461432},
      {483277.09399435978, 6085132.3503928604},
      {483153.19540110679, 6085061.1605968289},
      {482742.76043855213, 6084916.2936707232},
      {482310.50685580174, 6084910.1538892658},
  }};
  const std::vector<unsigned> tris = mapbox::earcut<unsigned>(ring);
  EXPECT_GT(tris.size(), 0u);
  EXPECT_EQ(tris.size() % 3, 0u);
  EXPECT_LE(tris.size() / 3, ring[0].size() - 2);
}
