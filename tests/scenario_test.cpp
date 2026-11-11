/// @file
/// GoogleTest tests of the scripted map motion (src/scenario.h): parsing, exact totals at
/// any frame rate, easing, tile waits and jumps.

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "scenario.h"

using namespace nav;

namespace {

/// A phone-like screen: 1170 x 2532 px, 3 px per point, wide zoom limits.
/// @param tiles_ready Value of Vrst_scenario_frame::tiles_ready.
/// @return Frame description.
Vrst_scenario_frame phone(bool tiles_ready = true) {
  return {1170, 2532, 3.0, 1e-6, 1e6, tiles_ready};
}

/// Plays a scenario to its end with a fixed frame interval.
/// @param s Scenario, started at 0 here.
/// @param dt Frame interval in seconds.
/// @return Final camera.
Vrst_view play(Vrst_scenario s, Vrst_float64 dt) {
  Vrst_view v{{100, -50}, 0.5, 0};
  s.start(0);
  for (Vrst_float64 t = 0; s.running() && t < 1000; t += dt) s.advance(t, v, phone());
  return v;
}

}  // namespace

/// The built-in scenario parses, and its steps have the expected kinds.
TEST(Scenario, StandardParses) {
  const Vrst_scenario s = Vrst_scenario::parse("standard", kStandardScenario);
  ASSERT_GE(s.steps().size(), 10u);
  EXPECT_EQ(s.steps().front().kind, Vrst_step_kind::WaitTiles);
  EXPECT_GT(s.max_seconds(), 20.0);
}

/// Bad lines are refused with their line number; comments and blank lines are fine.
TEST(Scenario, ParseErrorsNameTheLine) {
  EXPECT_NO_THROW(Vrst_scenario::parse("ok", "# comment\n\npan 1 2 3  # trailing\n"));
  try {
    Vrst_scenario::parse("bad", "wait 1\njump 3\n");
    FAIL() << "unknown step accepted";
  } catch (const std::runtime_error& e) {
    EXPECT_NE(std::string(e.what()).find("line 2"), std::string::npos) << e.what();
  }
  EXPECT_THROW(Vrst_scenario::parse("bad", "pan 1 x 2\n"), std::runtime_error);
  EXPECT_THROW(Vrst_scenario::parse("bad", "zoom 0 1\n"), std::runtime_error);
  EXPECT_THROW(Vrst_scenario::parse("bad", "wait -1\n"), std::runtime_error);
  EXPECT_THROW(Vrst_scenario::parse("bad", "# nothing\n"), std::runtime_error);
}

/// The total motion does not depend on the frame rate: 60 fps and 2 fps end at the same
/// camera, which equals applying each step at once.
TEST(Scenario, SameEndAtAnyFrameRate) {
  const Vrst_scenario s = Vrst_scenario::parse("mix", "pan 0 -300 2\nfling 600 200 1.2\nzoom 4 2\nrotate 45 1.5\n");
  const Vrst_view fast = play(s, 1.0 / 60), slow = play(s, 0.5);
  Vrst_view want{{100, -50}, 0.5, 0};
  const Vrst_scenario_frame f = phone();
  pan_by(want, 0, -300 * 3.0);
  pan_by(want, 600 * 3.0, 200 * 3.0);
  zoom_at(want, f.fb_w, f.fb_h, f.fb_w / 2.0, f.fb_h / 2.0, 4, f.min_scale, f.max_scale);
  rotate_at(want, f.fb_w, f.fb_h, f.fb_w / 2.0, f.fb_h / 2.0, 45 * kPi / 180);
  for (const Vrst_view& v : {fast, slow}) {
    EXPECT_NEAR(v.scale, want.scale, 1e-12);
    EXPECT_NEAR(v.angle, want.angle, 1e-12);
    EXPECT_NEAR(v.center.x, want.center.x, 1e-6);
    EXPECT_NEAR(v.center.y, want.center.y, 1e-6);
  }
}

/// A fling moves most of its way early; a pan is symmetric (half way at half time).
TEST(Scenario, FlingStartsFast) {
  for (const auto& [text, at_half_min, at_half_max] :
       {std::tuple{"fling 1000 0 1", 0.8, 0.95}, std::tuple{"pan 1000 0 1", 0.49, 0.51}}) {
    Vrst_scenario s = Vrst_scenario::parse("p", text);
    Vrst_view v{{0, 0}, 1, 0};
    s.start(0);
    s.advance(0.5, v, phone());
    // pan_by moves the center opposite to the drag: x went from 0 to -moved px.
    const Vrst_float64 moved = -v.center.x / 3000.0;
    EXPECT_GT(moved, at_half_min) << text;
    EXPECT_LT(moved, at_half_max) << text;
  }
}

/// wait_tiles ends as soon as the tiles are ready, or at its limit.
TEST(Scenario, WaitTilesEndsEarlyOrAtLimit) {
  Vrst_scenario s = Vrst_scenario::parse("w", "wait_tiles 5\nwait 1\n");
  Vrst_view v;
  s.start(0);
  s.advance(1, v, phone(false));
  EXPECT_EQ(s.step_index(), 0u);
  const Vrst_scenario_tick t = s.advance(2, v, phone(true));
  EXPECT_TRUE(t.step_changed);
  EXPECT_EQ(s.step_index(), 1u);
  EXPECT_TRUE(s.advance(3.1, v, phone()).finished);

  s.start(0);
  s.advance(4.9, v, phone(false));
  EXPECT_EQ(s.step_index(), 0u);
  s.advance(5.1, v, phone(false));
  EXPECT_EQ(s.step_index(), 1u);
}

/// goto is handed to the front end once, and later steps wait for the next call.
TEST(Scenario, GotoIsReturnedOnce) {
  Vrst_scenario s = Vrst_scenario::parse("g", "goto 47.32 5.04 20\npan 100 0 0\n");
  Vrst_view v;
  s.start(0);
  const Vrst_scenario_tick a = s.advance(0, v, phone());
  ASSERT_TRUE(a.jump.has_value());
  EXPECT_DOUBLE_EQ(a.jump->at.lat, 47.32);
  EXPECT_DOUBLE_EQ(a.jump->mpp, 20);
  EXPECT_EQ(s.step_index(), 1u);
  const Vrst_scenario_tick b = s.advance(0.01, v, phone());
  EXPECT_FALSE(b.jump.has_value());
  EXPECT_TRUE(b.finished);
}

/// stop() ends playing without touching the camera.
TEST(Scenario, StopHalts) {
  Vrst_scenario s = Vrst_scenario::parse("s", "pan 100 0 1\n");
  Vrst_view v;
  s.start(0);
  s.advance(0.2, v, phone());
  const Vrst_view before = v;
  s.stop();
  const Vrst_scenario_tick t = s.advance(0.8, v, phone());
  EXPECT_FALSE(s.running());
  EXPECT_FALSE(t.moved);
  EXPECT_DOUBLE_EQ(v.center.x, before.center.x);
}
