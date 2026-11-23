/// @file
/// GoogleTest tests of the activity recorder (src/session/activity_recorder.h): a recorded
/// camera path is a valid scenario, and playing it ends at the recorded camera.

#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "session/activity_recorder.h"
#include "session/scenario.h"

using namespace nav;

namespace {

/// World origin near Dijon, absolute Web Mercator metres.
const Vrst_vec2d kOrigin = mercator({47.32, 5.04});

/// A phone-like screen with the Dijon world origin.
/// @return Frame description.
Vrst_scenario_frame phone() {
  Vrst_scenario_frame f{1170, 2532, 3.0, 1e-6, 1e6, true};
  f.world_origin = kOrigin;
  return f;
}

}  // namespace

/// Pan, pause, zoom and rotate are recorded, parsed back and played to the same camera.
TEST(ActivityRecorder, PlaybackEndsAtTheRecordedCamera) {
  Vrst_activity_recorder rec;
  Vrst_view v{{0, 0}, 0.5, 0};
  rec.start(0, v, kOrigin, 3.0);
  Vrst_float64 t = 0;
  for (int i = 0; i < 60; ++i) {  // 1 s of panning at 60 fps
    t += 1.0 / 60;
    v.center.x += 20;
    rec.sample(t, v);
  }
  t += 2;  // stands still
  rec.sample(t, v);
  for (int i = 0; i < 30; ++i) {  // zoom in and rotate
    t += 1.0 / 60;
    v.scale *= 1.02;
    v.angle += 0.01;
    rec.sample(t, v);
  }
  const Vrst_view recorded = v;
  const std::string text = rec.finish(t);
  EXPECT_FALSE(rec.recording());
  EXPECT_NE(text.find("wait "), std::string::npos) << text;

  Vrst_scenario s = Vrst_scenario::parse("rec", text);
  EXPECT_NEAR(s.max_seconds(), t, 0.1);
  Vrst_view played{{5000, 5000}, 0.1, 1};  // playback starts anywhere: the first line jumps
  s.start(0);
  for (Vrst_float64 p = 0; s.running() && p < 100; p += 1.0 / 60) s.advance(p, played, phone());
  EXPECT_NEAR(played.center.x, recorded.center.x, 0.05);
  EXPECT_NEAR(played.center.y, recorded.center.y, 0.05);
  EXPECT_NEAR(played.scale / recorded.scale, 1.0, 1e-3);
  EXPECT_NEAR(played.angle, recorded.angle, 1e-3);
}

/// The view step rejects bad values like the other steps.
TEST(ActivityRecorder, ViewStepIsChecked) {
  EXPECT_NO_THROW(Vrst_scenario::parse("ok", "view 0.5 47.3 5.0 2.5 10"));
  EXPECT_THROW(Vrst_scenario::parse("bad", "view 0.5 47.3 5.0 0 10"), std::runtime_error);
  EXPECT_THROW(Vrst_scenario::parse("bad", "view 0.5 95 5.0 2 10"), std::runtime_error);
  EXPECT_THROW(Vrst_scenario::parse("bad", "view 0.5 47.3 5.0 2"), std::runtime_error);
}
