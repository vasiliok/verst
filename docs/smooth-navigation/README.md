# verst: smooth zoom and scroll on the phone (intention)

> **Status: intention, not started.** This is the plan to make panning and zooming on the
> iPhone feel like Apple Maps: 60 fps, no blank areas, no jumps, and the map keeps moving
> after the finger leaves. It is for the tiled OpenGL ES renderer on `main`.
> Nothing below is implemented yet.

## Context
The phone app (`ios/main.mm`) draws the tiled map (`src/map_tiles.h`, `src/tile_renderer.h`).
It works on the iPhone SE "Bruno" (A13, 60 Hz), but moving the map is not smooth yet:
- **The map stops dead** when the finger lifts. There is no momentum after a pan or a pinch.
- **Double tap and two-finger tap jump** to the new zoom in one frame, with no animation.
- **Holes when zooming out.** A missing tile is covered only by a cached *coarser* tile. When
  zooming out, the coarser level is usually not built yet and the finer tiles are not used,
  so parts of the screen stay empty until the workers finish.
- **Popping at level changes.** At 6, 24 and 96 m/px the map switches to another tile level
  in one frame, so roads and areas appear or disappear at once.
- **Slow tile builds.** Zoomed-out tiles simplify and re-triangulate every forest with
  earcut, and only 2 worker threads build tiles. On the phone, a z10 or z8 tile can take
  seconds.
- **Frame hitches** from work done inside one frame:
  - uploads of up to 8 tiles per frame, whatever their size;
  - deleting evicted tiles;
  - place labels: all 12,000 laid out every frame;
  - a hash lookup of the uniform name on every draw call (`m_u.at("...")`).
- **No measurements on the phone yet.** Every point above comes from reading the code and
  from desktop runs. Step 1 measures first, so the work goes where the time really goes.

The goal: 60 fps (16.7 ms) on the iPhone SE while panning, pinching and flinging across
Bourgogne, with no blank area and no visible pop.

## Snapshot zoom: move a picture while the finger moves, render when it lifts
**Decision:** during a pinch, a zoom animation or a momentum glide, the phone does not
render the map. It moves a cached picture of the map, and renders the real map once the
motion ends.

How it works:
1. **Gesture starts** (pinch began, double tap, Fit/Start animation, fling): render the map
   once into an off-screen texture, the *snapshot*.
   - The snapshot covers the screen plus a 50% margin on every side (iPhone SE: 750x1334 px ->
     about 1500x2000, within `GL_MAX_TEXTURE_SIZE`), so panning and zooming out do not show
     its edge at once.
   - It is rendered with 4x MSAA, then resolved into the texture with `glBlitFramebuffer`
     (GLES 3.0).
   - It holds the map only: roads, areas and lines. No labels, no GPS dot, no track.
2. **While the fingers move:** every frame draws one textured quad, moved, scaled and
   rotated by the difference between the snapshot camera and the current camera.
   - That is one draw call, whatever the map holds, so 60 fps is certain even on the
     iPhone SE.
   - The GPS dot and the track marker are drawn live on top, so they stay sharp and in place.
   - Labels are hidden. Scaled text looks wrong and stale names would sit on the wrong places.
3. **Refresh during a long gesture:** render a new snapshot at the current camera when:
   - the zoom has changed more than 1.6x (zooming in makes the picture blurry);
   - the zoom has changed less than 1/1.6x (zooming out shows the snapshot's edge);
   - the camera has moved past half the margin;
   - or the finger has rested for 150 ms.

   That costs one full render, about one frame, a few times per long pinch: not every frame.
4. **Meanwhile, load ahead:** every frame the loader is asked for the tiles of the *current*
   camera (step 3 of `select_tiles()`), so they are built while the gesture runs and are
   ready when it ends.
5. **Finger lifts:**
   - Without momentum, render the real map at the final camera.
   - With momentum, keep moving the snapshot until the glide stops, then render.
   - Cross-fade over about 150 ms: the real map underneath, the snapshot on top fading out.
     Tiles that are still missing are covered by the fading snapshot and then by the
     fallback tiles, so no blank appears.
   - Labels fade in with the real render.
6. **Panning with one finger:** a pan is only a translation, and the real render may be fast
   enough for it. Use the snapshot for pans only when the measured real frame time is over
   budget (over 12 ms on average over the last 10 frames). Pinch, animations and glides
   always use the snapshot.

Cost:
- one RGBA8 texture of about 12 MB, plus one MSAA renderbuffer for rendering into it;
- no extra tile memory.

This is shared C++ and GL, so navview uses it too, for wheel zoom and animated zoom.

## Gestures that feel right
- **Momentum (inertia):**
  - **Pan:** when the finger lifts, keep the release velocity
    (`UIPanGestureRecognizer velocityInView`) and slow it down each frame with a friction
    factor, about 0.998 per ms, like a `UIScrollView`. Stop below a few points per second.
  - **Pinch:** keep the zoom speed in the same way and zoom around the last centroid.
  - Any new touch stops the momentum at once.
- **Animated zoom:**
  - Double tap, two-finger tap, Fit and Start animate over about 250 ms with ease-out.
  - Zoom is animated in log scale and the center along a line, so the zoom speed looks even.
- **Animation clock:**
  - A `CADisplayLink`, running only while something moves: a gesture, momentum, an
    animation or tiles still loading. When nothing moves, nothing is drawn (battery).
  - It replaces the `paused = NO` loop of the GLKViewController.
- **Shared camera code:**
  - Momentum and animation are plain C++ in `src/map_scene.h` (`Vrst_camera_motion`), with
    no UIKit in it.
  - navview gets the same smoothness with the mouse: wheel zoom animated, drag with momentum.

## Never a blank screen
- **Fallback in both directions:**
  - For a missing tile, draw the nearest cached tile *coarser or finer*.
  - When zooming out, the finer tiles that are still cached cover the area until the coarse
    tile is ready.
  - Fallbacks draw under the real tiles, as today.
- **Prefetch:**
  - Ask the loader for a ring of tiles around the screen, one tile wide.
  - During a pan, extend the ring in the direction of motion.
  - During a pinch, ask for the next level in the zoom direction as well.
  - Visible tiles always come first in the queue, prefetch after them.
- **Keep the levels around the current zoom:** eviction prefers to drop tiles far from the
  screen and levels far from the current zoom, not simply the oldest ones.

## No pop at level changes
- **Hysteresis:** switch to the finer level only at max_mpp x 0.9, and back to the coarser
  one only at max_mpp x 1.1. A zoom that hovers at a threshold no longer flips the level.
- **Cross-fade:**
  - For about 200 ms after a level change, draw both levels.
  - The new level fades in with alpha. Fills and lines get a `u_alpha` uniform, and
    blending is on only during the fade.
- **Matching content at the borders:** z12 keeps the same road classes as z14 except
  service and living_street, both thin and grey, so the switch is hard to see. Check this
  with screenshots on both sides of each threshold.

## Faster tiles
- **More workers:** use (cores - 1), at least 2. The A13 has 6 cores.
- **Spread uploads:** upload by bytes per frame (about 2 MB), not by tile count. Split big
  tiles over several frames.
- **Delete later:** evicted tiles are deleted a few per frame.
- **Ready-made overview tiles:** osm2nav writes z8-z12 tiles ready to use, already
  simplified and triangulated (step 8 of `docs/tiling/README.md`, `tiles.dat`).
  - The phone then never runs earcut, and a zoomed-out tile is one memory-mapped read plus
    one upload.
  - Do this only if step 1 shows that overview builds are the bottleneck.
- **Cheaper draw calls:**
  - look up uniform locations once, into plain fields;
  - set a tile's matrix once per tile, not once per range;
  - merge neighbouring style ranges that share a pass into one draw call.
- **Labels:** keep places in a grid, so layout() reads only the cells on screen, not all
  12,000 places.

## Steps
1. **Measure on the phone.** A debug line in the stats overlay shows:
   - frame time (mean and worst of the last second);
   - tiles visible, missing and building;
   - build time per level;
   - upload MB per frame.

   Add `os_signpost` intervals for build, upload, draw and labels, and record a fling plus a
   pinch over Dijon and the countryside around it in Instruments. Write the numbers into this file.
2. **Snapshot zoom:** `Vrst_map_snapshot` in a new `src/map_snapshot.h`. It holds the
   off-screen render, the quad draw for a camera delta, the refresh rules and the fade.
   `ios/main.mm` uses it for pinch, and navview for wheel zoom.
3. **Momentum, animated zoom and the display link:** `Vrst_camera_motion` in `map_scene.h`.
   Glides and animations run on the snapshot.
4. **Fallback in both directions and prefetch** in `select_tiles()` (`src/tile_renderer.h`).
5. **Hysteresis and cross-fade** at level changes.
6. **Frame budget:** byte-limited uploads, spread deletes, uniform fields, merged draw calls,
   and the label grid.
7. **Faster builds:** more workers; ready-made overview tiles only if step 1 says so.
8. **Measure again** on the iPhone SE, then on an iPhone 13. Write before/after numbers here.

## Critical files
- New: `src/map_snapshot.h` (off-screen snapshot, quad draw, refresh rules, fade).
- Changed:
  - `ios/main.mm`: snapshot during pinch, display link, momentum, animated taps, debug stats.
  - `src/map_scene.h`: `Vrst_camera_motion`, with momentum and zoom animation.
  - `src/tile_renderer.h`: two-way fallback, prefetch, hysteresis, fade, upload budget, uniform fields.
  - `src/map_tiles.h`: loader priorities (visible before prefetch), thread count.
  - `src/map_labels.h`: place grid.
  - `src/navview.cpp`: the same motion for mouse wheel and drag.
- Optional:
  - `src/osm2nav.cpp`, `src/format.h`, `src/nav_files.h` for the ready-made overview tiles.

## Verification
1. **Instruments on the iPhone SE:**
   - a 10 s fling-and-pinch session over Dijon at 1-60 m/pt;
   - 99% of frames under 16.7 ms;
   - no frame over 33 ms after the first second.
2. **No blank area:** record the screen (`xcrun devicectl` screenshot series or a screen
   recording) while zooming from 1 to 400 m/pt and back fast. No frame may show empty
   background where the map has data.
3. **No pop:** screenshots just below and just above each level threshold look the same
   apart from the fade; hovering at a threshold does not flicker.
4. **Gesture feel, tested by hand:**
   - a fling glides and slows down smoothly;
   - a touch stops it at once;
   - a double tap zooms in an animation;
   - Fit and Start animate.
5. **Snapshot:**
   - during a 5 s pinch on the iPhone SE, every frame takes under 5 ms;
   - refreshes happen at the 1.6x steps and not every frame;
   - after release, the real render matches the last snapshot apart from the labels;
   - a screenshot at the end of the fade is the same as a fresh render.
6. **Unit tests (GoogleTest):**
   - the snapshot quad transform: snapshot camera plus current camera give a matrix that
     maps snapshot pixels onto screen pixels, tested on known points, rotated too;
   - the refresh rules fire at 1.6x, 1/1.6x, half the margin and 150 ms of rest;
   - `Vrst_camera_motion`: momentum slows to a stop, an animation ends exactly at the target,
     and a touch cancels the motion;
   - `select_tiles()` fallback: a finer cached tile covers a missing coarse tile;
   - hysteresis does not flip the level back and forth.
7. ctest passes, with 0 build warnings and 0 Doxygen warnings.

## Risks
- **Snapshot blur:** between refreshes, zooming in up to 1.6x shows a slightly soft
  picture. Apple Maps does the same while pinching. If it is too visible, lower the refresh
  step to 1.4x.
- **GPU memory on the phone:** prefetch and keeping neighbouring levels need more cache.
  Watch the 96 MB budget and the memory warnings, and evict harder on
  `didReceiveMemoryWarning`.
- **Fade cost:** drawing two levels with blending doubles the fill work for 200 ms. If the
  iPhone SE drops frames, fade only the lines and switch the areas at once.
- **OpenGL ES is deprecated on iOS.** It still runs on iOS 27, but a future iOS may drop it.
  The motion and tile code is API-free, so moving the renderer to Metal later would keep
  all of this. That is a separate intention.
