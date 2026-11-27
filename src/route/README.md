# route

[verst](../../README.md) > [src](../README.md) > route

`Vrst_router`: shortest routes over the nav files for a car driver, a cyclist or a
walker, the way back, and the nearest road at a point. `Vrst_route_tracker`: notices
when the traveller leaves the shown route, so the app reroutes.

| File | What |
|---|---|
| `router.{h,cpp}` | `route()`, `route_pair()`, `road_at()` |
| `route_tracker.{h,cpp}` | `update(fix, accuracy, t)`: distance to the route, metres left, off route |

How it works:
- **Snap**: the start and the destination snap to the nearest segment of the mode's
  main network (largest connected part, found once by union-find), so a point is never
  snapped onto a small cut-off road.
- **Search**: A* over the arcs the mode may use (mode bits in the arc weight, format 4);
  the heuristic is the straight distance at the mode's top speed.
- **Way back** (`route_pair()`): a second A* from the destination, where arcs off the
  outbound roads cost `kOffRouteFactor` (3) times more, so it keeps to the same roads
  and leaves them only where it must (a oneway). Twin segments of dual carriageways
  (same name or ref nearby) count as the same road.
- Maps made before format 4 route cars only.

Rerouting (`Vrst_route_tracker`, client types only):
- Each GPS fix is measured against the route line in local flat metres, in a window
  from 200 m behind the progress to 2 km ahead (the whole line for the first fix).
- Off = farther than max(car 40 / bike 30 / foot 25 m, 1.5 x accuracy). Fixes worse
  than 60 m decide nothing. Off for 5 s in a row = off route, reported once.
- Before the first fix reaches the line (the route starts on the nearest road), only
  moving farther away than the first fix counts as off.
- The last 200 m are arrival: the destination is often off the road, so leaving the
  road there is not off route.
- The iPhone app (`checkRoute` in `ios/main.mm`) follows only routes that start at the
  GPS position, reroutes with `route()` (the way back stays), at most every 10 s, and
  does not move the camera. Log: `route_check` (off_route) and `route` with `reroute`.

`router.cpp` is always built with `-O2`, also in Debug.

Uses: storage (`nav_files.h`, `map_tiles.h`), provider (`route_result.h`), geometry
(`geo.h`).
Used by: provider (`nav_map_provider` owns one router). Try it with `navroute`.
