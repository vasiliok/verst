# route

`Vrst_router`: shortest routes over the nav files for a car driver, a cyclist or a
walker, the way back, and the nearest road at a point.

| File | What |
|---|---|
| `router.{h,cpp}` | `route()`, `route_pair()`, `road_at()` |

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

`router.cpp` is always built with `-O2`, also in Debug.

Uses: storage (`nav_files.h`, `map_tiles.h`), provider (`route_result.h`).
Used by: provider (`nav_map_provider` owns one router). Try it with `navroute`.
