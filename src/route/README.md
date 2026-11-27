# route

[verst](../../README.md) > [src](../README.md) > route

`Vrst_router`: shortest routes over the nav files for a car driver, a cyclist or a
walker, the way back, the nearest road at a point, and routes from a moving car that
respect its direction of travel. `Vrst_route_tracker`: notices when the traveller leaves
the shown route or drives it the wrong way, so the app reroutes. `decide()`: the reroute
rules, read from the car's situation only (no user action).

| File | What |
|---|---|
| `router.{h,cpp}` | `route()`, `route_pair()`, `road_at()`, `route_moving()`, `situation()` |
| `route_tracker.{h,cpp}` | `update(fix, accuracy, t, course)`: distance to the route, metres left, wrong way, off route |
| `reroute_rules.{h,cpp}` | `decide(situation)`: Forward, UTurnAhead, NoUTurn or Any |

How it works:
- **Snap**: the start and the destination snap to the nearest segment of the mode's
  main network (largest connected part, found once by union-find), so a point is never
  snapped onto a small cut-off road.
- **Search**: A* over the arcs the mode may use (mode bits in the arc weight, format 4);
  the heuristic is the straight distance at the mode's top speed. Labels are per arc,
  not per vertex, so the arc a car came in on is known where it leaves: leaving on the
  same segment backwards is a U-turn and is priced (table below). The label arrays are
  kept per thread between searches.
- **U-turns** (cars; bikes and walkers turn anywhere for free):

  | Where | U-turn |
  |---|---|
  | motorway, trunk and their ramps | never |
  | plain road node (two roads meet, no intersection) | never |
  | intersection or dead end, in town | 1800 s (only when nothing else works) |
  | intersection or dead end, outside town | 60 s |

  A roundabout is not a U-turn (the car goes round on other segments). Town = class
  residential, living street or service, or a limit of 50 km/h or less.
- **Way back** (`route_pair()`): a second A* from the destination, where arcs off the
  outbound roads cost `kOffRouteFactor` (3) times more, so it keeps to the same roads
  and leaves them only where it must (a oneway). Twin segments of dual carriageways
  (same name or ref nearby) count as the same road.
- Maps made before format 4 route cars only.

Situation of a moving car (`situation()`, `Vrst_road_situation`): its road snapped in
the direction of travel (the car's own carriageway wins over a nearer one by 25 m), the
road type (Highway = motorway/trunk; Town = town street, or within 3 km of a city,
1.5 km of a town, 700 m of a village; else Rural), the road name, the speed, and the
events of the next 3 km: intersections, roundabouts, exits (ramps leaving), dead ends,
following the same name or number, else the straightest way.

Reroute rules (`decide()`, in order):
1. Highway above 50 km/h: Forward - on in the direction of travel to the next exit (the
   driver has not found a way to turn and is not about to stop). At 50 km/h or less on
   a highway this rule is off.
2. Town, whatever the speed: NoUTurn - round the blocks, every U-turn 1800 s.
3. A roundabout or an intersection ahead where turning back is allowed: UTurnAhead
   there (free U-turn at that vertex); a roundabout wins over a nearer intersection by
   up to 500 m.
4. Anything else while moving: Forward. Not moving (under 5 km/h) or no heading: Any.

`route_moving()` starts only in the direction of travel (unless Any) and returns the
situation, the strategy and the number of U-turns with the route.

Rerouting (`Vrst_route_tracker`, client types only):
- Each GPS fix is measured against the route line in local flat metres, in a window
  from 200 m behind the progress to 2 km ahead (the whole line for the first fix).
- Off = farther than max(car 40 / bike 30 / foot 25 m, 1.5 x accuracy), or on the line
  with a course more than 120 deg against the route (wrong way: the car turned round).
  Fixes worse than 60 m decide nothing. Off for 5 s in a row = off route, reported once.
- Before the first fix reaches the line (the route starts on the nearest road), only
  moving farther away than the first fix counts as off.
- The last 200 m are arrival: the destination is often off the road, so leaving the
  road there is not off route.
- The iPhone app (`checkRoute` in `ios/main.mm`) follows only routes that start at the
  GPS position, reroutes with `route_moving()` while the car moves (else `route()`; the
  way back stays), at most every 10 s, and does not move the camera. Log: `route_check`
  (off_route, wrong_way) and `route` with `reroute`, `strategy`, `road_type`, `road`,
  `kmh`, `heading`, `uturns` and `ahead` ("X120 R600 E900!", ! = no U-turn there). The
  stats box shows the situation once a second: `RURAL 70 km/h X503 X1155 -> uturn`.

`router.cpp` is always built with `-O2`, also in Debug.

Uses: storage (`nav_files.h`, `map_tiles.h`), provider (`route_result.h`), geometry
(`geo.h`).
Used by: provider (`nav_map_provider` owns one router). Try it with `navroute`
(`heading=<deg> speed=<kmh>` routes as a moving car and prints the situation;
`gpx=<file>` writes the route as a test drive).
