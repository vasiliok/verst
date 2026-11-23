# provider

Where a client gets map data from: one abstract `Vrst_map_provider` with a small API in
the client's terms (opaque keys, degrees, status and events), and today's
implementation on top of the nav files.

Client API (front ends may include these):

| File | What |
|---|---|
| `map_provider.{h,cpp}` | `Vrst_map_provider`: regions, tiles, status, search, routing (`route()`, `route_pair()`, `road_at()`); `open_map()` |
| `map_scene.{h,cpp}` | platform-independent map view: road class tables and colours, the 2D camera and its math, the map frame |
| `route_result.h` | `Vrst_travel_mode`, `Vrst_route`, `Vrst_route_pair` (route and way back), `Vrst_road_at` |
| `search_result.h` | `Vrst_search_kind`, `Vrst_search_result` |

Implementation (clients never include it):

| File | What |
|---|---|
| `nav_map_provider.{h,cpp}` | `Vrst_nav_map_provider`: memory-mapped nav files cut into tiles built by background threads; owns the router and the search index |

Uses: geometry, storage, route, search.
Used by: render, session, tools, the iOS app. The client types are also included by
storage, route and search, so they speak the client's terms at their edges.
