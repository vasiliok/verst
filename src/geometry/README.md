# geometry

Coordinates and plane geometry. No dependencies inside `src/` except the scalar types.

| File | What |
|---|---|
| `geo.h` | coordinates in the client's terms: `Vrst_latlon` (degrees), Web Mercator metres and boxes of both, conversions and distances |
| `geometry.h` | header-only plane geometry over any point type: line simplification, ring area, smallest enclosing rectangle, clipping a ring to a box, removing repeated points |
| `region_outline.{h,cpp}` | outline of a map region read from the Geofabrik `.poly` files (`parse_poly()`, `region_shape()`), drawn as the region border |

`region_outline.cpp` is built with `-ffp-contract=off` (one of `VRST_EARCUT_SOURCES` in
CMakeLists.txt) so earcut triangulates the same on arm64 as on x86.

Used by: everything that handles positions (storage, provider, search, session, render,
convert).
