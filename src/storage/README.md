# storage

[verst](../../README.md) > [src](../README.md) > storage

The nav files and the tiled map: the data side. Internal to the nav map provider;
clients never include these files (except the shared vocabulary in `map_kinds.h` and
`map_piece.h`, which the provider API hands out).

| File | What |
|---|---|
| `format.h` | binary layout of every nav file (header, sections), format version (4; version 3 still loads), arc and segment bits incl. travel modes |
| `map_kinds.h` | road classes, place, area and line kinds: shared vocabulary of the converter, the tiles and the renderer |
| `map_piece.h` | `Vrst_map_piece`: ready-to-draw content of one tile (area triangles, line pieces, dots) as the provider delivers it |
| `nav_files.{h,cpp}` | typed zero-copy views over the mapped files (`Vrst_graph`, `Vrst_map_layers`, `Vrst_nav_data`); loading checks magic, version and section bounds |
| `map_tiles.{h,cpp}` | the Web Mercator z/x/y grid, zoom levels, what each tile holds, the tile builder and the background loader |

`map_tiles.cpp` is built with `-ffp-contract=off` (earcut, see `VRST_EARCUT_SOURCES`).

Uses: base (mapped files), geometry, the client types in provider (`map_provider.h`,
`map_scene.h`).
Used by: provider (`nav_map_provider`), route, convert (format), navcheck.
