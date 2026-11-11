# verst - Step 1: OSM -> binary graph + viewer

Converts car roads from an `.osm.pbf` into four mmap-able files, validates them, and draws them.

| Tool | Usage |
|---|---|
| `osm2nav` | `osm2nav <input.osm.pbf> <out_dir>` |
| `navcheck` | `navcheck <out_dir>` (exit 0 = all PASS) - `navcheck <out_dir> --dump-way <osm_way_id>` |
| `navview` | `navview <out_dir> [--center <lat> <lon>] [--mpp <metres>] [--gpx <file.gpx> [--speed <x>] [--track-time <s>]] [--screenshot <file.bmp>]` (screenshot: renders 3 frames, saves the last, exits) |
| iPhone app | `ios/`: UIKit + OpenGL ES 3.0, same map code as navview; see [ios/README.md](ios/README.md) |

## Desktop and mobile

One C++ core, thin front ends:

| Layer | Files | Used by |
|---|---|---|
| map provider (client API: extent, places, show / take_ready, status, events) | `src/map_provider.h`, `src/map_piece.h`, `src/map_kinds.h`, `src/geo.h` | navview, iPhone app, renderer |
| nav map provider (today's implementation: nav files, z/x/y tiles, worker threads) | `src/nav_map_provider.h`, `src/map_tiles.h` | only `open_map()` and tests |
| nav files (mmap, typed views) | `src/format.h`, `src/mapped_file.h`, `src/nav_files.h` | osm2nav, navcheck, nav map provider |
| map view (class tables, camera, map frame) | `src/map_scene.h` | navview, iPhone app |
| OpenGL renderer (GL 3.3 core / GLES 3.0) | `src/tile_renderer.h`, `src/map_renderer.h`, `src/shaders.h`, `src/gl_api.h` | navview, iPhone app |
| desktop front end (GLFW, Dear ImGui, mouse/keys) | `src/navview.cpp` | macOS, Linux, Windows |
| iPhone front end (UIKit, GLKView, gestures) | `ios/main.mm` | iOS |

New features go into the core first; a front end only adds input and UI.

The front ends and the renderer get map data only through `Vrst_map_provider`
(`open_map(location)`). They never include a storage header (format.h, nav_files.h,
map_tiles.h), so the storage can change (for example to the pre-built tiles of
[docs/storage](docs/storage/README.md)) without touching them. The provider reports its
work for debugging: `status()` (threads, busy workers and how long, queue, missing,
`settled` = everything on screen loaded), `take_events()` (Started, Done, Failed,
Settled) and `describe()` (free text about the data).

## Build

```sh
git submodule update --init      # libosmium + protozero (or clone with: git clone --recursive <repo>)
mkdir -p data && curl -L -o data/andorra.osm.pbf https://download.geofabrik.de/europe/andorra-latest.osm.pbf

cmake -S . -B build                    # Release by default; -DNAV_TEST_PBF=<pbf> to use another file
cmake --build build -j
ctest --test-dir build --output-on-failure   # runs osm2nav + navcheck on NAV_TEST_PBF
```

Dependencies: zlib (system). GLFW 3.4, Dear ImGui v1.93.0 and FreeType 2.13.3 are fetched with FetchContent; the
DejaVu Sans label fonts are downloaded into `<build>/fonts` at configure time. The glad GL 3.3 core loader is
pre-generated in `third_party/glad/` (no Python needed at configure time; see its README to regenerate).
Use `-DNAV_BUILD_VIEWER=OFF` to build without GLFW/OpenGL.

## Docs

All comments are Doxygen (`///`, `///<`, `@file`, `@return`). All settings are in the
[`Doxyfile`](Doxyfile) at the repo root. Install Doxygen and Graphviz (`brew install doxygen
graphviz`), then either:

```sh
doxygen                               # from the repo root -> ../tmp/verst/doxygen/html/index.html
cmake --build build --target docs     # -> build/docs/html/index.html (adds the generated vrst_types.h)
```

Both also write XML next to `html/` (`xml/index.xml` lists every compound, one `<refid>.xml` per
class, file, namespace and page; schema in `xml/compound.xsd`), for tools that read the API.
Both write every warning to `warnings.log` next to `html/`; the project rule is 0 warnings.
Class, collaboration and include diagrams need Graphviz; without it set `HAVE_DOT = NO`.
The CMake target is skipped when Doxygen is not found.

Articles (also rendered as Doxygen pages):

- [How Vrst_graph works](docs/vrst_graph.md): forward/backward CSR, arc fields, how osm2nav builds it, file layout and costs, and how the nav_files.h classes and file ids link together.
- [Where the data lives](docs/storage/README.md): storage decision (proposal) - binary memory-mapped files for routing and pre-built tiles, SQLite (FTS5 + trigram + R*Tree) for search and user data, and the criteria and benchmark for choosing the storage engine.
- [Architectural bottlenecks & hard engineering challenges](docs/challenges/README.md): the risks on the way from map viewer to turn-by-turn navigation (turn restrictions, contraction hierarchies, map matching, label collisions, maneuvers, offline search, updates).

## Scalar types

All code uses the `Vrst_` scalar aliases (`Vrst_int8`...`Vrst_int64`, `Vrst_uint8`...`Vrst_uint64`, `Vrst_float32`,
`Vrst_float64`, `Vrst_size`). CMake generates them into `<build>/generated/vrst_types.h` from
`src/vrst_types.h.in`; set one with e.g. `-DVRST_TYPE_UINT32="unsigned int"`. Each alias is checked for size,
signedness and IEEE 754 at compile time, so a wrong setting fails the build.

## Viewer controls

drag = pan - wheel = zoom at cursor - `Q`/`E` = rotate - `1`-`9` = toggle motorway, trunk, primary,
secondary, tertiary, unclassified, residential (+living_street), service, ferry - `O` = highlight oneway -
`V` = vertices (red = dead end) - `R` = start view - `F` = fit map - `Esc` = quit. The Dear ImGui panel (top left) has the
same toggles plus dot size and rotation sliders; input over the panel does not move the map.
The viewer starts at `--center <lat> <lon>` with `--mpp` ground metres per screen point (smaller = closer).
Without them it uses the default point 47.322, 5.041 (`kDefaultPoint` in `navview.cpp`) at 5 m per point.
If the data does not contain the start point, it starts with the whole map in view. Example, the hairpins of
the CS 600 in Andorra: `navview <out_dir> --center 42.48014 1.45493 --mpp 1.1`. The window title shows counts, FPS, the
lat/lon under the cursor and the zoom.

**Place names.** Cities, towns, villages and hamlets (OSM `place=*` nodes with a name) are labelled with text drawn
from a FreeType signed-distance-field glyph atlas (`src/glyph_atlas.h`, DejaVu Sans) with a dark halo. Smaller
places appear only when zoomed in (town < 250, village < 45, hamlet < 9 m per point); labels are placed by
priority and dropped when they would overlap a more important one (`src/map_labels.h`). `L` toggles them.

**GPS track replay.** `--gpx <file.gpx>` loads a recorded track (`src/gpx_track.h`, shared with the iOS app) and
moves a red marker along it at the recorded times, multiplied by `--speed` (default 1). The track is blue, the
driven part yellow, and the camera follows the marker. `--track-time <s>` starts later in the track; without
`--center` the view starts at the marker. `Space` = play/pause, `C` = follow (a map drag turns it off); the panel
has time and speed sliders. Example: `navview <bourgogne_dir> --gpx drive.gpx --speed 2 --mpp 3`.

## File format (v2)

Every file: `FileHeader` (64 B) + `SectionEntry[n]` (24 B each) + sections at 64-byte aligned offsets.
Little-endian, offsets and indexes only. All four files of one import share `build_id`; the loader
refuses mixed sets. Layout is in `src/format.h`; the loader is `src/nav_files.h`.

| File | Sections |
|---|---|
| `graph.dat` `NAVGRAPH` | `VERTEX_COORD` Coord[V] - `FWD_FIRST_OUT` u32[V+1] - `FWD_ARCS` {head, weight ds, seg\|bit31 reversed}[A] - `BWD_FIRST_IN` u32[V+1] - `BWD_ARCS` {tail, weight, fwd_arc}[A] - `BBOX` {min,max}[1] |
| `edges.dat` `NAVEDGES` | `SEG_ATTR` {length_dm, name_id, ref_id, class, maxspeed, flags}[S] - `SEG_VERTICES` {a,b}[S] - `SEG_OSM_WAY` u64[S] |
| `geom.dat` `NAVGEOM` | `SEG_FIRST_POINT` u32[S+1] - `POINTS` Coord[P] (each range includes both end points) |
| `names.dat` `NAVNAMES` | `STR_OFFSETS` u32[N+1] - `STR_DATA` UTF-8 (id 0 = empty, deduplicated) - `PLACES` Place[L] 20 B (city/town/village/hamlet label: pos, name_id, population, kind; sorted by kind, then population) |

Coord = int32 lat/lon in 1e-7 deg. A segment is an undirected road piece between two vertices, stored in
OSM way order (a -> b). Two-way = 2 arcs, oneway = 1 arc. For `oneway=-1` the single arc runs b -> a with
bit31 set.

Additions beyond the original spec:
- `BBOX` section in `graph.dat`: the extent of **all nodes in the source file**. The PBF header bbox is
  not used because Geofabrik extracts contain complete ways that reach outside it.
- `SegFlags::kSegOnewayReverse` (bit 7): set with `kSegOneway` when the allowed direction is b -> a.

## Andorra (Geofabrik, 3.5 MB)

4,073 kept ways -> V 5,019 - S 5,966 - A 9,657 - P 44,266 - 553.8 km of road. The conversion takes about
55 ms with an 18 MiB peak RSS, and `navcheck` passes every check (32 components; the largest holds 98.7%
of the vertices).

## Known limits

- Car profile is simple: class default speeds, or `maxspeed` (km/h, mph, `FR:*`, `walk`). It ignores
  `maxspeed:forward/backward`, `access:conditional`, `oneway=reversible|alternating` (these are treated as
  two-way) and ferry `duration`.
- Integer `maxspeed` only (`"7.5"` is parsed as 7).
- Roads at the edge of an extract end in dead ends (red dots) where the extract cuts them.
- Float precision: positions are float metres relative to the map center, so the error is about 2 mm at
  25 km from the center. A country-sized extract will need tile-local origins at extreme zoom.
- Vertices are numbered in OSM node id order. Hilbert ordering goes in `number_vertices()` in `osm2nav.cpp`.
- 1 px lines only. No spatial index, routing or turn restrictions yet (`SEG_OSM_WAY` is kept for mapping
  restrictions later).
