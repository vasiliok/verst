# nav — Step 1: OSM → binary graph + viewer

Converts car roads from an `.osm.pbf` into four mmap-able files, validates them, and draws them.

| Tool | Usage |
|---|---|
| `osm2nav` | `osm2nav <input.osm.pbf> <out_dir>` |
| `navcheck` | `navcheck <out_dir>` (exit 0 = all PASS) · `navcheck <out_dir> --dump-way <osm_way_id>` |
| `navview` | `navview <out_dir>` · `navview <out_dir> --screenshot <file.bmp>` (renders 3 frames, saves the last, exits) |

## Build

```sh
git submodule update --init      # libosmium + protozero (or clone with: git clone --recursive <repo>)
mkdir -p data && curl -L -o data/andorra.osm.pbf https://download.geofabrik.de/europe/andorra-latest.osm.pbf

cmake -S . -B build                    # Release by default; -DNAV_TEST_PBF=<pbf> to use another file
cmake --build build -j
ctest --test-dir build --output-on-failure   # runs osm2nav + navcheck on NAV_TEST_PBF
```

Dependencies: zlib (system). GLFW 3.4 and Dear ImGui v1.93.0 are fetched with FetchContent. The glad GL 3.3 core loader is
pre-generated in `third_party/glad/` (no Python needed at configure time; see its README to regenerate).
Use `-DNAV_BUILD_VIEWER=OFF` to build without GLFW/OpenGL.

## Viewer controls

drag = pan · wheel = zoom at cursor · `Q`/`E` = rotate · `1`–`9` = toggle motorway, trunk, primary,
secondary, tertiary, unclassified, residential (+living_street), service, ferry · `O` = highlight oneway ·
`V` = vertices (red = dead end) · `R` = reset · `Esc` = quit. The Dear ImGui panel (top left) has the
same toggles plus dot size and rotation sliders; input over the panel does not move the map. The window title shows counts, FPS, the
lat/lon under the cursor and the zoom.

## File format (v1)

Every file: `FileHeader` (64 B) + `SectionEntry[n]` (24 B each) + sections at 64-byte aligned offsets.
Little-endian, offsets and indexes only. All four files of one import share `build_id`; the loader
refuses mixed sets. Layout is in `src/format.h`; the loader is `src/nav_files.h`.

| File | Sections |
|---|---|
| `graph.dat` `NAVGRAPH` | `VERTEX_COORD` Coord[V] · `FWD_FIRST_OUT` u32[V+1] · `FWD_ARCS` {head, weight ds, seg\|bit31 reversed}[A] · `BWD_FIRST_IN` u32[V+1] · `BWD_ARCS` {tail, weight, fwd_arc}[A] · `BBOX` {min,max}[1] |
| `edges.dat` `NAVEDGES` | `SEG_ATTR` {length_dm, name_id, ref_id, class, maxspeed, flags}[S] · `SEG_VERTICES` {a,b}[S] · `SEG_OSM_WAY` u64[S] |
| `geom.dat` `NAVGEOM` | `SEG_FIRST_POINT` u32[S+1] · `POINTS` Coord[P] (each range includes both end points) |
| `names.dat` `NAVNAMES` | `STR_OFFSETS` u32[N+1] · `STR_DATA` UTF-8 (id 0 = empty, deduplicated) |

Coord = int32 lat/lon in 1e-7°. A segment is an undirected road piece between two vertices, stored in
OSM way order (a → b). Two-way = 2 arcs, oneway = 1 arc. For `oneway=-1` the single arc runs b → a with
bit31 set.

Additions beyond the original spec:
- `BBOX` section in `graph.dat`: the extent of **all nodes in the source file**. The PBF header bbox is
  not used because Geofabrik extracts contain complete ways that reach outside it.
- `SegFlags::kSegOnewayReverse` (bit 7): set with `kSegOneway` when the allowed direction is b → a.

## Andorra (Geofabrik, 3.5 MB)

4,073 kept ways → V 5,019 · S 5,966 · A 9,657 · P 44,266 · 553.8 km of road. The conversion takes about
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
