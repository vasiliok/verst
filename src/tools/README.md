# tools

[verst](../../README.md) > [src](../README.md) > tools

Command-line programs, one `main()` per file. Built into the build directory by
CMakeLists.txt.

| Tool | Usage | What |
|---|---|---|
| `pbf_merge` | `pbf_merge <out.osm.pbf> <in1.osm.pbf> <in2.osm.pbf> ...` | merges neighbouring extracts into one |
| `osm2nav` | `osm2nav <input.osm.pbf> <out_dir>` | converts an extract into nav files, map.dat and search.db |
| `navcheck` | `navcheck <out_dir> [--dump-way <osm_way_id>]` | validates the nav files; exit 0 when all checks pass |
| `navroute` | `navroute <dir> <lat> <lon> <lat> <lon> [car\|bike\|foot]` | route and way back through the provider, as the app does |
| `navsearch` | `navsearch <dir> <text> [<lat> <lon>]` | address search in search.db, as the app does |
| `navview` | `navview <out_dir> [--center <lat> <lon>] [--mpp <m>] [--gpx <file>] ...` | desktop map viewer (GLFW + OpenGL 3.3); needs GLFW |

Source files: `pbf_merge_main.cpp`, `osm2nav.cpp`, `navcheck.cpp`, `navroute.cpp`,
`navsearch.cpp`, `navview.cpp`.

Uses: convert (pbf_merge, osm2nav), storage (navcheck), provider (navroute, navview),
search (navsearch), render and session (navview).
