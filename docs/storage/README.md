# verst: where the data lives (storage decision)

[verst](../../README.md) > [docs](../../README.md#documentation-map) > storage

> **Status: proposal, 2026-11-07.** The direction is agreed in discussion; the engine choice
> for each data type is to be confirmed by the benchmark at the end, on the real iPhone.
> Related: [challenges](../challenges/README.md), [tiling](../tiling/README.md).

## Summary

| Data | Storage | Why |
|---|---|---|
| Routing graph (`graph.dat`, `edges.dat`) | custom memory-mapped binary arrays | millions of tiny random reads per route; only array access is fast enough |
| Map data for drawing | pre-built tiles in one binary `tiles.dat` (index + blobs), memory-mapped | one lookup + one sequential read per tile; no geometry work on the phone |
| Search: streets, places, addresses, POIs | SQLite with FTS5 + trigram + R*Tree (`search.db`) | text and area queries, flexible |
| User data: favourites, history, settings, regions | SQLite in WAL mode (`user.db`) | written often, must survive map updates |

## Why routing and drawing data are not in SQLite
The core data is big arrays read in tight loops, and a flat memory-mapped file serves that
far better than a database:

| Need | Memory-mapped arrays | SQLite |
|---|---|---|
| Routing reads hundreds of thousands to millions of arcs per route | one array read, a few ns, neighbouring arcs side by side in memory | B-tree + SQL engine + page cache per lookup, about 1-5 us: 100-1000x slower |
| Start-up | nothing parsed or copied; `std::span` views point into the file | every query decodes rows |
| Drawing reads points, triangles, line pieces in bulk | whole ranges go straight to the GPU | row by row, or blobs we decode ourselves |
| Size | plain arrays, no row headers or index pages | typically 1.5-3x larger |
| Threads | read-only data, no locks | a connection per thread, every read through the engine |
| Memory on the phone | the OS pages the file in and out | SQLite keeps its own cache on top |

Production routers (OSRM, Valhalla, in-app navigation engines) keep the graph in flat binary
arrays for the same reasons.

## Routing data: how it should evolve
- **Turn restrictions -> edge-based graph:** a node is a road segment in one direction, an arc
  is an allowed turn; "no left turn" simply has no arc. About 2-3x the arcs, but the cleanest
  model and simple search code. Needs a new `kFormatVersion`; decide before routing code.
- **Locality:** number nodes along a space-filling curve (or BFS order) so a search stays in
  the same memory pages; on a phone this alone can give 2-5x.
- **Compact arcs:** 32-bit head, 32-bit time in deciseconds, flags packed in high bits.
- **Turn costs and speed profiles** computed in `osm2nav`, not at query time.
- **Contraction hierarchies / MLD** later, as extra arrays (node ranks, shortcut arcs, cell
  partitions) in the same style - only when measurements show bidirectional A* is too slow,
  likely only at country scale.

## Drawing data: pre-built tiles
Today the phone simplifies and triangulates every area for every tile at runtime. That is
where the zoom-10 hang came from (2026-11-07: workers stuck for minutes in
`mapbox::earcut` -> `cureLocalIntersections` on huge self-crossing forest outlines, leaving the
map blank) and it costs battery. The lasting design (step 8 of the tiling doc):
- `osm2nav` builds every tile of every level (z8-z14): simplified, clipped to the tile,
  triangulated, coordinates quantized to 16 bits inside the tile, bytes in the same layout as
  the GPU buffers.
- One `tiles.dat`: a sorted index (tile key -> offset, size) + the tile blobs, optionally
  compressed per tile (zstd or LZ4).
- On the phone a tile is: index lookup -> memory-mapped read (-> decompress) -> upload. No earcut,
  no simplification, no worker can hang on geometry; a tile costs well under 1 ms instead of
  10-100 ms (or forever).
- Hard cases (self-crossing outlines, huge forests, broken multipolygons) are cleaned once on
  the desktop and checked by `navcheck`.
- SQLite could hold tiles too (that is what `.mbtiles` is, and MapLibre uses it); for our own
  renderer a plain index + blobs file (like PMTiles) is simpler, smaller and memory-mappable.
  If verst ever switches to MapLibre, tiles become MBTiles.

## Search and user data: SQLite (FTS5 + trigram + R*Tree)
Covers about 80% of the need out of the box; the other 20% is ours whatever the storage.

| Part | Good at | Limits for verst |
|---|---|---|
| FTS5 | search as you type with prefix indexes (`prefix='2 3'`); compact, mature | `unicode61` folds é->e but not ß->ss, œ->oe, æ->ae, nor abbreviations (St/Saint, R./Rue): store our own `norm` column, normalized the same way at index and query time. bm25 ranking suits documents, not place names: rank ourselves |
| Trigram tokenizer | finds a name from any fragment; `LIKE`/`GLOB` use the index | not real typo tolerance (all trigrams must match); fuzzy search needs our own trigram-overlap table or `spellfix1`; index about 3-5x the text, so index unique names only |
| R*Tree | boxes in microseconds; near-me and area filters | boxes only: "nearest first" = box query + sort by distance in C++ |

- One table `item(id, kind, name, norm, lat, lon, importance, region, name_id)` with FTS5 on
  `norm`, a trigram index on unique names, an R*Tree on coordinates.
- Query: FTS5 prefix -> if fewer than N hits, fuzzy candidates from trigrams -> rank in C++
  (exact > word prefix > fuzzy, then importance, then distance to map center or GPS).
- `search.db` per region, read-only, opened with `immutable=1`; `user.db` in WAL mode,
  storing coordinates + names (never ids that change between map builds).
- Bundle the SQLite amalgamation (FetchContent) with `SQLITE_ENABLE_FTS5` and
  `SQLITE_ENABLE_RTREE`, not the system SQLite, so every platform has the same version.
- Deterministic output: sorted inserts, FTS5 `optimize`, `VACUUM`.
- Scale: Bourgogne (~60k street names) is tiny; France (~1M street names, tens of millions of
  addresses) still fine for FTS5, the trigram index and addresses need care (unique names only,
  number ranges per street). If measurements one day show it too big or slow, only search
  moves to an FST; the rest stays.

## Updates
- Split data by region (and tiles by area) so an update replaces only changed files.
- Every file carries the same `build_id` (exists today); a region's routing, tiles and
  search files must match or they are refused.
- Install side by side, then rename atomically; an interrupted update leaves the old region
  working.

## Choosing the storage engine for routing and drawing data

### Access patterns
- **Routing:** millions of tiny random reads per route (a node's arcs, their heads, weights,
  flags), tight loop, one thread, read-only.
- **Drawing:** per tile one lookup by `z/x/y` and one large sequential read (tens to hundreds
  of kB) straight to the GPU, on worker threads, read-only.
- **Updates:** rare, whole regions, never while the app runs.

### Must-haves (fail one = out)
1. Embedded and offline: a library in the app, no server.
2. Builds from source for iOS arm64 and desktop, C or C++ API, CMake.
3. Zero-copy reads: data used in place (memory-mapped or pointer into a page), no per-record
   decoding.
4. Safe to read from many threads, no locks on the hot path.
5. License fits App Store and closed or open distribution.
6. Read-only data never corrupted when the app is killed mid-read.

### Measured criteria (real iPhone, Bourgogne data)
| Criterion | Target |
|---|---|
| Random record read (one node's arcs) | <= 20 ns in memory; <= 100 ns per arc averaged over a route |
| Route-like workload: 1M random arc reads | < 50 ms on the iPhone SE |
| Tile lookup + read (100 kB) | < 1 ms cached, < 5 ms cold |
| Open time | < 20 ms per file |
| Size vs today's flat files (~80 MB routing, 342 MB map) | <= 1.1x for routing; tiles may shrink with compression |
| Own memory beyond data touched | < 10 MB |
| CPU per read | no per-record decoding for routing |
| Build time in `osm2nav` per region | minutes |

### Features (score 0-3)
Typed fixed-size records/arrays; blobs by key; ordered or spatial keys; per-blob compression;
format version + shared `build_id`; atomic region replacement; partial/delta updates;
validation and dump tools.

### Integration and risk (score 0-3)
Effort to adopt (wrappers, `osm2nav`, `nav_files`, tile loader); size added to the app;
maturity; control and debuggability (the earcut hang showed why); how much of today's
memory-mapped code stays.

### Candidates
| Candidate | Type |
|---|---|
| Custom memory-mapped files (today) + pre-built `tiles.dat` | baseline, flat arrays |
| LMDB | memory-mapped B+tree key-value, zero-copy |
| SQLite with blobs (MBTiles style) | relational, blobs by key |
| PMTiles | single-file tile archive (index + blobs), read-only |
| FlatBuffers / Cap'n Proto files, memory-mapped | zero-copy structured records |
| RocksDB / LevelDB | LSM key-value, compressed |

### Method
1. Drop candidates failing a must-have (RocksDB likely: it copies every value and its strength
   is heavy writing, which we do not do).
2. Weights: routing read speed 30%, tile read speed 20%, size and memory 15%, updates and
   consistency 15%, effort and control 20%.
3. Benchmark the top 2-3 on the iPhone with the same data: 1M random arc reads in a
   route-like pattern; 500 tile reads following the `cities` scenario; open time, size,
   memory peak. Record results in this file.
4. Decide per data type; routing and tiles may use different engines.

### Expected outcome (to be confirmed)
- Routing: custom memory-mapped arrays (LMDB if updates matter more). Nothing beats a flat
  array for this access pattern.
- Tiles: a PMTiles-style `tiles.dat` (index + compressed blobs); SQLite/MBTiles a reasonable
  second, and the natural one with MapLibre.
