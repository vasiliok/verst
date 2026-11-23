# verst: tiled map rendering (intention)

> **Status: intention, not started.** This is the plan to cut the map into square tiles and
> draw only the tiles on screen. It is for our own OpenGL / OpenGL ES renderer on `main`.
> Nothing below is implemented yet.
>
> Example pictures of the proposed look, drawn from the real Bourgogne data with a Python mock-up
> (not with verst): `~/src/tmp/verst/mockups/mock-{dijon-close,dijon,region}.png`.

## Context
Today `Vrst_map_mesh` (`src/provider/map_scene.h`) projects **every** shape point of the region into one
vertex buffer. `Vrst_map_renderer::draw()` (`src/render/map_renderer.h`) then draws all roads with
one `glDrawElements(GL_LINES)` call every frame, whatever the camera shows.
- Bourgogne (307k graph vertices) runs at about 23 fps on the Intel Mac, and about 15 fps with
  labels, even when the screen shows only one village.
- Memory and start time grow with the region. A whole country would not fit in a phone's GPU
  budget.
- The float vertex positions are relative to one map center. The precision is fine for a
  region but gets worse for a large country.

The goal is that frame cost depends on what is **on screen**, not on the size of the map.

## Tile model
- **Grid:** the standard Web Mercator quadtree, with tile `z/x/y` like OSM slippy tiles. A tile
  is a square in Mercator metres: side = 40075016.69 m / 2^z.
  - This is the same grid that MBTiles and MapLibre use, so the two approaches stay compatible.
- **Detail level (base):** z14. That is 2446 Mercator m per side, about 1.7 km of ground at 47 degN.
  The base tiles hold all roads at full detail.
- **Overview levels:** z12, z10 and z8, used when zoomed out.
  - Each overview level keeps only some road classes: z12 drops service and living_street, z10 keeps
    tertiary and up, z8 keeps primary and up.
  - Lines are simplified with Douglas-Peucker, with a tolerance of about 1/2 pixel at that level.
  - Without overviews, a zoomed-out view would still draw every road in many base tiles.
- **Level choice:** take the camera's metres per framebuffer pixel and pick the coarsest level
  whose detail still looks right. The table goes in `map_tiles.h` beside `kLabelStyles`.
- **Lines that cross tile borders:** add each line piece (point *i* -> *i+1*) to **every** tile its
  bounding box touches.
  - No clipping math, and no gaps at the borders.
  - Lines drawn twice at a border look the same, because they are opaque and the same color.

## Per-tile GPU data
- One VBO + IBO per tile, using the same `Vrst_gpu_point` (12 B) and the same shaders.
- Vertex positions are **relative to the tile's origin**, not the map center. This fixes float
  precision at any map size.
  - The CPU builds a view matrix per tile in double (`view_matrix` with `center - tile_origin`),
    so the shader does not change.
- Each tile's indices are sorted minor class -> major class, with an offset table per class.
  - Draw order: for each class band, draw that range in every visible tile.
  - Major roads therefore stay on top across tile borders, as today.
- Graph vertex dots (`m_point_verts`) and places for labels are put into tiles the same way.

## Which tiles to draw
- `visible_tiles(view, fb_w, fb_h, level)` maps the 4 screen corners with `screen_to_world`.
  This works when the map is rotated.
- It takes the Mercator bounding box of those 4 corners and lists the `x/y` range at that level.
  - The range is cut down to the data bbox.
  - Tiles with no data are skipped through a set of non-empty tile ids.
- **Cache:** an LRU list of built tiles, with a byte budget (e.g. 64 MB desktop, 24 MB iOS).
  - A tile that is visible but not built yet uses a coarser cached parent tile until it is
    ready, so the map never shows holes.

## More map layers: forest, water, railways, bridges
Today the nav files hold only the car road network. A map also needs the land around the roads.
The new layers are **for drawing only**: routing never reads them, so they go in their own file
and the graph files stay unchanged.

| Layer | OSM tags | Geometry | Drawn as |
|---|---|---|---|
| Water areas | `natural=water`, `water=*`, `waterway=riverbank`, `landuse=reservoir/basin` | polygon | blue fill |
| Rivers, canals, streams | `waterway=river/canal/stream/ditch` | line | blue line, wider for river/canal |
| Forest | `landuse=forest`, `natural=wood` | polygon | dark green fill |
| Other land | `landuse=residential/industrial/farmland/meadow/grass`, `leisure=park` | polygon | faint fill, different per kind |
| Railways | `railway=rail/light_rail/narrow_gauge/tram/subway` | line | grey line with a dashed pattern |
| Bridges | `kSegBridge` (already set on road segments), plus `bridge=*` on railways | line | wider dark casing under the road, drawn above everything below it |
| Tunnels | `kSegTunnel` (already set) | line | road drawn faded or dashed |
| Buildings | `building=*` | polygon -> rectangle | light grey squares, only when zoomed in |

- **Import (osm2nav):**
  - Areas are closed ways plus `type=multipolygon` relations. They are assembled with
    libosmium's area assembler, which osm2nav already links.
  - Polygons are triangulated once at import with **earcut** (mapbox/earcut.hpp, a header-only
    file, fetched with FetchContent). The app then only draws triangles and never triangulates.
  - Lines (waterways, railways) are stored like the road shapes: points plus first-point indices.
  - Areas below about one pixel at the z14 level are dropped, and outlines are simplified with
    Douglas-Peucker, so a forest of 10 000 points does not slow the map.
- **New file `map.dat`** (`FileType::Map = 5`, format version 3), with these sections:
  - `AreaKind[A]`, `AreaFirstTri[A+1]`, `AreaPoints` (Vrst_coord), `AreaIndices` (uint32 triangles);
  - `LineKind[L]`, `LineFlags[L]` (bridge, tunnel), `LineFirstPoint[L+1]`, `LinePoints`.
  - navcheck checks the sections the same way as the others: index ranges, kinds, points
    inside the bbox.
- **Buildings as squares:**
  - Each `building=*` outline is replaced at import by its **smallest rotated rectangle**: 4
    corners and 2 triangles. It is turned to match the building, so the rectangles line up with
    the street.
  - Big or unusual buildings (L-shapes, courtyards, churches) keep their real outline. They are
    still simplified and cut into triangles with earcut. The rule: keep the real outline when
    the rectangle is more than ~25% bigger than the building.
  - There are a lot of buildings: Bourgogne has about a million. A rectangle costs 4 points + 2
    triangles, against ~7 points + ~5 triangles for a raw outline, so the file and the GPU memory
    stay about half the size.
  - They are drawn only from about **3 m per point** and closer, from the z14 tiles only.
    Overview levels never contain buildings, and a zoomed-out city does not draw a million
    squares.
  - Fill slightly lighter than the background, with a 1-px darker outline, so neighbouring
    houses stay apart.
  - Stored in `map.dat` in their own sections (`BuildingPoints`, `BuildingIndices`,
    `BuildingFirstPoint`), separate from the other areas. The renderer can then skip them all
    at once.
- **Street names and road numbers:**
  - **No new data.** Every road segment already has `name_id` and `ref_id` in `Vrst_seg_attr`
    (`src/storage/format.h`), and the strings are in names.dat.
  - **Chains:** a long street is many segments in the graph. When a tile is built, connected
    segments with the same `name_id` are joined into one line, so the label can run along the
    whole street.
  - **Names along the line:** in `Vrst_map_labels::layout()`, every letter is placed along the
    line at the next pen position, turned to the line's direction.
    - Each glyph is already its own quad in `Vrst_text_vertex`, so the text shader does not
      change; the quad corners are only rotated.
    - Text always reads left to right: if the line runs right to left on screen, it is walked
      backwards.
    - A name is skipped where the line bends more than 30 deg between two letters, or where the
      line is shorter than the text.
    - Long streets repeat the name about every 600 px. The same name never comes closer than
      400 px to itself.
    - Shown from about **3 m per point** and closer. Text height is 11 pt, or 13-14 pt when
      very close.
  - **Road numbers** (`ref`, e.g. D 965, M 905, A 31): a small box in the road's class colour
    with dark bold text, put at the middle of the road.
    - Shown at every zoom for primary roads and up. Secondary roads are added when zoomed in.
    - The box is a fill quad drawn under the glyphs: one more shader uniform, or a quad in the
      fill shader.
  - **Priority** in the shared collision test: place names, then road numbers, then street
    names, major roads first. Important labels win, as today.
  - **No jumping while panning:** label positions along a chain are fixed in world metres, not
    screen pixels.
- **Text and fonts:** names are **not painted into the tiles**.
  - A tile holds only where a name can go: the joined street lines (tile-relative), each
    with its `name_id`, `ref_id` and road class.
  - The letters come from the glyph atlas we already have (`src/render/glyph_atlas.h`). It is built
    once at start: FreeType renders DejaVu Sans and Sans Bold into one distance-field texture,
    with only the letters the names use.
    - French street names add no new letters beyond the place names, so the texture stays
      about 1024x256.
  - Every name's width is measured once at load (`Vrst_map_labels::measure()`).
  - Every frame:
    ```
    visible tiles --> street lines + name_id --+
                                               +-> layout(): letters along each line,
    font atlas (built once) -- letter shapes --+    collision test across all tiles
                                                          |
                                             one vertex list of letter squares
                                                          |
                                   one draw call: text shader + atlas texture
    ```
  - Why not painted into the tiles:
    - **Rotation:** painted text turns with the map and ends up upside down.
    - **Tile borders:** a name crossing a border would be cut in half or doubled.
    - **Zoom:** painted text grows and shrinks while zooming, but the distance-field font stays
      sharp and the same size on screen.
    - **Collisions:** the overlap test has to see all tiles at once.
    - **Halo:** the dark outline comes from the distance field at no extra cost.
  - Fonts per label:
    - places: bold for cities and towns, regular for villages;
    - streets: regular;
    - road numbers: bold dark text in a box;
    - later, rivers in DejaVu Sans Oblique (a third atlas font) for blue italic water names.
- **Draw order inside a tile** (bottom to top):
  1. background;
  2. land fills (farmland, residential, park);
  3. forest;
  4. water areas;
  5. buildings (below rivers and roads, so streets stay readable);
  6. waterways;
  7. tunnels (faded);
  8. railways;
  9. roads, minor to major;
  10. bridge casings + bridge roads;
  11. GPS track;
  12. labels.
  - Bridges must be drawn after the roads and rivers they cross, otherwise a bridge looks like a
    junction.
- **Shaders:**
  - A new fill shader for triangles. It takes one colour per area kind from a uniform table,
    like `kClassColors` today.
  - The line shader gets a dash pattern (a distance along the line, passed as a vertex
    attribute) for railways and tunnels.
  - Wider lines (rivers, bridge casings) are drawn as quads built at import, not with
    `glLineWidth`. Core-profile GL and GLES only guarantee 1-pixel lines.
- **Tiling:** areas and lines are added to tiles the same way as roads, by bounding box.
  - Overview levels keep only big forests and lakes plus the rivers and main rail lines.
- **Toggles:** each layer group gets a key in navview (like 1..9 for road classes) and a toolbar
  switch on iOS.

## Steps
1. **Tile math:** new `src/storage/map_tiles.h`.
   - `Vrst_tile_id {z, x, y}`, tile bounds, the level table, `visible_tiles()`.
   - Pure C++, tested by a small ctest: known lat/lon -> known `z/x/y`, and a rotated view lists
     the expected tiles.
2. **Runtime tiler, base level only:** `Vrst_tile_mesh` is built from the nav files for one tile.
   - At load time, a spatial index (segment -> tiles) is filled once.
   - Building one tile only reads the segments it lists.
   - `Vrst_map_renderer` draws the visible tiles instead of the one big buffer.
   - Debug key `T` in navview draws the tile borders and their `z/x/y`.
   - The ImGui panel shows tiles visible / cached / built per second and the number of draw calls.
3. **Overview levels:** class filters + Douglas-Peucker per level. Check that zoomed-out fps no
   longer depends on how many minor roads the region has.
4. **Lazy building on a worker thread:**
   - Tiles are built on a background thread. The GL thread only uploads the finished tiles,
     with a limit per frame so panning stays smooth.
   - The parent-tile fallback covers tiles that are not built yet.
5. **More layers:** osm2nav writes `map.dat` (areas with earcut, waterways, railways).
   - The renderer adds the fill shader, the dashed lines and the bridge casings, and draws in
     the layer order above.
   - First water + forest, then railways, then bridges and tunnels, then buildings, with a screenshot check after each.
6. **Street names and road numbers:**
   - Join segments with the same `name_id` into chains when a tile is built.
   - `Vrst_map_labels::layout()` places letters along the chains and the road-number boxes.
   - The atlas, text shader and `draw_labels()` stay as they are.
7. **iOS:** the same code, because `map_tiles.h` and the renderer are shared. Only the cache
   budget changes.
8. **Optional - tiles stored in the nav files:**
   - osm2nav writes the tiles ready to use: a new `tiles.dat`, in the next format version, that holds
     each tile's vertex and index arrays.
   - The app then memory-maps a tile and uploads it with no CPU work.
   - Decide after measuring step 4: do it only if building tiles at runtime is too slow on the
     phone.

## Critical files
- New: `src/storage/map_tiles.h` (grid math, level table, visible tiles, tile mesh, cache), plus a test.
- Changed:
  - `src/provider/map_scene.h`: `Vrst_map_mesh` becomes the source for the tiles.
  - `src/render/map_renderer.h`: one VAO per tile, a draw loop per class band.
  - `src/render/map_labels.h`: places and street chains come from the visible tiles; letters along
    lines; road-number boxes.
  - `src/tools/navview.cpp`: key `T`, tile statistics.
  - `ios/main.mm`: cache budget.
  - `README.md`.
- More layers: `src/storage/format.h` (`FileType::Map`, new sections), `src/tools/osm2nav.cpp`,
  `src/storage/nav_files.h`, `src/tools/navcheck.cpp`, `src/render/shaders.h` (fill shader, dashes), CMake (earcut).
- Optional step 8: `src/storage/format.h`, `src/tools/osm2nav.cpp`, `src/storage/nav_files.h`, `src/tools/navcheck.cpp`.

## Verification
1. **Same picture:**
   - Take a navview `--screenshot` at the same `--center/--mpp` before and after tiling, at
     Dijon mpp 5, at mpp 30, and fitted to the whole map.
   - Diff the images. Only anti-aliasing at tile borders may change.
2. **Speed:**
   - Bourgogne fps at Dijon mpp 5 must be well above 23.
   - Fitted to the whole map, fps must not be lower than today.
3. Layers: in Dijon and the villages around it, forests and water sit under the roads, buildings show as
   rectangles turned along the streets from 3 m/pt, railways are dashed, and
   bridges show casings over rivers and rails.
4. Names: at Dijon 0.8 and 2.5 m/pt, street names follow the streets and read left to right,
   road numbers sit in boxes, nothing overlaps, and labels do not jump while panning.
5. Tile border overlay (`T`) at several zoom levels and while rotated: no holes, no missing roads
   at borders.
6. The cache stays within its budget while panning across the whole region.
7. iOS Simulator screenshot is the same as before.
8. 0 build warnings and 0 Doxygen warnings; ctest passes, including the new tile test.

## Risks
- **Many draw calls:** about 30 visible tiles x class bands. That is fine for GL, but stays
  under watch on iOS.
  - If needed, merge the class bands with `glMultiDrawElements` on desktop, or with one index
    range per tile on iOS.
- **Thin lines at overview levels:** simplification can change their look. Tune the
  Douglas-Peucker tolerance per level against screenshots.
- **Label cost per frame:** walking letters along thousands of street chains every frame can
  be slow. Cache each chain's screen-free layout (letter positions in world metres) per tile,
  and only redo the collision test per frame.
- **The worker thread:** it must not touch GL. Only finished CPU arrays cross over to the GL
  thread.
