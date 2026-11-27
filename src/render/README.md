# render

[verst](../../README.md) > [src](../README.md) > render

The OpenGL map renderer shared by the desktop viewer (OpenGL 3.3 core) and the iOS app
(OpenGL ES 3.0). Built as `vrst_render`.

| File | What |
|---|---|
| `gl_api.h` | picks the API: OpenGL ES 3.0 on iOS, desktop OpenGL 3.3 core through glad elsewhere |
| `shaders.h` | map shaders for both APIs (no `#version` line; the renderer prepends it) |
| `tile_renderer.{h,cpp}` | `Vrst_tile_renderer`: uploads the map pieces the provider delivers, keeps a byte-limited cache, draws visible tiles by layer |
| `map_renderer.{h,cpp}` | `Vrst_map_renderer`: GL setup, overlays (GPS track, position dot or direction arrow, pins, route line, region border) and labels |
| `map_labels.{h,cpp}` | `Vrst_map_labels`: which place names to show at the current zoom and their glyph quads |
| `glyph_atlas.{h,cpp}` | `Vrst_glyph_atlas`: signed-distance-field glyph atlas rendered with FreeType |

Uses: provider (`map_provider.h`, `map_scene.h`), storage (`map_piece.h`), geometry
(`region_outline.h`). Never reads nav files.
Used by: navview, the iOS app.
