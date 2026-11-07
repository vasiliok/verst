# verst: what a shader is, explained with the project's own shaders

> Reference for explaining shaders in a new session. The examples are the real shaders in
> `src/shaders.h` and `src/tile_renderer.h`. They run on desktop OpenGL 3.3 (navview) and on
> OpenGL ES 3.0 (iPhone app).

## 1. What a shader is
A **shader** is a small program that runs **on the GPU**, not on the CPU. It is written in
**GLSL** (a C-like language), stored in the C++ code as a text string, and compiled by the
graphics driver while the app runs.

The GPU runs the same shader many times **in parallel**: once per vertex, or once per pixel.
That is why a map with a million line pieces can still draw in a few milliseconds. A CPU loop
over a million pieces every frame would be far too slow.

Every draw call uses one **program**, which is two shaders linked together:

| Shader | Runs once per | Its job | Main output |
|---|---|---|---|
| **Vertex shader** | vertex (corner of a triangle) | Where is this point on the screen? | `gl_Position` |
| **Fragment shader** | pixel the triangle covers | What colour is this pixel? | `o_color` (or `discard`) |

Between the two, the GPU itself does **rasterisation**: it takes the triangles that the vertex
shader placed on the screen and finds every pixel they cover. The fragment shader then runs once
for each of those pixels.

```
C++ (CPU)                      GPU
vertex buffer ──► vertex shader ──► triangles ──► rasteriser ──► fragment shader ──► screen
uniforms ────────────┘ (per vertex)                 (pixels)       (per pixel)
```

## 2. The three kinds of shader inputs
| GLSL keyword | Meaning | Changes | verst example |
|---|---|---|---|
| `in` in the vertex shader (**attribute**) | data of one vertex, read from a GPU buffer | per vertex | `a_pos`: a point in metres; `a_attr`: the road class |
| `uniform` | a value set by C++ before the draw call, the same for every vertex and pixel | per draw call | `u_view`: the camera matrix; `u_colors[]`: the colour table |
| `out` in the vertex shader → `in` in the fragment shader (**varying**) | a value handed from a vertex to the pixels between vertices | per pixel | `v_kind`, `v_attr`, `v_along` |

A varying is normally **interpolated**: a pixel halfway between two vertices gets the average of
their values. With **`flat`** it is not interpolated: the pixels take the value of one vertex. verst
uses `flat` for integer ids such as the road class, because averaging "primary" and "service"
means nothing.

Naming used in verst: `a_` attribute, `u_` uniform, `v_` varying, `o_` output.

## 3. The simplest example: area fills (forest, water, buildings)
From `src/tile_renderer.h`. Every area (a lake, a forest, a house) was cut into triangles at import
time. Each vertex is a `Vrst_gpu_point {x, y, attr}`: a position in metres from the tile corner,
plus the `AreaKind` as `attr`.

```glsl
// Vertex shader: runs for each triangle corner.
layout(location = 0) in vec2 a_pos;   // metres from the tile origin (from the buffer)
layout(location = 1) in uint a_attr;  // AreaKind: 0 farmland ... 5 forest, 6 water, 7 building
uniform mat3 u_view;                  // camera: metres -> screen, set by C++
flat out uint v_kind;                 // passed on to the pixels, not interpolated
void main() {
  vec3 p = u_view * vec3(a_pos, 1.0); // move, rotate and zoom with one matrix multiply
  gl_Position = vec4(p.xy, 0.0, 1.0); // screen position in -1..1 ("NDC")
  v_kind = a_attr;
}
```

```glsl
// Fragment shader: runs for each pixel inside the triangle.
flat in uint v_kind;
uniform vec3 u_colors[8];             // one RGB colour per AreaKind (kAreaColors in C++)
out vec4 o_color;
void main() { o_color = vec4(u_colors[v_kind], 1.0); }
```

Points to notice:
- **The camera is a uniform.** Panning or zooming changes only `u_view`, nine floats. The
  vertex buffers on the GPU are never touched again, which is what makes moving the map cheap.
- **The colours are a uniform table.** The buffer stores only the *kind*, and the shader looks up
  the colour, so changing the style needs no new upload.
- **Screen coordinates.** `gl_Position` uses "normalised device coordinates": x and y run from
  −1 to +1 across the window, whatever its size in pixels.

The camera matrix comes from `view_matrix()` in `src/map_scene.h`. Each tile has its own origin,
so the vertices hold small numbers that stay precise in 32-bit floats. For each tile, the C++ code
subtracts the tile origin from the camera centre before computing that tile's matrix.

## 4. A vertex shader that builds geometry: wide map lines
Core OpenGL and OpenGL ES only promise lines **1 pixel wide**, but a road must be several pixels
wide and grow when zooming in. So verst draws each line piece as a **rectangle (quad)**, and the
vertex shader builds that rectangle.

- The buffer holds one `Vrst_line_instance {x0, y0, x1, y1, d0, attr}` per piece: start, end,
  distance along the line, and style.
- C++ calls `glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, count)`: 4 vertices per piece. Each
  instance reads the next piece from the buffer.
- **The 4 vertices have no data of their own.** The built-in `gl_VertexID` (0, 1, 2, 3) says which
  corner of the rectangle this vertex is.

```glsl
uint style = a_attr & 0xFFu;
vec2 s0 = (u_view * vec3(a_p0, 1.0)).xy * u_half_fb;  // both ends, in framebuffer pixels
vec2 s1 = (u_view * vec3(a_p1, 1.0)).xy * u_half_fb;
vec2 dir = normalize(s1 - s0);                         // along the line (zero-length case handled in the source)
vec2 n = vec2(-dir.y, dir.x);                          // sideways
float hw = 0.5 * max(u_width[style] + u_extra_px, 1.0);  // half width in pixels
float end  = float(gl_VertexID & 1);                   // 0 = start corner, 1 = end corner
float side = (gl_VertexID & 2) != 0 ? 1.0 : -1.0;      // left or right edge
vec2 p = mix(s0 - dir * hw, s1 + dir * hw, end) + n * side * hw;
gl_Position = vec4(p / u_half_fb, 0.0, 1.0);
```

Points to notice:
- **Widths are in pixels**, from the uniform array `u_width[style]`. C++ fills it each frame from
  the zoom (`line_widths()`), so roads keep a readable width at every zoom without rebuilding
  any buffer.
- **Hiding is done in the shader.** When a road class is switched off in `u_class_mask`, the
  vertex shader moves the quad off screen (`gl_Position = vec4(2, 2, 2, 1)`), so the GPU skips it.
- Each end is extended by half the width (square caps), so consecutive pieces overlap and the
  joints have no gaps.

## 5. A fragment shader with several passes: casings, dashes, tunnels
The same line program draws several looks, chosen by the uniform `u_pass`:

```glsl
vec3 c = u_colors[style];
if (u_pass == 1) c = u_pass_color;                       // bridge casing: draw wider, in a fixed colour
if (u_pass == 2) {                                       // railway dashes
  if (mod(v_along, 2.0 * u_dash_px) > u_dash_px) discard;
  c = u_pass_color;
}
if (u_pass == 3) {                                       // tunnel: dashed and faded into the background
  if (mod(v_along, 2.0 * u_dash_px) > u_dash_px) discard;
  c = mix(u_background, c, 0.45);
}
o_color = vec4(c, 1.0);
```

- **`discard`** throws the pixel away, so nothing is drawn there. Dashes are gaps made this way.
- **`v_along`** is a varying: the distance along the line in pixels, interpolated across the
  quad. `mod(v_along, period)` makes a repeating pattern, here dashes.
- **Draw order makes the layers.** C++ draws:
  1. areas;
  2. tunnels (pass 3);
  3. ground lines (pass 0, then railway dashes with pass 2);
  4. bridges: casing (pass 1, wider), then the road on top (pass 0).

  The GPU paints later draws over earlier ones, like layers of paint.

## 6. Round points without triangles: vertex dots and the GPS marker
With `GL_POINTS`, each vertex becomes a square of `gl_PointSize` pixels. Inside that square, the
fragment shader gets `gl_PointCoord` (0..1) and cuts out a circle:

```glsl
vec2 d = gl_PointCoord - vec2(0.5);
if (dot(d, d) > 0.25) discard;               // outside the circle of radius 0.5
o_color = r2 > 0.12 ? vec4(1.0) : u_color;   // white ring around a coloured centre (GPS dot)
```

## 7. Sharp text at any size: signed distance field labels
Place names (`kTextVs`/`kTextFs` in `src/shaders.h`) use a texture, the **glyph atlas**, built with
FreeType. Each texel stores a **distance to the letter's edge** instead of a plain on/off pixel:
0.5 is exactly on the outline, larger is inside, smaller is outside.

```glsl
float d = texture(u_atlas, v_uv).r;               // sample the distance (a texture read)
float w = max(fwidth(d), 1e-4);                   // how much d changes per screen pixel
float fill  = smoothstep(0.5 - w, 0.5 + w, d);    // inside the letter: soft 1-pixel edge
float edge  = max(0.5 - u_halo_px * w, 0.02);     // halo boundary, u_halo_px pixels outside
float alpha = smoothstep(edge - w, edge + w, d);
if (alpha <= 0.0) discard;
o_color = vec4(mix(u_halo_color, u_text_color, fill), alpha);
```

- A **texture** is an image on the GPU, and `texture()` reads it at a coordinate. The `v_uv`
  varying is interpolated, so each pixel reads its own spot in the letter.
- `fwidth()` gives the screen-space rate of change, so edges are exactly one pixel soft at any
  label size. One small atlas serves every size.
- `alpha` below 1 needs **blending**, which C++ turns on for text: the pixel is mixed with what
  is already on the screen.

## 8. How C++ gets a shader onto the GPU
In `Vrst_map_renderer::compile()` / `link()` (`src/map_renderer.h`):
1. **Source:** the GLSL text lives in `std::string_view` constants.
   - It has no `#version` line. C++ prepends `kGlslHeader`: `#version 330 core` on desktop, or
     `#version 300 es` plus `precision highp float;` on iOS.
   - Only features both versions share are used, so the same source works on both.
2. **Compile:** `glCreateShader` → `glShaderSource` → `glCompileShader`, once for the vertex shader
   and once for the fragment shader.
3. **Link:** `glCreateProgram` → `glAttachShader` ×2 → `glLinkProgram`. A compile or link error
   throws with the driver's log.
4. **Look up uniforms:** `glGetUniformLocation(prog, "u_view")` gives a number used to set the
   value later, for example `glUniformMatrix3fv(loc, 1, GL_FALSE, m.data())`.
5. **Describe the buffer layout:** a **VAO** (vertex array object) records how bytes in a buffer map
   to attributes. For example, `glVertexAttribPointer(0, 2, GL_FLOAT, …)` means location 0 = two
   floats. `glVertexAttribIPointer` is used for integer attributes such as `a_attr`, and
   `glVertexAttribDivisor(a, 1)` makes an attribute advance per instance rather than per vertex.
6. **Draw:** `glUseProgram` → set uniforms → `glBindVertexArray` → `glDrawArrays` /
   `glDrawArraysInstanced`.

## 9. All shaders in the project
| Program | File | Draws | Primitive |
|---|---|---|---|
| `kFillVs/Fs` | `src/tile_renderer.h` | areas: farmland … water, buildings | triangles |
| `kTileLineVs/Fs` | `src/tile_renderer.h` | roads, rivers, railways, building outlines, tile borders | instanced quads |
| `kPointVs/Fs` | `src/shaders.h` | graph vertex dots, dead ends in red | points |
| `kOverlayVs/Fs` | `src/shaders.h` | GPS track line, track marker, GPS position dot | lines / points |
| `kTextVs/Fs` | `src/shaders.h` | place names (SDF) | triangles, textured |
| `kLineVs/Fs` | `src/shaders.h` | old single-mesh road view (1 px lines) | `GL_LINES` |

## 10. Rules of thumb this project follows
- **Upload once, change uniforms per frame.** Geometry goes to the GPU when a tile loads, and a
  camera move only changes `u_view`.
- **Store ids in buffers, not colours or widths.** The shader looks them up in uniform tables, so
  the style can change without uploading again.
- **Few draw calls.** The GPU is fast per vertex but each draw call has a CPU cost, so all tiles
  of one style are drawn in one loop with only the matrix changing.
- **Precision:** the GPU works in 32-bit floats, so positions are stored relative to a nearby
  origin (the tile corner), never as raw world coordinates.
- **GLSL shared by GL 3.3 and GLES 3.0:** no `#version` in the source, explicit
  `layout(location = …)`, `flat` for integer varyings, and no desktop-only built-ins.
