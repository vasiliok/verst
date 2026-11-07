/// @file
/// Map shaders shared by desktop OpenGL 3.3 core and OpenGL ES 3.0 (iOS).
///
/// The sources have no `#version` line: Vrst_map_renderer prepends kGlslHeader for the
/// platform, `#version 330 core` or `#version 300 es` with precision qualifiers. The bodies
/// use only features both languages share (flat uint varyings, gl_PointCoord).
#pragma once

#include <string_view>

namespace nav::shaders {

/// Road lines, vertex shader. `a_attr = road_class | flags << 16`.
inline constexpr std::string_view kLineVs = R"(
layout(location = 0) in vec2 a_pos;
layout(location = 1) in uint a_attr;
uniform mat3 u_view;
flat out uint v_attr;
void main() {
  vec3 p = u_view * vec3(a_pos, 1.0);
  gl_Position = vec4(p.xy, 0.0, 1.0);
  v_attr = a_attr;
}
)";

/// Road lines, fragment shader: class color, class mask and oneway highlight.
inline constexpr std::string_view kLineFs = R"(
flat in uint v_attr;
uniform vec3 u_colors[16];
uniform uint u_class_mask;
uniform bool u_oneway;
out vec4 o_color;
void main() {
  uint cls = v_attr & 0xFFu;
  uint flags = v_attr >> 16;
  if (((u_class_mask >> cls) & 1u) == 0u) discard;
  vec3 c = u_colors[cls];
  if (u_oneway) c = (flags & 1u) != 0u ? vec3(1.0, 0.25, 0.9) : c * 0.3;
  o_color = vec4(c, 1.0);
}
)";

/// Graph vertices, vertex shader. `a_attr` = 1 for dead ends (undirected degree 1).
inline constexpr std::string_view kPointVs = R"(
layout(location = 0) in vec2 a_pos;
layout(location = 1) in uint a_attr;
uniform mat3 u_view;
uniform float u_point_px;
flat out uint v_dead;
void main() {
  vec3 p = u_view * vec3(a_pos, 1.0);
  gl_Position = vec4(p.xy, 0.0, 1.0);
  v_dead = a_attr;
  gl_PointSize = a_attr != 0u ? 2.0 * u_point_px : u_point_px;
}
)";

/// Graph vertices, fragment shader: round dots, dead ends highlighted.
inline constexpr std::string_view kPointFs = R"(
flat in uint v_dead;
out vec4 o_color;
void main() {
  vec2 d = gl_PointCoord - vec2(0.5);
  if (dot(d, d) > 0.25) discard;
  o_color = v_dead != 0u ? vec4(1.0, 0.15, 0.15, 1.0) : vec4(0.85, 0.85, 0.9, 1.0);
}
)";

/// Overlays (GPS track line, position marker), vertex shader. One color per draw call.
inline constexpr std::string_view kOverlayVs = R"(
layout(location = 0) in vec2 a_pos;
uniform mat3 u_view;
uniform float u_point_px;
void main() {
  vec3 p = u_view * vec3(a_pos, 1.0);
  gl_Position = vec4(p.xy, 0.0, 1.0);
  gl_PointSize = u_point_px;
}
)";

/// Overlays, fragment shader. Lines use u_color; with u_round points become a filled
/// circle in u_color with a white ring, so the marker stays visible on any road color.
inline constexpr std::string_view kOverlayFs = R"(
uniform vec4 u_color;
uniform bool u_round;
out vec4 o_color;
void main() {
  if (u_round) {
    vec2 d = gl_PointCoord - vec2(0.5);
    float r2 = dot(d, d);
    if (r2 > 0.25) discard;
    o_color = r2 > 0.12 ? vec4(1.0) : u_color;
  } else {
    o_color = u_color;
  }
}
)";

/// Label glyphs, vertex shader: framebuffer px (y down) -> NDC.
inline constexpr std::string_view kTextVs = R"(
layout(location = 0) in vec2 a_pos;
layout(location = 1) in vec2 a_uv;
uniform vec2 u_fb_size;
out vec2 v_uv;
void main() {
  gl_Position = vec4(a_pos.x / u_fb_size.x * 2.0 - 1.0, 1.0 - a_pos.y / u_fb_size.y * 2.0, 0.0, 1.0);
  v_uv = a_uv;
}
)";

/// Label glyphs, fragment shader: signed distance field (0.5 = outline) drawn as
/// light text with a dark halo of u_halo_px screen pixels. fwidth() gives the change
/// of the distance per screen pixel, so edges stay sharp and the halo keeps its width
/// at every label size.
inline constexpr std::string_view kTextFs = R"(
in vec2 v_uv;
uniform sampler2D u_atlas;
uniform float u_halo_px;
uniform vec3 u_text_color;
uniform vec3 u_halo_color;
out vec4 o_color;
void main() {
  float d = texture(u_atlas, v_uv).r;
  float w = max(fwidth(d), 1e-4);
  float fill = smoothstep(0.5 - w, 0.5 + w, d);
  float edge = max(0.5 - u_halo_px * w, 0.02);
  float alpha = smoothstep(edge - w, edge + w, d);
  if (alpha <= 0.0) discard;
  o_color = vec4(mix(u_halo_color, u_text_color, fill), alpha);
}
)";

}  // namespace nav::shaders
