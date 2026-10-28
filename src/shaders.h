/// @file
/// Map shaders shared by desktop OpenGL 3.3 core and OpenGL ES 3.0 (iOS).
///
/// The sources have no `#version` line: Vrst_map_renderer prepends kGlslHeader for the
/// platform, `#version 330 core` or `#version 300 es` with precision qualifiers. The bodies
/// use only features both languages share (flat uint varyings, gl_PointCoord).
#pragma once

namespace nav::shaders {

/// Road lines, vertex shader. `a_attr = road_class | flags << 16`.
inline constexpr const char* kLineVs = R"(
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
inline constexpr const char* kLineFs = R"(
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
inline constexpr const char* kPointVs = R"(
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
inline constexpr const char* kPointFs = R"(
flat in uint v_dead;
out vec4 o_color;
void main() {
  vec2 d = gl_PointCoord - vec2(0.5);
  if (dot(d, d) > 0.25) discard;
  o_color = v_dead != 0u ? vec4(1.0, 0.15, 0.15, 1.0) : vec4(0.85, 0.85, 0.9, 1.0);
}
)";

}  // namespace nav::shaders
