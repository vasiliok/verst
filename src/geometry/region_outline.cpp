/// @file
/// parse_poly() and region_shape(). Declarations and documentation are in region_outline.h.
#include "geometry/region_outline.h"

#include <array>
#include <cstdlib>
#include <stdexcept>
#include <string>

#include <mapbox/earcut.hpp>

namespace nav {

namespace {

/// Removes spaces and tabs at both ends.
/// @param s Text.
/// @return Trimmed view.
std::string_view trim(std::string_view s) {
  while (!s.empty() && (s.front() == ' ' || s.front() == '\t' || s.front() == '\r')) s.remove_prefix(1);
  while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\r')) s.remove_suffix(1);
  return s;
}

/// Reads one number.
/// @param s Text; advanced past the number.
/// @param out Value.
/// @return false when there is no number.
bool read_number(std::string_view& s, Vrst_float64& out) {
  s = trim(s);
  // strtod needs a terminated string; the app never changes the "C" locale (decimal dot).
  const std::string word(s.substr(0, s.find_first_of(" \t")));
  if (word.empty()) return false;
  char* end = nullptr;
  out = std::strtod(word.c_str(), &end);
  if (end != word.c_str() + word.size()) return false;
  s.remove_prefix(word.size());
  return true;
}

}  // namespace

std::vector<Vrst_poly_ring> parse_poly(std::string_view text) {
  std::vector<Vrst_poly_ring> rings;
  Vrst_size line_no = 0;
  bool in_ring = false, named = false, ended = false;
  while (!text.empty() && !ended) {
    const Vrst_size nl = text.find('\n');
    const std::string_view line = trim(text.substr(0, nl));
    text = nl == std::string_view::npos ? std::string_view{} : text.substr(nl + 1);
    ++line_no;
    const auto bad = [&](const char* why) {
      return std::runtime_error(".poly line " + std::to_string(line_no) + ": " + why);
    };
    if (!named) {  // first line: the region name
      named = true;
      continue;
    }
    if (line.empty()) continue;
    if (!in_ring) {
      if (line == "END") {
        ended = true;
      } else {
        rings.push_back({{}, line.front() == '!'});
        in_ring = true;
      }
      continue;
    }
    if (line == "END") {
      if (rings.back().points.size() < 3) throw bad("ring with fewer than 3 points");
      in_ring = false;
      continue;
    }
    std::string_view rest = line;
    Vrst_float64 lon = 0, lat = 0;
    if (!read_number(rest, lon) || !read_number(rest, lat) || !trim(rest).empty()) throw bad("expected lon lat");
    if (lat < -90 || lat > 90 || lon < -180 || lon > 180) throw bad("lon lat out of range");
    rings.back().points.push_back({lat, lon});
  }
  if (in_ring) throw std::runtime_error(".poly: last ring has no END");
  if (rings.empty() || rings.front().hole) throw std::runtime_error(".poly: no outer ring");
  return rings;
}

Vrst_region_shape region_shape(const std::vector<Vrst_poly_ring>& rings) {
  Vrst_region_shape shape;
  // One polygon = one outer ring and the holes after it, triangulated together.
  std::vector<std::vector<std::array<Vrst_float64, 2>>> poly;
  std::vector<Vrst_vec2d> points;  // the polygon's points in earcut index order
  const auto flush = [&] {
    if (poly.empty()) return;
    for (const Vrst_uint32 i : mapbox::earcut<Vrst_uint32>(poly)) shape.triangles.push_back(points[i]);
    poly.clear();
    points.clear();
  };
  for (const Vrst_poly_ring& r : rings) {
    if (!r.hole) flush();
    if (poly.empty() && r.hole) continue;  // a hole without an outer ring: ignored
    std::vector<Vrst_vec2d> ring;
    for (const Vrst_latlon& p : r.points) ring.push_back(mercator(p));
    if (ring.size() > 1 && ring.front().x == ring.back().x && ring.front().y == ring.back().y) ring.pop_back();
    poly.emplace_back();
    for (const Vrst_vec2d& p : ring) {
      poly.back().push_back({p.x, p.y});
      points.push_back(p);
    }
    ring.push_back(ring.front());  // the border is a closed strip
    shape.outlines.push_back(std::move(ring));
  }
  flush();
  return shape;
}

}  // namespace nav
