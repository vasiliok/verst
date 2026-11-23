/// @file
/// search_index(): ranking and result text of the address search. Documentation is in
/// search_rank.h.
#include "search/search_rank.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <tuple>

#include "search/search_text.h"

namespace nav {

namespace {

/// Candidates asked from the index; ranking picks the best of them.
constexpr Vrst_size kCandidates = 200;

/// Position of a stored coordinate.
/// @param c Coordinate in 1e-7 degrees.
/// @return Degrees.
Vrst_latlon to_latlon(Vrst_coord c) { return {c.lat_e7 * 1e-7, c.lon_e7 * 1e-7}; }

/// Distance for ranking and display (equirectangular, fine within a region).
/// @param a First point.
/// @param b Second point.
/// @return Metres.
Vrst_float64 distance_m(Vrst_latlon a, Vrst_latlon b) {
  const Vrst_float64 rad = kPi / 180.0;
  const Vrst_float64 x = (b.lon - a.lon) * rad * std::cos((a.lat + b.lat) / 2 * rad);
  const Vrst_float64 y = (b.lat - a.lat) * rad;
  return kEarthRadius * std::sqrt(x * x + y * y);
}

/// Distance as text: "350 m", "1.2 km", "12 km".
/// @param m Metres.
/// @return Text.
std::string distance_text(Vrst_float64 m) {
  char buf[32];
  if (m < 1000) std::snprintf(buf, sizeof buf, "%.0f m", m);
  else if (m < 10000) std::snprintf(buf, sizeof buf, "%.1f km", m / 1000);
  else std::snprintf(buf, sizeof buf, "%.0f km", m / 1000);
  return buf;
}

/// Name of a place kind.
/// @param k Kind.
/// @return "city", "town", "village" or "hamlet".
const char* place_word(PlaceKind k) {
  switch (k) {
    case PlaceKind::City: return "city";
    case PlaceKind::Town: return "town";
    case PlaceKind::Village: return "village";
    default: return "hamlet";
  }
}

/// Zoom that shows a place well.
/// @param k Kind.
/// @return Ground metres per screen point.
Vrst_float64 place_zoom(PlaceKind k) {
  switch (k) {
    case PlaceKind::City: return 100;
    case PlaceKind::Town: return 50;
    case PlaceKind::Village: return 30;
    default: return 10;
  }
}

/// Whether every term is a whole word of a record's text (not only a prefix of one).
/// @param s Record.
/// @param terms Normalized query words.
/// @return true when all are whole words.
bool whole_words(const Vrst_search_street& s, const std::vector<std::string>& terms) {
  const std::string text = " " + normalize(s.name + " " + s.city + " " + s.postcode) + " ";
  return std::ranges::all_of(terms, [&](const std::string& t) { return text.find(" " + t + " ") != std::string::npos; });
}

/// A text without the spaces around it.
/// @param s Text.
/// @return Trimmed text.
std::string trimmed(std::string_view s) {
  while (!s.empty() && s.front() == ' ') s.remove_prefix(1);
  while (!s.empty() && s.back() == ' ') s.remove_suffix(1);
  return std::string(s);
}

}  // namespace

Vrst_search_result link_result(const Vrst_geo_link& link, std::optional<Vrst_latlon> near) {
  char coords[64];
  std::snprintf(coords, sizeof coords, "%.6f, %.6f", link.pos.lat, link.pos.lon);
  std::string detail = coords;
  if (!link.exact) detail += " (map center of the link)";
  if (near) detail += " - " + distance_text(distance_m(*near, link.pos));
  constexpr Vrst_float64 kPointZoom = 2;
  return {link.pos, Vrst_search_kind::Point, link.name.empty() ? "Point" : link.name, detail, kPointZoom};
}

std::vector<Vrst_search_result> search_link_address(const Vrst_search_index& index, std::string_view text,
                                                    std::optional<Vrst_latlon> near) {
  std::vector<std::string> parts;
  for (Vrst_size b = 0; b <= text.size();) {
    const Vrst_size e = std::min(text.find(',', b), text.size());
    if (std::string p = trimmed(text.substr(b, e - b)); !p.empty()) parts.push_back(std::move(p));
    b = e + 1;
  }
  // The city: the part with a postcode (digits), else the last but one (the last is the country).
  Vrst_size city = parts.size() >= 2 ? parts.size() - 2 : 0;
  for (Vrst_size i = 0; i < parts.size(); ++i)
    if (std::ranges::any_of(parts[i], [](char c) { return c >= '0' && c <= '9'; }) &&
        std::ranges::any_of(parts[i], [](char c) { return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z'); }) &&
        parse_query(parts[i]).number.empty())
      city = i;
  const auto from_link = [](Vrst_search_result r) {
    r.detail = "from the link's address - " + r.detail;
    return std::vector<Vrst_search_result>{std::move(r)};
  };
  for (Vrst_size i = 0; i < parts.size(); ++i) {
    if (i == city) continue;
    const std::string query = parts[i] + " " + (city < parts.size() ? parts[city] : std::string());
    for (Vrst_search_result& r : search_index(index, query, near, 1)) {
      // Only a street or number whose own words are all in this part, not a near miss.
      const std::string want = " " + normalize(parts[i]) + " ";
      const std::string got = normalize(r.title);
      bool all = r.kind != Vrst_search_kind::Place;
      for (Vrst_size b = 0; all && b < got.size();) {
        const Vrst_size e = std::min(got.find(' ', b), got.size());
        all = want.find(" " + got.substr(b, e - b) + " ") != std::string::npos;
        b = e + 1;
      }
      if (all) return from_link(std::move(r));
    }
  }
  if (city < parts.size())
    for (Vrst_search_result& r : search_index(index, parts[city], near, 1)) return from_link(std::move(r));
  return {};
}

std::vector<Vrst_search_result> search_index(const Vrst_search_index& index, std::string_view text,
                                             std::optional<Vrst_latlon> near, Vrst_size max) {
  const Vrst_search_query q = parse_query(text);
  const std::vector<Vrst_search_street> hits = index.find_streets(q.terms, kCandidates);
  const bool one_word = q.terms.size() == 1;

  // One candidate with what ranks it, compared in member order.
  struct Vrst_ranked {
    bool missing_number;  // A number was asked and this street does not have it.
    bool partial;         // Some term matches only as a prefix.
    bool not_place_first; // Not a place in a one-word query.
    int size;             // Place kind (city first); streets after places.
    Vrst_float64 dist;    // Metres from near.
    Vrst_size index;      // Into hits; also makes the order stable.
    std::optional<Vrst_coord> number;  // The house number's position, when found.
  };
  std::vector<Vrst_ranked> ranked;
  ranked.reserve(hits.size());
  for (Vrst_size i = 0; i < hits.size(); ++i) {
    const Vrst_search_street& s = hits[i];
    const bool place = s.kind == Vrst_search_kind::Place;
    std::optional<Vrst_coord> num;
    if (!q.number.empty() && !place) num = index.find_number(s.id, q.number);
    ranked.push_back({!q.number.empty() && !num, !whole_words(s, q.terms), !(one_word && place),
                      place ? static_cast<int>(s.place) : kPlaceKindCount,
                      near ? distance_m(*near, to_latlon(s.pos)) : 0.0, i, num});
  }
  std::ranges::sort(ranked, {}, [](const Vrst_ranked& r) {
    return std::tuple(r.missing_number, r.partial, r.not_place_first, r.size, r.dist, r.index);
  });

  std::vector<Vrst_search_result> out;
  for (const Vrst_ranked& rk : ranked) {
    if (out.size() >= max) break;
    const Vrst_search_street& s = hits[rk.index];
    Vrst_search_result r{to_latlon(s.pos), s.kind, s.name, {}, 3};
    std::string where;
    if (s.kind == Vrst_search_kind::Place) {
      where = place_word(s.place);
      r.metres_per_point = place_zoom(s.place);
    } else {
      where = s.city + (s.postcode.empty() ? "" : (s.city.empty() ? "" : " ") + s.postcode);
      if (rk.number) {
        r.pos = to_latlon(*rk.number);
        r.kind = Vrst_search_kind::Address;
        r.title = number_display(q.number) + " " + s.name;
        r.metres_per_point = 1;
      } else if (!q.number.empty()) {
        where = "no number " + number_display(q.number) + (where.empty() ? "" : " - " + where);
      }
    }
    if (near) where += (where.empty() ? "" : " - ") + distance_text(distance_m(*near, r.pos));
    r.detail = std::move(where);
    out.push_back(std::move(r));
  }
  return out;
}

}  // namespace nav
