/// @file
/// Vrst_address_collector: merging addresses, roads and places into search records.
/// Documentation is in osm_addresses.h.
#include "convert/osm_addresses.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>

#include "geometry/geo.h"
#include "search/search_text.h"

namespace nav {

namespace {

/// Grid cell of the city lookup, in 1e-7 degrees (0.05 deg, about 5 km).
constexpr Vrst_float64 kCellE7 = 500'000;
/// Rings of cells searched around a point before falling back to all cities (about 40 km).
constexpr int kMaxRing = 8;
/// Metres per 1e-7 degree of latitude.
constexpr Vrst_float64 kMetresPerE7 = kEarthRadius * kPi / 180.0 * 1e-7;

/// Finds the nearest city, town or village of a point.
class Vrst_city_grid {
 public:
  /// Indexes the cities.
  /// @param cities Position and name id of each city, town and village.
  explicit Vrst_city_grid(std::vector<std::pair<Vrst_coord, Vrst_uint32>> cities);
  /// Name of the nearest city.
  /// @param p Position.
  /// @return Name id; 0 when there are no cities.
  Vrst_uint32 nearest(Vrst_coord p) const;

 private:
  /// Cell of a coordinate.
  /// @param v Latitude or longitude in 1e-7 degrees.
  /// @return Cell index.
  static int cell(Vrst_int32 v) { return static_cast<int>(std::floor(v / kCellE7)); }
  /// Map key of a cell.
  /// @param cx Longitude cell.
  /// @param cy Latitude cell.
  /// @return Key.
  static Vrst_uint64 key(int cx, int cy) {
    return (static_cast<Vrst_uint64>(static_cast<Vrst_uint32>(cx)) << 32) | static_cast<Vrst_uint32>(cy);
  }
  /// Squared distance in metres (equirectangular).
  /// @param a First point.
  /// @param b Second point.
  /// @param kx cos(latitude) of the query point.
  /// @return Metres squared.
  static Vrst_float64 d2(Vrst_coord a, Vrst_coord b, Vrst_float64 kx) {
    const Vrst_float64 x = (b.lon_e7 - a.lon_e7) * kx * kMetresPerE7, y = (b.lat_e7 - a.lat_e7) * kMetresPerE7;
    return x * x + y * y;
  }

  /// The cities.
  std::vector<std::pair<Vrst_coord, Vrst_uint32>> m_cities;
  /// Cell -> indices into m_cities.
  std::unordered_map<Vrst_uint64, std::vector<Vrst_uint32>> m_cells;
};

Vrst_city_grid::Vrst_city_grid(std::vector<std::pair<Vrst_coord, Vrst_uint32>> cities) : m_cities(std::move(cities)) {
  for (Vrst_size i = 0; i < m_cities.size(); ++i)
    m_cells[key(cell(m_cities[i].first.lon_e7), cell(m_cities[i].first.lat_e7))].push_back(static_cast<Vrst_uint32>(i));
}

Vrst_uint32 Vrst_city_grid::nearest(Vrst_coord p) const {
  if (m_cities.empty()) return 0;
  const Vrst_float64 kx = std::cos(p.lat_e7 * 1e-7 * kPi / 180.0);
  // Every point of ring r is at least (r - 1) cells away; a longitude cell is the shorter side.
  const Vrst_float64 cell_m = kCellE7 * kMetresPerE7 * kx;
  const int cx = cell(p.lon_e7), cy = cell(p.lat_e7);
  Vrst_float64 best = std::numeric_limits<Vrst_float64>::max();
  Vrst_uint32 name = 0;
  const auto visit = [&](int x, int y) {
    const auto it = m_cells.find(key(x, y));
    if (it == m_cells.end()) return;
    for (const Vrst_uint32 i : it->second)
      if (const Vrst_float64 d = d2(p, m_cities[i].first, kx); d < best) {
        best = d;
        name = m_cities[i].second;
      }
  };
  for (int r = 0; r <= kMaxRing; ++r) {
    const Vrst_float64 ring_min = (r - 1) * cell_m;
    if (name != 0 && r > 0 && ring_min * ring_min > best) return name;
    for (int x = cx - r; x <= cx + r; ++x) {
      visit(x, cy - r);
      if (r > 0) visit(x, cy + r);
    }
    for (int y = cy - r + 1; y <= cy + r - 1; ++y) {
      visit(cx - r, y);
      visit(cx + r, y);
    }
  }
  if (name != 0) return name;
  for (const auto& [pos, n] : m_cities)
    if (const Vrst_float64 d = d2(p, pos, kx); d < best) {
      best = d;
      name = n;
    }
  return name;
}

}  // namespace

void Vrst_address_collector::add_address(Vrst_coord pos, const Vrst_address_tags& tags) {
  const std::string number = number_key(tags.number);
  const std::string_view street = !tags.street.empty() ? tags.street : tags.place;
  if (number.empty() || street.empty()) return;
  m_rows.push_back({pos, m_strings.intern(number), m_strings.intern(street), m_strings.intern(tags.city),
                    m_strings.intern(tags.postcode)});
  ++m_numbers;
}

void Vrst_address_collector::add_road(Vrst_coord pos, std::string_view name) {
  if (!name.empty()) m_rows.push_back({pos, 0, m_strings.intern(name), 0, 0});
}

void Vrst_address_collector::add_place(Vrst_coord pos, PlaceKind kind, std::string_view name) {
  if (!name.empty()) m_places.push_back({pos, kind, m_strings.intern(name)});
}

Vrst_search_stats Vrst_address_collector::write(Vrst_search_index_writer& writer, Vrst_uint64 build_id) {
  std::vector<std::pair<Vrst_coord, Vrst_uint32>> cities;
  for (const Vrst_place_row& p : m_places)
    if (p.kind != PlaceKind::Hamlet) cities.emplace_back(p.pos, p.name);
  const Vrst_city_grid grid(std::move(cities));

  // Spellings that normalize the same ("DIJON", "Dijon") are one name.
  std::vector<Vrst_uint32> canon(m_strings.strings().size(), std::numeric_limits<Vrst_uint32>::max());
  std::unordered_map<std::string, Vrst_uint32> canon_of;
  const auto canonical = [&](Vrst_uint32 id) {
    if (canon[id] == std::numeric_limits<Vrst_uint32>::max())
      canon[id] = canon_of.try_emplace(normalize(m_strings.str(id)), id).first->second;
    return canon[id];
  };

  // Streets: one per (name, city), with the sum of their points for the middle.
  struct Vrst_street_acc {
    Vrst_uint32 name, city, postcode;  // first spelling seen
    Vrst_int64 lat_sum, lon_sum;
    Vrst_uint32 count;
    Vrst_coord best;                   // row nearest to the middle
    Vrst_float64 best_d;
  };
  std::vector<Vrst_street_acc> streets;
  std::unordered_map<Vrst_uint64, Vrst_uint32> street_of;
  std::vector<Vrst_uint32> row_street(m_rows.size());
  for (Vrst_size i = 0; i < m_rows.size(); ++i) {
    Vrst_row& r = m_rows[i];
    if (r.city == 0) r.city = grid.nearest(r.pos);
    const Vrst_uint64 k = (static_cast<Vrst_uint64>(canonical(r.street)) << 32) | (r.city ? canonical(r.city) : 0);
    const auto [it, added] = street_of.try_emplace(k, static_cast<Vrst_uint32>(streets.size()));
    if (added) streets.push_back({r.street, r.city, r.postcode, 0, 0, 0, r.pos, std::numeric_limits<Vrst_float64>::max()});
    Vrst_street_acc& s = streets[it->second];
    s.lat_sum += r.pos.lat_e7;
    s.lon_sum += r.pos.lon_e7;
    ++s.count;
    if (s.postcode == 0) s.postcode = r.postcode;
    row_street[i] = it->second;
  }
  for (Vrst_size i = 0; i < m_rows.size(); ++i) {
    Vrst_street_acc& s = streets[row_street[i]];
    const Vrst_float64 dy = static_cast<Vrst_float64>(m_rows[i].pos.lat_e7) - static_cast<Vrst_float64>(s.lat_sum) / s.count;
    const Vrst_float64 dx = static_cast<Vrst_float64>(m_rows[i].pos.lon_e7) - static_cast<Vrst_float64>(s.lon_sum) / s.count;
    if (const Vrst_float64 d = dx * dx + dy * dy; d < s.best_d) {
      s.best_d = d;
      s.best = m_rows[i].pos;
    }
  }

  // Ids in a fixed order, so the same input always gives the same file: places, then streets.
  std::ranges::sort(m_places, {}, [&](const Vrst_place_row& p) {
    return std::tuple(p.kind, std::string_view(m_strings.str(p.name)), p.pos.lat_e7, p.pos.lon_e7);
  });
  std::vector<Vrst_uint32> street_order(streets.size());
  for (Vrst_size i = 0; i < street_order.size(); ++i) street_order[i] = static_cast<Vrst_uint32>(i);
  std::ranges::sort(street_order, {}, [&](Vrst_uint32 i) {
    return std::tuple(std::string_view(m_strings.str(streets[i].name)), std::string_view(m_strings.str(streets[i].city)),
                      i);
  });
  std::vector<Vrst_uint32> street_id(streets.size());
  Vrst_uint32 next = 0;
  for (const Vrst_place_row& p : m_places)
    writer.add_street({next++, Vrst_search_kind::Place, p.kind, m_strings.str(p.name), {}, {}, p.pos});
  for (const Vrst_uint32 i : street_order) {
    const Vrst_street_acc& s = streets[i];
    street_id[i] = next;
    writer.add_street({next++, Vrst_search_kind::Street, PlaceKind::City, m_strings.str(s.name), m_strings.str(s.city),
                       m_strings.str(s.postcode), s.best});
  }

  std::vector<Vrst_uint32> numbered;
  for (Vrst_size i = 0; i < m_rows.size(); ++i)
    if (m_rows[i].number != 0) numbered.push_back(static_cast<Vrst_uint32>(i));
  std::ranges::sort(numbered, {}, [&](Vrst_uint32 i) {
    return std::tuple(street_id[row_street[i]], std::string_view(m_strings.str(m_rows[i].number)), i);
  });
  for (const Vrst_uint32 i : numbered)
    writer.add_address({street_id[row_street[i]], m_strings.str(m_rows[i].number), m_rows[i].pos});

  Vrst_search_stats st;
  st.addresses = numbered.size();
  st.streets = streets.size();
  st.places = m_places.size();
  st.bytes = writer.finish(build_id);
  return st;
}

}  // namespace nav
