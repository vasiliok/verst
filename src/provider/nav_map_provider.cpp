/// @file
/// Vrst_nav_map_provider and open_map().
/// Declarations and documentation are in nav_map_provider.h and map_provider.h.
#include "provider/nav_map_provider.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <exception>
#include <limits>

#include "search/search_rank.h"

namespace nav {

namespace {

/// Milliseconds since a moment.
/// @param since Start.
/// @return Elapsed ms.
Vrst_float64 ms_since(std::chrono::steady_clock::time_point since) {
  return std::chrono::duration<Vrst_float64, std::milli>(std::chrono::steady_clock::now() - since).count();
}

/// One area of a build as text: "area <id> <kind> <stored>><simplified>><clipped> pts <ms> ms"
/// (0 for a step not reached yet).
/// @param a Area probe.
/// @return Text.
std::string area_text(const Vrst_area_probe& a) {
  static constexpr const char* kKinds[] = {"farmland", "meadow", "residential", "industrial",
                                           "park",     "forest", "water",       "building"};
  static_assert(std::size(kKinds) == kAreaKindCount);
  const char* kind = a.kind >= 0 && a.kind < kAreaKindCount ? kKinds[a.kind] : "?";
  std::array<char, 128> buf{};
  const int n = std::snprintf(buf.data(), buf.size(), "area %u %s %zu>%zu>%zu pts %.0f ms", a.id, kind, a.raw_points,
                              a.simplified, a.clipped, a.ms);
  return {buf.data(), static_cast<Vrst_size>(std::clamp(n, 0, static_cast<int>(buf.size()) - 1))};
}

/// Where a build is, as text: "<stage> <done>/<total>", and in the areas stage the
/// current area and its step, e.g. "areas 812/4031: area 991234 forest 18234>812>640 pts
/// 3200 ms, triangulate".
/// @param w Build snapshot.
/// @return Text.
std::string step_text(const Vrst_build_snapshot& w) {
  static constexpr const char* kStages[] = {"idle", "roads", "lines", "areas", "dots", "finish"};
  static constexpr const char* kSteps[] = {"start", "simplify", "clip", "triangulate", "copy"};
  std::array<char, 48> buf{};
  const int n = std::snprintf(buf.data(), buf.size(), "%s %zu/%zu", kStages[static_cast<int>(w.stage)], w.done,
                              w.total);
  std::string out(buf.data(), static_cast<Vrst_size>(std::clamp(n, 0, static_cast<int>(buf.size()) - 1)));
  if (w.stage == Vrst_build_stage::Areas) out += ": " + area_text(w.area) + ", " + kSteps[static_cast<int>(w.step)];
  return out;
}

}  // namespace

Vrst_nav_map_provider::Vrst_nav_map_provider(const std::string& dir, const Vrst_map_options& options)
    : m_record(options.record_events), m_loader(m_builder, options.threads) {
  const auto t0 = std::chrono::steady_clock::now();
  m_data.open(dir);
  m_open_ms = ms_since(t0);
  const auto t1 = std::chrono::steady_clock::now();
  m_index.build(m_data);
  m_index_ms = ms_since(t1);
  m_loader.record_events(m_record);
  // The extent is the box of the road shape points: what the map draws.
  Vrst_int32 lat0 = std::numeric_limits<Vrst_int32>::max(), lon0 = lat0;
  Vrst_int32 lat1 = std::numeric_limits<Vrst_int32>::min(), lon1 = lat1;
  for (const Vrst_coord& c : m_data.geom().points()) {
    lat0 = std::min(lat0, c.lat_e7), lat1 = std::max(lat1, c.lat_e7);
    lon0 = std::min(lon0, c.lon_e7), lon1 = std::max(lon1, c.lon_e7);
  }
  if (lat0 <= lat1) m_extent = {to_latlon({lat0, lon0}), to_latlon({lat1, lon1})};
  // The map works without search: a missing or broken index only turns search off.
  try {
    m_search = open_search_index(dir);
    if (!m_search) {
      m_search_note = "no search index";
    } else if (m_search->build_id() != m_data.graph().file().build_id()) {
      m_search.reset();
      m_search_note = "search index of another build";
    }
  } catch (const std::exception& e) {
    m_search_note = std::string("search index unreadable: ") + e.what();
  }
}

std::vector<Vrst_search_result> Vrst_nav_map_provider::search(std::string_view text, std::optional<Vrst_latlon> near,
                                                              Vrst_size max) const {
  if (const std::optional<Vrst_geo_link> link = parse_geo_link(text)) {
    Vrst_search_result r = link_result(*link, near);
    const Vrst_latlon p = link->pos;
    if (p.lat < m_extent.south_west.lat || p.lat > m_extent.north_east.lat || p.lon < m_extent.south_west.lon ||
        p.lon > m_extent.north_east.lon)
      r.detail += " - outside this map";
    return {r};
  }
  if (!m_search) return {};
  if (const std::optional<std::string> address = parse_link_address(text))
    return search_link_address(*m_search, *address, near);
  return search_index(*m_search, text, near, max);
}

std::vector<Vrst_map_place> Vrst_nav_map_provider::places() const {
  std::vector<Vrst_map_place> out;
  out.reserve(m_data.names().places().size());
  for (const Vrst_place& p : m_data.names().places())
    out.push_back({to_latlon(p.pos), static_cast<PlaceKind>(p.kind), p.population,
                   std::string(m_data.names().get(p.name_id))});
  return out;
}

std::vector<Vrst_piece_need> Vrst_nav_map_provider::show(const Vrst_map_area& area, const Vrst_has_piece& has) {
  const int level = level_for(area.metres_per_px);
  std::vector<Vrst_piece_need> needs;
  std::vector<Vrst_tile_id> missing;
  for (const Vrst_tile_id& t : visible_tiles(area, kTileLevels[level].z)) {
    if (!m_index.contains(t)) continue;  // nothing there
    Vrst_piece_need n{tile_key(t), {}};
    for (int l = level + 1; l < kTileLevelCount; ++l) {
      const int shift = t.z - kTileLevels[l].z;
      n.fallbacks.push_back(tile_key({kTileLevels[l].z, t.x >> shift, t.y >> shift}));
    }
    if (!has(n.key)) missing.push_back(t);
    needs.push_back(std::move(n));
  }
  m_missing = missing.size();
  m_loader.request(missing);
  const bool now_settled = settled(m_loader.status());
  if (now_settled && !m_settled && m_record) m_settled_at.push_back(Vrst_clock::now());
  m_settled = now_settled;
  return needs;
}

std::vector<Vrst_map_piece> Vrst_nav_map_provider::take_ready(Vrst_size max) {
  std::vector<Vrst_map_piece> out;
  for (std::unique_ptr<Vrst_map_piece>& p : m_loader.take(max)) out.push_back(std::move(*p));
  return out;
}

Vrst_provider_status Vrst_nav_map_provider::status() const {
  const Vrst_loader_status s = m_loader.status();
  Vrst_provider_status out;
  out.threads = s.workers.size();
  out.queued = s.queued;
  out.working = s.building;
  out.ready = s.done;
  out.done_total = s.built;
  out.failed_total = s.failed;
  out.dropped_total = s.dropped;
  out.missing = m_missing;
  out.settled = settled(s);
  for (const Vrst_worker_state& w : s.workers)
    out.workers.push_back({w.busy, w.busy ? tile_key(w.tile) : 0, w.busy_ms, w.built, w.busy ? step_text(w.work) : ""});
  return out;
}

std::vector<Vrst_work_event> Vrst_nav_map_provider::take_events() {
  std::vector<Vrst_work_event> out;
  for (const Vrst_loader_event& e : m_loader.take_events()) {
    static constexpr Vrst_work_event_kind kKind[] = {Vrst_work_event_kind::Started, Vrst_work_event_kind::Done,
                                                     Vrst_work_event_kind::Failed};
    out.push_back({kKind[static_cast<int>(e.kind)], tile_key(e.tile), e.worker, e.build_ms, e.age_ms,
                   e.slowest.ms > 0 ? "slowest " + area_text(e.slowest) : ""});
  }
  const Vrst_clock::time_point now = Vrst_clock::now();
  for (const Vrst_clock::time_point at : m_settled_at)
    out.push_back({Vrst_work_event_kind::Settled, 0, 0, 0,
                   std::chrono::duration<Vrst_float64, std::milli>(now - at).count(), ""});
  m_settled_at.clear();
  std::ranges::stable_sort(out, std::ranges::greater{}, &Vrst_work_event::age_ms);  // oldest first
  return out;
}

std::vector<std::string> Vrst_nav_map_provider::work_dump(int worker) const {
  const Vrst_build_snapshot w = m_loader.work_of(worker);
  std::vector<std::string> out{"step: " + step_text(w)};
  if (w.slowest.ms > 0) out.push_back("slowest: " + area_text(w.slowest));
  if (!w.ring.empty()) {
    // The exact earcut input: absolute Web Mercator metres, %.17g so the doubles come
    // back bit for bit; "x,y" pairs separated by spaces.
    std::string ring = "ring: ";
    std::array<char, 64> buf{};
    for (const Vrst_vec2d& p : w.ring) {
      const int n = std::snprintf(buf.data(), buf.size(), "%.17g,%.17g ", p.x, p.y);
      ring.append(buf.data(), static_cast<Vrst_size>(std::clamp(n, 0, static_cast<int>(buf.size()) - 1)));
    }
    out.push_back(std::move(ring));
  }
  return out;
}

std::string Vrst_nav_map_provider::piece_name(Vrst_uint64 key) const {
  const Vrst_tile_id t = tile_of_key(key);
  return std::to_string(t.z) + "/" + std::to_string(t.x) + "/" + std::to_string(t.y);
}

std::vector<std::string> Vrst_nav_map_provider::describe() const {
  const Vrst_graph& g = m_data.graph();
  char sizes[96], content[96], timing[96];
  std::snprintf(sizes, sizeof sizes, "V %u  S %u  A %u  P %zu", g.num_vertices(), m_data.edges().num_segments(),
                g.num_arcs(), m_data.geom().points().size());
  std::snprintf(content, sizeof content, "dead ends %zu  places %zu%s", m_index.dead_ends(),
                m_data.names().places().size(), m_data.has_map() ? "" : "  roads only (no map.dat)");
  std::snprintf(timing, sizeof timing, "open %.0f ms  index %.0f ms  %zu z14 tiles", m_open_ms, m_index_ms,
                m_index.size(0));
  return {sizes, content, timing, m_search ? "search index ok" : m_search_note};
}

std::unique_ptr<Vrst_map_provider> open_map(const std::string& location, const Vrst_map_options& options) {
  return std::make_unique<Vrst_nav_map_provider>(location, options);
}

}  // namespace nav
