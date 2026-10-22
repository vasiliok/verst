// osm2nav <input.osm.pbf> <out_dir>
//
// Converts OSM car roads into graph.dat / edges.dat / geom.dat / names.dat.
// Pass 1 reads ways, pass 2 reads node coordinates of road nodes only.

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <optional>
#include <random>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#ifndef _WIN32
#include <sys/resource.h>
#endif

#include <osmium/io/pbf_input.hpp>
#include <osmium/osm/node.hpp>
#include <osmium/osm/way.hpp>

#include "format.h"

namespace fs = std::filesystem;
using namespace nav;

namespace {

// ===========================================================================
// OSM source interface (libosmium now, own protozero PBF reader later)
// ===========================================================================

struct OsmTag {
  std::string_view key;
  std::string_view value;
};

struct OsmWay {
  uint64_t id;
  std::span<const uint64_t> refs;
  std::span<const OsmTag> tags;

  std::string_view tag(std::string_view key) const {
    for (const OsmTag& t : tags)
      if (t.key == key) return t.value;
    return {};
  }
};

struct OsmNode {
  uint64_t id;
  Coord coord;
};

class IOsmSource {
 public:
  virtual ~IOsmSource() = default;
  virtual std::optional<BBox> header_bbox() = 0;
  virtual void for_each_way(const std::function<void(const OsmWay&)>& fn) = 0;
  virtual void for_each_node(const std::function<void(const OsmNode&)>& fn) = 0;
};

class OsmiumSource final : public IOsmSource {
 public:
  explicit OsmiumSource(std::string path) : path_(std::move(path)) {}

  std::optional<BBox> header_bbox() override {
    osmium::io::Reader reader{path_, osmium::osm_entity_bits::nothing};
    const osmium::Box box = reader.header().box();
    reader.close();
    if (!box.valid()) return std::nullopt;
    return BBox{{box.bottom_left().y(), box.bottom_left().x()}, {box.top_right().y(), box.top_right().x()}};
  }

  void for_each_way(const std::function<void(const OsmWay&)>& fn) override {
    osmium::io::Reader reader{path_, osmium::osm_entity_bits::way};
    std::vector<uint64_t> refs;
    std::vector<OsmTag> tags;
    while (osmium::memory::Buffer buffer = reader.read()) {
      for (const osmium::Way& w : buffer.select<osmium::Way>()) {
        refs.clear();
        tags.clear();
        for (const osmium::NodeRef& nr : w.nodes()) refs.push_back(static_cast<uint64_t>(nr.ref()));
        for (const osmium::Tag& t : w.tags()) tags.push_back({t.key(), t.value()});
        fn(OsmWay{static_cast<uint64_t>(w.id()), refs, tags});
      }
    }
    reader.close();
  }

  void for_each_node(const std::function<void(const OsmNode&)>& fn) override {
    osmium::io::Reader reader{path_, osmium::osm_entity_bits::node};
    while (osmium::memory::Buffer buffer = reader.read()) {
      for (const osmium::Node& n : buffer.select<osmium::Node>()) {
        const osmium::Location loc = n.location();
        if (!loc.valid()) continue;
        fn(OsmNode{static_cast<uint64_t>(n.id()), Coord{loc.y(), loc.x()}});
      }
    }
    reader.close();
  }

 private:
  std::string path_;
};

// ===========================================================================
// Car profile
// ===========================================================================

struct ClassDef {
  std::string_view highway;  // empty: not selected by highway tag
  RoadClass cls;
  uint8_t speed_kmh;         // default when maxspeed is unknown
  bool link;
};

// Indexed by RoadClass.
constexpr ClassDef kClasses[] = {
    {"motorway", RoadClass::Motorway, 110, false},
    {"motorway_link", RoadClass::MotorwayLink, 60, true},
    {"trunk", RoadClass::Trunk, 90, false},
    {"trunk_link", RoadClass::TrunkLink, 50, true},
    {"primary", RoadClass::Primary, 70, false},
    {"primary_link", RoadClass::PrimaryLink, 40, true},
    {"secondary", RoadClass::Secondary, 60, false},
    {"secondary_link", RoadClass::SecondaryLink, 40, true},
    {"tertiary", RoadClass::Tertiary, 50, false},
    {"tertiary_link", RoadClass::TertiaryLink, 30, true},
    {"unclassified", RoadClass::Unclassified, 40, false},
    {"residential", RoadClass::Residential, 30, false},
    {"living_street", RoadClass::LivingStreet, 10, false},
    {"service", RoadClass::Service, 15, false},
    {"", RoadClass::Ferry, 10, false},
};
static_assert(std::size(kClasses) == kRoadClassCount);
static_assert([] {
  for (int i = 0; i < kRoadClassCount; ++i)
    if (static_cast<int>(kClasses[i].cls) != i) return false;
  return true;
}());

const ClassDef* find_class(std::string_view highway) {
  if (highway.empty()) return nullptr;
  for (const ClassDef& c : kClasses)
    if (c.highway == highway) return &c;
  return nullptr;
}

// Returns km/h, 0 = unknown / no limit.
uint8_t parse_maxspeed(std::string_view v) {
  if (v.empty() || v == "none" || v == "signals" || v == "variable") return 0;
  if (v == "walk") return 6;
  if (v == "FR:urban") return 50;
  if (v == "FR:rural") return 80;
  if (v == "FR:motorway") return 130;
  int ivalue = 0;  // integer from_chars: Apple libc++ lacks the floating-point overload
  auto [p, ec] = std::from_chars(v.data(), v.data() + v.size(), ivalue);
  if (ec != std::errc() || ivalue <= 0) return 0;
  double value = ivalue;
  std::string_view unit(p, v.data() + v.size() - p);
  while (!unit.empty() && unit.front() == ' ') unit.remove_prefix(1);
  if (unit.starts_with("mph"))
    value *= 1.609344;
  else if (!(unit.empty() || unit.front() == ';' || unit == "km/h" || unit == "kmh" || unit == "kph"))
    return 0;
  return static_cast<uint8_t>(std::clamp(std::lround(value), 1L, 255L));
}

enum class Dir : uint8_t { Both, Fwd, Bwd };

struct WayInfo {
  uint64_t osm_id;
  uint32_t name_id;
  uint32_t ref_id;
  RoadClass cls;
  Dir dir;
  uint8_t maxspeed_kmh;
  uint16_t flags;
};

bool is_yes(std::string_view v) { return v == "yes" || v == "designated"; }
bool is_set(std::string_view v) { return !v.empty() && v != "no"; }

// Returns nullopt when the way is not a car road.
std::optional<WayInfo> classify(const OsmWay& w) {
  const ClassDef* cd = find_class(w.tag("highway"));
  const std::string_view mc = w.tag("motorcar");
  const std::string_view mv = w.tag("motor_vehicle");
  const std::string_view car = !mc.empty() ? mc : mv;  // most specific wins

  if (!cd) {
    if (w.tag("route") != "ferry" || !is_yes(car)) return std::nullopt;
    cd = &kClasses[static_cast<int>(RoadClass::Ferry)];
  }
  if (w.tag("area") == "yes") return std::nullopt;
  if (car == "no") return std::nullopt;
  const std::string_view access = w.tag("access");
  if ((access == "no" || access == "private") && !is_yes(car)) return std::nullopt;

  WayInfo info{};
  info.osm_id = w.id;
  info.cls = cd->cls;
  info.maxspeed_kmh = parse_maxspeed(w.tag("maxspeed"));

  const std::string_view junction = w.tag("junction");
  const bool roundabout = junction == "roundabout" || junction == "circular";
  if (roundabout) info.flags |= kSegRoundabout;
  if (cd->link) info.flags |= kSegLink;
  if (w.tag("toll") == "yes") info.flags |= kSegToll;
  if (is_set(w.tag("tunnel"))) info.flags |= kSegTunnel;
  if (is_set(w.tag("bridge"))) info.flags |= kSegBridge;
  if (cd->cls == RoadClass::Ferry) info.flags |= kSegFerry;

  const std::string_view oneway = w.tag("oneway");
  if (oneway == "yes" || oneway == "true" || oneway == "1")
    info.dir = Dir::Fwd;
  else if (oneway == "-1" || oneway == "reverse")
    info.dir = Dir::Bwd;
  else if (oneway == "no" || oneway == "false" || oneway == "0")
    info.dir = Dir::Both;
  else
    info.dir = (roundabout || cd->cls == RoadClass::Motorway) ? Dir::Fwd : Dir::Both;
  if (info.dir != Dir::Both) info.flags |= kSegOneway;
  if (info.dir == Dir::Bwd) info.flags |= kSegOnewayReverse;
  return info;
}

uint32_t speed_kmh(const WayInfo& w) {
  return w.maxspeed_kmh ? w.maxspeed_kmh : kClasses[static_cast<int>(w.cls)].speed_kmh;
}

// ===========================================================================
// Helpers
// ===========================================================================

class StringPool {
 public:
  StringPool() { strings_.emplace_back(); }
  uint32_t intern(std::string_view s) {
    if (s.empty()) return 0;
    auto [it, inserted] = ids_.try_emplace(std::string(s), static_cast<uint32_t>(strings_.size()));
    if (inserted) strings_.emplace_back(s);
    return it->second;
  }
  const std::vector<std::string>& strings() const { return strings_; }

 private:
  std::unordered_map<std::string, uint32_t> ids_;
  std::vector<std::string> strings_;
};

constexpr Coord kInvalidCoord{std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::min()};
constexpr uint32_t kNone = std::numeric_limits<uint32_t>::max();

double haversine_m(Coord p, Coord q) {
  constexpr double kEarthRadiusM = 6371008.8;
  constexpr double kDeg = 1e-7 * 3.14159265358979323846 / 180.0;
  const double lat1 = p.lat_e7 * kDeg, lat2 = q.lat_e7 * kDeg;
  const double dlat = lat2 - lat1, dlon = (q.lon_e7 - p.lon_e7) * kDeg;
  const double h = std::sin(dlat / 2) * std::sin(dlat / 2) +
                   std::cos(lat1) * std::cos(lat2) * std::sin(dlon / 2) * std::sin(dlon / 2);
  return 2 * kEarthRadiusM * std::asin(std::min(1.0, std::sqrt(h)));
}

class FileWriter {
 public:
  FileWriter(FileType type, uint64_t build_id) : type_(type), build_id_(build_id) {}

  template <class T>
  void add(SectionId id, std::span<const T> data) {
    sections_.push_back({id, sizeof(T), data.size(), data.data()});
  }
  template <class T>
  void add(SectionId id, const std::vector<T>& data) {
    add(id, std::span<const T>(data));
  }

  uint64_t write(const fs::path& path) const {
    FileHeader h{};
    std::memcpy(h.magic, magic_for(type_), 8);
    h.format_version = kFormatVersion;
    h.file_type = static_cast<uint32_t>(type_);
    h.build_id = build_id_;
    h.section_count = static_cast<uint32_t>(sections_.size());

    auto align = [](uint64_t x) { return (x + kSectionAlign - 1) & ~(kSectionAlign - 1); };
    std::vector<SectionEntry> table;
    uint64_t offset = sizeof(FileHeader) + sections_.size() * sizeof(SectionEntry);
    for (const Sec& s : sections_) {
      offset = align(offset);
      table.push_back({static_cast<uint32_t>(s.id), s.elem_size, offset, s.count});
      offset += s.count * s.elem_size;
    }

    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) throw std::runtime_error("cannot create " + path.string());
    out.write(reinterpret_cast<const char*>(&h), sizeof h);
    out.write(reinterpret_cast<const char*>(table.data()), table.size() * sizeof(SectionEntry));
    uint64_t pos = sizeof(FileHeader) + table.size() * sizeof(SectionEntry);
    static constexpr char kZeros[kSectionAlign] = {};
    for (size_t i = 0; i < sections_.size(); ++i) {
      out.write(kZeros, static_cast<std::streamsize>(table[i].offset - pos));
      const uint64_t bytes = sections_[i].count * sections_[i].elem_size;
      out.write(static_cast<const char*>(sections_[i].data), static_cast<std::streamsize>(bytes));
      pos = table[i].offset + bytes;
    }
    if (!out.flush()) throw std::runtime_error("write failed: " + path.string());
    return pos;
  }

 private:
  struct Sec {
    SectionId id;
    uint32_t elem_size;
    uint64_t count;
    const void* data;
  };
  FileType type_;
  uint64_t build_id_;
  std::vector<Sec> sections_;
};

class PhaseTimer {
 public:
  void phase(const char* name) {
    const auto now = std::chrono::steady_clock::now();
    std::printf("  %-22s %8.1f ms\n", name, std::chrono::duration<double, std::milli>(now - last_).count());
    last_ = now;
  }
  double total_ms() const {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_).count();
  }

 private:
  std::chrono::steady_clock::time_point start_ = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point last_ = start_;
};

double peak_rss_mb() {
#ifdef _WIN32
  return 0;
#else
  rusage ru{};
  getrusage(RUSAGE_SELF, &ru);
#ifdef __APPLE__
  return ru.ru_maxrss / (1024.0 * 1024.0);  // bytes
#else
  return ru.ru_maxrss / 1024.0;  // KiB
#endif
#endif
}

uint64_t make_build_id(const std::string& input) {
  std::random_device rd;
  uint64_t id = (uint64_t(rd()) << 32) ^ rd();
  id ^= std::hash<std::string>{}(input);
  id ^= static_cast<uint64_t>(std::chrono::system_clock::now().time_since_epoch().count()) * 0x9E3779B97F4A7C15ull;
  return id ? id : 1;
}

// ===========================================================================
// Segments
// ===========================================================================

struct SegTmp {
  uint32_t a_node;  // dense node index
  uint32_t b_node;
  uint32_t way;
  uint32_t length_dm;
};

// The single place that decides vertex order. Returns node index -> vertex id
// (kNone for non-vertices) and fills vertex coordinates.
// TODO(hilbert): sort used vertex nodes by Hilbert key of their coordinate.
std::vector<uint32_t> number_vertices(const std::vector<SegTmp>& segs, const std::vector<Coord>& node_coord,
                                      std::vector<Coord>& vertex_coord) {
  std::vector<uint32_t> node_to_vertex(node_coord.size(), kNone);
  for (const SegTmp& s : segs) node_to_vertex[s.a_node] = node_to_vertex[s.b_node] = 0;
  uint32_t next = 0;
  vertex_coord.clear();
  for (size_t n = 0; n < node_to_vertex.size(); ++n) {
    if (node_to_vertex[n] == kNone) continue;
    node_to_vertex[n] = next++;
    vertex_coord.push_back(node_coord[n]);
  }
  return node_to_vertex;
}

int run(const std::string& input, const fs::path& out_dir) {
  PhaseTimer timer;
  OsmiumSource source(input);

  // --- pass 1: ways --------------------------------------------------------
  std::vector<WayInfo> ways;
  std::vector<uint64_t> way_first_ref{0};
  std::vector<uint64_t> refs;
  StringPool names;
  uint64_t ways_read = 0;

  source.for_each_way([&](const OsmWay& w) {
    ++ways_read;
    if (w.refs.size() < 2) return;
    std::optional<WayInfo> info = classify(w);
    if (!info) return;
    info->name_id = names.intern(w.tag("name"));
    info->ref_id = names.intern(w.tag("ref"));
    ways.push_back(*info);
    for (uint64_t r : w.refs)
      if (refs.size() == way_first_ref.back() || refs.back() != r) refs.push_back(r);  // drop repeated refs
    way_first_ref.push_back(refs.size());
  });
  timer.phase("pass 1 (ways)");

  // --- node table ----------------------------------------------------------
  std::vector<uint64_t> node_ids = refs;
  std::sort(node_ids.begin(), node_ids.end());
  node_ids.erase(std::unique(node_ids.begin(), node_ids.end()), node_ids.end());
  const size_t num_nodes = node_ids.size();

  std::vector<uint32_t> way_nodes(refs.size());
  for (size_t i = 0; i < refs.size(); ++i)
    way_nodes[i] = static_cast<uint32_t>(std::lower_bound(node_ids.begin(), node_ids.end(), refs[i]) - node_ids.begin());
  std::vector<uint64_t>().swap(refs);

  std::vector<uint8_t> uses(num_nodes, 0);
  std::vector<uint8_t> is_vertex(num_nodes, 0);
  for (size_t w = 0; w < ways.size(); ++w) {
    const uint64_t b = way_first_ref[w], e = way_first_ref[w + 1];
    for (uint64_t i = b; i < e; ++i)
      if (uses[way_nodes[i]] < 2) ++uses[way_nodes[i]];
    is_vertex[way_nodes[b]] = is_vertex[way_nodes[e - 1]] = 1;
  }
  for (size_t n = 0; n < num_nodes; ++n)
    if (uses[n] >= 2) is_vertex[n] = 1;
  std::vector<uint8_t>().swap(uses);
  timer.phase("node table");

  // --- pass 2: node coordinates --------------------------------------------
  std::vector<Coord> node_coord(num_nodes, kInvalidCoord);
  BBox extent{{std::numeric_limits<int32_t>::max(), std::numeric_limits<int32_t>::max()},
              {std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::min()}};
  uint64_t nodes_read = 0;
  source.for_each_node([&](const OsmNode& n) {
    ++nodes_read;
    extent.min.lat_e7 = std::min(extent.min.lat_e7, n.coord.lat_e7);
    extent.min.lon_e7 = std::min(extent.min.lon_e7, n.coord.lon_e7);
    extent.max.lat_e7 = std::max(extent.max.lat_e7, n.coord.lat_e7);
    extent.max.lon_e7 = std::max(extent.max.lon_e7, n.coord.lon_e7);
    auto it = std::lower_bound(node_ids.begin(), node_ids.end(), n.id);
    if (it != node_ids.end() && *it == n.id) node_coord[it - node_ids.begin()] = n.coord;
  });
  std::vector<uint64_t>().swap(node_ids);
  timer.phase("pass 2 (nodes)");

  // --- segments ------------------------------------------------------------
  // Ends of every run of known nodes become vertices: a missing node breaks the way.
  uint64_t missing_refs = 0, missing_breaks = 0, ways_with_missing = 0;
  for (size_t w = 0; w < ways.size(); ++w) {
    const uint64_t b = way_first_ref[w], e = way_first_ref[w + 1];
    bool any_missing = false, seen_valid = false, gap = false;
    for (uint64_t i = b; i < e; ++i) {
      const uint32_t n = way_nodes[i];
      if (node_coord[n] == kInvalidCoord) {
        ++missing_refs;
        any_missing = true;
        if (seen_valid) gap = true;
        continue;
      }
      if (i == b || node_coord[way_nodes[i - 1]] == kInvalidCoord) is_vertex[n] = 1;
      if (i + 1 == e || node_coord[way_nodes[i + 1]] == kInvalidCoord) is_vertex[n] = 1;
      if (gap) ++missing_breaks;
      gap = false;
      seen_valid = true;
    }
    ways_with_missing += any_missing;
  }

  std::vector<SegTmp> segs;
  std::vector<Coord> points;
  std::vector<uint32_t> seg_first_point{0};
  uint64_t loops_split = 0;
  double total_m = 0;

  auto emit = [&](uint32_t way, std::span<const uint32_t> nodes) {
    const size_t first = points.size();
    double len = 0;
    for (size_t k = 0; k < nodes.size(); ++k) {
      const Coord c = node_coord[nodes[k]];
      if (points.size() > first) {
        if (points.back() == c) {
          // Drop the duplicate, unless it is the end point of a zero-length segment.
          const bool keep = k + 1 == nodes.size() && points.size() == first + 1;
          if (!keep) continue;
        } else {
          len += haversine_m(points.back(), c);
        }
      }
      points.push_back(c);
    }
    seg_first_point.push_back(static_cast<uint32_t>(points.size()));
    const uint32_t len_dm = static_cast<uint32_t>(std::llround(len * 10));
    segs.push_back({nodes.front(), nodes.back(), way, len_dm});
    total_m += len;
  };

  std::vector<uint32_t> cur;
  for (size_t w = 0; w < ways.size(); ++w) {
    const uint64_t b = way_first_ref[w], e = way_first_ref[w + 1];
    cur.clear();
    for (uint64_t i = b; i < e; ++i) {
      const uint32_t n = way_nodes[i];
      if (node_coord[n] == kInvalidCoord) {
        cur.clear();
        continue;
      }
      cur.push_back(n);
      if (cur.size() < 2 || !is_vertex[n]) {
        continue;
      }
      if (cur.front() == cur.back()) {
        // Closed loop without intersections: split at the middle shape point.
        const size_t mid = cur.size() / 2;
        emit(static_cast<uint32_t>(w), std::span(cur).first(mid + 1));
        emit(static_cast<uint32_t>(w), std::span(cur).subspan(mid));
        ++loops_split;
      } else {
        emit(static_cast<uint32_t>(w), cur);
      }
      cur.assign(1, n);
    }
  }
  std::vector<uint32_t>().swap(way_nodes);
  std::vector<uint8_t>().swap(is_vertex);
  timer.phase("segments");

  // --- vertices, arcs, CSR -------------------------------------------------
  std::vector<Coord> vertex_coord;
  const std::vector<uint32_t> node_to_vertex = number_vertices(segs, node_coord, vertex_coord);
  std::vector<Coord>().swap(node_coord);
  const uint32_t V = static_cast<uint32_t>(vertex_coord.size());
  const uint32_t S = static_cast<uint32_t>(segs.size());

  std::vector<SegAttr> seg_attr(S);
  std::vector<SegVertices> seg_vertices(S);
  std::vector<uint64_t> seg_osm_way(S);
  struct ArcTmp {
    uint32_t tail, head, weight, seg;
  };
  std::vector<ArcTmp> arcs;
  arcs.reserve(size_t(S) * 2);
  uint64_t oneway_segs = 0;
  for (uint32_t s = 0; s < S; ++s) {
    const SegTmp& t = segs[s];
    const WayInfo& w = ways[t.way];
    const uint32_t a = node_to_vertex[t.a_node], b = node_to_vertex[t.b_node];
    seg_attr[s] = {t.length_dm, w.name_id, w.ref_id, static_cast<uint8_t>(w.cls), w.maxspeed_kmh, w.flags};
    seg_vertices[s] = {a, b};
    seg_osm_way[s] = w.osm_id;
    const uint32_t weight = std::max<uint32_t>(1, static_cast<uint32_t>(std::llround(t.length_dm * 3.6 / speed_kmh(w))));
    if (w.dir != Dir::Bwd) arcs.push_back({a, b, weight, s});
    if (w.dir != Dir::Fwd) arcs.push_back({b, a, weight, s | kSegReversed});
    oneway_segs += w.dir != Dir::Both;
  }
  std::vector<SegTmp>().swap(segs);
  const uint32_t A = static_cast<uint32_t>(arcs.size());

  // Forward CSR: counting sort by tail (stable).
  std::vector<uint32_t> fwd_first_out(V + 1, 0);
  for (const ArcTmp& a : arcs) ++fwd_first_out[a.tail + 1];
  for (uint32_t v = 0; v < V; ++v) fwd_first_out[v + 1] += fwd_first_out[v];
  std::vector<FwdArc> fwd_arcs(A);
  std::vector<uint32_t> tail_of(A);
  {
    std::vector<uint32_t> pos(fwd_first_out.begin(), fwd_first_out.end() - 1);
    for (const ArcTmp& a : arcs) {
      const uint32_t i = pos[a.tail]++;
      fwd_arcs[i] = {a.head, a.weight, a.seg};
      tail_of[i] = a.tail;
    }
  }
  std::vector<ArcTmp>().swap(arcs);

  // Backward CSR: counting sort of forward arcs by head.
  std::vector<uint32_t> bwd_first_in(V + 1, 0);
  for (const FwdArc& a : fwd_arcs) ++bwd_first_in[a.head + 1];
  for (uint32_t v = 0; v < V; ++v) bwd_first_in[v + 1] += bwd_first_in[v];
  std::vector<BwdArc> bwd_arcs(A);
  {
    std::vector<uint32_t> pos(bwd_first_in.begin(), bwd_first_in.end() - 1);
    for (uint32_t i = 0; i < A; ++i) bwd_arcs[pos[fwd_arcs[i].head]++] = {tail_of[i], fwd_arcs[i].weight, i};
  }
  timer.phase("graph (CSR)");

  // --- names ---------------------------------------------------------------
  std::vector<uint32_t> str_offsets{0};
  std::vector<char> str_data;
  for (const std::string& s : names.strings()) {
    str_data.insert(str_data.end(), s.begin(), s.end());
    str_offsets.push_back(static_cast<uint32_t>(str_data.size()));
  }

  // --- write ---------------------------------------------------------------
  fs::create_directories(out_dir);
  const uint64_t build_id = make_build_id(input);
  const std::vector<BBox> bbox{extent};

  FileWriter graph(FileType::Graph, build_id);
  graph.add(SectionId::VertexCoord, vertex_coord);
  graph.add(SectionId::FwdFirstOut, fwd_first_out);
  graph.add(SectionId::FwdArcs, fwd_arcs);
  graph.add(SectionId::BwdFirstIn, bwd_first_in);
  graph.add(SectionId::BwdArcs, bwd_arcs);
  graph.add(SectionId::Bbox, bbox);

  FileWriter edges(FileType::Edges, build_id);
  edges.add(SectionId::SegAttr, seg_attr);
  edges.add(SectionId::SegVertices, seg_vertices);
  edges.add(SectionId::SegOsmWay, seg_osm_way);

  FileWriter geom(FileType::Geom, build_id);
  geom.add(SectionId::SegFirstPoint, seg_first_point);
  geom.add(SectionId::Points, points);

  FileWriter strs(FileType::Names, build_id);
  strs.add(SectionId::StrOffsets, str_offsets);
  strs.add(SectionId::StrData, str_data);

  const uint64_t sz_graph = graph.write(out_dir / file_name_for(FileType::Graph));
  const uint64_t sz_edges = edges.write(out_dir / file_name_for(FileType::Edges));
  const uint64_t sz_geom = geom.write(out_dir / file_name_for(FileType::Geom));
  const uint64_t sz_names = strs.write(out_dir / file_name_for(FileType::Names));
  timer.phase("write");

  // --- stats ---------------------------------------------------------------
  const std::optional<BBox> hb = source.header_bbox();
  auto kib = [](uint64_t b) { return b / 1024.0; };
  std::printf("\n");
  std::printf("ways read            %12llu\n", (unsigned long long)ways_read);
  std::printf("ways kept            %12zu\n", ways.size());
  std::printf("nodes read           %12llu\n", (unsigned long long)nodes_read);
  std::printf("road nodes           %12zu\n", num_nodes);
  std::printf("missing node refs    %12llu  (%llu ways, %llu breaks)\n", (unsigned long long)missing_refs,
              (unsigned long long)ways_with_missing, (unsigned long long)missing_breaks);
  std::printf("closed loops split   %12llu\n", (unsigned long long)loops_split);
  std::printf("vertices V           %12u\n", V);
  std::printf("segments S           %12u  (%llu oneway)\n", S, (unsigned long long)oneway_segs);
  std::printf("arcs A               %12u\n", A);
  std::printf("points P             %12zu\n", points.size());
  std::printf("names N              %12zu\n", names.strings().size());
  std::printf("road length          %12.1f km\n", total_m / 1000);
  std::printf("node extent          lat [%.7f, %.7f] lon [%.7f, %.7f]\n", extent.min.lat_e7 * 1e-7,
              extent.max.lat_e7 * 1e-7, extent.min.lon_e7 * 1e-7, extent.max.lon_e7 * 1e-7);
  if (hb)
    std::printf("header bbox          lat [%.7f, %.7f] lon [%.7f, %.7f]\n", hb->min.lat_e7 * 1e-7,
                hb->max.lat_e7 * 1e-7, hb->min.lon_e7 * 1e-7, hb->max.lon_e7 * 1e-7);
  std::printf("build_id             %016llx\n", (unsigned long long)build_id);
  std::printf("graph.dat            %12.1f KiB\n", kib(sz_graph));
  std::printf("edges.dat            %12.1f KiB\n", kib(sz_edges));
  std::printf("geom.dat             %12.1f KiB\n", kib(sz_geom));
  std::printf("names.dat            %12.1f KiB\n", kib(sz_names));
  std::printf("total time           %12.1f ms\n", timer.total_ms());
  std::printf("peak RSS             %12.1f MiB\n", peak_rss_mb());
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::fprintf(stderr, "usage: %s <input.osm.pbf> <out_dir>\n", argv[0]);
    return 2;
  }
  try {
    return run(argv[1], argv[2]);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "osm2nav: %s\n", e.what());
    return 1;
  }
}
