/// @file
/// `osm2nav <input.osm.pbf> <out_dir>`
///
/// Converts OSM car roads into graph.dat / edges.dat / geom.dat / names.dat.
/// Pass 1 reads ways, pass 2 reads node coordinates of road nodes only.

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

/// One key=value tag of an OSM object, as seen by the converter.
/// Part of the source-neutral input model, so the profile code does not depend on
/// libosmium types.
struct Vrst_osm_tag {
  /// Views into the reader's buffer: valid only during the callback. No copy per tag,
  /// which matters at tens of millions of ways.
  std::string_view key;
  std::string_view value;  ///< Tag value; same lifetime as key.
};

/// One OSM way as delivered by Vrst_osm_source: id, node refs and tags.
/// Valid only during the callback. Gives classify() and the segment splitter one
/// input form whatever reader produced it.
struct Vrst_osm_way {
  /// OSM way id, kept per segment for turn restrictions and debugging.
  Vrst_uint64 id;
  /// Node ids in way order. Ids, not coordinates: PBF ways store only refs, and the
  /// shared ids are what reveals intersections.
  std::span<const Vrst_uint64> refs;
  /// All tags of the way; the profile decides which matter.
  std::span<const Vrst_osm_tag> tags;

  std::string_view tag(std::string_view key) const {
    for (const Vrst_osm_tag& t : tags)
      if (t.key == key) return t.value;
    return {};
  }
};

/// One OSM node as delivered by Vrst_osm_source: id and position only.
/// Tags are left out: the car graph needs no node tags yet (barriers and traffic
/// signals may come later).
struct Vrst_osm_node {
  /// OSM node id, matched against the sorted road-node refs.
  Vrst_uint64 id;
  /// Position in 1e-7 degrees, already in file format.
  Vrst_coord coord;
};

/// Abstract OSM input, read in passes: header bbox, then all ways, then all nodes.
/// Isolates the converter from the PBF reader, so libosmium can later be replaced by
/// an own protozero reader (or a test source) without touching the conversion.
class Vrst_osm_source {
 public:
  virtual ~Vrst_osm_source() = default;
  virtual std::optional<Vrst_bbox> header_bbox() = 0;
  virtual void for_each_way(const std::function<void(const Vrst_osm_way&)>& fn) = 0;
  virtual void for_each_node(const std::function<void(const Vrst_osm_node&)>& fn) = 0;
};

/// Vrst_osm_source implemented with libosmium.
/// Current reader. Each pass opens a new osmium::io::Reader filtered to one entity
/// type, so only ways or only nodes are decoded.
class Vrst_osmium_source final : public Vrst_osm_source {
 public:
  explicit Vrst_osmium_source(std::string path) : path_(std::move(path)) {}

  std::optional<Vrst_bbox> header_bbox() override {
    osmium::io::Reader reader{path_, osmium::osm_entity_bits::nothing};
    const osmium::Box box = reader.header().box();
    reader.close();
    if (!box.valid()) return std::nullopt;
    return Vrst_bbox{{box.bottom_left().y(), box.bottom_left().x()}, {box.top_right().y(), box.top_right().x()}};
  }

  void for_each_way(const std::function<void(const Vrst_osm_way&)>& fn) override {
    osmium::io::Reader reader{path_, osmium::osm_entity_bits::way};
    std::vector<Vrst_uint64> refs;
    std::vector<Vrst_osm_tag> tags;
    while (osmium::memory::Buffer buffer = reader.read()) {
      for (const osmium::Way& w : buffer.select<osmium::Way>()) {
        refs.clear();
        tags.clear();
        for (const osmium::NodeRef& nr : w.nodes()) refs.push_back(static_cast<Vrst_uint64>(nr.ref()));
        for (const osmium::Tag& t : w.tags()) tags.push_back({t.key(), t.value()});
        fn(Vrst_osm_way{static_cast<Vrst_uint64>(w.id()), refs, tags});
      }
    }
    reader.close();
  }

  void for_each_node(const std::function<void(const Vrst_osm_node&)>& fn) override {
    osmium::io::Reader reader{path_, osmium::osm_entity_bits::node};
    while (osmium::memory::Buffer buffer = reader.read()) {
      for (const osmium::Node& n : buffer.select<osmium::Node>()) {
        const osmium::Location loc = n.location();
        if (!loc.valid()) continue;
        fn(Vrst_osm_node{static_cast<Vrst_uint64>(n.id()), Vrst_coord{loc.y(), loc.x()}});
      }
    }
    reader.close();
  }

 private:
  /// The PBF is opened once per pass: libosmium readers cannot rewind, and two
  /// passes (ways, then nodes) avoid holding all node coordinates in memory.
  std::string path_;
};

// ===========================================================================
// Car profile
// ===========================================================================

/// One row of the car profile table kClasses: how a highway=* value maps to a
/// RoadClass and its default speed.
/// Keeps every profile decision in one table instead of if-chains, so speeds and
/// classes can be tuned in one place and the table is checked at compile time.
struct Vrst_class_def {
  /// highway=* value that selects this class; empty for classes chosen by other tags
  /// (ferry is route=ferry).
  std::string_view highway;
  /// Must equal the row index (checked by static_assert), so lookup by class is O(1).
  RoadClass cls;
  /// Default speed when maxspeed is missing or "none". Most OSM ways have no
  /// maxspeed, so without it most weights would be undefined.
  Vrst_uint8 speed_kmh;
  /// `*_link` ramp. Sets kSegLink; guidance says "take the exit" instead of naming the
  /// ramp as a road.
  bool link;
};

/// Car profile table, indexed by RoadClass (checked by static_assert).
constexpr Vrst_class_def kClasses[] = {
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

const Vrst_class_def* find_class(std::string_view highway) {
  if (highway.empty()) return nullptr;
  for (const Vrst_class_def& c : kClasses)
    if (c.highway == highway) return &c;
  return nullptr;
}

/// Parses an OSM maxspeed tag value.
/// @return km/h, 0 = unknown / no limit.
Vrst_uint8 parse_maxspeed(std::string_view v) {
  if (v.empty() || v == "none" || v == "signals" || v == "variable") return 0;
  if (v == "walk") return 6;
  if (v == "FR:urban") return 50;
  if (v == "FR:rural") return 80;
  if (v == "FR:motorway") return 130;
  int ivalue = 0;  // integer from_chars: Apple libc++ lacks the floating-point overload
  auto [p, ec] = std::from_chars(v.data(), v.data() + v.size(), ivalue);
  if (ec != std::errc() || ivalue <= 0) return 0;
  Vrst_float64 value = ivalue;
  std::string_view unit(p, v.data() + v.size() - p);
  while (!unit.empty() && unit.front() == ' ') unit.remove_prefix(1);
  if (unit.starts_with("mph"))
    value *= 1.609344;
  else if (!(unit.empty() || unit.front() == ';' || unit == "km/h" || unit == "kmh" || unit == "kph"))
    return 0;
  return static_cast<Vrst_uint8>(std::clamp(std::lround(value), 1L, 255L));
}

/// Which ways a car may drive along an OSM way, relative to its node order.
/// Both: two arcs per segment. Fwd: oneway along the node order (oneway=yes,
/// roundabouts, motorways). Bwd: oneway against it (oneway=-1). Resolved once per
/// way so the arc builder only has to read this, not the tags.
enum class Dir : Vrst_uint8 { Both, Fwd, Bwd };

/// Converter-side summary of one road way after pass 1: the result of classify().
/// All segments cut from the way share it by index, so tags are parsed once per way,
/// not once per segment, and only these few bytes stay in memory instead of all tags.
struct Vrst_way_info {
  /// Copied to SEG_OSM_WAY.
  Vrst_uint64 osm_id;
  /// Interned name; strings are stored once in Vrst_string_pool.
  Vrst_uint32 name_id;
  /// Interned ref (road number), same pool as name_id.
  Vrst_uint32 ref_id;
  /// Road class from the profile table.
  RoadClass cls;
  /// Allowed direction after resolving oneway, implied oneway (roundabout, motorway)
  /// and explicit oneway=no. Decides 1 or 2 arcs per segment, and which way.
  Dir dir;
  /// Parsed maxspeed in km/h, 0 = unknown.
  Vrst_uint8 maxspeed_kmh;
  /// SegFlags derived from tags.
  Vrst_uint16 flags;
};

bool is_yes(std::string_view v) { return v == "yes" || v == "designated"; }
bool is_set(std::string_view v) { return !v.empty() && v != "no"; }

/// Applies the car profile to one way.
/// @return the way summary, or nullopt when the way is not a car road.
std::optional<Vrst_way_info> classify(const Vrst_osm_way& w) {
  const Vrst_class_def* cd = find_class(w.tag("highway"));
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

  Vrst_way_info info{};
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

Vrst_uint32 speed_kmh(const Vrst_way_info& w) {
  return w.maxspeed_kmh ? w.maxspeed_kmh : kClasses[static_cast<int>(w.cls)].speed_kmh;
}

// ===========================================================================
// Helpers
// ===========================================================================

/// Interns strings to dense ids, id 0 = "".
/// Builds names.dat: thousands of segments share a few hundred names, so each name
/// is stored once and segments carry a 4-byte id.
class Vrst_string_pool {
 public:
  Vrst_string_pool() { strings_.emplace_back(); }
  Vrst_uint32 intern(std::string_view s) {
    if (s.empty()) return 0;
    auto [it, inserted] = ids_.try_emplace(std::string(s), static_cast<Vrst_uint32>(strings_.size()));
    if (inserted) strings_.emplace_back(s);
    return it->second;
  }
  const std::vector<std::string>& strings() const { return strings_; }

 private:
  /// String -> id for deduplication. Thousands of segments share a few names; without
  /// it names.dat would hold one copy per way.
  std::unordered_map<std::string, Vrst_uint32> ids_;
  /// Strings in id order (id 0 = ""), written as names.dat. Needed besides ids_
  /// because the map has no order.
  std::vector<std::string> strings_;
};

/// Coordinate of a road node not found in pass 2 (way refers to a node outside the
/// extract). Such nodes split the way, so no segment spans the missing point.
constexpr Vrst_coord kInvalidCoord{std::numeric_limits<Vrst_int32>::min(), std::numeric_limits<Vrst_int32>::min()};
/// "No index" marker, e.g. a node that is not a graph vertex.
constexpr Vrst_uint32 kNone = std::numeric_limits<Vrst_uint32>::max();

Vrst_float64 haversine_m(Vrst_coord p, Vrst_coord q) {
  constexpr Vrst_float64 kEarthRadiusM = 6371008.8;
  constexpr Vrst_float64 kDeg = 1e-7 * 3.14159265358979323846 / 180.0;
  const Vrst_float64 lat1 = p.lat_e7 * kDeg, lat2 = q.lat_e7 * kDeg;
  const Vrst_float64 dlat = lat2 - lat1, dlon = (q.lon_e7 - p.lon_e7) * kDeg;
  const Vrst_float64 h = std::sin(dlat / 2) * std::sin(dlat / 2) +
                   std::cos(lat1) * std::cos(lat2) * std::sin(dlon / 2) * std::sin(dlon / 2);
  return 2 * kEarthRadiusM * std::asin(std::min(1.0, std::sqrt(h)));
}

/// Writes one nav file: header, section table and 64-byte aligned sections.
/// The write-side counterpart of Vrst_nav_file. Computing offsets, alignment and
/// elem_size in one place guarantees every file passes the loader's checks.
class Vrst_file_writer {
 public:
  Vrst_file_writer(FileType type, Vrst_uint64 build_id) : type_(type), build_id_(build_id) {}

  template <class T>
  void add(SectionId id, std::span<const T> data) {
    sections_.push_back({id, sizeof(T), data.size(), data.data()});
  }
  template <class T>
  void add(SectionId id, const std::vector<T>& data) {
    add(id, std::span<const T>(data));
  }

  Vrst_uint64 write(const fs::path& path) const {
    Vrst_file_header h{};
    std::memcpy(h.magic, magic_for(type_), 8);
    h.format_version = kFormatVersion;
    h.file_type = static_cast<Vrst_uint32>(type_);
    h.build_id = build_id_;
    h.section_count = static_cast<Vrst_uint32>(sections_.size());

    auto align = [](Vrst_uint64 x) { return (x + kSectionAlign - 1) & ~(kSectionAlign - 1); };
    std::vector<Vrst_section_entry> table;
    Vrst_uint64 offset = sizeof(Vrst_file_header) + sections_.size() * sizeof(Vrst_section_entry);
    for (const Vrst_section& s : sections_) {
      offset = align(offset);
      table.push_back({static_cast<Vrst_uint32>(s.id), s.elem_size, offset, s.count});
      offset += s.count * s.elem_size;
    }

    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) throw std::runtime_error("cannot create " + path.string());
    out.write(reinterpret_cast<const char*>(&h), sizeof h);
    out.write(reinterpret_cast<const char*>(table.data()), table.size() * sizeof(Vrst_section_entry));
    Vrst_uint64 pos = sizeof(Vrst_file_header) + table.size() * sizeof(Vrst_section_entry);
    static constexpr char kZeros[kSectionAlign] = {};
    for (Vrst_size i = 0; i < sections_.size(); ++i) {
      out.write(kZeros, static_cast<std::streamsize>(table[i].offset - pos));
      const Vrst_uint64 bytes = sections_[i].count * sections_[i].elem_size;
      out.write(static_cast<const char*>(sections_[i].data), static_cast<std::streamsize>(bytes));
      pos = table[i].offset + bytes;
    }
    if (!out.flush()) throw std::runtime_error("write failed: " + path.string());
    return pos;
  }

 private:
  /// One section queued for writing: id, element size, count and a pointer to the data.
  /// Only a view: the caller keeps the data alive until write(). Lets `add<T>()` keep
  /// type information that write(), which only sees bytes, would otherwise lose.
  struct Vrst_section {
    /// Section id written to the table.
    SectionId id;
    /// sizeof(T) captured at add() time, where T is still known; write() only has bytes.
    Vrst_uint32 elem_size;
    /// Number of elements.
    Vrst_uint64 count;
    /// First element; count * elem_size bytes are written from here.
    const void* data;
  };
  /// Selects magic and file_type in the header.
  FileType type_;
  /// Same value for all four writers of one run; see Vrst_file_header::build_id.
  Vrst_uint64 build_id_;
  /// Sections in output order. Offsets are computed only in write(), so sections can
  /// be added in any order without manual layout.
  std::vector<Vrst_section> sections_;
};

/// Prints the wall time of each converter phase and the total.
/// Shows which phase takes the time, the first thing to look at when a large
/// extract converts slowly.
class Vrst_phase_timer {
 public:
  void phase(const char* name) {
    const auto now = std::chrono::steady_clock::now();
    std::printf("  %-22s %8.1f ms\n", name, std::chrono::duration<Vrst_float64, std::milli>(now - last_).count());
    last_ = now;
  }
  Vrst_float64 total_ms() const {
    return std::chrono::duration<Vrst_float64, std::milli>(std::chrono::steady_clock::now() - start_).count();
  }

 private:
  /// Program start, for the total time.
  std::chrono::steady_clock::time_point start_ = std::chrono::steady_clock::now();
  /// End of the previous phase: each phase prints only its own time.
  std::chrono::steady_clock::time_point last_ = start_;
};

Vrst_float64 peak_rss_mb() {
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

Vrst_uint64 make_build_id(const std::string& input) {
  std::random_device rd;
  Vrst_uint64 id = (Vrst_uint64(rd()) << 32) ^ rd();
  id ^= std::hash<std::string>{}(input);
  id ^= static_cast<Vrst_uint64>(std::chrono::system_clock::now().time_since_epoch().count()) * 0x9E3779B97F4A7C15ull;
  return id ? id : 1;
}

// ===========================================================================
// Segments
// ===========================================================================

/// Segment before vertex renumbering: one road piece between two graph vertices,
/// referring to nodes by dense node index.
/// Exists because vertex ids are only known after number_vertices(), which needs all
/// segments first; this is the record that is renumbered and turned into arcs.
struct Vrst_seg_tmp {
  /// Dense node index of the first point. Node, not vertex id: vertex ids exist only
  /// after number_vertices(), which needs all segments first.
  Vrst_uint32 a_node;
  /// Dense node index of the last point.
  Vrst_uint32 b_node;
  /// Index into ways: name, class, direction and speed are taken from there instead
  /// of being copied into every segment.
  Vrst_uint32 way;
  /// Measured while the points are emitted, when the coordinates are at hand.
  Vrst_uint32 length_dm;
};

/// The single place that decides vertex order. Fills vertex_coord.
/// @return node index -> vertex id (kNone for non-vertices).
/// @todo Sort used vertex nodes by Hilbert key of their coordinate.
std::vector<Vrst_uint32> number_vertices(const std::vector<Vrst_seg_tmp>& segs, const std::vector<Vrst_coord>& node_coord,
                                      std::vector<Vrst_coord>& vertex_coord) {
  std::vector<Vrst_uint32> node_to_vertex(node_coord.size(), kNone);
  for (const Vrst_seg_tmp& s : segs) node_to_vertex[s.a_node] = node_to_vertex[s.b_node] = 0;
  Vrst_uint32 next = 0;
  vertex_coord.clear();
  for (Vrst_size n = 0; n < node_to_vertex.size(); ++n) {
    if (node_to_vertex[n] == kNone) continue;
    node_to_vertex[n] = next++;
    vertex_coord.push_back(node_coord[n]);
  }
  return node_to_vertex;
}

int run(const std::string& input, const fs::path& out_dir) {
  Vrst_phase_timer timer;
  Vrst_osmium_source source(input);

  // --- pass 1: ways --------------------------------------------------------
  std::vector<Vrst_way_info> ways;
  std::vector<Vrst_uint64> way_first_ref{0};
  std::vector<Vrst_uint64> refs;
  Vrst_string_pool names;
  Vrst_uint64 ways_read = 0;

  source.for_each_way([&](const Vrst_osm_way& w) {
    ++ways_read;
    if (w.refs.size() < 2) return;
    std::optional<Vrst_way_info> info = classify(w);
    if (!info) return;
    info->name_id = names.intern(w.tag("name"));
    info->ref_id = names.intern(w.tag("ref"));
    ways.push_back(*info);
    for (Vrst_uint64 r : w.refs)
      if (refs.size() == way_first_ref.back() || refs.back() != r) refs.push_back(r);  // drop repeated refs
    way_first_ref.push_back(refs.size());
  });
  timer.phase("pass 1 (ways)");

  // --- node table ----------------------------------------------------------
  std::vector<Vrst_uint64> node_ids = refs;
  std::sort(node_ids.begin(), node_ids.end());
  node_ids.erase(std::unique(node_ids.begin(), node_ids.end()), node_ids.end());
  const Vrst_size num_nodes = node_ids.size();

  std::vector<Vrst_uint32> way_nodes(refs.size());
  for (Vrst_size i = 0; i < refs.size(); ++i)
    way_nodes[i] = static_cast<Vrst_uint32>(std::lower_bound(node_ids.begin(), node_ids.end(), refs[i]) - node_ids.begin());
  std::vector<Vrst_uint64>().swap(refs);

  std::vector<Vrst_uint8> uses(num_nodes, 0);
  std::vector<Vrst_uint8> is_vertex(num_nodes, 0);
  for (Vrst_size w = 0; w < ways.size(); ++w) {
    const Vrst_uint64 b = way_first_ref[w], e = way_first_ref[w + 1];
    for (Vrst_uint64 i = b; i < e; ++i)
      if (uses[way_nodes[i]] < 2) ++uses[way_nodes[i]];
    is_vertex[way_nodes[b]] = is_vertex[way_nodes[e - 1]] = 1;
  }
  for (Vrst_size n = 0; n < num_nodes; ++n)
    if (uses[n] >= 2) is_vertex[n] = 1;
  std::vector<Vrst_uint8>().swap(uses);
  timer.phase("node table");

  // --- pass 2: node coordinates --------------------------------------------
  std::vector<Vrst_coord> node_coord(num_nodes, kInvalidCoord);
  Vrst_bbox extent{{std::numeric_limits<Vrst_int32>::max(), std::numeric_limits<Vrst_int32>::max()},
              {std::numeric_limits<Vrst_int32>::min(), std::numeric_limits<Vrst_int32>::min()}};
  Vrst_uint64 nodes_read = 0;
  source.for_each_node([&](const Vrst_osm_node& n) {
    ++nodes_read;
    extent.min.lat_e7 = std::min(extent.min.lat_e7, n.coord.lat_e7);
    extent.min.lon_e7 = std::min(extent.min.lon_e7, n.coord.lon_e7);
    extent.max.lat_e7 = std::max(extent.max.lat_e7, n.coord.lat_e7);
    extent.max.lon_e7 = std::max(extent.max.lon_e7, n.coord.lon_e7);
    auto it = std::lower_bound(node_ids.begin(), node_ids.end(), n.id);
    if (it != node_ids.end() && *it == n.id) node_coord[it - node_ids.begin()] = n.coord;
  });
  std::vector<Vrst_uint64>().swap(node_ids);
  timer.phase("pass 2 (nodes)");

  // --- segments ------------------------------------------------------------
  // Ends of every run of known nodes become vertices: a missing node breaks the way.
  Vrst_uint64 missing_refs = 0, missing_breaks = 0, ways_with_missing = 0;
  for (Vrst_size w = 0; w < ways.size(); ++w) {
    const Vrst_uint64 b = way_first_ref[w], e = way_first_ref[w + 1];
    bool any_missing = false, seen_valid = false, gap = false;
    for (Vrst_uint64 i = b; i < e; ++i) {
      const Vrst_uint32 n = way_nodes[i];
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

  std::vector<Vrst_seg_tmp> segs;
  std::vector<Vrst_coord> points;
  std::vector<Vrst_uint32> seg_first_point{0};
  Vrst_uint64 loops_split = 0;
  Vrst_float64 total_m = 0;

  auto emit = [&](Vrst_uint32 way, std::span<const Vrst_uint32> nodes) {
    const Vrst_size first = points.size();
    Vrst_float64 len = 0;
    for (Vrst_size k = 0; k < nodes.size(); ++k) {
      const Vrst_coord c = node_coord[nodes[k]];
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
    seg_first_point.push_back(static_cast<Vrst_uint32>(points.size()));
    const Vrst_uint32 len_dm = static_cast<Vrst_uint32>(std::llround(len * 10));
    segs.push_back({nodes.front(), nodes.back(), way, len_dm});
    total_m += len;
  };

  std::vector<Vrst_uint32> cur;
  for (Vrst_size w = 0; w < ways.size(); ++w) {
    const Vrst_uint64 b = way_first_ref[w], e = way_first_ref[w + 1];
    cur.clear();
    for (Vrst_uint64 i = b; i < e; ++i) {
      const Vrst_uint32 n = way_nodes[i];
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
        const Vrst_size mid = cur.size() / 2;
        emit(static_cast<Vrst_uint32>(w), std::span(cur).first(mid + 1));
        emit(static_cast<Vrst_uint32>(w), std::span(cur).subspan(mid));
        ++loops_split;
      } else {
        emit(static_cast<Vrst_uint32>(w), cur);
      }
      cur.assign(1, n);
    }
  }
  std::vector<Vrst_uint32>().swap(way_nodes);
  std::vector<Vrst_uint8>().swap(is_vertex);
  timer.phase("segments");

  // --- vertices, arcs, CSR -------------------------------------------------
  std::vector<Vrst_coord> vertex_coord;
  const std::vector<Vrst_uint32> node_to_vertex = number_vertices(segs, node_coord, vertex_coord);
  std::vector<Vrst_coord>().swap(node_coord);
  const Vrst_uint32 V = static_cast<Vrst_uint32>(vertex_coord.size());
  const Vrst_uint32 S = static_cast<Vrst_uint32>(segs.size());

  std::vector<Vrst_seg_attr> seg_attr(S);
  std::vector<Vrst_seg_vertices> seg_vertices(S);
  std::vector<Vrst_uint64> seg_osm_way(S);
  /// Arc before CSR, with an explicit tail.
  /// Arcs are produced in segment order; the counting sort by tail then groups them
  /// per vertex and makes the tail implicit in Vrst_fwd_arc.
  struct Vrst_arc_tmp {
    Vrst_uint32 tail, head, weight, seg;
  };
  std::vector<Vrst_arc_tmp> arcs;
  arcs.reserve(Vrst_size(S) * 2);
  Vrst_uint64 oneway_segs = 0;
  for (Vrst_uint32 s = 0; s < S; ++s) {
    const Vrst_seg_tmp& t = segs[s];
    const Vrst_way_info& w = ways[t.way];
    const Vrst_uint32 a = node_to_vertex[t.a_node], b = node_to_vertex[t.b_node];
    seg_attr[s] = {t.length_dm, w.name_id, w.ref_id, static_cast<Vrst_uint8>(w.cls), w.maxspeed_kmh, w.flags};
    seg_vertices[s] = {a, b};
    seg_osm_way[s] = w.osm_id;
    const Vrst_uint32 weight = std::max<Vrst_uint32>(1, static_cast<Vrst_uint32>(std::llround(t.length_dm * 3.6 / speed_kmh(w))));
    if (w.dir != Dir::Bwd) arcs.push_back({a, b, weight, s});
    if (w.dir != Dir::Fwd) arcs.push_back({b, a, weight, s | kSegReversed});
    oneway_segs += w.dir != Dir::Both;
  }
  std::vector<Vrst_seg_tmp>().swap(segs);
  const Vrst_uint32 A = static_cast<Vrst_uint32>(arcs.size());

  // Forward CSR: counting sort by tail (stable).
  std::vector<Vrst_uint32> fwd_first_out(V + 1, 0);
  for (const Vrst_arc_tmp& a : arcs) ++fwd_first_out[a.tail + 1];
  for (Vrst_uint32 v = 0; v < V; ++v) fwd_first_out[v + 1] += fwd_first_out[v];
  std::vector<Vrst_fwd_arc> fwd_arcs(A);
  std::vector<Vrst_uint32> tail_of(A);
  {
    std::vector<Vrst_uint32> pos(fwd_first_out.begin(), fwd_first_out.end() - 1);
    for (const Vrst_arc_tmp& a : arcs) {
      const Vrst_uint32 i = pos[a.tail]++;
      fwd_arcs[i] = {a.head, a.weight, a.seg};
      tail_of[i] = a.tail;
    }
  }
  std::vector<Vrst_arc_tmp>().swap(arcs);

  // Backward CSR: counting sort of forward arcs by head.
  std::vector<Vrst_uint32> bwd_first_in(V + 1, 0);
  for (const Vrst_fwd_arc& a : fwd_arcs) ++bwd_first_in[a.head + 1];
  for (Vrst_uint32 v = 0; v < V; ++v) bwd_first_in[v + 1] += bwd_first_in[v];
  std::vector<Vrst_bwd_arc> bwd_arcs(A);
  {
    std::vector<Vrst_uint32> pos(bwd_first_in.begin(), bwd_first_in.end() - 1);
    for (Vrst_uint32 i = 0; i < A; ++i) bwd_arcs[pos[fwd_arcs[i].head]++] = {tail_of[i], fwd_arcs[i].weight, i};
  }
  timer.phase("graph (CSR)");

  // --- names ---------------------------------------------------------------
  std::vector<Vrst_uint32> str_offsets{0};
  std::vector<char> str_data;
  for (const std::string& s : names.strings()) {
    str_data.insert(str_data.end(), s.begin(), s.end());
    str_offsets.push_back(static_cast<Vrst_uint32>(str_data.size()));
  }

  // --- write ---------------------------------------------------------------
  fs::create_directories(out_dir);
  const Vrst_uint64 build_id = make_build_id(input);
  const std::vector<Vrst_bbox> bbox{extent};

  Vrst_file_writer graph(FileType::Graph, build_id);
  graph.add(SectionId::VertexCoord, vertex_coord);
  graph.add(SectionId::FwdFirstOut, fwd_first_out);
  graph.add(SectionId::FwdArcs, fwd_arcs);
  graph.add(SectionId::BwdFirstIn, bwd_first_in);
  graph.add(SectionId::BwdArcs, bwd_arcs);
  graph.add(SectionId::Bbox, bbox);

  Vrst_file_writer edges(FileType::Edges, build_id);
  edges.add(SectionId::SegAttr, seg_attr);
  edges.add(SectionId::SegVertices, seg_vertices);
  edges.add(SectionId::SegOsmWay, seg_osm_way);

  Vrst_file_writer geom(FileType::Geom, build_id);
  geom.add(SectionId::SegFirstPoint, seg_first_point);
  geom.add(SectionId::Points, points);

  Vrst_file_writer strs(FileType::Names, build_id);
  strs.add(SectionId::StrOffsets, str_offsets);
  strs.add(SectionId::StrData, str_data);

  const Vrst_uint64 sz_graph = graph.write(out_dir / file_name_for(FileType::Graph));
  const Vrst_uint64 sz_edges = edges.write(out_dir / file_name_for(FileType::Edges));
  const Vrst_uint64 sz_geom = geom.write(out_dir / file_name_for(FileType::Geom));
  const Vrst_uint64 sz_names = strs.write(out_dir / file_name_for(FileType::Names));
  timer.phase("write");

  // --- stats ---------------------------------------------------------------
  const std::optional<Vrst_bbox> hb = source.header_bbox();
  auto kib = [](Vrst_uint64 b) { return b / 1024.0; };
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
