/// @file
/// `osm2nav <input.osm.pbf> <out_dir>`
///
/// Converts OSM car roads into graph.dat / edges.dat / geom.dat / names.dat.
/// Pass 1 reads ways, pass 2 reads node coordinates of road nodes only.
/// Then the drawing-only layers (water, forest, land use, buildings, rivers,
/// railways) are read in their own passes into map.dat (see osm_map_layers.h).

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
#include "osm_map_layers.h"

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
class Vrst_osm_way {
 public:
  /// Wraps the reader's buffers; nothing is copied.
  /// @param id OSM way id.
  /// @param refs Node ids in way order.
  /// @param tags All tags of the way.
  Vrst_osm_way(Vrst_uint64 id, std::span<const Vrst_uint64> refs, std::span<const Vrst_osm_tag> tags)
      : m_id(id), m_refs(refs), m_tags(tags) {}

  /// Looks up one tag by linear search; ways have few tags.
  /// @param key Tag key.
  /// @return Tag value, or "" when the way does not have the tag.
  std::string_view tag(std::string_view key) const {
    for (const Vrst_osm_tag& t : m_tags)
      if (t.key == key) return t.value;
    return {};
  }

  /// @copydoc Vrst_osm_way::m_id
  /// @return The way id.
  Vrst_uint64 id() const { return m_id; }
  /// @copydoc Vrst_osm_way::m_refs
  /// @return View into the reader's buffer, valid during the callback.
  std::span<const Vrst_uint64> refs() const { return m_refs; }
  /// @copydoc Vrst_osm_way::m_tags
  /// @return View into the reader's buffer, valid during the callback.
  std::span<const Vrst_osm_tag> tags() const { return m_tags; }

 private:
  /// OSM way id, kept per segment for turn restrictions and debugging.
  Vrst_uint64 m_id;
  /// Node ids in way order. Ids, not coordinates: PBF ways store only refs, and the
  /// shared ids are what reveals intersections.
  std::span<const Vrst_uint64> m_refs;
  /// All tags of the way; the profile decides which matter.
  std::span<const Vrst_osm_tag> m_tags;
};

/// One OSM node as delivered by Vrst_osm_source: id, position and the few tags the
/// converter uses. Other tags are left out: the car graph needs no node tags yet
/// (barriers and traffic signals may come later).
struct Vrst_osm_node {
  /// OSM node id, matched against the sorted road-node refs.
  Vrst_uint64 id;
  /// Position in 1e-7 degrees, already in file format.
  Vrst_coord coord;
  /// place=* value ("" if none); settlements become map labels (Vrst_place).
  std::string_view place;
  /// name=* value ("" if none); the label text of a place.
  std::string_view name;
  /// population=* value ("" if none); orders labels of the same kind.
  std::string_view population;
};

/// Abstract OSM input, read in passes: header bbox, then all ways, then all nodes.
/// Isolates the converter from the PBF reader, so libosmium can later be replaced by
/// an own protozero reader (or a test source) without touching the conversion.
class Vrst_osm_source {
 public:
  /// Virtual: sources are owned and used through this base class.
  virtual ~Vrst_osm_source() = default;
  /// Bounding box from the file header, if the file has one. Only printed in the stats.
  /// @return Box, or nullopt when the header has none.
  virtual std::optional<Vrst_bbox> header_bbox() = 0;
  /// Reads all ways in file order (pass 1).
  /// @param fn Called once per way; the way is valid only during the call.
  virtual void for_each_way(const std::function<void(const Vrst_osm_way&)>& fn) = 0;
  /// Reads all nodes with a valid location in file order (pass 2).
  /// @param fn Called once per node.
  virtual void for_each_node(const std::function<void(const Vrst_osm_node&)>& fn) = 0;
};

/// Vrst_osm_source implemented with libosmium.
/// Current reader. Each pass opens a new osmium::io::Reader filtered to one entity
/// type, so only ways or only nodes are decoded.
class Vrst_osmium_source final : public Vrst_osm_source {
 public:
  /// Remembers the input; nothing is read yet.
  /// @param path .osm.pbf file; it is opened again for every pass.
  explicit Vrst_osmium_source(std::string path) : m_path(std::move(path)) {}

  /// @copydoc Vrst_osm_source::header_bbox
  std::optional<Vrst_bbox> header_bbox() override {
    osmium::io::Reader reader{m_path, osmium::osm_entity_bits::nothing};
    const osmium::Box box = reader.header().box();
    reader.close();
    if (!box.valid()) return std::nullopt;
    return Vrst_bbox{{box.bottom_left().y(), box.bottom_left().x()}, {box.top_right().y(), box.top_right().x()}};
  }

  /// @copydoc Vrst_osm_source::for_each_way
  void for_each_way(const std::function<void(const Vrst_osm_way&)>& fn) override {
    osmium::io::Reader reader{m_path, osmium::osm_entity_bits::way};
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

  /// @copydoc Vrst_osm_source::for_each_node
  void for_each_node(const std::function<void(const Vrst_osm_node&)>& fn) override {
    osmium::io::Reader reader{m_path, osmium::osm_entity_bits::node};
    while (osmium::memory::Buffer buffer = reader.read()) {
      for (const osmium::Node& n : buffer.select<osmium::Node>()) {
        const osmium::Location loc = n.location();
        if (!loc.valid()) continue;
        const osmium::TagList& tags = n.tags();
        auto tag = [&](const char* key) {
          const char* v = tags.get_value_by_key(key);
          return v ? std::string_view(v) : std::string_view();
        };
        // The string views point into the buffer and are valid during fn only.
        fn(Vrst_osm_node{static_cast<Vrst_uint64>(n.id()), Vrst_coord{loc.y(), loc.x()}, tag("place"), tag("name"),
                         tag("population")});
      }
    }
    reader.close();
  }

 private:
  /// The PBF is opened once per pass: libosmium readers cannot rewind, and two
  /// passes (ways, then nodes) avoid holding all node coordinates in memory.
  std::string m_path;
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

/// Profile row for a highway=* value.
/// @param highway Tag value.
/// @return Row in kClasses, or nullptr when the value is empty or not a car road.
const Vrst_class_def* find_class(std::string_view highway) {
  if (highway.empty()) return nullptr;
  for (const Vrst_class_def& c : kClasses)
    if (c.highway == highway) return &c;
  return nullptr;
}

/// Parses an OSM maxspeed tag value: plain km/h, "mph" values, a few French zone
/// codes and "walk"; anything else counts as unknown.
/// @param v Tag value, e.g. "50", "30 mph", "FR:urban".
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

/// Whether an access-style tag grants access.
/// @param v Tag value.
/// @return true for "yes" and "designated".
bool is_yes(std::string_view v) { return v == "yes" || v == "designated"; }
/// Whether a flag tag such as tunnel=* or bridge=* is present.
/// @param v Tag value.
/// @return true for any value except "" and "no".
bool is_set(std::string_view v) { return !v.empty() && v != "no"; }

/// Applies the car profile to one way: road class, access, flags, speed and direction.
/// @param w Way from pass 1.
/// @return The way summary, or nullopt when the way is not a car road. name_id and
///         ref_id are left 0 for the caller to fill.
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
  info.osm_id = w.id();
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

/// Speed used for the arc weight.
/// @param w Way summary.
/// @return Tagged maxspeed, or the class default when it is unknown.
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
  /// Creates the pool with the empty string as id 0.
  Vrst_string_pool() { m_strings.emplace_back(); }
  /// Adds a string once.
  /// @param s String to add.
  /// @return Its id; equal strings get the same id and "" is always 0.
  Vrst_uint32 intern(std::string_view s) {
    if (s.empty()) return 0;
    auto [it, inserted] = m_ids.try_emplace(std::string(s), static_cast<Vrst_uint32>(m_strings.size()));
    if (inserted) m_strings.emplace_back(s);
    return it->second;
  }
  /// All strings, ready for names.dat.
  /// @return Strings in id order.
  const std::vector<std::string>& strings() const { return m_strings; }

 private:
  /// String -> id for deduplication. Thousands of segments share a few names; without
  /// it names.dat would hold one copy per way.
  std::unordered_map<std::string, Vrst_uint32> m_ids;
  /// Strings in id order (id 0 = ""), written as names.dat. Needed besides m_ids
  /// because the map has no order.
  std::vector<std::string> m_strings;
};

/// Place kind of a place=* value.
/// @param v Tag value.
/// @return Kind, or nullopt for values that get no label (suburb, locality, ...).
std::optional<PlaceKind> place_kind(std::string_view v) {
  if (v == "city") return PlaceKind::City;
  if (v == "town") return PlaceKind::Town;
  if (v == "village") return PlaceKind::Village;
  if (v == "hamlet") return PlaceKind::Hamlet;
  return std::nullopt;
}

/// Leading digits of a population=* value ("12 345" and "12,345" count as 12345).
/// @param v Tag value.
/// @return Population, 0 when the value has no digits.
Vrst_uint32 parse_population(std::string_view v) {
  Vrst_uint64 n = 0;
  for (char c : v) {
    if (c >= '0' && c <= '9') n = std::min<Vrst_uint64>(n * 10 + Vrst_uint64(c - '0'), 0xFFFF'FFFFu);
    else if (c != ' ' && c != ',' && c != '.') break;
  }
  return static_cast<Vrst_uint32>(n);
}

/// Coordinate of a road node not found in pass 2 (way refers to a node outside the
/// extract). Such nodes split the way, so no segment spans the missing point.
constexpr Vrst_coord kInvalidCoord{std::numeric_limits<Vrst_int32>::min(), std::numeric_limits<Vrst_int32>::min()};
/// "No index" marker, e.g. a node that is not a graph vertex.
constexpr Vrst_uint32 kNone = std::numeric_limits<Vrst_uint32>::max();

/// Great-circle distance on a sphere of mean Earth radius; within 0.5% of the
/// ellipsoid, enough for lengths and weights.
/// @param p First point.
/// @param q Second point.
/// @return Distance in metres.
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
  /// Starts an empty file description.
  /// @param type File to write; selects magic and file_type.
  /// @param build_id Import id, the same for all four writers of one run.
  Vrst_file_writer(FileType type, Vrst_uint64 build_id) : m_type(type), m_build_id(build_id) {}

  /// Queues a section; only the view is kept, nothing is copied.
  /// @tparam T Element type, written as raw bytes.
  /// @param id Section id.
  /// @param data Elements; must stay alive until write().
  template <class T>
  void add(SectionId id, std::span<const T> data) {
    m_sections.push_back({id, sizeof(T), data.size(), data.data()});
  }
  /// Queues a vector as a section; see the span overload.
  /// @tparam T Element type, written as raw bytes.
  /// @param id Section id.
  /// @param data Elements; the vector must stay alive and unchanged until write().
  template <class T>
  void add(SectionId id, const std::vector<T>& data) {
    add(id, std::span<const T>(data));
  }

  /// Writes header, section table and sections, padding each to kSectionAlign.
  /// @param path Output file, replaced if it exists.
  /// @return File size in bytes.
  /// @throws std::runtime_error if the file cannot be created or written.
  Vrst_uint64 write(const fs::path& path) const {
    Vrst_file_header h{};
    std::memcpy(h.magic, magic_for(m_type), 8);
    h.format_version = kFormatVersion;
    h.file_type = static_cast<Vrst_uint32>(m_type);
    h.build_id = m_build_id;
    h.section_count = static_cast<Vrst_uint32>(m_sections.size());

    auto align = [](Vrst_uint64 x) { return (x + kSectionAlign - 1) & ~(kSectionAlign - 1); };
    std::vector<Vrst_section_entry> table;
    Vrst_uint64 offset = sizeof(Vrst_file_header) + m_sections.size() * sizeof(Vrst_section_entry);
    for (const Vrst_section& s : m_sections) {
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
    for (Vrst_size i = 0; i < m_sections.size(); ++i) {
      out.write(kZeros, static_cast<std::streamsize>(table[i].offset - pos));
      const Vrst_uint64 bytes = m_sections[i].count * m_sections[i].elem_size;
      out.write(static_cast<const char*>(m_sections[i].data), static_cast<std::streamsize>(bytes));
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
  FileType m_type;
  /// Same value for all four writers of one run; see Vrst_file_header::build_id.
  Vrst_uint64 m_build_id;
  /// Sections in output order. Offsets are computed only in write(), so sections can
  /// be added in any order without manual layout.
  std::vector<Vrst_section> m_sections;
};

/// Prints the wall time of each converter phase and the total.
/// Shows which phase takes the time, the first thing to look at when a large
/// extract converts slowly.
class Vrst_phase_timer {
 public:
  /// Prints the time since the previous phase (or the start) and starts the next one.
  /// @param name Phase label.
  void phase(const char* name) {
    const auto now = std::chrono::steady_clock::now();
    std::printf("  %-22s %8.1f ms\n", name, std::chrono::duration<Vrst_float64, std::milli>(now - m_last).count());
    m_last = now;
  }
  /// Time since the timer was created.
  /// @return Milliseconds.
  Vrst_float64 total_ms() const {
    return std::chrono::duration<Vrst_float64, std::milli>(std::chrono::steady_clock::now() - m_start).count();
  }

 private:
  /// Program start, for the total time.
  std::chrono::steady_clock::time_point m_start = std::chrono::steady_clock::now();
  /// End of the previous phase: each phase prints only its own time.
  std::chrono::steady_clock::time_point m_last = m_start;
};

/// Peak resident memory of the process.
/// @return MiB; 0 on Windows.
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

/// Random 64-bit id for one import, written to all four files. Mixes random_device,
/// the input path and the clock, so two runs practically never share an id even where
/// random_device is weak.
/// @param input Input path.
/// @return Non-zero id.
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

/// The single place that decides vertex order.
/// @param segs All segments, ends given as node indices.
/// @param node_coord Coordinate of each node index.
/// @param[out] vertex_coord Receives the coordinate of each vertex, in vertex id order.
/// @return Node index -> vertex id, kNone for nodes that are not vertices.
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

/// Converts one extract: pass 1 ways, node table, pass 2 nodes, segments, CSR, names,
/// write, stats.
/// @param input .osm.pbf file.
/// @param out_dir Output directory, created if missing.
/// @return 0.
/// @throws std::runtime_error on a read or write error.
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
    if (w.refs().size() < 2) return;
    std::optional<Vrst_way_info> info = classify(w);
    if (!info) return;
    info->name_id = names.intern(w.tag("name"));
    info->ref_id = names.intern(w.tag("ref"));
    ways.push_back(*info);
    for (Vrst_uint64 r : w.refs())
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
  std::vector<Vrst_place> places;
  source.for_each_node([&](const Vrst_osm_node& n) {
    ++nodes_read;
    if (!n.place.empty() && !n.name.empty()) {
      if (const std::optional<PlaceKind> k = place_kind(n.place))
        places.push_back({n.coord, names.intern(n.name), parse_population(n.population), Vrst_uint8(*k), {}});
    }
    extent.min.lat_e7 = std::min(extent.min.lat_e7, n.coord.lat_e7);
    extent.min.lon_e7 = std::min(extent.min.lon_e7, n.coord.lon_e7);
    extent.max.lat_e7 = std::max(extent.max.lat_e7, n.coord.lat_e7);
    extent.max.lon_e7 = std::max(extent.max.lon_e7, n.coord.lon_e7);
    auto it = std::lower_bound(node_ids.begin(), node_ids.end(), n.id);
    if (it != node_ids.end() && *it == n.id) node_coord[it - node_ids.begin()] = n.coord;
  });
  std::vector<Vrst_uint64>().swap(node_ids);
  // Label priority order: kind, then population, then name id for a stable order.
  std::sort(places.begin(), places.end(), [](const Vrst_place& a, const Vrst_place& b) {
    if (a.kind != b.kind) return a.kind < b.kind;
    if (a.population != b.population) return a.population > b.population;
    return a.name_id < b.name_id;
  });
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
  strs.add(SectionId::Places, places);

  const Vrst_uint64 sz_graph = graph.write(out_dir / file_name_for(FileType::Graph));
  const Vrst_uint64 sz_edges = edges.write(out_dir / file_name_for(FileType::Edges));
  const Vrst_uint64 sz_geom = geom.write(out_dir / file_name_for(FileType::Geom));
  const Vrst_uint64 sz_names = strs.write(out_dir / file_name_for(FileType::Names));
  timer.phase("write");

  // --- map layers: map.dat --------------------------------------------------
  Vrst_map_layer_builder layers;
  layers.read(input);
  layers.finish();
  const Vrst_map_layer_data& ml = layers.data();
  Vrst_file_writer map(FileType::Map, build_id);
  map.add(SectionId::AreaFirstPoint, ml.area_first_point);
  map.add(SectionId::AreaFirstIndex, ml.area_first_index);
  map.add(SectionId::AreaKinds, ml.area_kinds);
  map.add(SectionId::AreaPoints, ml.area_points);
  map.add(SectionId::AreaIndices, ml.area_indices);
  map.add(SectionId::LineFirstPoint, ml.line_first_point);
  map.add(SectionId::LineAttr, ml.line_attr);
  map.add(SectionId::LinePoints, ml.line_points);
  map.add(SectionId::AreaOuterSize, ml.area_outer_size);
  const Vrst_uint64 sz_map = map.write(out_dir / file_name_for(FileType::Map));
  timer.phase("map layers");

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
  {
    Vrst_size per_kind[kPlaceKindCount] = {};
    for (const Vrst_place& pl : places) ++per_kind[pl.kind];
    std::printf("places L             %12zu  (%zu city, %zu town, %zu village, %zu hamlet)\n", places.size(),
                per_kind[0], per_kind[1], per_kind[2], per_kind[3]);
  }
  {
    Vrst_size per_kind[kAreaKindCount] = {};
    for (const Vrst_uint8 k : ml.area_kinds) ++per_kind[k];
    std::printf("areas R              %12zu  (%zu water, %zu forest, %zu buildings: %zu rect, %zu outline)\n",
                ml.area_kinds.size(), per_kind[int(AreaKind::Water)], per_kind[int(AreaKind::Forest)],
                per_kind[int(AreaKind::Building)], layers.rectangles(), layers.outlines());
    std::printf("area triangles       %12zu\n", ml.area_indices.size() / 3);
    std::printf("lines L              %12zu  (%zu points)\n", ml.line_attr.size(), ml.line_points.size());
  }
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
  std::printf("map.dat              %12.1f KiB\n", kib(sz_map));
  std::printf("total time           %12.1f ms\n", timer.total_ms());
  std::printf("peak RSS             %12.1f MiB\n", peak_rss_mb());
  return 0;
}

}  // namespace

/// osm2nav entry point.
/// @param argc Argument count.
/// @param argv `<input.osm.pbf> <out_dir>`.
/// @return 0 on success, 1 on error, 2 on wrong usage.
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
