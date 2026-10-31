/// @file
/// `navcheck <out_dir> [--dump-way <osm_way_id>]`
///
/// Validates the four nav files. Exit code 0 = all checks pass.

#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <numeric>
#include <string>
#include <vector>

#include "nav_files.h"

namespace fs = std::filesystem;
using namespace nav;

namespace {

/// Number of failed checks so far; non-zero gives exit code 1 and skips later
/// checks that would index through broken data.
int g_failures = 0;

/// Prints one PASS/FAIL line and counts failures in g_failures.
/// @param ok Check result.
/// @param name Check name, printed in the left column.
/// @param detail Numbers that explain the result.
void report(bool ok, const char* name, const std::string& detail) {
  std::printf("[%s] %-34s %s\n", ok ? "PASS" : "FAIL", name, detail.c_str());
  if (!ok) ++g_failures;
}

/// printf into a std::string, for report() details.
/// @param f printf format.
/// @param args Values for f.
/// @return Formatted text, cut at 511 characters.
std::string fmt(const char* f, auto... args) {
  char buf[512];
  std::snprintf(buf, sizeof buf, f, args...);
  return buf;
}

/// Fixed point to degrees.
/// @param e7 Degrees * 1e7.
/// @return Degrees.
Vrst_float64 deg(Vrst_int32 e7) { return e7 * 1e-7; }

/// Whether a coordinate is a possible WGS84 position.
/// @param c Coordinate.
/// @return true if latitude is in [-90, 90] and longitude in [-180, 180].
bool valid_coord(Vrst_coord c) {
  return c.lat_e7 >= -900'000'000 && c.lat_e7 <= 900'000'000 && c.lon_e7 >= -1'800'000'000 &&
         c.lon_e7 <= 1'800'000'000;
}

/// Whether a coordinate lies in a box.
/// @param c Coordinate.
/// @param b Box, edges included.
/// @return true if c is inside b.
bool in_bbox(Vrst_coord c, const Vrst_bbox& b) {
  return c.lat_e7 >= b.min.lat_e7 && c.lat_e7 <= b.max.lat_e7 && c.lon_e7 >= b.min.lon_e7 &&
         c.lon_e7 <= b.max.lon_e7;
}

/// Tail of a forward arc. The tail is implicit in the CSR, so it is found by binary
/// search (upper_bound) in fwd_first_out.
/// @param g Graph.
/// @param i Forward arc index, less than A.
/// @return Vertex whose out-arc run contains arc i.
Vrst_uint32 tail_of(const Vrst_graph& g, Vrst_uint32 i) {
  auto it = std::upper_bound(g.fwd_first_out().begin(), g.fwd_first_out().end(), i);
  return static_cast<Vrst_uint32>(it - g.fwd_first_out().begin()) - 1;
}

/// Checks a CSR offset (prefix) array: starts at 0, never decreases, ends at last.
/// @param off Offsets to check.
/// @param last Expected last value: the size of the array the offsets index.
/// @param[out] bad_at Index of the first bad entry; the last index when only the
///             end value is wrong or all is well.
/// @return true if off is a valid prefix array over last elements.
bool check_offsets(std::span<const Vrst_uint32> off, Vrst_uint64 last, Vrst_uint64* bad_at) {
  if (off.empty() || off.front() != 0) {
    *bad_at = 0;
    return false;
  }
  for (Vrst_size i = 1; i < off.size(); ++i)
    if (off[i] < off[i - 1]) {
      *bad_at = i;
      return false;
    }
  *bad_at = off.size() - 1;
  return off.back() == last;
}

/// Disjoint-set forest over vertices, with path halving and union by size.
/// Counts the connected components of the road graph and the largest one's share in
/// near-linear time. Many small components usually mean broken data or a too-strict
/// profile, which no per-file check can see.
class Vrst_union_find {
 public:
  /// Creates n singleton sets, one per vertex.
  /// @param n Number of vertices.
  explicit Vrst_union_find(Vrst_uint32 n) : m_parent(n), m_size(n, 1) {
    std::iota(m_parent.begin(), m_parent.end(), 0u);
  }
  /// Root of the set containing a vertex. Path halving keeps later finds short.
  /// @param x Vertex.
  /// @return Root vertex of its set.
  Vrst_uint32 find(Vrst_uint32 x) {
    while (m_parent[x] != x) x = m_parent[x] = m_parent[m_parent[x]];
    return x;
  }
  /// Merges the sets of two vertices, the smaller under the larger.
  /// @param a First vertex.
  /// @param b Second vertex.
  void unite(Vrst_uint32 a, Vrst_uint32 b) {
    a = find(a), b = find(b);
    if (a == b) return;
    if (m_size[a] < m_size[b]) std::swap(a, b);
    m_parent[b] = a;
    m_size[a] += m_size[b];
  }
  /// Size of a set.
  /// @param root Root vertex, as returned by find().
  /// @return Number of vertices in the set.
  Vrst_uint32 set_size(Vrst_uint32 root) const { return m_size[root]; }

 private:
  /// parent[v]: next vertex towards the set root (root: parent[v] == v).
  std::vector<Vrst_uint32> m_parent;
  /// size[root]: vertices in the set. Union by size keeps trees shallow, and the
  /// sizes give the largest-component share without a second pass.
  std::vector<Vrst_uint32> m_size;
};

/// Opens each file on its own so every failure is reported, then checks that the
/// files share one build_id and agree on the segment count.
/// @param dir Directory with the four nav files.
/// @param[out] d Receives the opened files.
/// @return true if all four files opened, share one build_id and agree on the
///         segment count.
bool check_files(const fs::path& dir, Vrst_nav_data& d) {
  bool all_ok = true;
  auto try_open = [&](const char* name, auto&& open) {
    try {
      open();
      report(true, name, "magic, version, sections, elem_size ok");
    } catch (const std::exception& e) {
      report(false, name, e.what());
      all_ok = false;
    }
  };
  try_open("graph.dat header", [&] { d.open_file(FileType::Graph, dir); });
  try_open("edges.dat header", [&] { d.open_file(FileType::Edges, dir); });
  try_open("geom.dat header", [&] { d.open_file(FileType::Geom, dir); });
  try_open("names.dat header", [&] { d.open_file(FileType::Names, dir); });
  if (!all_ok) return false;

  const Vrst_uint64 id = d.graph().file().build_id();
  const bool same = d.edges().file().build_id() == id && d.geom().file().build_id() == id && d.names().file().build_id() == id;
  report(same, "same build_id", fmt("%016" PRIx64, id));
  const bool seg_counts = d.geom().seg_first_point().size() == Vrst_size(d.edges().num_segments()) + 1;
  report(seg_counts, "segment count edges/geom",
         fmt("S=%u, SEG_FIRST_POINT=%zu", d.edges().num_segments(), d.geom().seg_first_point().size()));
  return same && seg_counts;
}

/// Checks that both CSR offset arrays are valid prefix arrays over A arcs.
/// @param g Graph.
void check_csr(const Vrst_graph& g) {
  const Vrst_uint32 A = g.num_arcs();
  Vrst_uint64 at = 0;
  bool ok = check_offsets(g.fwd_first_out(), A, &at);
  report(ok, "FWD_FIRST_OUT monotonic, last==A", ok ? fmt("V=%u A=%u", g.num_vertices(), A) : fmt("bad at %" PRIu64, at));
  ok = check_offsets(g.bwd_first_in(), A, &at);
  report(ok, "BWD_FIRST_IN monotonic, last==A", ok ? fmt("V=%u A=%u", g.num_vertices(), A) : fmt("bad at %" PRIu64, at));
}

/// Checks every arc: head and segment ids in range, tail/head equal the segment ends,
/// one arc per oneway segment and two otherwise, positive weights, and backward arcs
/// that mirror the forward arcs one to one. Indexes through the CSR, so main() runs
/// it only when the earlier checks passed.
/// @param d Opened files.
void check_arcs(const Vrst_nav_data& d) {
  const Vrst_graph& g = d.graph();
  const Vrst_uint32 V = g.num_vertices(), A = g.num_arcs(), S = d.edges().num_segments();

  Vrst_uint64 bad_head = 0, bad_seg = 0, bad_seg_ends = 0;
  std::vector<Vrst_uint8> arcs_per_seg(S, 0);
  for (Vrst_uint32 v = 0; v < V; ++v) {
    for (const Vrst_fwd_arc& a : g.out_arcs(v)) {
      if (a.head >= V) ++bad_head;
      const Vrst_uint32 s = a.seg & kSegIdMask;
      if (s >= S) {
        ++bad_seg;
        continue;
      }
      const Vrst_seg_vertices sv = d.edges().vertices()[s];
      const bool rev = a.seg & kSegReversed;
      if ((rev ? sv.b : sv.a) != v || (rev ? sv.a : sv.b) != a.head) ++bad_seg_ends;
      if (arcs_per_seg[s] < 255) ++arcs_per_seg[s];
    }
  }
  report(bad_head == 0, "Vrst_fwd_arc.head < V", fmt("%" PRIu64 " bad", bad_head));
  report(bad_seg == 0, "Vrst_fwd_arc.seg < S", fmt("%" PRIu64 " bad", bad_seg));
  report(bad_seg_ends == 0, "Vrst_fwd_arc tail/head == seg a/b", fmt("%" PRIu64 " bad", bad_seg_ends));

  Vrst_uint64 bad_oneway = 0;
  for (Vrst_uint32 s = 0; s < S; ++s) {
    const bool oneway = d.edges().attr()[s].flags & kSegOneway;
    if (arcs_per_seg[s] != (oneway ? 1 : 2)) ++bad_oneway;
  }
  report(bad_oneway == 0, "arcs per segment match oneway", fmt("%" PRIu64 " bad", bad_oneway));

  Vrst_uint64 bad_tail = 0, bad_ref = 0, bad_match = 0;
  std::vector<Vrst_uint8> seen(A, 0);
  for (Vrst_uint32 v = 0; v < V; ++v) {
    for (const Vrst_bwd_arc& b : g.in_arcs(v)) {
      if (b.tail >= V) ++bad_tail;
      if (b.fwd_arc >= A) {
        ++bad_ref;
        continue;
      }
      const Vrst_fwd_arc& f = g.fwd_arcs()[b.fwd_arc];
      if (f.head != v || f.weight != b.weight || tail_of(g, b.fwd_arc) != b.tail || seen[b.fwd_arc]++) ++bad_match;
    }
  }
  const Vrst_uint64 unseen = static_cast<Vrst_uint64>(std::count(seen.begin(), seen.end(), 0));
  report(bad_tail == 0, "Vrst_bwd_arc.tail < V", fmt("%" PRIu64 " bad", bad_tail));
  report(bad_ref == 0, "Vrst_bwd_arc.fwd_arc < A", fmt("%" PRIu64 " bad", bad_ref));
  report(bad_match == 0 && unseen == 0, "Vrst_bwd_arc <-> Vrst_fwd_arc bijection",
         fmt("%" PRIu64 " mismatched, %" PRIu64 " fwd arcs without bwd", bad_match, unseen));

  Vrst_uint64 zero_weight = 0;
  for (const Vrst_fwd_arc& a : g.fwd_arcs()) zero_weight += a.weight == 0;
  report(zero_weight == 0, "arc weight >= 1", fmt("%" PRIu64 " zero", zero_weight));
}

/// Checks that SEG_FIRST_POINT is a valid prefix array and that every segment has at
/// least 2 points that start and end on its vertices.
/// @param d Opened files.
void check_geometry(const Vrst_nav_data& d) {
  const Vrst_geom& geo = d.geom();
  const Vrst_uint32 S = d.edges().num_segments();
  Vrst_uint64 at = 0;
  const bool mono = check_offsets(geo.seg_first_point(), geo.points().size(), &at);
  report(mono, "SEG_FIRST_POINT monotonic, last==P",
         mono ? fmt("S=%u P=%zu", S, geo.points().size()) : fmt("bad at %" PRIu64, at));
  if (!mono) return;

  Vrst_uint64 short_segs = 0, bad_ends = 0;
  for (Vrst_uint32 s = 0; s < S; ++s) {
    auto pts = geo.segment_points(s);
    if (pts.size() < 2) {
      ++short_segs;
      continue;
    }
    const Vrst_seg_vertices sv = d.edges().vertices()[s];
    if (sv.a >= d.graph().num_vertices() || sv.b >= d.graph().num_vertices() ||
        pts.front() != d.graph().vertex_coord()[sv.a] || pts.back() != d.graph().vertex_coord()[sv.b])
      ++bad_ends;
  }
  report(short_segs == 0, "segments have >= 2 points", fmt("%" PRIu64 " bad", short_segs));
  report(bad_ends == 0, "segment ends == vertex coords", fmt("%" PRIu64 " bad", bad_ends));
}

/// Checks that all vertex and shape coordinates are valid WGS84 and inside the
/// stored bbox.
/// @param d Opened files.
void check_coords(const Vrst_nav_data& d) {
  const Vrst_bbox& bb = d.graph().bbox();
  Vrst_uint64 invalid = 0, outside = 0;
  auto test = [&](Vrst_coord c) {
    if (!valid_coord(c))
      ++invalid;
    else if (!in_bbox(c, bb))
      ++outside;
  };
  for (Vrst_coord c : d.graph().vertex_coord()) test(c);
  for (Vrst_coord c : d.geom().points()) test(c);
  report(invalid == 0, "no invalid coords", fmt("%" PRIu64 " invalid", invalid));
  report(outside == 0, "coords inside bbox",
         fmt("%" PRIu64 " outside lat [%.7f, %.7f] lon [%.7f, %.7f]", outside, deg(bb.min.lat_e7),
             deg(bb.max.lat_e7), deg(bb.min.lon_e7), deg(bb.max.lon_e7)));
}

/// Checks segment attributes and names: name/ref ids exist, road classes are valid,
/// string offsets never decrease and string 0 is empty. Places must have a name, a
/// valid kind, a position inside the bbox, and be sorted by kind.
/// @param d Opened files.
void check_attrs(const Vrst_nav_data& d) {
  const Vrst_uint32 N = d.names().size();
  Vrst_uint64 bad_name = 0, bad_class = 0;
  for (const Vrst_seg_attr& a : d.edges().attr()) {
    bad_name += (a.name_id >= N) + (a.ref_id >= N);
    bad_class += a.road_class >= kRoadClassCount;
  }
  report(bad_name == 0, "name_id/ref_id < N", fmt("N=%u, %" PRIu64 " bad", N, bad_name));
  report(bad_class == 0, "road_class valid", fmt("%" PRIu64 " bad", bad_class));
  bool offs_ok = d.names().offsets().front() == 0 && d.names().offsets()[1] == 0;
  for (Vrst_size i = 1; i < d.names().offsets().size(); ++i) offs_ok &= d.names().offsets()[i] >= d.names().offsets()[i - 1];
  report(offs_ok, "STR_OFFSETS monotonic, id 0 empty", fmt("data %zu bytes", d.names().data().size()));

  const Vrst_bbox bb = d.graph().bbox();
  Vrst_uint64 bad_place = 0;
  Vrst_uint8 prev_kind = 0;
  for (const Vrst_place& pl : d.names().places()) {
    const bool inside = pl.pos.lat_e7 >= bb.min.lat_e7 && pl.pos.lat_e7 <= bb.max.lat_e7 &&
                        pl.pos.lon_e7 >= bb.min.lon_e7 && pl.pos.lon_e7 <= bb.max.lon_e7;
    bad_place += pl.name_id == 0 || pl.name_id >= N || pl.kind >= kPlaceKindCount || pl.kind < prev_kind || !inside;
    prev_kind = pl.kind;
  }
  report(bad_place == 0, "places named, valid kind, sorted, inside bbox",
         fmt("%zu places, %" PRIu64 " bad", d.names().places().size(), bad_place));
}

/// Prints sizes, road length, dead ends and connected components. main() runs it
/// only when all checks passed.
/// @param d Opened files.
void stats(const Vrst_nav_data& d) {
  const Vrst_uint32 V = d.graph().num_vertices();
  std::vector<Vrst_uint32> degree(V, 0);
  Vrst_union_find uf(V);
  for (const Vrst_seg_vertices& sv : d.edges().vertices()) {
    ++degree[sv.a];
    ++degree[sv.b];
    uf.unite(sv.a, sv.b);
  }
  const auto dead_ends = std::count(degree.begin(), degree.end(), 1u);
  Vrst_uint32 components = 0, largest = 0;
  for (Vrst_uint32 v = 0; v < V; ++v)
    if (uf.find(v) == v) {
      ++components;
      largest = std::max(largest, uf.set_size(v));
    }
  Vrst_uint64 length_dm = 0;
  for (const Vrst_seg_attr& a : d.edges().attr()) length_dm += a.length_dm;
  std::printf("\nstats:\n");
  std::printf("  V=%u  S=%u  A=%u  P=%zu  N=%u\n", V, d.edges().num_segments(), d.graph().num_arcs(),
              d.geom().points().size(), d.names().size());
  std::printf("  road length          %.1f km\n", length_dm / 1e4);
  std::printf("  dead-end vertices    %td\n", dead_ends);
  std::printf("  components           %u\n", components);
  std::printf("  largest component    %u vertices (%.1f%%)\n", largest, V ? 100.0 * largest / V : 0.0);
}

/// Prints the attributes, segments and shape points made from one OSM way
/// (--dump-way), for comparison with openstreetmap.org.
/// @param d Opened files.
/// @param way_id OSM way id.
void dump_way(const Vrst_nav_data& d, Vrst_uint64 way_id) {
  static const char* kClassNames[] = {"motorway",      "motorway_link", "trunk",          "trunk_link",
                                      "primary",       "primary_link",  "secondary",      "secondary_link",
                                      "tertiary",      "tertiary_link", "unclassified",   "residential",
                                      "living_street", "service",       "ferry"};
  static_assert(std::size(kClassNames) == kRoadClassCount);
  bool found = false;
  for (Vrst_uint32 s = 0; s < d.edges().num_segments(); ++s) {
    if (d.edges().osm_way()[s] != way_id) continue;
    const Vrst_seg_attr& a = d.edges().attr()[s];
    const Vrst_seg_vertices sv = d.edges().vertices()[s];
    if (!found)
      std::printf("way %" PRIu64 "  name=\"%.*s\" ref=\"%.*s\" class=%s maxspeed=%u flags=0x%x\n", way_id,
                  int(d.names().get(a.name_id).size()), d.names().get(a.name_id).data(),
                  int(d.names().get(a.ref_id).size()), d.names().get(a.ref_id).data(), kClassNames[a.road_class],
                  a.maxspeed_kmh, a.flags);
    found = true;
    std::printf("segment %u  v%u -> v%u  %.1f m\n", s, sv.a, sv.b, a.length_dm / 10.0);
    for (Vrst_coord c : d.geom().segment_points(s)) std::printf("  %.7f %.7f\n", deg(c.lat_e7), deg(c.lon_e7));
  }
  if (!found) std::printf("way %" PRIu64 " not found\n", way_id);
}

}  // namespace

/// navcheck entry point.
/// @param argc Argument count.
/// @param argv `<out_dir> [--dump-way <osm_way_id>]`.
/// @return 0 when all checks pass (or the dump ran), 1 on a failed check or load
///         error, 2 on wrong usage.
int main(int argc, char** argv) {
  if (argc != 2 && !(argc == 4 && std::string(argv[2]) == "--dump-way")) {
    std::fprintf(stderr, "usage: %s <out_dir> [--dump-way <osm_way_id>]\n", argv[0]);
    return 2;
  }
  const fs::path dir = argv[1];
  Vrst_nav_data d;
  if (argc == 4) {
    try {
      d.open(dir);
    } catch (const std::exception& e) {
      std::fprintf(stderr, "navcheck: %s\n", e.what());
      return 1;
    }
    dump_way(d, std::strtoull(argv[3], nullptr, 10));
    return 0;
  }

  if (!check_files(dir, d)) {
    std::printf("\nFAILED (%d)\n", g_failures);
    return 1;
  }
  check_csr(d.graph());
  if (g_failures == 0) check_arcs(d);  // arc checks index through the CSR
  check_geometry(d);
  check_coords(d);
  check_attrs(d);
  if (g_failures == 0) stats(d);
  std::printf("\n%s\n", g_failures == 0 ? "ALL PASS" : ("FAILED (" + std::to_string(g_failures) + ")").c_str());
  return g_failures == 0 ? 0 : 1;
}
