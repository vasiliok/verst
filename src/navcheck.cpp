// navcheck <out_dir> [--dump-way <osm_way_id>]
//
// Validates the four nav files. Exit code 0 = all checks pass.

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

int g_failures = 0;

void report(bool ok, const char* name, const std::string& detail) {
  std::printf("[%s] %-34s %s\n", ok ? "PASS" : "FAIL", name, detail.c_str());
  if (!ok) ++g_failures;
}

std::string fmt(const char* f, auto... args) {
  char buf[512];
  std::snprintf(buf, sizeof buf, f, args...);
  return buf;
}

double deg(int32_t e7) { return e7 * 1e-7; }

bool valid_coord(Coord c) {
  return c.lat_e7 >= -900'000'000 && c.lat_e7 <= 900'000'000 && c.lon_e7 >= -1'800'000'000 &&
         c.lon_e7 <= 1'800'000'000;
}

bool in_bbox(Coord c, const BBox& b) {
  return c.lat_e7 >= b.min.lat_e7 && c.lat_e7 <= b.max.lat_e7 && c.lon_e7 >= b.min.lon_e7 &&
         c.lon_e7 <= b.max.lon_e7;
}

// Tail of forward arc i (upper_bound in first_out).
uint32_t tail_of(const Graph& g, uint32_t i) {
  auto it = std::upper_bound(g.fwd_first_out.begin(), g.fwd_first_out.end(), i);
  return static_cast<uint32_t>(it - g.fwd_first_out.begin()) - 1;
}

bool check_offsets(std::span<const uint32_t> off, uint64_t last, uint64_t* bad_at) {
  if (off.empty() || off.front() != 0) {
    *bad_at = 0;
    return false;
  }
  for (size_t i = 1; i < off.size(); ++i)
    if (off[i] < off[i - 1]) {
      *bad_at = i;
      return false;
    }
  *bad_at = off.size() - 1;
  return off.back() == last;
}

struct UnionFind {
  std::vector<uint32_t> parent, size;
  explicit UnionFind(uint32_t n) : parent(n), size(n, 1) { std::iota(parent.begin(), parent.end(), 0u); }
  uint32_t find(uint32_t x) {
    while (parent[x] != x) x = parent[x] = parent[parent[x]];
    return x;
  }
  void unite(uint32_t a, uint32_t b) {
    a = find(a), b = find(b);
    if (a == b) return;
    if (size[a] < size[b]) std::swap(a, b);
    parent[b] = a;
    size[a] += size[b];
  }
};

// Check 1 opens each file on its own so every failure is reported.
bool check_files(const fs::path& dir, NavData& d) {
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
  try_open("graph.dat header", [&] { d.graph.open((dir / "graph.dat").string()); });
  try_open("edges.dat header", [&] { d.edges.open((dir / "edges.dat").string()); });
  try_open("geom.dat header", [&] { d.geom.open((dir / "geom.dat").string()); });
  try_open("names.dat header", [&] { d.names.open((dir / "names.dat").string()); });
  if (!all_ok) return false;

  const uint64_t id = d.graph.file.build_id();
  const bool same = d.edges.file.build_id() == id && d.geom.file.build_id() == id && d.names.file.build_id() == id;
  report(same, "same build_id", fmt("%016" PRIx64, id));
  const bool seg_counts = d.geom.seg_first_point.size() == size_t(d.edges.num_segments()) + 1;
  report(seg_counts, "segment count edges/geom",
         fmt("S=%u, SEG_FIRST_POINT=%zu", d.edges.num_segments(), d.geom.seg_first_point.size()));
  return same && seg_counts;
}

void check_csr(const Graph& g) {
  const uint32_t A = g.num_arcs();
  uint64_t at = 0;
  bool ok = check_offsets(g.fwd_first_out, A, &at);
  report(ok, "FWD_FIRST_OUT monotonic, last==A", ok ? fmt("V=%u A=%u", g.num_vertices(), A) : fmt("bad at %" PRIu64, at));
  ok = check_offsets(g.bwd_first_in, A, &at);
  report(ok, "BWD_FIRST_IN monotonic, last==A", ok ? fmt("V=%u A=%u", g.num_vertices(), A) : fmt("bad at %" PRIu64, at));
}

void check_arcs(const NavData& d) {
  const Graph& g = d.graph;
  const uint32_t V = g.num_vertices(), A = g.num_arcs(), S = d.edges.num_segments();

  uint64_t bad_head = 0, bad_seg = 0, bad_seg_ends = 0;
  std::vector<uint8_t> arcs_per_seg(S, 0);
  for (uint32_t v = 0; v < V; ++v) {
    for (const FwdArc& a : g.out_arcs(v)) {
      if (a.head >= V) ++bad_head;
      const uint32_t s = a.seg & kSegIdMask;
      if (s >= S) {
        ++bad_seg;
        continue;
      }
      const SegVertices sv = d.edges.vertices[s];
      const bool rev = a.seg & kSegReversed;
      if ((rev ? sv.b : sv.a) != v || (rev ? sv.a : sv.b) != a.head) ++bad_seg_ends;
      if (arcs_per_seg[s] < 255) ++arcs_per_seg[s];
    }
  }
  report(bad_head == 0, "FwdArc.head < V", fmt("%" PRIu64 " bad", bad_head));
  report(bad_seg == 0, "FwdArc.seg < S", fmt("%" PRIu64 " bad", bad_seg));
  report(bad_seg_ends == 0, "FwdArc tail/head == seg a/b", fmt("%" PRIu64 " bad", bad_seg_ends));

  uint64_t bad_oneway = 0;
  for (uint32_t s = 0; s < S; ++s) {
    const bool oneway = d.edges.attr[s].flags & kSegOneway;
    if (arcs_per_seg[s] != (oneway ? 1 : 2)) ++bad_oneway;
  }
  report(bad_oneway == 0, "arcs per segment match oneway", fmt("%" PRIu64 " bad", bad_oneway));

  uint64_t bad_tail = 0, bad_ref = 0, bad_match = 0;
  std::vector<uint8_t> seen(A, 0);
  for (uint32_t v = 0; v < V; ++v) {
    for (const BwdArc& b : g.in_arcs(v)) {
      if (b.tail >= V) ++bad_tail;
      if (b.fwd_arc >= A) {
        ++bad_ref;
        continue;
      }
      const FwdArc& f = g.fwd_arcs[b.fwd_arc];
      if (f.head != v || f.weight != b.weight || tail_of(g, b.fwd_arc) != b.tail || seen[b.fwd_arc]++) ++bad_match;
    }
  }
  const uint64_t unseen = static_cast<uint64_t>(std::count(seen.begin(), seen.end(), 0));
  report(bad_tail == 0, "BwdArc.tail < V", fmt("%" PRIu64 " bad", bad_tail));
  report(bad_ref == 0, "BwdArc.fwd_arc < A", fmt("%" PRIu64 " bad", bad_ref));
  report(bad_match == 0 && unseen == 0, "BwdArc <-> FwdArc bijection",
         fmt("%" PRIu64 " mismatched, %" PRIu64 " fwd arcs without bwd", bad_match, unseen));

  uint64_t zero_weight = 0;
  for (const FwdArc& a : g.fwd_arcs) zero_weight += a.weight == 0;
  report(zero_weight == 0, "arc weight >= 1", fmt("%" PRIu64 " zero", zero_weight));
}

void check_geometry(const NavData& d) {
  const Geom& geo = d.geom;
  const uint32_t S = d.edges.num_segments();
  uint64_t at = 0;
  const bool mono = check_offsets(geo.seg_first_point, geo.points.size(), &at);
  report(mono, "SEG_FIRST_POINT monotonic, last==P",
         mono ? fmt("S=%u P=%zu", S, geo.points.size()) : fmt("bad at %" PRIu64, at));
  if (!mono) return;

  uint64_t short_segs = 0, bad_ends = 0;
  for (uint32_t s = 0; s < S; ++s) {
    auto pts = geo.segment_points(s);
    if (pts.size() < 2) {
      ++short_segs;
      continue;
    }
    const SegVertices sv = d.edges.vertices[s];
    if (sv.a >= d.graph.num_vertices() || sv.b >= d.graph.num_vertices() ||
        pts.front() != d.graph.vertex_coord[sv.a] || pts.back() != d.graph.vertex_coord[sv.b])
      ++bad_ends;
  }
  report(short_segs == 0, "segments have >= 2 points", fmt("%" PRIu64 " bad", short_segs));
  report(bad_ends == 0, "segment ends == vertex coords", fmt("%" PRIu64 " bad", bad_ends));
}

void check_coords(const NavData& d) {
  const BBox& bb = d.graph.bbox;
  uint64_t invalid = 0, outside = 0;
  auto test = [&](Coord c) {
    if (!valid_coord(c))
      ++invalid;
    else if (!in_bbox(c, bb))
      ++outside;
  };
  for (Coord c : d.graph.vertex_coord) test(c);
  for (Coord c : d.geom.points) test(c);
  report(invalid == 0, "no invalid coords", fmt("%" PRIu64 " invalid", invalid));
  report(outside == 0, "coords inside bbox",
         fmt("%" PRIu64 " outside lat [%.7f, %.7f] lon [%.7f, %.7f]", outside, deg(bb.min.lat_e7),
             deg(bb.max.lat_e7), deg(bb.min.lon_e7), deg(bb.max.lon_e7)));
}

void check_attrs(const NavData& d) {
  const uint32_t N = d.names.size();
  uint64_t bad_name = 0, bad_class = 0;
  for (const SegAttr& a : d.edges.attr) {
    bad_name += (a.name_id >= N) + (a.ref_id >= N);
    bad_class += a.road_class >= kRoadClassCount;
  }
  report(bad_name == 0, "name_id/ref_id < N", fmt("N=%u, %" PRIu64 " bad", N, bad_name));
  report(bad_class == 0, "road_class valid", fmt("%" PRIu64 " bad", bad_class));
  bool offs_ok = d.names.offsets.front() == 0 && d.names.offsets[1] == 0;
  for (size_t i = 1; i < d.names.offsets.size(); ++i) offs_ok &= d.names.offsets[i] >= d.names.offsets[i - 1];
  report(offs_ok, "STR_OFFSETS monotonic, id 0 empty", fmt("data %zu bytes", d.names.data.size()));
}

void stats(const NavData& d) {
  const uint32_t V = d.graph.num_vertices();
  std::vector<uint32_t> degree(V, 0);
  UnionFind uf(V);
  for (const SegVertices& sv : d.edges.vertices) {
    ++degree[sv.a];
    ++degree[sv.b];
    uf.unite(sv.a, sv.b);
  }
  const auto dead_ends = std::count(degree.begin(), degree.end(), 1u);
  uint32_t components = 0, largest = 0;
  for (uint32_t v = 0; v < V; ++v)
    if (uf.find(v) == v) {
      ++components;
      largest = std::max(largest, uf.size[v]);
    }
  uint64_t length_dm = 0;
  for (const SegAttr& a : d.edges.attr) length_dm += a.length_dm;
  std::printf("\nstats:\n");
  std::printf("  V=%u  S=%u  A=%u  P=%zu  N=%u\n", V, d.edges.num_segments(), d.graph.num_arcs(),
              d.geom.points.size(), d.names.size());
  std::printf("  road length          %.1f km\n", length_dm / 1e4);
  std::printf("  dead-end vertices    %td\n", dead_ends);
  std::printf("  components           %u\n", components);
  std::printf("  largest component    %u vertices (%.1f%%)\n", largest, V ? 100.0 * largest / V : 0.0);
}

void dump_way(const NavData& d, uint64_t way_id) {
  static const char* kClassNames[] = {"motorway",      "motorway_link", "trunk",          "trunk_link",
                                      "primary",       "primary_link",  "secondary",      "secondary_link",
                                      "tertiary",      "tertiary_link", "unclassified",   "residential",
                                      "living_street", "service",       "ferry"};
  static_assert(std::size(kClassNames) == kRoadClassCount);
  bool found = false;
  for (uint32_t s = 0; s < d.edges.num_segments(); ++s) {
    if (d.edges.osm_way[s] != way_id) continue;
    const SegAttr& a = d.edges.attr[s];
    const SegVertices sv = d.edges.vertices[s];
    if (!found)
      std::printf("way %" PRIu64 "  name=\"%.*s\" ref=\"%.*s\" class=%s maxspeed=%u flags=0x%x\n", way_id,
                  int(d.names.get(a.name_id).size()), d.names.get(a.name_id).data(),
                  int(d.names.get(a.ref_id).size()), d.names.get(a.ref_id).data(), kClassNames[a.road_class],
                  a.maxspeed_kmh, a.flags);
    found = true;
    std::printf("segment %u  v%u -> v%u  %.1f m\n", s, sv.a, sv.b, a.length_dm / 10.0);
    for (Coord c : d.geom.segment_points(s)) std::printf("  %.7f %.7f\n", deg(c.lat_e7), deg(c.lon_e7));
  }
  if (!found) std::printf("way %" PRIu64 " not found\n", way_id);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2 && !(argc == 4 && std::string(argv[2]) == "--dump-way")) {
    std::fprintf(stderr, "usage: %s <out_dir> [--dump-way <osm_way_id>]\n", argv[0]);
    return 2;
  }
  const fs::path dir = argv[1];
  NavData d;
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
  check_csr(d.graph);
  if (g_failures == 0) check_arcs(d);  // arc checks index through the CSR
  check_geometry(d);
  check_coords(d);
  check_attrs(d);
  if (g_failures == 0) stats(d);
  std::printf("\n%s\n", g_failures == 0 ? "ALL PASS" : ("FAILED (" + std::to_string(g_failures) + ")").c_str());
  return g_failures == 0 ? 0 : 1;
}
