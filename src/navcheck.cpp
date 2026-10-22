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

Vrst_float64 deg(Vrst_int32 e7) { return e7 * 1e-7; }

bool valid_coord(Vrst_coord c) {
  return c.lat_e7 >= -900'000'000 && c.lat_e7 <= 900'000'000 && c.lon_e7 >= -1'800'000'000 &&
         c.lon_e7 <= 1'800'000'000;
}

bool in_bbox(Vrst_coord c, const Vrst_bbox& b) {
  return c.lat_e7 >= b.min.lat_e7 && c.lat_e7 <= b.max.lat_e7 && c.lon_e7 >= b.min.lon_e7 &&
         c.lon_e7 <= b.max.lon_e7;
}

// Tail of forward arc i (upper_bound in first_out).
Vrst_uint32 tail_of(const Vrst_graph& g, Vrst_uint32 i) {
  auto it = std::upper_bound(g.fwd_first_out.begin(), g.fwd_first_out.end(), i);
  return static_cast<Vrst_uint32>(it - g.fwd_first_out.begin()) - 1;
}

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

struct Vrst_union_find {
  std::vector<Vrst_uint32> parent, size;
  explicit Vrst_union_find(Vrst_uint32 n) : parent(n), size(n, 1) { std::iota(parent.begin(), parent.end(), 0u); }
  Vrst_uint32 find(Vrst_uint32 x) {
    while (parent[x] != x) x = parent[x] = parent[parent[x]];
    return x;
  }
  void unite(Vrst_uint32 a, Vrst_uint32 b) {
    a = find(a), b = find(b);
    if (a == b) return;
    if (size[a] < size[b]) std::swap(a, b);
    parent[b] = a;
    size[a] += size[b];
  }
};

// Check 1 opens each file on its own so every failure is reported.
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
  try_open("graph.dat header", [&] { d.graph.open((dir / "graph.dat").string()); });
  try_open("edges.dat header", [&] { d.edges.open((dir / "edges.dat").string()); });
  try_open("geom.dat header", [&] { d.geom.open((dir / "geom.dat").string()); });
  try_open("names.dat header", [&] { d.names.open((dir / "names.dat").string()); });
  if (!all_ok) return false;

  const Vrst_uint64 id = d.graph.file.build_id();
  const bool same = d.edges.file.build_id() == id && d.geom.file.build_id() == id && d.names.file.build_id() == id;
  report(same, "same build_id", fmt("%016" PRIx64, id));
  const bool seg_counts = d.geom.seg_first_point.size() == Vrst_size(d.edges.num_segments()) + 1;
  report(seg_counts, "segment count edges/geom",
         fmt("S=%u, SEG_FIRST_POINT=%zu", d.edges.num_segments(), d.geom.seg_first_point.size()));
  return same && seg_counts;
}

void check_csr(const Vrst_graph& g) {
  const Vrst_uint32 A = g.num_arcs();
  Vrst_uint64 at = 0;
  bool ok = check_offsets(g.fwd_first_out, A, &at);
  report(ok, "FWD_FIRST_OUT monotonic, last==A", ok ? fmt("V=%u A=%u", g.num_vertices(), A) : fmt("bad at %" PRIu64, at));
  ok = check_offsets(g.bwd_first_in, A, &at);
  report(ok, "BWD_FIRST_IN monotonic, last==A", ok ? fmt("V=%u A=%u", g.num_vertices(), A) : fmt("bad at %" PRIu64, at));
}

void check_arcs(const Vrst_nav_data& d) {
  const Vrst_graph& g = d.graph;
  const Vrst_uint32 V = g.num_vertices(), A = g.num_arcs(), S = d.edges.num_segments();

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
      const Vrst_seg_vertices sv = d.edges.vertices[s];
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
    const bool oneway = d.edges.attr[s].flags & kSegOneway;
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
      const Vrst_fwd_arc& f = g.fwd_arcs[b.fwd_arc];
      if (f.head != v || f.weight != b.weight || tail_of(g, b.fwd_arc) != b.tail || seen[b.fwd_arc]++) ++bad_match;
    }
  }
  const Vrst_uint64 unseen = static_cast<Vrst_uint64>(std::count(seen.begin(), seen.end(), 0));
  report(bad_tail == 0, "Vrst_bwd_arc.tail < V", fmt("%" PRIu64 " bad", bad_tail));
  report(bad_ref == 0, "Vrst_bwd_arc.fwd_arc < A", fmt("%" PRIu64 " bad", bad_ref));
  report(bad_match == 0 && unseen == 0, "Vrst_bwd_arc <-> Vrst_fwd_arc bijection",
         fmt("%" PRIu64 " mismatched, %" PRIu64 " fwd arcs without bwd", bad_match, unseen));

  Vrst_uint64 zero_weight = 0;
  for (const Vrst_fwd_arc& a : g.fwd_arcs) zero_weight += a.weight == 0;
  report(zero_weight == 0, "arc weight >= 1", fmt("%" PRIu64 " zero", zero_weight));
}

void check_geometry(const Vrst_nav_data& d) {
  const Vrst_geom& geo = d.geom;
  const Vrst_uint32 S = d.edges.num_segments();
  Vrst_uint64 at = 0;
  const bool mono = check_offsets(geo.seg_first_point, geo.points.size(), &at);
  report(mono, "SEG_FIRST_POINT monotonic, last==P",
         mono ? fmt("S=%u P=%zu", S, geo.points.size()) : fmt("bad at %" PRIu64, at));
  if (!mono) return;

  Vrst_uint64 short_segs = 0, bad_ends = 0;
  for (Vrst_uint32 s = 0; s < S; ++s) {
    auto pts = geo.segment_points(s);
    if (pts.size() < 2) {
      ++short_segs;
      continue;
    }
    const Vrst_seg_vertices sv = d.edges.vertices[s];
    if (sv.a >= d.graph.num_vertices() || sv.b >= d.graph.num_vertices() ||
        pts.front() != d.graph.vertex_coord[sv.a] || pts.back() != d.graph.vertex_coord[sv.b])
      ++bad_ends;
  }
  report(short_segs == 0, "segments have >= 2 points", fmt("%" PRIu64 " bad", short_segs));
  report(bad_ends == 0, "segment ends == vertex coords", fmt("%" PRIu64 " bad", bad_ends));
}

void check_coords(const Vrst_nav_data& d) {
  const Vrst_bbox& bb = d.graph.bbox;
  Vrst_uint64 invalid = 0, outside = 0;
  auto test = [&](Vrst_coord c) {
    if (!valid_coord(c))
      ++invalid;
    else if (!in_bbox(c, bb))
      ++outside;
  };
  for (Vrst_coord c : d.graph.vertex_coord) test(c);
  for (Vrst_coord c : d.geom.points) test(c);
  report(invalid == 0, "no invalid coords", fmt("%" PRIu64 " invalid", invalid));
  report(outside == 0, "coords inside bbox",
         fmt("%" PRIu64 " outside lat [%.7f, %.7f] lon [%.7f, %.7f]", outside, deg(bb.min.lat_e7),
             deg(bb.max.lat_e7), deg(bb.min.lon_e7), deg(bb.max.lon_e7)));
}

void check_attrs(const Vrst_nav_data& d) {
  const Vrst_uint32 N = d.names.size();
  Vrst_uint64 bad_name = 0, bad_class = 0;
  for (const Vrst_seg_attr& a : d.edges.attr) {
    bad_name += (a.name_id >= N) + (a.ref_id >= N);
    bad_class += a.road_class >= kRoadClassCount;
  }
  report(bad_name == 0, "name_id/ref_id < N", fmt("N=%u, %" PRIu64 " bad", N, bad_name));
  report(bad_class == 0, "road_class valid", fmt("%" PRIu64 " bad", bad_class));
  bool offs_ok = d.names.offsets.front() == 0 && d.names.offsets[1] == 0;
  for (Vrst_size i = 1; i < d.names.offsets.size(); ++i) offs_ok &= d.names.offsets[i] >= d.names.offsets[i - 1];
  report(offs_ok, "STR_OFFSETS monotonic, id 0 empty", fmt("data %zu bytes", d.names.data.size()));
}

void stats(const Vrst_nav_data& d) {
  const Vrst_uint32 V = d.graph.num_vertices();
  std::vector<Vrst_uint32> degree(V, 0);
  Vrst_union_find uf(V);
  for (const Vrst_seg_vertices& sv : d.edges.vertices) {
    ++degree[sv.a];
    ++degree[sv.b];
    uf.unite(sv.a, sv.b);
  }
  const auto dead_ends = std::count(degree.begin(), degree.end(), 1u);
  Vrst_uint32 components = 0, largest = 0;
  for (Vrst_uint32 v = 0; v < V; ++v)
    if (uf.find(v) == v) {
      ++components;
      largest = std::max(largest, uf.size[v]);
    }
  Vrst_uint64 length_dm = 0;
  for (const Vrst_seg_attr& a : d.edges.attr) length_dm += a.length_dm;
  std::printf("\nstats:\n");
  std::printf("  V=%u  S=%u  A=%u  P=%zu  N=%u\n", V, d.edges.num_segments(), d.graph.num_arcs(),
              d.geom.points.size(), d.names.size());
  std::printf("  road length          %.1f km\n", length_dm / 1e4);
  std::printf("  dead-end vertices    %td\n", dead_ends);
  std::printf("  components           %u\n", components);
  std::printf("  largest component    %u vertices (%.1f%%)\n", largest, V ? 100.0 * largest / V : 0.0);
}

void dump_way(const Vrst_nav_data& d, Vrst_uint64 way_id) {
  static const char* kClassNames[] = {"motorway",      "motorway_link", "trunk",          "trunk_link",
                                      "primary",       "primary_link",  "secondary",      "secondary_link",
                                      "tertiary",      "tertiary_link", "unclassified",   "residential",
                                      "living_street", "service",       "ferry"};
  static_assert(std::size(kClassNames) == kRoadClassCount);
  bool found = false;
  for (Vrst_uint32 s = 0; s < d.edges.num_segments(); ++s) {
    if (d.edges.osm_way[s] != way_id) continue;
    const Vrst_seg_attr& a = d.edges.attr[s];
    const Vrst_seg_vertices sv = d.edges.vertices[s];
    if (!found)
      std::printf("way %" PRIu64 "  name=\"%.*s\" ref=\"%.*s\" class=%s maxspeed=%u flags=0x%x\n", way_id,
                  int(d.names.get(a.name_id).size()), d.names.get(a.name_id).data(),
                  int(d.names.get(a.ref_id).size()), d.names.get(a.ref_id).data(), kClassNames[a.road_class],
                  a.maxspeed_kmh, a.flags);
    found = true;
    std::printf("segment %u  v%u -> v%u  %.1f m\n", s, sv.a, sv.b, a.length_dm / 10.0);
    for (Vrst_coord c : d.geom.segment_points(s)) std::printf("  %.7f %.7f\n", deg(c.lat_e7), deg(c.lon_e7));
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
  check_csr(d.graph);
  if (g_failures == 0) check_arcs(d);  // arc checks index through the CSR
  check_geometry(d);
  check_coords(d);
  check_attrs(d);
  if (g_failures == 0) stats(d);
  std::printf("\n%s\n", g_failures == 0 ? "ALL PASS" : ("FAILED (" + std::to_string(g_failures) + ")").c_str());
  return g_failures == 0 ? 0 : 1;
}
