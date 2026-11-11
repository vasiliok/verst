/// @file
/// Binary layout shared by osm2nav, navcheck and navview.
///
/// Every file: Vrst_file_header, Vrst_section_entry[section_count], then sections.
/// Sections start at 64-byte aligned offsets. Little-endian, no pointers.
#pragma once

#include <bit>
#include <string_view>
#include <type_traits>

#include "map_kinds.h"
#include "vrst_types.h"

static_assert(std::endian::native == std::endian::little, "nav files are little-endian only");

namespace nav {

/// Layout version written to Vrst_file_header::format_version. Bump on any change to
/// an on-disk struct or enum value; loaders refuse every other version.
inline constexpr Vrst_uint32 kFormatVersion = 3;
/// Section offset alignment in bytes: one cache line, and enough for any element type.
inline constexpr Vrst_uint64 kSectionAlign = 64;

/// A type that may be stored in a section as raw bytes: copied with memcpy semantics and
/// with a fixed layout, so the bytes written by osm2nav read back as the same values.
/// Rejects types that own memory (std::string, std::vector) at compile time.
/// @tparam T Section element type.
template <class T>
concept Vrst_plain_data = std::is_trivially_copyable_v<T> && std::is_standard_layout_v<T>;

/// Which nav file a file is; stored in Vrst_file_header::file_type.
/// Each loader states the type it expects, so passing edges.dat where graph.dat is
/// wanted fails at open() instead of misreading sections. Starts at 1 so a
/// zero-filled header never matches any type.
enum class FileType : Vrst_uint32 { Graph = 1, Edges = 2, Geom = 3, Names = 4, Map = 5 };

inline constexpr char kMagicGraph[8] = {'N', 'A', 'V', 'G', 'R', 'A', 'P', 'H'};  ///< graph.dat magic.
inline constexpr char kMagicEdges[8] = {'N', 'A', 'V', 'E', 'D', 'G', 'E', 'S'};  ///< edges.dat magic.
inline constexpr char kMagicGeom[8] = {'N', 'A', 'V', 'G', 'E', 'O', 'M', '\0'};  ///< geom.dat magic.
inline constexpr char kMagicNames[8] = {'N', 'A', 'V', 'N', 'A', 'M', 'E', 'S'};  ///< names.dat magic.
inline constexpr char kMagicMap[8] = {'N', 'A', 'V', 'M', 'A', 'P', '\0', '\0'};  ///< map.dat magic.

/// Magic bytes of a file type.
/// @param t File type.
/// @return The 8 magic bytes; empty for an unknown type.
constexpr std::string_view magic_for(FileType t) {
  switch (t) {
    case FileType::Graph: return {kMagicGraph, 8};
    case FileType::Edges: return {kMagicEdges, 8};
    case FileType::Geom: return {kMagicGeom, 8};
    case FileType::Names: return {kMagicNames, 8};
    case FileType::Map: return {kMagicMap, 8};
  }
  return {};
}

/// File name of a file type inside an output directory.
/// @param t File type.
/// @return "graph.dat", "edges.dat", "geom.dat", "names.dat" or "map.dat"; empty for an unknown type.
constexpr std::string_view file_name_for(FileType t) {
  switch (t) {
    case FileType::Graph: return "graph.dat";
    case FileType::Edges: return "edges.dat";
    case FileType::Geom: return "geom.dat";
    case FileType::Names: return "names.dat";
    case FileType::Map: return "map.dat";
  }
  return {};
}

/// Id of one array inside a nav file, stored in Vrst_section_entry::id.
/// Readers look sections up by id, never by position, so sections can be added or
/// reordered without breaking old readers. Ids are grouped by file in blocks of 16
/// (graph 1.., edges 16.., geom 32.., names 48.., map 64..) so a new section fits next to its
/// file without renumbering. Values are on-disk: never change an existing one.
enum class SectionId : Vrst_uint32 {
  // graph.dat
  VertexCoord = 1,    ///< Vrst_coord[V]
  FwdFirstOut = 2,    ///< uint32[V+1]
  FwdArcs = 3,        ///< Vrst_fwd_arc[A]
  BwdFirstIn = 4,     ///< uint32[V+1]
  BwdArcs = 5,        ///< Vrst_bwd_arc[A]
  Bbox = 6,           ///< Vrst_bbox[1]  extent of all nodes in the source file
  // edges.dat
  SegAttr = 16,       ///< Vrst_seg_attr[S]
  SegVertices = 17,   ///< Vrst_seg_vertices[S]
  SegOsmWay = 18,     ///< uint64[S]
  // geom.dat
  SegFirstPoint = 32, ///< uint32[S+1]
  Points = 33,        ///< Vrst_coord[P]
  // names.dat
  StrOffsets = 48,    ///< uint32[N+1]
  StrData = 49,       ///< char[]
  Places = 50,        ///< Vrst_place[L]  city/town/village/hamlet labels, by priority
  // map.dat
  AreaFirstPoint = 64,  ///< uint32[R+1]  first point of each area in AREA_POINTS
  AreaFirstIndex = 65,  ///< uint32[R+1]  first triangle index of each area in AREA_INDICES
  AreaKinds = 66,       ///< uint8[R]     AreaKind
  AreaPoints = 67,      ///< Vrst_coord[]  outline points, outer ring first
  AreaIndices = 68,     ///< uint32[]     triangles, 3 per triangle, relative to the area's first point
  LineFirstPoint = 69,  ///< uint32[L+1]  first point of each line in LINE_POINTS
  LineAttr = 70,        ///< Vrst_line_attr[L]
  LinePoints = 71,      ///< Vrst_coord[]
  AreaOuterSize = 72,   ///< uint32[R]    points of the outer ring; the rest are holes
};

/// First 64 bytes of every nav file.
/// Identifies the file (magic, type, version) and ties it to one import (build_id)
/// before any offset in it is trusted. Without it a loader would cast whatever bytes
/// it was given: a wrong, old or truncated file would give garbage routes, not an error.
/// Size: 64 B (checked by static_assert).
struct Vrst_file_header {
  /// "NAVGRAPH" etc. Rejects a wrong or truncated file before any offset is trusted;
  /// without it a renamed geom.dat would be read as a graph and index out of bounds.
  char magic[8];
  /// Layout version. Bumped on any struct change so an old loader refuses new files
  /// instead of casting them with the wrong layout (mmap means no parse step to catch it).
  Vrst_uint32 format_version;
  /// FileType. Redundant with magic, but numeric: lets tools switch on it and catches
  /// a magic that matches by accident.
  Vrst_uint32 file_type;
  /// Same random id in all four files of one import. Vertex/segment ids are only valid
  /// inside one build: a graph.dat from today with a geom.dat from yesterday has
  /// matching sizes but wrong geometry. This is the only way to detect such a mix.
  Vrst_uint64 build_id;
  /// Number of Vrst_section_entry right after the header. Sections are looked up by
  /// id, so new sections can be added without breaking old readers.
  Vrst_uint32 section_count;
  /// Per-file feature bits, 0 for now. Room for e.g. "geometry delta-compressed"
  /// without a format_version bump.
  Vrst_uint32 flags;
  /// Zero. Keeps the header at 64 B so the section table stays aligned when
  /// fields are added later.
  Vrst_uint8 reserved[32];
};

/// One row of the section table that follows the header: where an array is and
/// what it holds. Turns a file into a set of typed arrays found by id, with enough
/// information (offset, elem_size, count) to bounds-check each one before the
/// zero-copy cast. It is what lets a file be used by mmap alone, with no parsing.
/// Size: 24 B (checked by static_assert).
struct Vrst_section_entry {
  /// SectionId. Sections are found by id, not by position, so order can change and
  /// unknown sections are skipped.
  Vrst_uint32 id;
  /// sizeof one element as written. The loader compares it with sizeof(T) before the
  /// cast: a struct that grew on one platform (padding, different alias type) is
  /// refused instead of being read with a shifted stride.
  Vrst_uint32 elem_size;
  /// From file start, 64-aligned. Alignment makes the reinterpret_cast legal for any
  /// element type and keeps arrays cache-line aligned; 64-bit so files > 4 GB work.
  Vrst_uint64 offset;
  /// Number of elements (not bytes). With elem_size it bounds-checks the section
  /// against the file size, so a truncated file cannot be read past its end.
  Vrst_uint64 count;
};

/// Geographic position in OSM's own fixed-point form (degrees * 1e7).
/// The one coordinate type for vertices, shape points and the bbox. Keeping OSM's
/// integer form means no precision is lost on import, equality is exact, and the
/// file reads the same on every platform.
/// Size: 8 B (checked by static_assert).
struct Vrst_coord {
  /// Latitude * 1e7. Fixed point instead of float: exact OSM precision (~1 cm), the
  /// same bits on every platform, and exact equality for "same node" tests, which
  /// float rounding would break.
  Vrst_int32 lat_e7;
  /// Longitude * 1e7. +-180 deg * 1e7 still fits in int32.
  Vrst_int32 lon_e7;
  /// Exact equality, i.e. the same OSM position. Reliable because the coordinates are
  /// fixed point; with floats two equal nodes could compare unequal.
  /// @param a First coordinate.
  /// @param b Second coordinate.
  /// @return true if latitude and longitude are bit-identical.
  friend bool operator==(const Vrst_coord& a, const Vrst_coord& b) = default;
};

/// Axis-aligned latitude/longitude rectangle, stored once in graph.dat.
/// Gives the data's extent without scanning all points: navcheck uses it for a
/// sanity check, the viewer to decide whether the default point is in the data.
/// Size: 16 B (checked by static_assert).
struct Vrst_bbox {
  /// South-west corner (min lat, min lon).
  Vrst_coord min;
  /// North-east corner. Extent of all source nodes; used by navcheck to catch
  /// swapped lat/lon or garbage coordinates, and by the viewer to test whether a
  /// point is inside the data. Does not handle the antimeridian (min.lon > max.lon).
  Vrst_coord max;
};

// --- graph.dat -------------------------------------------------------------

inline constexpr Vrst_uint32 kSegReversed = 0x8000'0000u;  ///< Vrst_fwd_arc::seg bit31: arc runs b -> a.
inline constexpr Vrst_uint32 kSegIdMask = 0x7FFF'FFFFu;    ///< Segment id part of Vrst_fwd_arc::seg.

/// Directed road arc in the forward CSR: one arc for each direction a car may drive
/// along a segment. Arcs of vertex v are fwd_arcs[fwd_first_out[v] .. fwd_first_out[v+1]).
/// This is the array the router reads most, so it holds only what a forward search
/// needs: where the arc goes, what it costs, and a link back to its segment.
/// Size: 12 B (checked by static_assert).
struct Vrst_fwd_arc {
  /// Target vertex. The tail is implicit (the CSR row), which saves 4 B per arc on
  /// the router's hottest array.
  Vrst_uint32 head;
  /// Travel time in deciseconds, precomputed so the router does no division and no
  /// attribute lookup. Seconds would round short city segments to 0 or 1 and make
  /// many routes tie; deciseconds keep them apart.
  Vrst_uint32 weight;
  /// Segment id | kSegReversed. The router never needs it, but path unpacking does:
  /// it is the only link from an arc to its geometry, name and OSM way. Bit31 says
  /// the arc runs b -> a, so the shape must be read backwards; without it a reverse
  /// oneway or the second arc of a two-way road would be drawn and guided backwards.
  Vrst_uint32 seg;
};

/// The same arc seen from its head: entry of the backward (incoming) CSR.
/// Bidirectional Dijkstra and contraction hierarchies search backwards from the
/// target; without an incoming-arc index that would need a scan of all arcs per
/// vertex. 12 B like Vrst_fwd_arc, so both search directions read the same
/// amount of memory per arc.
/// Size: 12 B (checked by static_assert).
struct Vrst_bwd_arc {
  /// Source vertex of the incoming arc. Needed by the backward search of
  /// bidirectional Dijkstra / CH, which walks arcs against their direction.
  Vrst_uint32 tail;
  /// Copy of the forward weight. Duplicated so the backward search stays inside
  /// this array and does not jump to random Vrst_fwd_arc entries (cache misses).
  Vrst_uint32 weight;
  /// Index of the same arc in FWD_ARCS. When the two searches meet, the backward
  /// half of the path must be turned into forward arcs to get segments and geometry;
  /// also the stable key for per-arc data (turn costs, traffic) shared by both
  /// directions.
  Vrst_uint32 fwd_arc;
};

// --- edges.dat -------------------------------------------------------------


/// Bit flags of a segment, stored in Vrst_seg_attr::flags.
/// Yes/no facts from tags that guidance, avoid-options and rendering need and that
/// the graph structure alone cannot tell. A plain enum, not enum class, so flags
/// combine with | into the 16-bit field without casts.
enum SegFlags : Vrst_uint16 {
  kSegOneway = 1u << 0,         ///< exactly one arc exists
  kSegRoundabout = 1u << 1,     ///< junction=roundabout/circular; guidance counts exits
  kSegLink = 1u << 2,           ///< *_link ramp; guidance says "take the exit"
  kSegToll = 1u << 3,           ///< toll=yes; for avoid-tolls
  kSegTunnel = 1u << 4,         ///< tunnel=* except no; for guidance (GPS is lost inside) and rendering
  kSegBridge = 1u << 5,         ///< bridge=* except no; for rendering and guidance
  kSegFerry = 1u << 6,          ///< route=ferry; for avoid-ferries
  kSegOnewayReverse = 1u << 7,  ///< with kSegOneway: allowed direction is b -> a
};

/// Attributes of one segment (a road piece between two vertices), indexed by
/// segment id. Holds everything that is not needed during the shortest-path search
/// but is needed right after it: lengths, names and road types for guidance, and
/// the inputs to recompute weights. Kept out of graph.dat so routing does not page
/// it in.
/// Size: 16 B (checked by static_assert).
struct Vrst_seg_attr {
  /// Geodesic length in decimetres. Kept apart from weight: distance-to-turn,
  /// trip length and a re-weighting with another speed profile all need metres,
  /// and recomputing it from geom.dat would touch the shape points.
  Vrst_uint32 length_dm;
  /// Street name, index into names.dat, 0 = none. Needed for guidance
  /// ("turn left onto ...") and to merge consecutive segments of the same street.
  Vrst_uint32 name_id;
  /// Road number like "CG-1", 0 = none. Separate from the name: many main roads
  /// have only a ref, and signs show both.
  Vrst_uint32 ref_id;
  /// RoadClass. Drives rendering order/color, default speed and the router's road
  /// hierarchy; turn instructions need it to tell "stay on motorway" from "exit".
  Vrst_uint8 road_class;
  /// Signed limit in km/h, 0 = unknown. Kept even though the weight is baked: speed
  /// warnings, and re-weighting when the default speed table changes, cannot be
  /// done from the weight alone.
  Vrst_uint8 maxspeed_kmh;
  /// SegFlags. Oneway/reverse tell which arcs exist without searching the graph;
  /// roundabout, link, toll, tunnel, bridge and ferry are needed for guidance
  /// ("take the 2nd exit"), avoid-options and rendering.
  Vrst_uint16 flags;
};

/// The two end vertices of a segment, indexed by segment id.
/// The link from a segment back into the graph. Arcs point from vertices to
/// segments; this points the other way, which snapping (position -> segment ->
/// start vertices) and navcheck's consistency checks need.
/// Size: 8 B (checked by static_assert).
struct Vrst_seg_vertices {
  /// Vertex at the first shape point. Arcs only give head (and tail by CSR row), so
  /// going from a segment back to the graph, e.g. snapping a GPS position onto a
  /// segment and starting a route from both of its ends, needs these.
  Vrst_uint32 a;
  /// Vertex at the last shape point. Also defines "forward" for kSegReversed.
  Vrst_uint32 b;
};

// --- names.dat -------------------------------------------------------------


/// Named settlement to label on the map: an OSM node with place=city/town/village/hamlet.
/// Roads alone do not tell where a town is or what it is called; these points are
/// what the map labels with text. Stored in names.dat because the label text is a
/// names.dat string. The section is sorted by kind, then by population (largest
/// first), so a label layout that walks it in order places the important names
/// first and drops the ones that would overlap them.
/// Size: 20 B (checked by static_assert).
struct Vrst_place {
  /// Position of the place node, the label anchor.
  Vrst_coord pos;
  /// Name, index into the names.dat strings; never 0 (unnamed places are skipped).
  Vrst_uint32 name_id;
  /// population=* tag, 0 = unknown. Orders places of the same kind, so the bigger
  /// town wins when two labels collide.
  Vrst_uint32 population;
  /// PlaceKind.
  Vrst_uint8 kind;
  /// Zero. Keeps the size a multiple of 4.
  Vrst_uint8 reserved[3];
};

// --- map.dat ---------------------------------------------------------------



/// Bit flags of a map.dat line, stored in Vrst_line_attr::flags. Same meaning as the
/// kSegBridge / kSegTunnel road flags, so the renderer treats railways and roads alike.
enum LineFlags : Vrst_uint8 {
  kLineBridge = 1u << 0,  ///< bridge=* except no: drawn above what it crosses, with a casing
  kLineTunnel = 1u << 1,  ///< tunnel=* except no: drawn faded below everything else
};

/// Attributes of one map.dat line (river, railway), indexed by line id.
/// Size: 4 B (checked by static_assert).
struct Vrst_line_attr {
  /// LineKind.
  Vrst_uint8 kind;
  /// LineFlags.
  Vrst_uint8 flags;
  /// Zero. Keeps the size a multiple of 4.
  Vrst_uint16 reserved;
};

static_assert(sizeof(Vrst_file_header) == 64);
static_assert(sizeof(Vrst_section_entry) == 24);
static_assert(sizeof(Vrst_coord) == 8);
static_assert(sizeof(Vrst_bbox) == 16);
static_assert(sizeof(Vrst_fwd_arc) == 12);
static_assert(sizeof(Vrst_bwd_arc) == 12);
static_assert(sizeof(Vrst_seg_attr) == 16);
static_assert(sizeof(Vrst_seg_vertices) == 8);
static_assert(sizeof(Vrst_place) == 20);
static_assert(sizeof(Vrst_line_attr) == 4);

}  // namespace nav
