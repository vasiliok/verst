// Binary layout shared by osm2nav, navcheck and navview.
//
// Every file: Vrst_file_header, Vrst_section_entry[section_count], then sections.
// Sections start at 64-byte aligned offsets. Little-endian, no pointers.
#pragma once

#include <bit>

#include "vrst_types.h"

static_assert(std::endian::native == std::endian::little, "nav files are little-endian only");

namespace nav {

inline constexpr Vrst_uint32 kFormatVersion = 1;
inline constexpr Vrst_uint64 kSectionAlign = 64;

enum class FileType : Vrst_uint32 { Graph = 1, Edges = 2, Geom = 3, Names = 4 };

inline constexpr char kMagicGraph[8] = {'N', 'A', 'V', 'G', 'R', 'A', 'P', 'H'};
inline constexpr char kMagicEdges[8] = {'N', 'A', 'V', 'E', 'D', 'G', 'E', 'S'};
inline constexpr char kMagicGeom[8] = {'N', 'A', 'V', 'G', 'E', 'O', 'M', '\0'};
inline constexpr char kMagicNames[8] = {'N', 'A', 'V', 'N', 'A', 'M', 'E', 'S'};

inline const char* magic_for(FileType t) {
  switch (t) {
    case FileType::Graph: return kMagicGraph;
    case FileType::Edges: return kMagicEdges;
    case FileType::Geom: return kMagicGeom;
    case FileType::Names: return kMagicNames;
  }
  return nullptr;
}

inline const char* file_name_for(FileType t) {
  switch (t) {
    case FileType::Graph: return "graph.dat";
    case FileType::Edges: return "edges.dat";
    case FileType::Geom: return "geom.dat";
    case FileType::Names: return "names.dat";
  }
  return nullptr;
}

enum class SectionId : Vrst_uint32 {
  // graph.dat
  VertexCoord = 1,    // Vrst_coord[V]
  FwdFirstOut = 2,    // uint32[V+1]
  FwdArcs = 3,        // Vrst_fwd_arc[A]
  BwdFirstIn = 4,     // uint32[V+1]
  BwdArcs = 5,        // Vrst_bwd_arc[A]
  Bbox = 6,           // Vrst_bbox[1]  extent of all nodes in the source file
  // edges.dat
  SegAttr = 16,       // Vrst_seg_attr[S]
  SegVertices = 17,   // Vrst_seg_vertices[S]
  SegOsmWay = 18,     // uint64[S]
  // geom.dat
  SegFirstPoint = 32, // uint32[S+1]
  Points = 33,        // Vrst_coord[P]
  // names.dat
  StrOffsets = 48,    // uint32[N+1]
  StrData = 49,       // char[]
};

struct Vrst_file_header {  // 64 B
  char magic[8];
  Vrst_uint32 format_version;
  Vrst_uint32 file_type;  // FileType
  Vrst_uint64 build_id;
  Vrst_uint32 section_count;
  Vrst_uint32 flags;
  Vrst_uint8 reserved[32];
};

struct Vrst_section_entry {  // 24 B
  Vrst_uint32 id;         // SectionId
  Vrst_uint32 elem_size;
  Vrst_uint64 offset;     // from file start, 64-aligned
  Vrst_uint64 count;
};

struct Vrst_coord {  // 8 B, 1e-7 degrees
  Vrst_int32 lat_e7;
  Vrst_int32 lon_e7;
  friend bool operator==(const Vrst_coord&, const Vrst_coord&) = default;
};

struct Vrst_bbox {  // 16 B
  Vrst_coord min;
  Vrst_coord max;
};

// --- graph.dat -------------------------------------------------------------

inline constexpr Vrst_uint32 kSegReversed = 0x8000'0000u;  // Vrst_fwd_arc::seg bit31
inline constexpr Vrst_uint32 kSegIdMask = 0x7FFF'FFFFu;

struct Vrst_fwd_arc {     // 12 B
  Vrst_uint32 head;
  Vrst_uint32 weight;  // deciseconds
  Vrst_uint32 seg;     // segment id | kSegReversed
};

struct Vrst_bwd_arc {      // 12 B
  Vrst_uint32 tail;
  Vrst_uint32 weight;
  Vrst_uint32 fwd_arc;  // index into Vrst_fwd_arc
};

// --- edges.dat -------------------------------------------------------------

enum class RoadClass : Vrst_uint8 {
  Motorway, MotorwayLink,
  Trunk, TrunkLink,
  Primary, PrimaryLink,
  Secondary, SecondaryLink,
  Tertiary, TertiaryLink,
  Unclassified,
  Residential,
  LivingStreet,
  Service,
  Ferry,
  Count
};
inline constexpr int kRoadClassCount = static_cast<int>(RoadClass::Count);

enum SegFlags : Vrst_uint16 {
  kSegOneway = 1u << 0,         // exactly one arc exists
  kSegRoundabout = 1u << 1,
  kSegLink = 1u << 2,
  kSegToll = 1u << 3,
  kSegTunnel = 1u << 4,
  kSegBridge = 1u << 5,
  kSegFerry = 1u << 6,
  kSegOnewayReverse = 1u << 7,  // with kSegOneway: allowed direction is b -> a
};

struct Vrst_seg_attr {  // 16 B
  Vrst_uint32 length_dm;
  Vrst_uint32 name_id;  // 0 = none
  Vrst_uint32 ref_id;   // 0 = none
  Vrst_uint8 road_class;
  Vrst_uint8 maxspeed_kmh;  // 0 = unknown
  Vrst_uint16 flags;        // SegFlags
};

struct Vrst_seg_vertices {  // 8 B
  Vrst_uint32 a;
  Vrst_uint32 b;
};

static_assert(sizeof(Vrst_file_header) == 64);
static_assert(sizeof(Vrst_section_entry) == 24);
static_assert(sizeof(Vrst_coord) == 8);
static_assert(sizeof(Vrst_bbox) == 16);
static_assert(sizeof(Vrst_fwd_arc) == 12);
static_assert(sizeof(Vrst_bwd_arc) == 12);
static_assert(sizeof(Vrst_seg_attr) == 16);
static_assert(sizeof(Vrst_seg_vertices) == 8);

}  // namespace nav
