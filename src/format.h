// Binary layout shared by osm2nav, navcheck and navview.
//
// Every file: FileHeader, SectionEntry[section_count], then sections.
// Sections start at 64-byte aligned offsets. Little-endian, no pointers.
#pragma once

#include <bit>
#include <cstdint>

static_assert(std::endian::native == std::endian::little, "nav files are little-endian only");

namespace nav {

inline constexpr uint32_t kFormatVersion = 1;
inline constexpr uint64_t kSectionAlign = 64;

enum class FileType : uint32_t { Graph = 1, Edges = 2, Geom = 3, Names = 4 };

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

enum class SectionId : uint32_t {
  // graph.dat
  VertexCoord = 1,    // Coord[V]
  FwdFirstOut = 2,    // uint32[V+1]
  FwdArcs = 3,        // FwdArc[A]
  BwdFirstIn = 4,     // uint32[V+1]
  BwdArcs = 5,        // BwdArc[A]
  Bbox = 6,           // BBox[1]  extent of all nodes in the source file
  // edges.dat
  SegAttr = 16,       // SegAttr[S]
  SegVertices = 17,   // SegVertices[S]
  SegOsmWay = 18,     // uint64[S]
  // geom.dat
  SegFirstPoint = 32, // uint32[S+1]
  Points = 33,        // Coord[P]
  // names.dat
  StrOffsets = 48,    // uint32[N+1]
  StrData = 49,       // char[]
};

struct FileHeader {  // 64 B
  char magic[8];
  uint32_t format_version;
  uint32_t file_type;  // FileType
  uint64_t build_id;
  uint32_t section_count;
  uint32_t flags;
  uint8_t reserved[32];
};

struct SectionEntry {  // 24 B
  uint32_t id;         // SectionId
  uint32_t elem_size;
  uint64_t offset;     // from file start, 64-aligned
  uint64_t count;
};

struct Coord {  // 8 B, 1e-7 degrees
  int32_t lat_e7;
  int32_t lon_e7;
  friend bool operator==(const Coord&, const Coord&) = default;
};

struct BBox {  // 16 B
  Coord min;
  Coord max;
};

// --- graph.dat -------------------------------------------------------------

inline constexpr uint32_t kSegReversed = 0x8000'0000u;  // FwdArc::seg bit31
inline constexpr uint32_t kSegIdMask = 0x7FFF'FFFFu;

struct FwdArc {     // 12 B
  uint32_t head;
  uint32_t weight;  // deciseconds
  uint32_t seg;     // segment id | kSegReversed
};

struct BwdArc {      // 12 B
  uint32_t tail;
  uint32_t weight;
  uint32_t fwd_arc;  // index into FwdArc
};

// --- edges.dat -------------------------------------------------------------

enum class RoadClass : uint8_t {
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

enum SegFlags : uint16_t {
  kSegOneway = 1u << 0,         // exactly one arc exists
  kSegRoundabout = 1u << 1,
  kSegLink = 1u << 2,
  kSegToll = 1u << 3,
  kSegTunnel = 1u << 4,
  kSegBridge = 1u << 5,
  kSegFerry = 1u << 6,
  kSegOnewayReverse = 1u << 7,  // with kSegOneway: allowed direction is b -> a
};

struct SegAttr {  // 16 B
  uint32_t length_dm;
  uint32_t name_id;  // 0 = none
  uint32_t ref_id;   // 0 = none
  uint8_t road_class;
  uint8_t maxspeed_kmh;  // 0 = unknown
  uint16_t flags;        // SegFlags
};

struct SegVertices {  // 8 B
  uint32_t a;
  uint32_t b;
};

static_assert(sizeof(FileHeader) == 64);
static_assert(sizeof(SectionEntry) == 24);
static_assert(sizeof(Coord) == 8);
static_assert(sizeof(BBox) == 16);
static_assert(sizeof(FwdArc) == 12);
static_assert(sizeof(BwdArc) == 12);
static_assert(sizeof(SegAttr) == 16);
static_assert(sizeof(SegVertices) == 8);

}  // namespace nav
