/// @file
/// The kinds of things a map shows: road classes, place kinds, area kinds and line
/// kinds. Shared vocabulary of the data side (osm2nav writes the values into the nav
/// files, format.h) and the client side (renderer colours, toggles, labels), so a
/// client can style the map without including any storage header.
#pragma once

#include "vrst_types.h"

namespace nav {

/// Car-relevant road type, derived from highway=* (and route=ferry for Ferry).
/// Groups dozens of OSM tag values into a fixed set that the profile, router,
/// guidance and renderer can index arrays by (kClasses, kClassColors, class_mask
/// bits). Values are stored in Vrst_seg_attr::road_class, so reordering them is a
/// format change; add new classes before Count and bump kFormatVersion.
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
/// Number of road classes: size of every per-class table and bits used in class masks.
inline constexpr int kRoadClassCount = static_cast<int>(RoadClass::Count);

/// Settlement size from OSM place=*, stored in Vrst_place::kind.
/// Decides label priority, font size and the zoom from which a label is shown; a
/// village label at country zoom would cover the map. Ordered from most to least
/// important; values are on-disk, add new kinds before Count and bump kFormatVersion.
enum class PlaceKind : Vrst_uint8 { City, Town, Village, Hamlet, Count };
/// Number of place kinds: size of every per-kind table.
inline constexpr int kPlaceKindCount = static_cast<int>(PlaceKind::Count);

/// What an area of map.dat is, stored in AREA_KIND. Drawing-only layers: routing never
/// reads map.dat. Ordered bottom to top as the renderer draws them, so sorting areas
/// by kind is the draw order. Values are on-disk; add new kinds before Count and bump
/// kFormatVersion.
enum class AreaKind : Vrst_uint8 {
  Farmland,     ///< landuse=farmland/farmyard/orchard/vineyard
  Meadow,       ///< landuse=meadow/grass, natural=grassland/scrub/heath
  Residential,  ///< landuse=residential
  Industrial,   ///< landuse=industrial/commercial/retail/railway
  Park,         ///< leisure=park/garden/pitch, landuse=cemetery/recreation_ground
  Forest,       ///< landuse=forest, natural=wood
  Water,        ///< natural=water, waterway=riverbank, landuse=reservoir/basin
  Building,     ///< building=*, usually reduced to its smallest rotated rectangle
  Count
};
/// Number of area kinds: size of every per-kind table.
inline constexpr int kAreaKindCount = static_cast<int>(AreaKind::Count);

/// What a line of map.dat is, stored in Vrst_line_attr::kind. Roads are not here: they
/// are the routing segments of geom.dat. Values are on-disk.
enum class LineKind : Vrst_uint8 {
  Stream,     ///< waterway=stream/ditch/drain
  River,      ///< waterway=river/canal
  Rail,       ///< railway=rail/narrow_gauge
  MinorRail,  ///< railway=light_rail/tram/subway
  Count
};
/// Number of line kinds: size of every per-kind table.
inline constexpr int kLineKindCount = static_cast<int>(LineKind::Count);

}  // namespace nav
