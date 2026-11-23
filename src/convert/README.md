# convert

[verst](../../README.md) > [src](../README.md) > convert

OSM extract -> nav files, as a library (`vrst_convert`) so both the desktop tools and
the iPhone app can run it on an extract they downloaded.

| File | What |
|---|---|
| `pbf_merge.{h,cpp}` | `merge_pbf()`: streaming merge of sorted extracts (e.g. Bourgogne + Franche-Comte) |
| `osm_convert.{h,cpp}` | `convert_osm()`: roads per travel mode into graph.dat, edges.dat, geom.dat, names.dat; drives the other builders |
| `osm_map_layers.{h,cpp}` | `Vrst_map_layer_builder`: water, forest, land use, buildings (triangulated), rivers and railways into map.dat |
| `osm_addresses.{h,cpp}` | `Vrst_address_collector`: house numbers, named roads and places into the search index |

`osm_map_layers.cpp` is built with `-ffp-contract=off` (earcut, see
`VRST_EARCUT_SOURCES`).

Uses: base (string pool, progress), geometry, storage (`format.h`), search (index
writer, text rules); libosmium and protozero.
Used by: osm2nav, pbf_merge, the iOS download manager.
