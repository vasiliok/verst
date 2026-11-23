# search

[verst](../../README.md) > [src](../README.md) > search

Offline address search (places, streets, house numbers) and points from map links.
The database engine sits behind an interface, so SQLite FTS5 can be replaced without
touching the converter, the ranking or the app.

| File | What |
|---|---|
| `search_text.{h,cpp}` | text rules shared by writer and query: `normalize()`, `parse_query()`, house number keys (abbreviations st, bd, rte, ...) |
| `search_engine.{h,cpp}` | search records, the engine interfaces `Vrst_search_index_writer` / `Vrst_search_index`, `make_search_writer()`, `open_search_index()` |
| `search_sqlite.{h,cpp}` | the SQLite FTS5 engine: `search.db` in the region folder; the only file that includes SQLite |
| `search_rank.{h,cpp}` | `search_index()`: parse the query, ask the engine for candidates, rank them and make results |
| `geo_link.{h,cpp}` | `parse_geo_link()`: points from Google Maps / OpenStreetMap / geo: links and coordinate text; `is_short_map_link()` |

`geo_link.cpp` is also compiled into the iOS share extension (`verst_share`).

Uses: geometry, storage (`format.h`), provider (`search_result.h`).
Used by: provider (`nav_map_provider`), convert (index writer), navsearch, the iOS app
and its share extension.
