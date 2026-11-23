/// @file
/// Points from map links and coordinate text: a Google Maps link (place, search or
/// coordinate link), an OpenStreetMap or geo: link, or coordinates as text, so a point
/// found in another app can be shown on our map. Google Maps uses WGS84 like OSM
/// (outside China), so the numbers are taken as they are.
#pragma once

#include <optional>
#include <string>
#include <string_view>

#include "geometry/geo.h"

namespace nav {

/// A point read from a link or text.
struct Vrst_geo_link {
  Vrst_latlon pos;   ///< The point.
  std::string name;  ///< Place name from the link (Google /place/<name>/), may be empty.
  bool exact = true; ///< false when only the map center of the link was found (Google /@lat,lon).
};

/// Reads a point. Tried in this order, the first that gives a valid point wins:
/// 1. Google's place pin `!3d<lat>!4d<lon>` (the last one in the link);
/// 2. a `q`, `query`, `ll`, `center`, `destination`, `daddr` or `mlat`/`mlon` parameter
///    with "lat,lon" (also "loc:lat,lon", "lat+lon", "lat%2Clon");
/// 3. `geo:lat,lon` and OSM's `#map=<zoom>/<lat>/<lon>`;
/// 4. Google's map center `/@lat,lon,` (exact = false);
/// 5. the whole text as decimal "lat, lon" (with a decimal point) or degrees-minutes-seconds
///    (`47 19'20.0"N 5 02'30.0"E`, with or without the degree sign).
/// Percent escapes are decoded first, so a link inside another link (Google's consent
/// page `continue=`) is found too.
/// @param text Link or text.
/// @return The point, or nullopt when the text has none.
std::optional<Vrst_geo_link> parse_geo_link(std::string_view text);

/// The place text of a map link that has no coordinates, such as a shared Google Maps
/// place ("maps.google.com?q=Cafe du Port, 12 Rue de la Liberte, 21000 Dijon,
/// France&ftid=..."): the `q` or `query` value, decoded.
/// @param text Link.
/// @return The place text, or nullopt when the text is not such a link.
std::optional<std::string> parse_link_address(std::string_view text);

/// Whether a text is a short Google Maps link that must be opened (followed to where it
/// redirects) before it shows a point: maps.app.goo.gl, goo.gl/maps, g.co/kgs.
/// @param text Link or text.
/// @return true for such a link.
bool is_short_map_link(std::string_view text);

}  // namespace nav
