/// @file
/// parse_geo_link() and is_short_map_link(). Documentation is in geo_link.h.
#include "geo_link.h"

#include <cmath>
#include <cstdlib>
#include <regex>

#include "vrst_types.h"

namespace nav {

namespace {

/// Percent escapes decoded and '+' read as a space, as in a URL query.
/// @param s Text.
/// @return Decoded text; bad escapes are kept as they are.
std::string decode_once(std::string_view s) {
  const auto hex = [](char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
  };
  std::string out;
  out.reserve(s.size());
  for (Vrst_size i = 0; i < s.size(); ++i) {
    if (s[i] == '%' && i + 2 < s.size() && hex(s[i + 1]) >= 0 && hex(s[i + 2]) >= 0) {
      out += static_cast<char>(hex(s[i + 1]) * 16 + hex(s[i + 2]));
      i += 2;
    } else {
      out += s[i] == '+' ? ' ' : s[i];
    }
  }
  return out;
}

/// A point when both numbers are a valid latitude and longitude.
/// @param lat Latitude text.
/// @param lon Longitude text.
/// @return Point, or nullopt.
std::optional<Vrst_latlon> point(const std::string& lat, const std::string& lon) {
  const Vrst_float64 a = std::strtod(lat.c_str(), nullptr), b = std::strtod(lon.c_str(), nullptr);
  if (!std::isfinite(a) || !std::isfinite(b) || std::abs(a) > 90 || std::abs(b) > 180) return std::nullopt;
  return Vrst_latlon{a, b};
}

/// The last match of a two-number pattern that is a valid point.
/// @param text Decoded text.
/// @param re Pattern whose groups 1 and 2 are latitude and longitude.
/// @return Point, or nullopt.
std::optional<Vrst_latlon> last_point(const std::string& text, const std::regex& re) {
  std::optional<Vrst_latlon> found;
  for (std::sregex_iterator it(text.begin(), text.end(), re), end; it != end; ++it)
    if (const std::optional<Vrst_latlon> p = point((*it)[1], (*it)[2])) found = p;
  return found;
}

/// Degrees, minutes, seconds as degrees.
/// @param d Degrees.
/// @param m Minutes.
/// @param s Seconds.
/// @param hemisphere N, S, E or W.
/// @return Signed degrees.
Vrst_float64 dms(const std::string& d, const std::string& m, const std::string& s, const std::string& hemisphere) {
  const Vrst_float64 v = std::strtod(d.c_str(), nullptr) + std::strtod(m.c_str(), nullptr) / 60 +
                         std::strtod(s.c_str(), nullptr) / 3600;
  return hemisphere == "S" || hemisphere == "W" ? -v : v;
}

}  // namespace

std::optional<Vrst_geo_link> parse_geo_link(std::string_view raw) {
  // Decoded up to three times: a link inside a link is escaped once more each time.
  std::string text(raw);
  for (int i = 0; i < 3; ++i) {
    std::string d = decode_once(text);
    if (d == text) break;
    text = std::move(d);
  }
  static const std::string kNum = R"((-?\d{1,3}(?:\.\d+)?))";
  static const std::regex kPin(R"(!3d)" + kNum + R"(!4d)" + kNum);
  static const std::regex kParam(R"([?&](?:q|query|ll|sll|center|destination|daddr)=(?:loc:)?\s*)" + kNum +
                                 R"(\s*[, ]\s*)" + kNum);
  static const std::regex kOsmMarker(R"([?&]mlat=)" + kNum + R"(&mlon=)" + kNum);
  static const std::regex kGeo(R"(geo:)" + kNum + R"(,)" + kNum);
  static const std::regex kOsmMap(R"(#map=\d+/)" + kNum + R"(/)" + kNum);
  static const std::regex kCenter(R"(/@)" + kNum + R"(,)" + kNum + R"([,z/])");
  static const std::string kDec = R"((-?\d{1,3}\.\d+))";  // typed numbers like "12 3" are not a point
  static const std::regex kPlain(R"(^\s*)" + kDec + R"(\s*[,; ]\s*)" + kDec + R"(\s*$)");
  static const std::regex kDms(
      R"re((\d{1,3})\D{1,3}(\d{1,2})['\x60]\s*(\d{1,2}(?:\.\d+)?)"\s*([NS])[\s,]*(\d{1,3})\D{1,3}(\d{1,2})['\x60]\s*(\d{1,2}(?:\.\d+)?)"\s*([EW]))re");
  static const std::regex kPlace(R"(/place/([^/@?]+))");

  Vrst_geo_link link;
  std::smatch m;
  if (std::regex_search(text, m, kPlace)) link.name = m[1];
  for (const std::regex* re : {&kPin, &kParam, &kOsmMarker, &kGeo, &kOsmMap}) {
    if (const std::optional<Vrst_latlon> p = last_point(text, *re)) {
      link.pos = *p;
      return link;
    }
  }
  if (const std::optional<Vrst_latlon> p = last_point(text, kCenter)) {
    link.pos = *p;
    link.exact = false;
    return link;
  }
  if (const std::optional<Vrst_latlon> p = last_point(text, kPlain)) {
    link.pos = *p;
    return link;
  }
  if (std::regex_search(text, m, kDms)) {
    const Vrst_float64 lat = dms(m[1], m[2], m[3], m[4]), lon = dms(m[5], m[6], m[7], m[8]);
    if (std::abs(lat) <= 90 && std::abs(lon) <= 180) {
      link.pos = {lat, lon};
      return link;
    }
  }
  return std::nullopt;
}

std::optional<std::string> parse_link_address(std::string_view raw) {
  if (raw.find("://") == std::string_view::npos) return std::nullopt;  // links only, not typed text
  std::string text(raw);
  for (int i = 0; i < 3; ++i) {
    std::string d = decode_once(text);
    if (d == text) break;
    text = std::move(d);
  }
  static const std::regex kQuery(R"([?&](?:q|query)=([^&#]+))");
  std::smatch m;
  if (!std::regex_search(text, m, kQuery)) return std::nullopt;
  std::string q = m[1];
  while (!q.empty() && q.back() == ' ') q.pop_back();
  if (q.empty()) return std::nullopt;
  return q;
}

bool is_short_map_link(std::string_view text) {
  for (const std::string_view host : {"maps.app.goo.gl/", "goo.gl/maps", "g.co/kgs/"})
    if (text.find(host) != std::string_view::npos) return true;
  return false;
}

}  // namespace nav
