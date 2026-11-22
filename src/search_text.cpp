/// @file
/// normalize(), number_key(), number_display(), parse_query(). Documentation is in
/// search_text.h.
#include "search_text.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstddef>
#include <utility>

#include "vrst_types.h"

namespace nav {

namespace {

/// Latin letters with accents and ligatures (U+00C0 .. U+017F) and what they become.
struct Vrst_fold_range {
  char32_t first;    ///< First code point.
  char32_t last;     ///< Last code point, inclusive.
  const char* text;  ///< Replacement, lowercase ASCII.
};

/// Folding table, sorted by code point. Upper and lower case share a range where they
/// are neighbours (Latin Extended-A alternates them).
constexpr std::array<Vrst_fold_range, 50> kFold{{
    {0xC0, 0xC5, "a"},   {0xC6, 0xC6, "ae"},  {0xC7, 0xC7, "c"},   {0xC8, 0xCB, "e"},   {0xCC, 0xCF, "i"},
    {0xD0, 0xD0, "d"},   {0xD1, 0xD1, "n"},   {0xD2, 0xD6, "o"},   {0xD7, 0xD7, " "},   {0xD8, 0xD8, "o"},
    {0xD9, 0xDC, "u"},   {0xDD, 0xDD, "y"},   {0xDE, 0xDE, "th"},  {0xDF, 0xDF, "ss"},  {0xE0, 0xE5, "a"},
    {0xE6, 0xE6, "ae"},  {0xE7, 0xE7, "c"},   {0xE8, 0xEB, "e"},   {0xEC, 0xEF, "i"},   {0xF0, 0xF0, "d"},
    {0xF1, 0xF1, "n"},   {0xF2, 0xF6, "o"},   {0xF7, 0xF7, " "},   {0xF8, 0xF8, "o"},   {0xF9, 0xFC, "u"},
    {0xFD, 0xFD, "y"},   {0xFE, 0xFE, "th"},  {0xFF, 0xFF, "y"},   {0x100, 0x105, "a"}, {0x106, 0x10D, "c"},
    {0x10E, 0x111, "d"}, {0x112, 0x11B, "e"}, {0x11C, 0x123, "g"}, {0x124, 0x127, "h"}, {0x128, 0x131, "i"},
    {0x132, 0x133, "ij"}, {0x134, 0x135, "j"}, {0x136, 0x138, "k"}, {0x139, 0x142, "l"}, {0x143, 0x14B, "n"},
    {0x14C, 0x151, "o"}, {0x152, 0x153, "oe"}, {0x154, 0x159, "r"}, {0x15A, 0x161, "s"}, {0x162, 0x167, "t"},
    {0x168, 0x173, "u"}, {0x174, 0x175, "w"}, {0x176, 0x178, "y"}, {0x179, 0x17E, "z"}, {0x17F, 0x17F, "s"},
}};

/// Query abbreviations that are not a prefix of the word they stand for; prefixes
/// ("av" for avenue) already match as they are.
constexpr std::array<std::pair<std::string_view, std::string_view>, 6> kAbbreviations{{
    {"st", "saint"}, {"ste", "sainte"}, {"r", "rue"}, {"bd", "boulevard"}, {"rte", "route"}, {"fg", "faubourg"},
}};

/// Decodes one UTF-8 code point.
/// @param s Text.
/// @param i Position of the first byte; moved past the code point.
/// @return Code point, or U+FFFD for a bad sequence (one byte consumed).
char32_t next_code_point(std::string_view s, Vrst_size& i) {
  const auto byte = [&](Vrst_size k) { return static_cast<unsigned char>(s[k]); };
  const unsigned char b = byte(i);
  const Vrst_size len = b < 0x80 ? 1 : (b >> 5) == 0x6 ? 2 : (b >> 4) == 0xE ? 3 : (b >> 3) == 0x1E ? 4 : 0;
  if (len == 0 || i + len > s.size()) {
    ++i;
    return 0xFFFD;
  }
  char32_t c = len == 1 ? b : b & (0x7F >> len);
  for (Vrst_size k = 1; k < len; ++k) {
    if ((byte(i + k) & 0xC0) != 0x80) {
      ++i;
      return 0xFFFD;
    }
    c = (c << 6) | (byte(i + k) & 0x3F);
  }
  i += len;
  return c;
}

/// Splits normalized text into its words.
/// @param text Output of normalize().
/// @return Words.
std::vector<std::string> words_of(std::string_view text) {
  std::vector<std::string> words;
  for (Vrst_size b = 0; b < text.size();) {
    const Vrst_size e = std::min(text.find(' ', b), text.size());
    words.emplace_back(text.substr(b, e - b));
    b = e + 1;
  }
  return words;
}

/// Whether a word starts with a digit.
/// @param w Word.
/// @return true for "12", "12b".
bool starts_with_digit(std::string_view w) { return !w.empty() && std::isdigit(static_cast<unsigned char>(w[0])); }

}  // namespace

std::string normalize(std::string_view text) {
  std::string out;
  out.reserve(text.size());
  const auto space = [&out] {
    if (!out.empty() && out.back() != ' ') out += ' ';
  };
  for (Vrst_size i = 0; i < text.size();) {
    const Vrst_size start = i;
    const char32_t c = next_code_point(text, i);
    if (c < 0x80) {
      const auto a = static_cast<unsigned char>(c);
      if (std::isalnum(a)) out += static_cast<char>(std::tolower(a));
      else space();
      continue;
    }
    const auto r = std::ranges::find_if(kFold, [c](const Vrst_fold_range& f) { return c >= f.first && c <= f.last; });
    if (r != kFold.end()) {
      if (r->text[0] == ' ') space();
      else out += r->text;
    } else if (c < 0xC0 || (c >= 0x2000 && c <= 0x206F) || c == 0xFFFD) {
      space();  // Latin-1 punctuation, general punctuation (curly apostrophe), bad bytes
    } else {
      out.append(text.substr(start, i - start));
    }
  }
  if (!out.empty() && out.back() == ' ') out.pop_back();
  return out;
}

std::string number_key(std::string_view number) {
  std::string key = normalize(number);
  std::erase(key, ' ');
  if (std::ranges::none_of(key, [](char ch) { return std::isdigit(static_cast<unsigned char>(ch)) != 0; })) return {};
  return key;
}

std::string number_display(std::string_view key) {
  const Vrst_size digits = static_cast<Vrst_size>(
      std::ranges::find_if(key, [](char ch) { return !std::isdigit(static_cast<unsigned char>(ch)); }) - key.begin());
  if (key.size() - digits < 2 || digits == 0) return std::string(key);
  return std::string(key.substr(0, digits)) + " " + std::string(key.substr(digits));
}

Vrst_search_query parse_query(std::string_view text) {
  constexpr Vrst_size kMaxNumberDigits = 4;
  const std::vector<std::string> words = words_of(normalize(text));
  Vrst_search_query q;
  std::string number;
  for (Vrst_size i = 0; i < words.size(); ++i) {
    const std::string& w = words[i];
    const auto digits = static_cast<Vrst_size>(
        std::ranges::find_if(w, [](char ch) { return !std::isdigit(static_cast<unsigned char>(ch)); }) - w.begin());
    if (number.empty() && starts_with_digit(w) && digits <= kMaxNumberDigits) {
      number = w;
      if (i + 1 < words.size()) {
        const std::string& n = words[i + 1];
        // A letter suffix is A to D; "12 r de la paix" keeps "r" as rue.
        const bool suffix = n == "bis" || n == "ter" || n == "quater" ||
                            (n.size() == 1 && n[0] >= 'a' && n[0] <= 'd' && digits == w.size());
        if (suffix) {
          number += n;
          ++i;
        }
      }
      continue;
    }
    const auto a = std::ranges::find(kAbbreviations, std::string_view(w), &std::pair<std::string_view, std::string_view>::first);
    q.terms.emplace_back(a != kAbbreviations.end() ? std::string(a->second) : w);
  }
  if (q.terms.empty()) {
    if (!number.empty()) q.terms.push_back(number);  // only a number: search it as a word
  } else {
    q.number = number_key(number);
  }
  return q;
}

}  // namespace nav
