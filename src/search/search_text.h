/// @file
/// Text rules of the address search, shared by the index writer and the query side so
/// both see the same words: normalize() and parse_query().
#pragma once

#include <string>
#include <string_view>
#include <vector>

namespace nav {

/// Normalized form for matching: lowercase, Latin accents folded ("e" for e acute),
/// ligatures expanded (ae, oe, ss), punctuation and apostrophes turned into spaces,
/// spaces collapsed and trimmed. Letters outside Latin are kept as they are.
/// @param text UTF-8 text.
/// @return Normalized text, words separated by one space.
std::string normalize(std::string_view text);

/// A house number as stored and looked up: normalized, without spaces ("12 bis" ->
/// "12bis", "12B" -> "12b").
/// @param number House number text.
/// @return Key; empty when the text has no digit.
std::string number_key(std::string_view number);

/// A house number key for display: a suffix of two letters or more gets a space
/// ("12bis" -> "12 bis"; "12b" stays).
/// @param key Key from number_key().
/// @return Display text.
std::string number_display(std::string_view key);

/// What the user typed, split for the search.
struct Vrst_search_query {
  /// Normalized words to match as prefixes, abbreviations expanded ("st" -> "saint").
  std::vector<std::string> terms;
  /// House number key (number_key()), empty when the text has none.
  std::string number;
};

/// Splits a query into words and an optional house number. The number is the first
/// word that starts with a digit and has at most 4 digits (5 digits is a postcode, kept
/// as a word), plus a following "bis", "ter", "quater" or letter a to d. It is only taken
/// when other words remain. Abbreviations that are not a prefix of their word are
/// expanded (st, ste, r, bd, rte, fg); the others ("av") already match as prefixes.
/// @param text UTF-8 query.
/// @return Terms and number.
Vrst_search_query parse_query(std::string_view text);

}  // namespace nav
