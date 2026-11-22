/// @file
/// Vrst_string_pool: strings interned to dense ids, used by the converter for names.dat
/// and for the address index.
#pragma once

#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "vrst_types.h"

namespace nav {

/// Interns strings to dense ids, id 0 = "".
/// Thousands of segments or addresses share a few hundred names, so each name is
/// stored once and the records carry a 4-byte id.
class Vrst_string_pool {
 public:
  /// Creates the pool with the empty string as id 0.
  Vrst_string_pool() { m_strings.emplace_back(); }
  /// Adds a string once.
  /// @param s String to add.
  /// @return Its id; equal strings get the same id and "" is always 0.
  Vrst_uint32 intern(std::string_view s) {
    if (s.empty()) return 0;
    auto [it, inserted] = m_ids.try_emplace(std::string(s), static_cast<Vrst_uint32>(m_strings.size()));
    if (inserted) m_strings.emplace_back(s);
    return it->second;
  }
  /// All strings, ready for names.dat.
  /// @return Strings in id order.
  const std::vector<std::string>& strings() const { return m_strings; }
  /// One string.
  /// @param id Id from intern().
  /// @return The string.
  const std::string& str(Vrst_uint32 id) const { return m_strings[id]; }

 private:
  /// String -> id for deduplication. Thousands of segments share a few names; without
  /// it names.dat would hold one copy per way.
  std::unordered_map<std::string, Vrst_uint32> m_ids;
  /// Strings in id order (id 0 = ""), written as names.dat. Needed besides m_ids
  /// because the map has no order.
  std::vector<std::string> m_strings;
};

}  // namespace nav
