/// @file
/// merge_pbf(): streaming merge of sorted OSM extracts.
/// Declarations and documentation are in pbf_merge.h.
#include "pbf_merge.h"

#include <cstdio>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <utility>

#include <osmium/io/input_iterator.hpp>
#include <osmium/io/pbf_input.hpp>
#include <osmium/io/pbf_output.hpp>
#include <osmium/memory/buffer.hpp>
#include <osmium/osm/object.hpp>

namespace nav {

namespace {

/// (type, id) of an object: the sort key of an extract (node < way < relation, then id).
using Vrst_object_key = std::pair<int, osmium::object_id_type>;

/// Sort key of an object.
/// @param o Object.
/// @return Key.
Vrst_object_key key_of(const osmium::OSMObject& o) {
  return {static_cast<int>(o.type()), o.id()};
}

/// One input being read: its reader and its position.
struct Vrst_merge_input {
  using Iter = osmium::io::InputIterator<osmium::io::Reader, osmium::OSMObject>;  ///< Object iterator.
  std::unique_ptr<osmium::io::Reader> reader;  ///< Reader; owned, iterators point into it.
  Iter it;                                     ///< Next object.
  Iter end;                                    ///< End.
  Vrst_object_key last{0, 0};                  ///< Key of the last object taken, to check the order.
};

/// Bytes of a buffer worth writing out.
constexpr Vrst_size kFlushBytes = Vrst_size{1} << 23;

}  // namespace

Vrst_merge_stats merge_pbf(const std::vector<std::string>& inputs, const std::string& output,
                           Vrst_convert_progress& progress) {
  progress.begin(Vrst_convert_phase::Merge);
  std::vector<Vrst_merge_input> in;
  osmium::io::Header header;
  for (const std::string& path : inputs) {
    auto r = std::make_unique<osmium::io::Reader>(path);
    for (const osmium::Box& b : r->header().boxes()) header.add_box(b);
    Vrst_merge_input::Iter it{*r};
    in.push_back({std::move(r), it, {}, {0, 0}});
  }
  header.set("generator", "verst merge_pbf");
  Vrst_merge_stats st;
  try {
    osmium::io::Writer writer{output, header, osmium::io::overwrite::allow};
    osmium::memory::Buffer buffer{2 * kFlushBytes, osmium::memory::Buffer::auto_grow::yes};
    for (;;) {
      // The smallest (type, id) over all inputs; of equal ones the highest version.
      const osmium::OSMObject* best = nullptr;
      for (const Vrst_merge_input& s : in)
        if (s.it != s.end && (!best || key_of(*s.it) < key_of(*best))) best = &*s.it;
      if (!best) break;
      for (const Vrst_merge_input& s : in)
        if (s.it != s.end && key_of(*s.it) == key_of(*best) && s.it->version() > best->version()) best = &*s.it;
      buffer.add_item(*best);
      buffer.commit();
      ++st.written;
      const Vrst_object_key k = key_of(*best);
      int same = 0;
      for (Vrst_size i = 0; i < in.size(); ++i) {
        Vrst_merge_input& s = in[i];
        if (s.it == s.end || key_of(*s.it) != k) continue;
        if (k < s.last) throw std::runtime_error("not sorted like a Geofabrik extract: " + inputs[i]);
        s.last = k;
        ++s.it;
        ++same;
      }
      st.duplicates += static_cast<Vrst_uint64>(same - 1);
      if (buffer.committed() > kFlushBytes) {
        writer(std::move(buffer));
        buffer = osmium::memory::Buffer{2 * kFlushBytes, osmium::memory::Buffer::auto_grow::yes};
        // Progress of the first input stands for all: they are read side by side.
        const osmium::io::Reader& r0 = *in.front().reader;
        if (r0.file_size() > 0)
          progress.set_fraction(static_cast<Vrst_float64>(r0.offset()) / static_cast<Vrst_float64>(r0.file_size()));
        progress.check();
      }
    }
    writer(std::move(buffer));
    writer.close();
  } catch (...) {
    std::error_code ec;
    std::filesystem::remove(output, ec);  // no half-written merge left behind
    throw;
  }
  for (Vrst_merge_input& s : in) s.reader->close();
  return st;
}

}  // namespace nav
