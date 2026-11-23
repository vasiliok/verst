/// @file
/// Client-side helpers of the map provider API (map_provider.h).

#include "provider/map_provider.h"

#include <algorithm>
#include <array>
#include <cstdio>

namespace nav {

namespace {

/// Debug line buffer.
using Vrst_line_buf = std::array<char, 160>;

/// Text snprintf wrote into a buffer.
/// @param buf Buffer.
/// @param n snprintf result.
/// @return Text, cut at the buffer size.
std::string text(const Vrst_line_buf& buf, int n) {
  return {buf.data(), static_cast<Vrst_size>(std::clamp(n, 0, static_cast<int>(buf.size()) - 1))};
}

}  // namespace

std::vector<std::string> status_lines(const Vrst_provider_status& s, const Vrst_map_provider& map) {
  std::vector<std::string> out;
  Vrst_line_buf b{};
  out.push_back(s.settled ? std::string("loading done")
                          : text(b, std::snprintf(b.data(), b.size(), "loading: %zu missing, %zu queued, %zu working, %zu ready",
                                                  s.missing, s.queued, s.working, s.ready)));
  const Vrst_size busy = static_cast<Vrst_size>(std::ranges::count(s.workers, true, &Vrst_worker_info::busy));
  out.push_back(text(b, std::snprintf(b.data(), b.size(), "workers %zu of %zu busy, built %zu, failed %zu, dropped %zu",
                                      busy, s.threads, s.done_total, s.failed_total, s.dropped_total)));
  for (Vrst_size w = 0; w < s.workers.size(); ++w) {
    const Vrst_worker_info& i = s.workers[w];
    const int n = i.busy ? std::snprintf(b.data(), b.size(), "  w%zu %s for %.0f ms (done %zu)", w,
                                         map.piece_name(i.key).c_str(), i.busy_ms, i.done)
                         : std::snprintf(b.data(), b.size(), "  w%zu idle (done %zu)", w, i.done);
    out.push_back(text(b, n));
    if (i.busy && !i.step.empty()) out.push_back("     " + i.step);
  }
  return out;
}

}  // namespace nav
