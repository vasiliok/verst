/// @file
/// Where a client (a front end, the renderer) gets map data from: one abstract
/// provider with a small API in the client's terms.
///
/// The client never learns how the data is stored or made: no files, no memory
/// mapping, no on-disk formats, no road graph, no tile grid and no threads. It opens a
/// map at an opaque location, says what the screen shows, draws the pieces the
/// provider tells it to and collects new pieces when they are ready. Status and
/// events report the provider's work (how busy, how many threads, when loading is
/// complete) for overlays and the session log.
///
/// Typical frame:
/// @code
/// for (Vrst_map_piece& p : map.take_ready(8)) renderer.add(p, frame);
/// const auto needs = map.show(area, [&](Vrst_uint64 k) { return renderer.has(k); });
/// // draw each need's key, or the first of its fallbacks the renderer has
/// @endcode
#pragma once

#include <array>
#include <functional>
#include <memory>
#include <optional>
#include <string_view>
#include <string>
#include <vector>

#include "geo.h"
#include "map_kinds.h"
#include "map_piece.h"
#include "search_result.h"
#include "vrst_types.h"

namespace nav {

/// A named place for labels. An owned copy: valid as long as the client keeps it.
struct Vrst_map_place {
  Vrst_latlon pos;              ///< Label anchor.
  PlaceKind kind{};             ///< City, town, village or hamlet.
  Vrst_uint32 population = 0;   ///< 0 = unknown; orders places of the same kind.
  std::string name;             ///< UTF-8 name.
};

/// What the screen shows now.
struct Vrst_map_area {
  /// Screen corners, absolute Web Mercator metres; four corners so a rotated view is
  /// covered.
  std::array<Vrst_vec2d, 4> corners{};
  /// Screen center, absolute Mercator metres; pieces near it load first.
  Vrst_vec2d center{};
  /// Mercator metres per framebuffer pixel; picks the detail of the pieces.
  Vrst_float64 metres_per_px = 0;
};

/// One piece of the map to draw this frame: draw `key` when the client has it,
/// otherwise the first of `fallbacks` it has (coarser pieces covering the same
/// ground), so the map never shows holes while pieces load.
struct Vrst_piece_need {
  Vrst_uint64 key = 0;                  ///< The piece wanted at this detail.
  std::vector<Vrst_uint64> fallbacks;   ///< Coarser pieces, best first.
};

/// What one worker thread of the provider is doing now.
struct Vrst_worker_info {
  bool busy = false;          ///< Working on a piece.
  Vrst_uint64 key = 0;        ///< That piece, when busy.
  Vrst_float64 busy_ms = 0;   ///< How long it has been on it, when busy.
  Vrst_size done = 0;         ///< Pieces this worker finished since start.
  std::string step;           ///< When busy: what it does inside the piece, free text for logs.
};

/// Snapshot of the provider's work, for overlays and the session log.
struct Vrst_provider_status {
  Vrst_size threads = 0;        ///< Worker threads.
  Vrst_size queued = 0;         ///< Pieces waiting for a worker.
  Vrst_size working = 0;        ///< Pieces being made now.
  Vrst_size ready = 0;          ///< Finished, not taken by take_ready() yet.
  Vrst_size done_total = 0;     ///< Finished since start.
  Vrst_size failed_total = 0;   ///< Failed since start; asked again on the next show().
  Vrst_size dropped_total = 0;  ///< Queued, then no longer needed.
  Vrst_size missing = 0;        ///< Pieces the last show() needed that the client lacked.
  /// Nothing queued, working or waiting for take_ready(), and the last show() had no
  /// missing piece: everything on screen is loaded.
  bool settled = false;
  std::vector<Vrst_worker_info> workers;  ///< One entry per worker thread.
};

/// Kind of a Vrst_work_event.
enum class Vrst_work_event_kind {
  Started,  ///< A worker began a piece.
  Done,     ///< A worker finished a piece.
  Failed,   ///< A worker could not make a piece.
  Settled,  ///< Everything on screen is loaded (Vrst_provider_status::settled became true).
};

/// One thing that happened in the provider.
struct Vrst_work_event {
  Vrst_work_event_kind kind = Vrst_work_event_kind::Started;  ///< What happened.
  Vrst_uint64 key = 0;      ///< The piece; 0 for Settled.
  int worker = 0;           ///< Which worker thread; 0 for Settled.
  Vrst_float64 ms = 0;      ///< Done / Failed: time spent on the piece.
  Vrst_float64 age_ms = 0;  ///< How long before take_events() it happened.
  std::string detail;       ///< Done / Failed: free text for logs (e.g. the slowest part of the piece).
};

/// Asks the client whether it holds a piece (Vrst_map_provider::show()).
using Vrst_has_piece = std::function<bool(Vrst_uint64 key)>;

/// Where the client gets map data from. Non-blocking: show() returns at once and the
/// pieces arrive later through take_ready(). Call from one thread (the client's).
class Vrst_map_provider {
 public:
  Vrst_map_provider() = default;                                      ///< Interface: nothing to set up.
  virtual ~Vrst_map_provider() = default;                             ///< Stops any background work.
  Vrst_map_provider(const Vrst_map_provider&) = delete;               ///< Owns work in flight: not copyable.
  Vrst_map_provider& operator=(const Vrst_map_provider&) = delete;    ///< Owns work in flight: not copyable.
  Vrst_map_provider(Vrst_map_provider&&) = delete;                    ///< Workers may point at it: not movable.
  Vrst_map_provider& operator=(Vrst_map_provider&&) = delete;         ///< Workers may point at it: not movable.

  /// Area the map covers.
  /// @return Box in degrees.
  virtual Vrst_geo_box extent() const = 0;

  /// Named places for labels.
  /// @return Places, most important first (kind, then population).
  virtual std::vector<Vrst_map_place> places() const = 0;

  /// Tells the provider what the screen shows. Starts making the pieces the client
  /// lacks, nearest to the screen center first, and drops queued pieces that are no
  /// longer needed.
  /// @param area What the screen shows.
  /// @param has Whether the client holds a piece.
  /// @return Pieces to draw, nearest to the screen center first.
  virtual std::vector<Vrst_piece_need> show(const Vrst_map_area& area, const Vrst_has_piece& has) = 0;

  /// Takes pieces that became ready.
  /// @param max At most this many; uploading them is the client's per-frame cost.
  /// @return Ready pieces, oldest first.
  virtual std::vector<Vrst_map_piece> take_ready(Vrst_size max) = 0;

  /// Snapshot of the work: threads, busy workers, queue, missing, settled.
  /// @return Status now.
  virtual Vrst_provider_status status() const = 0;

  /// Takes the work events recorded since the last call (only with
  /// Vrst_map_options::record_events).
  /// @return Events, oldest first, with their age.
  virtual std::vector<Vrst_work_event> take_events() = 0;

  /// Debug name of a piece, for logs and overlays (e.g. "10/523/356").
  /// @param key Piece key.
  /// @return Name; the client prints it without reading it.
  virtual std::string piece_name(Vrst_uint64 key) const = 0;

  /// Free-text lines about the data, for debug overlays (sizes, layers present).
  /// @return Lines; the client prints them without reading them.
  virtual std::vector<std::string> describe() const = 0;

  /// Everything known about what one worker is doing, for a log record when it seems
  /// stuck: its step and, when it has one, the exact input it is working on, so the
  /// problem can be reproduced elsewhere.
  /// @param worker Worker index (Vrst_provider_status::workers).
  /// @return Free-text lines "name: value"; the client logs them without reading them.
  virtual std::vector<std::string> work_dump(int worker) const = 0;

  /// Whether the map has an address search index (maps made before it have none).
  /// @return true when search() can find anything.
  virtual bool has_search() const = 0;
  /// Finds places, streets and house numbers ("12 rue de la liberte dijon"), or reads a
  /// point from a map link or coordinates (a Google Maps place or coordinate link, an
  /// OSM or geo: link, "47.32, 5.04"); a link with only a place text (a shared Google
  /// place) is searched by its address. Short links (maps.app.goo.gl) must be opened by
  /// the client first. Fast enough to call on every keystroke.
  /// @param text What the user typed.
  /// @param near Map center or GPS position: nearer results first, distances in the
  ///        detail; nullopt = none.
  /// @param max At most this many results.
  /// @return Results, best first; empty without an index or a word.
  virtual std::vector<Vrst_search_result> search(std::string_view text, std::optional<Vrst_latlon> near,
                                                 Vrst_size max) const = 0;
};

/// Settings a client may give when opening a map; none of them is about storage.
struct Vrst_map_options {
  int threads = 2;             ///< Worker threads, at least 1.
  bool record_events = false;  ///< Record work events for take_events().
};

/// Opens a map.
/// @param location Where the map is; opaque to the client (today the folder with the
///        bundled map data).
/// @param options Threads and event recording.
/// @return The provider.
/// @throws std::runtime_error when nothing usable is at the location.
std::unique_ptr<Vrst_map_provider> open_map(const std::string& location, const Vrst_map_options& options = {});

/// The provider's status as text for debug overlays, the same in every front end:
/// - line 1: loading progress, or "loading done" when settled;
/// - line 2: busy workers of all threads and the totals (built, failed, dropped);
/// - then one line per worker: the piece it works on and for how long, or idle; a busy
///   worker gets a second line with its step inside the piece.
/// @param s Status from Vrst_map_provider::status().
/// @param map The provider, for piece names.
/// @return Lines.
std::vector<std::string> status_lines(const Vrst_provider_status& s, const Vrst_map_provider& map);

}  // namespace nav
