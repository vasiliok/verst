/// @file
/// Today's implementation of Vrst_map_provider: the map is read from the nav files
/// (memory-mapped) and cut into z/x/y tiles that background threads build on demand.
/// Only open_map() and the tests see this class; clients use map_provider.h.
#pragma once

#include <chrono>
#include <string>
#include <vector>

#include "map_provider.h"
#include "map_tiles.h"
#include "nav_files.h"

namespace nav {

/// Map provider over the nav files: Vrst_nav_data, Vrst_tile_index,
/// Vrst_tile_builder and Vrst_tile_loader behind the client API. A piece is one tile;
/// its key is tile_key().
class Vrst_nav_map_provider final : public Vrst_map_provider {
 public:
  /// Opens the nav files, builds the tile index and starts the workers.
  /// @param dir Folder with graph.dat, edges.dat, geom.dat, names.dat (and map.dat).
  /// @param options Threads and event recording.
  /// @throws std::runtime_error when a file is missing or invalid.
  Vrst_nav_map_provider(const std::string& dir, const Vrst_map_options& options);

  Vrst_geo_box extent() const override { return m_extent; }
  std::vector<Vrst_map_place> places() const override;
  std::vector<Vrst_piece_need> show(const Vrst_map_area& area, const Vrst_has_piece& has) override;
  std::vector<Vrst_map_piece> take_ready(Vrst_size max) override;
  Vrst_provider_status status() const override;
  std::vector<Vrst_work_event> take_events() override;
  std::string piece_name(Vrst_uint64 key) const override;
  std::vector<std::string> describe() const override;
  std::vector<std::string> work_dump(int worker) const override;

 private:
  /// Clock of the Settled events.
  using Vrst_clock = std::chrono::steady_clock;

  /// Whether everything on screen is loaded, from a loader snapshot.
  /// @param s Loader status.
  /// @return true when nothing is queued, built or waiting and nothing was missing.
  bool settled(const Vrst_loader_status& s) const {
    return s.queued == 0 && s.building == 0 && s.done == 0 && m_missing == 0;
  }

  /// Mapped nav files.
  Vrst_nav_data m_data;
  /// Which roads, areas and lines touch which tile.
  Vrst_tile_index m_index;
  /// Builds tiles from m_data and m_index; read-only, shared by the workers.
  Vrst_tile_builder m_builder{m_data, m_index};
  /// Box of all road shape points, the area the map covers.
  Vrst_geo_box m_extent{};
  /// Time to open the files and to build the index, ms (describe()).
  Vrst_float64 m_open_ms = 0;
  Vrst_float64 m_index_ms = 0;  ///< See m_open_ms.
  /// Pieces the last show() needed that the client lacked.
  Vrst_size m_missing = 0;
  /// Settled after the last show(); a change to true records a Settled event.
  bool m_settled = false;
  /// Record events (Vrst_map_options::record_events).
  bool m_record = false;
  /// Settled moments not taken yet.
  std::vector<Vrst_clock::time_point> m_settled_at;
  /// Background workers; declared last so they stop before the data they read goes.
  Vrst_tile_loader m_loader;
};

}  // namespace nav
