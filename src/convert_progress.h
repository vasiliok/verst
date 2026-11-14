/// @file
/// Live progress and cancel switch of the map pipeline (merge extracts, convert to nav
/// files), shared by the desktop tools (osm2nav, pbf_merge) and the iPhone app.
///
/// The working thread reports its phase and how far through the phase it is; any other
/// thread reads them (a progress bar) or asks it to stop. Stopping is cooperative: the
/// pipeline calls check() between blocks of the input and throws
/// Vrst_convert_cancelled, so it never stops in the middle of writing a file.
#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <stdexcept>
#include <utility>

#include "vrst_types.h"

namespace nav {

/// Phase of the pipeline, in the order they run.
enum class Vrst_convert_phase : int {
  Idle,        ///< Not started.
  Merge,       ///< Several extracts into one file (pbf_merge).
  Ways,        ///< Road ways (convert pass 1).
  NodeTable,   ///< Road node table.
  Nodes,       ///< Road node coordinates and places (pass 2).
  Segments,    ///< Ways cut into segments.
  Graph,       ///< Arcs and the two CSR tables.
  WriteRoads,  ///< graph.dat, edges.dat, geom.dat, names.dat.
  Relations,   ///< Multipolygon relations.
  AreaWays,    ///< Which nodes areas and lines need.
  AreaNodes,   ///< Locations of those nodes.
  Areas,       ///< Areas assembled, triangulated; lines.
  WriteMap,    ///< map.dat.
  Done,        ///< Finished.
};
/// Number of phases.
inline constexpr int kConvertPhaseCount = static_cast<int>(Vrst_convert_phase::Done) + 1;

/// Short name of a phase, for logs and the app ("areas").
/// @param p Phase.
/// @return Name.
inline const char* phase_name(Vrst_convert_phase p) {
  static constexpr const char* kNames[] = {"idle",      "merge",     "ways",       "node table", "nodes",
                                           "segments",  "graph",     "write roads", "relations", "area ways",
                                           "area nodes", "areas",    "write map",  "done"};
  static_assert(std::size(kNames) == kConvertPhaseCount);
  return kNames[static_cast<int>(p)];
}

/// Thrown by Vrst_convert_progress::check() after cancel().
class Vrst_convert_cancelled : public std::runtime_error {
 public:
  Vrst_convert_cancelled() : std::runtime_error("cancelled") {}  ///< "cancelled".
};

/// Progress of one pipeline run; see the file comment. Written by the working thread,
/// read and cancelled from any thread.
class Vrst_convert_progress {
 public:
  /// Called on the working thread when a phase ends.
  using Vrst_phase_done = std::function<void(Vrst_convert_phase, Vrst_float64 ms)>;

  /// Sets the function called at the end of every phase (osm2nav prints a line, the
  /// app logs a record). Set before the run starts.
  /// @param fn Callback; may be empty.
  void on_phase_done(Vrst_phase_done fn) { m_on_done = std::move(fn); }

  /// Ends the current phase (calls the callback) and starts the next one at 0.
  /// @param p New phase.
  /// @throws Vrst_convert_cancelled when cancel() was called.
  void begin(Vrst_convert_phase p) {
    end_phase();
    m_fraction.store(0, std::memory_order_relaxed);
    m_phase.store(static_cast<int>(p), std::memory_order_relaxed);
    check();
  }
  /// Ends the last phase; phase() is Done.
  void finish() { begin(Vrst_convert_phase::Done); }

  /// How far the current phase is.
  /// @param f 0..1.
  void set_fraction(Vrst_float64 f) { m_fraction.store(f, std::memory_order_relaxed); }

  /// Asks the run to stop at its next check().
  void cancel() { m_cancel.store(true, std::memory_order_relaxed); }
  /// @return true after cancel().
  bool cancelled() const { return m_cancel.load(std::memory_order_relaxed); }
  /// Stops the run when it was cancelled.
  /// @throws Vrst_convert_cancelled after cancel().
  void check() const {
    if (cancelled()) throw Vrst_convert_cancelled();
  }

  /// @return The current phase.
  Vrst_convert_phase phase() const { return static_cast<Vrst_convert_phase>(m_phase.load(std::memory_order_relaxed)); }
  /// @return How far the current phase is, 0..1.
  Vrst_float64 fraction() const { return m_fraction.load(std::memory_order_relaxed); }

 private:
  /// Clock of the phase times.
  using Vrst_clock = std::chrono::steady_clock;

  /// Reports the end of the current phase, if one is running.
  void end_phase() {
    const auto now = Vrst_clock::now();
    const auto p = phase();
    if (p != Vrst_convert_phase::Idle && p != Vrst_convert_phase::Done && m_on_done)
      m_on_done(p, std::chrono::duration<Vrst_float64, std::milli>(now - m_start).count());
    m_start = now;
  }

  /// Vrst_convert_phase as int.
  std::atomic<int> m_phase{0};
  /// Fraction of the current phase.
  std::atomic<Vrst_float64> m_fraction{0};
  /// cancel() was called.
  std::atomic<bool> m_cancel{false};
  /// Start of the current phase (working thread only).
  Vrst_clock::time_point m_start = Vrst_clock::now();
  /// Phase end callback (working thread only).
  Vrst_phase_done m_on_done;
};

}  // namespace nav
