/// @file
/// Session log: one JSON Lines file per app run, for finding out afterwards what the app
/// did while the user moved the map (frame timing, touches, gestures, tile builds, stalls).
///
/// Records are collected in memory and written to the file when flush_if_due() finds the
/// last write older than kFlushSeconds, so logging never waits for the disk inside a frame.
/// Every record starts with its type and `t`, seconds since the run started; the caller
/// passes `t`, so the log, the on-screen clock and the touch timestamps share one clock.
/// Not thread-safe: use it from one thread (the main thread on iOS).
#pragma once

#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <vector>

#include "vrst_types.h"

namespace nav {

class Vrst_session_log;

/// One record being written; appended to the log when it goes out of scope. Built with
/// chained calls: `log.record("frame", t).num("ms", 4.2).text("kind", "pan");`.
class Vrst_log_record {
 public:
  /// Starts a record.
  /// @param log Log the record is appended to; must outlive the record.
  /// @param type Record type, e.g. "frame".
  /// @param t Seconds since the run started.
  Vrst_log_record(Vrst_session_log& log, std::string_view type, Vrst_float64 t);
  /// Closes the record and appends it to the log.
  ~Vrst_log_record();
  Vrst_log_record(const Vrst_log_record&) = delete;             ///< Appends once: not copyable.
  Vrst_log_record& operator=(const Vrst_log_record&) = delete;  ///< Appends once: not copyable.
  Vrst_log_record(Vrst_log_record&&) = delete;                  ///< Appends once: not movable.
  Vrst_log_record& operator=(Vrst_log_record&&) = delete;       ///< Appends once: not movable.

  /// Adds a number with a fixed number of decimals.
  /// @param key Field name.
  /// @param v Value.
  /// @param decimals Digits after the point; 0 writes an integer.
  /// @return This record, for chaining.
  Vrst_log_record& num(std::string_view key, Vrst_float64 v, int decimals = 3);
  /// Adds an integer.
  /// @param key Field name.
  /// @param v Value.
  /// @return This record, for chaining.
  Vrst_log_record& count(std::string_view key, Vrst_uint64 v);
  /// Adds a text, JSON-escaped.
  /// @param key Field name.
  /// @param v Value.
  /// @return This record, for chaining.
  Vrst_log_record& text(std::string_view key, std::string_view v);
  /// Adds a true/false value.
  /// @param key Field name.
  /// @param v Value.
  /// @return This record, for chaining.
  Vrst_log_record& flag(std::string_view key, bool v);

 private:
  /// Starts a field: separator and quoted key.
  /// @param key Field name.
  void key(std::string_view key);

  /// Log the record goes to.
  Vrst_session_log& m_log;
  /// JSON text built so far, without the closing brace.
  std::string m_line;
};

/// Session log file; see the file comment.
class Vrst_session_log {
 public:
  /// Seconds between two writes to the file.
  static constexpr Vrst_float64 kFlushSeconds = 1.0;

  Vrst_session_log() = default;  ///< Closed; open() starts a session.
  /// Writes what is left and closes the file.
  ~Vrst_session_log();
  Vrst_session_log(const Vrst_session_log&) = delete;             ///< Owns the file: not copyable.
  Vrst_session_log& operator=(const Vrst_session_log&) = delete;  ///< Owns the file: not copyable.
  Vrst_session_log(Vrst_session_log&&) = delete;                  ///< Records point at it: not movable.
  Vrst_session_log& operator=(Vrst_session_log&&) = delete;       ///< Records point at it: not movable.

  /// Starts a new session file `session-<YYYYMMDD-HHMMSS>.jsonl` in a directory and
  /// deletes the oldest session files beyond the newest `keep`.
  /// @param dir Directory; created if missing.
  /// @param keep Session files to keep, the new one included.
  /// @throws std::runtime_error when the file cannot be created.
  void open(const std::filesystem::path& dir, Vrst_size keep);

  /// Starts a record; it is appended when the returned object goes out of scope.
  /// @param type Record type.
  /// @param t Seconds since the run started.
  /// @return The record, for chained fields.
  Vrst_log_record record(std::string_view type, Vrst_float64 t) { return {*this, type, t}; }

  /// Writes the buffer to the file when the last write is kFlushSeconds old.
  /// @param t Seconds since the run started.
  void flush_if_due(Vrst_float64 t);
  /// Writes the buffer to the file now (before sharing the file, or on going to background).
  void flush();

  /// Session files in the directory, oldest first.
  /// @return Paths; empty when open() was not called.
  std::vector<std::filesystem::path> files() const;
  /// Bytes of all session files, the buffer included.
  /// @return Bytes.
  Vrst_uint64 total_bytes() const;
  /// Deletes every session file except the current one.
  void delete_old();
  /// @return Path of the current session file; empty when not open.
  const std::filesystem::path& path() const { return m_path; }

 private:
  friend class Vrst_log_record;
  /// Appends one finished line to the buffer.
  /// @param line JSON object without the newline.
  void append(std::string_view line);

  /// Directory of the session files.
  std::filesystem::path m_dir;
  /// Current session file.
  std::filesystem::path m_path;
  /// Open file, appended to by flush().
  std::ofstream m_file;
  /// Lines not written yet.
  std::string m_buffer;
  /// `t` of the last write.
  Vrst_float64 m_flushed_t = 0;
};

}  // namespace nav
