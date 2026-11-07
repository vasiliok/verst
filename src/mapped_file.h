/// @file
/// Read-only memory-mapped file (POSIX + Windows). Throws std::runtime_error.
#pragma once

#include <span>
#include <stdexcept>
#include <string>
#include <utility>

#include "vrst_types.h"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace nav {

/// Read-only memory mapping of a whole file, with POSIX and Windows back ends.
/// The nav files are used in place: the OS pages in only what is touched, opening is
/// O(1) whatever the file size, and several processes share the same pages. Move-only
/// so exactly one owner unmaps; all spans into the data die with it.
class Vrst_mapped_file {
 public:
  /// Creates a closed mapping; call open() before use.
  Vrst_mapped_file() = default;
  /// Maps a file at once.
  /// @param path File to map read-only.
  /// @throws std::runtime_error if the file cannot be opened, stat'ed or mapped.
  explicit Vrst_mapped_file(const std::string& path) { open(path); }
  /// Unmaps the file; spans into it become invalid.
  ~Vrst_mapped_file() { close(); }

  /// Not copyable: two owners would unmap the same pages twice.
  Vrst_mapped_file(const Vrst_mapped_file&) = delete;
  /// Not copyable: two owners would unmap the same pages twice.
  Vrst_mapped_file& operator=(const Vrst_mapped_file&) = delete;
  /// Takes over another mapping.
  /// @param o Source; it is left closed.
  Vrst_mapped_file(Vrst_mapped_file&& o) noexcept { swap(o); }
  /// Closes this mapping and takes over another one.
  /// @param o Source; it is left closed.
  /// @return *this.
  Vrst_mapped_file& operator=(Vrst_mapped_file&& o) noexcept;

  /// Maps a file read-only, closing any previous mapping first. An empty file gives an
  /// empty bytes().
  /// @param path File to map.
  /// @throws std::runtime_error if the file cannot be opened, stat'ed or mapped.
  void open(const std::string& path);

  /// Unmaps the file and returns to the closed state. Safe to call when already closed.
  void close();

  /// @copydoc m_bytes
  /// @return The mapped bytes, empty when closed or for an empty file.
  std::span<const Vrst_uint8> bytes() const { return m_bytes; }
  /// Size of the mapping.
  /// @return Mapped length in bytes, 0 when closed.
  Vrst_size size() const { return m_bytes.size(); }
  /// Path of the mapped file.
  /// @return Path given to open(), empty when closed.
  const std::string& path() const { return m_path; }

 private:
  /// Exchanges state with another mapping; the building block of the move operations.
  /// @param o Other mapping.
  void swap(Vrst_mapped_file& o) noexcept {
    std::swap(m_bytes, o.m_bytes);
    std::swap(m_path, o.m_path);
  }

  /// The read-only mapping, empty when closed or for an empty file. All typed views
  /// point into it, so it must outlive them; munmap needs its size, and every section
  /// bound is checked against it.
  std::span<const Vrst_uint8> m_bytes;
  /// For error messages only: with four files open, "bad magic" alone does not say
  /// which file is broken.
  std::string m_path;
};

}  // namespace nav
