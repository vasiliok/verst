/// @file
/// Read-only memory-mapped file (POSIX + Windows). Throws std::runtime_error.
#pragma once

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
  Vrst_mapped_file& operator=(Vrst_mapped_file&& o) noexcept {
    if (this != &o) {
      close();
      swap(o);
    }
    return *this;
  }

  /// Maps a file read-only, closing any previous mapping first. An empty file gives
  /// size() 0 and data() nullptr.
  /// @param path File to map.
  /// @throws std::runtime_error if the file cannot be opened, stat'ed or mapped.
  void open(const std::string& path) {
    close();
#ifdef _WIN32
    HANDLE f = CreateFileA(path.c_str(), GENERIC_READ, FILE_SHARE_READ, nullptr, OPEN_EXISTING,
                           FILE_ATTRIBUTE_NORMAL, nullptr);
    if (f == INVALID_HANDLE_VALUE) throw std::runtime_error("cannot open " + path);
    LARGE_INTEGER sz;
    if (!GetFileSizeEx(f, &sz)) {
      CloseHandle(f);
      throw std::runtime_error("cannot stat " + path);
    }
    m_size = static_cast<Vrst_size>(sz.QuadPart);
    if (m_size > 0) {
      HANDLE m = CreateFileMappingA(f, nullptr, PAGE_READONLY, 0, 0, nullptr);
      CloseHandle(f);
      if (!m) throw std::runtime_error("cannot map " + path);
      m_data = static_cast<const Vrst_uint8*>(MapViewOfFile(m, FILE_MAP_READ, 0, 0, 0));
      CloseHandle(m);
      if (!m_data) throw std::runtime_error("cannot map " + path);
    } else {
      CloseHandle(f);
    }
#else
    int fd = ::open(path.c_str(), O_RDONLY);
    if (fd < 0) throw std::runtime_error("cannot open " + path);
    struct stat st;
    if (fstat(fd, &st) != 0) {
      ::close(fd);
      throw std::runtime_error("cannot stat " + path);
    }
    m_size = static_cast<Vrst_size>(st.st_size);
    if (m_size > 0) {
      void* p = mmap(nullptr, m_size, PROT_READ, MAP_SHARED, fd, 0);
      ::close(fd);
      if (p == MAP_FAILED) throw std::runtime_error("cannot map " + path);
      m_data = static_cast<const Vrst_uint8*>(p);
    } else {
      ::close(fd);
    }
#endif
    m_path = path;
  }

  /// Unmaps the file and returns to the closed state. Safe to call when already closed.
  void close() {
    if (m_data) {
#ifdef _WIN32
      UnmapViewOfFile(m_data);
#else
      munmap(const_cast<Vrst_uint8*>(m_data), m_size);
#endif
    }
    m_data = nullptr;
    m_size = 0;
    m_path.clear();
  }

  /// Start of the mapping.
  /// @return First byte, nullptr when closed or for an empty file.
  const Vrst_uint8* data() const { return m_data; }
  /// Size of the mapping.
  /// @return Mapped length in bytes, 0 when closed.
  Vrst_size size() const { return m_size; }
  /// Path of the mapped file.
  /// @return Path given to open(), empty when closed.
  const std::string& path() const { return m_path; }

 private:
  /// Exchanges state with another mapping; the building block of the move operations.
  /// @param o Other mapping.
  void swap(Vrst_mapped_file& o) noexcept {
    std::swap(m_data, o.m_data);
    std::swap(m_size, o.m_size);
    std::swap(m_path, o.m_path);
  }

  /// Start of the read-only mapping, nullptr when closed or for an empty file. All
  /// typed views point into it, so it must outlive them.
  const Vrst_uint8* m_data = nullptr;
  /// Mapped length in bytes. munmap needs it, and every section bound is checked
  /// against it.
  Vrst_size m_size = 0;
  /// For error messages only: with four files open, "bad magic" alone does not say
  /// which file is broken.
  std::string m_path;
};

}  // namespace nav
