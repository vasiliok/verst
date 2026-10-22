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
  Vrst_mapped_file() = default;
  explicit Vrst_mapped_file(const std::string& path) { open(path); }
  ~Vrst_mapped_file() { close(); }

  Vrst_mapped_file(const Vrst_mapped_file&) = delete;
  Vrst_mapped_file& operator=(const Vrst_mapped_file&) = delete;
  Vrst_mapped_file(Vrst_mapped_file&& o) noexcept { swap(o); }
  Vrst_mapped_file& operator=(Vrst_mapped_file&& o) noexcept {
    if (this != &o) {
      close();
      swap(o);
    }
    return *this;
  }

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
    size_ = static_cast<Vrst_size>(sz.QuadPart);
    if (size_ > 0) {
      HANDLE m = CreateFileMappingA(f, nullptr, PAGE_READONLY, 0, 0, nullptr);
      CloseHandle(f);
      if (!m) throw std::runtime_error("cannot map " + path);
      data_ = static_cast<const Vrst_uint8*>(MapViewOfFile(m, FILE_MAP_READ, 0, 0, 0));
      CloseHandle(m);
      if (!data_) throw std::runtime_error("cannot map " + path);
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
    size_ = static_cast<Vrst_size>(st.st_size);
    if (size_ > 0) {
      void* p = mmap(nullptr, size_, PROT_READ, MAP_SHARED, fd, 0);
      ::close(fd);
      if (p == MAP_FAILED) throw std::runtime_error("cannot map " + path);
      data_ = static_cast<const Vrst_uint8*>(p);
    } else {
      ::close(fd);
    }
#endif
    path_ = path;
  }

  void close() {
    if (data_) {
#ifdef _WIN32
      UnmapViewOfFile(data_);
#else
      munmap(const_cast<Vrst_uint8*>(data_), size_);
#endif
    }
    data_ = nullptr;
    size_ = 0;
    path_.clear();
  }

  const Vrst_uint8* data() const { return data_; }
  Vrst_size size() const { return size_; }
  const std::string& path() const { return path_; }

 private:
  void swap(Vrst_mapped_file& o) noexcept {
    std::swap(data_, o.data_);
    std::swap(size_, o.size_);
    std::swap(path_, o.path_);
  }

  /// Start of the read-only mapping, nullptr when closed or for an empty file. All
  /// typed views point into it, so it must outlive them.
  const Vrst_uint8* data_ = nullptr;
  /// Mapped length in bytes. munmap needs it, and every section bound is checked
  /// against it.
  Vrst_size size_ = 0;
  /// For error messages only: with four files open, "bad magic" alone does not say
  /// which file is broken.
  std::string path_;
};

}  // namespace nav
