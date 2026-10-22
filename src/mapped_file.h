// Read-only memory-mapped file (POSIX + Windows). Throws std::runtime_error.
#pragma once

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

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

class MappedFile {
 public:
  MappedFile() = default;
  explicit MappedFile(const std::string& path) { open(path); }
  ~MappedFile() { close(); }

  MappedFile(const MappedFile&) = delete;
  MappedFile& operator=(const MappedFile&) = delete;
  MappedFile(MappedFile&& o) noexcept { swap(o); }
  MappedFile& operator=(MappedFile&& o) noexcept {
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
    size_ = static_cast<size_t>(sz.QuadPart);
    if (size_ > 0) {
      HANDLE m = CreateFileMappingA(f, nullptr, PAGE_READONLY, 0, 0, nullptr);
      CloseHandle(f);
      if (!m) throw std::runtime_error("cannot map " + path);
      data_ = static_cast<const uint8_t*>(MapViewOfFile(m, FILE_MAP_READ, 0, 0, 0));
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
    size_ = static_cast<size_t>(st.st_size);
    if (size_ > 0) {
      void* p = mmap(nullptr, size_, PROT_READ, MAP_SHARED, fd, 0);
      ::close(fd);
      if (p == MAP_FAILED) throw std::runtime_error("cannot map " + path);
      data_ = static_cast<const uint8_t*>(p);
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
      munmap(const_cast<uint8_t*>(data_), size_);
#endif
    }
    data_ = nullptr;
    size_ = 0;
    path_.clear();
  }

  const uint8_t* data() const { return data_; }
  size_t size() const { return size_; }
  const std::string& path() const { return path_; }

 private:
  void swap(MappedFile& o) noexcept {
    std::swap(data_, o.data_);
    std::swap(size_, o.size_);
    std::swap(path_, o.path_);
  }

  const uint8_t* data_ = nullptr;
  size_t size_ = 0;
  std::string path_;
};

}  // namespace nav
