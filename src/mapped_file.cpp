/// @file
/// Vrst_mapped_file: mapping and unmapping (POSIX and Windows).
/// Declarations and documentation are in mapped_file.h.
#include "mapped_file.h"

namespace nav {

Vrst_mapped_file& Vrst_mapped_file::operator=(Vrst_mapped_file&& o) noexcept {
  if (this != &o) {
    close();
    swap(o);
  }
  return *this;
}

void Vrst_mapped_file::open(const std::string& path) {
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
  const auto size = static_cast<Vrst_size>(sz.QuadPart);
  if (size > 0) {
    HANDLE m = CreateFileMappingA(f, nullptr, PAGE_READONLY, 0, 0, nullptr);
    CloseHandle(f);
    if (!m) throw std::runtime_error("cannot map " + path);
    const LPVOID p = MapViewOfFile(m, FILE_MAP_READ, 0, 0, 0);
    CloseHandle(m);
    if (!p) throw std::runtime_error("cannot map " + path);
    m_bytes = {static_cast<const Vrst_uint8*>(p), size};
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
  const auto size = static_cast<Vrst_size>(st.st_size);
  if (size > 0) {
    const auto p = mmap(nullptr, size, PROT_READ, MAP_SHARED, fd, 0);
    ::close(fd);
    if (p == MAP_FAILED) throw std::runtime_error("cannot map " + path);
    m_bytes = {static_cast<const Vrst_uint8*>(p), size};
  } else {
    ::close(fd);
  }
#endif
  m_path = path;
}

void Vrst_mapped_file::close() {
  if (!m_bytes.empty()) {
#ifdef _WIN32
    UnmapViewOfFile(m_bytes.data());
#else
    munmap(const_cast<Vrst_uint8*>(m_bytes.data()), m_bytes.size());
#endif
  }
  m_bytes = {};
  m_path.clear();
}

}  // namespace nav
