/// @file
/// Vrst_mapped_file: mapping and unmapping (POSIX and Windows).
/// Declarations and documentation are in mapped_file.h.
#include "base/mapped_file.h"

namespace nav {

#ifndef _WIN32
namespace {

/// Owns a POSIX file descriptor and closes it when it goes out of scope.
class Vrst_fd {
 public:
  /// Takes ownership of a descriptor.
  /// @param fd Descriptor from ::open(); negative means none.
  explicit Vrst_fd(int fd) : m_fd(fd) {}
  /// Closes the descriptor, if any.
  ~Vrst_fd() {
    if (m_fd >= 0) ::close(m_fd);
  }
  Vrst_fd(const Vrst_fd&) = delete;             ///< Owns the descriptor: not copyable.
  Vrst_fd& operator=(const Vrst_fd&) = delete;  ///< Owns the descriptor: not copyable.
  Vrst_fd(Vrst_fd&&) = delete;                  ///< Scope guard: not movable.
  Vrst_fd& operator=(Vrst_fd&&) = delete;       ///< Scope guard: not movable.
  /// @return The descriptor; negative when open failed.
  int get() const { return m_fd; }

 private:
  /// The descriptor; negative when there is none.
  int m_fd;
};

}  // namespace
#endif

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
  const Vrst_fd fd(::open(path.c_str(), O_RDONLY));
  if (fd.get() < 0) throw std::runtime_error("cannot open " + path);
  struct stat st{};
  if (fstat(fd.get(), &st) != 0) throw std::runtime_error("cannot stat " + path);
  const auto size = static_cast<Vrst_size>(st.st_size);
  if (size > 0) {
    // The mapping stays valid after the descriptor is closed.
    const auto p = mmap(nullptr, size, PROT_READ, MAP_SHARED, fd.get(), 0);
    if (p == MAP_FAILED) throw std::runtime_error("cannot map " + path);
    m_bytes = {static_cast<const Vrst_uint8*>(p), size};
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
