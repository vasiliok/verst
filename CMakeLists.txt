cmake_minimum_required(VERSION 3.20)
project(verst LANGUAGES C CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(NAV_BUILD_VIEWER "Build navview (fetches GLFW)" ON)
set(NAV_TEST_PBF "${CMAKE_SOURCE_DIR}/data/andorra.osm.pbf" CACHE FILEPATH "PBF used by the CTest")

function(nav_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra)
  endif()
endfunction()

# --- scalar types: src/vrst_types.h.in -> <build>/generated/vrst_types.h -------
set(VRST_TYPE_INT8    "std::int8_t"   CACHE STRING "C++ type for Vrst_int8")
set(VRST_TYPE_INT16   "std::int16_t"  CACHE STRING "C++ type for Vrst_int16")
set(VRST_TYPE_INT32   "std::int32_t"  CACHE STRING "C++ type for Vrst_int32")
set(VRST_TYPE_INT64   "std::int64_t"  CACHE STRING "C++ type for Vrst_int64")
set(VRST_TYPE_UINT8   "std::uint8_t"  CACHE STRING "C++ type for Vrst_uint8")
set(VRST_TYPE_UINT16  "std::uint16_t" CACHE STRING "C++ type for Vrst_uint16")
set(VRST_TYPE_UINT32  "std::uint32_t" CACHE STRING "C++ type for Vrst_uint32")
set(VRST_TYPE_UINT64  "std::uint64_t" CACHE STRING "C++ type for Vrst_uint64")
set(VRST_TYPE_FLOAT32 "float"         CACHE STRING "C++ type for Vrst_float32")
set(VRST_TYPE_FLOAT64 "double"        CACHE STRING "C++ type for Vrst_float64")
set(VRST_TYPE_SIZE    "std::size_t"   CACHE STRING "C++ type for Vrst_size")
configure_file(src/vrst_types.h.in "${CMAKE_BINARY_DIR}/generated/vrst_types.h" @ONLY)
add_library(vrst_types INTERFACE)
target_include_directories(vrst_types INTERFACE "${CMAKE_BINARY_DIR}/generated")

# --- text: FreeType + label font, shared by navview and the iOS app ---------
# Map labels are drawn from a FreeType SDF glyph atlas (src/glyph_atlas.h). FreeType
# is built from source so desktop and iOS (cross-compiled) use the same version.
if(NAV_BUILD_VIEWER OR IOS)
  include(FetchContent)
  foreach(_ft_opt FT_DISABLE_ZLIB FT_DISABLE_BZIP2 FT_DISABLE_PNG FT_DISABLE_HARFBUZZ FT_DISABLE_BROTLI)
    set(${_ft_opt} ON CACHE BOOL "" FORCE)
  endforeach()
  FetchContent_Declare(freetype
    GIT_REPOSITORY https://github.com/freetype/freetype.git
    GIT_TAG VER-2-13-3
    GIT_SHALLOW TRUE)
  FetchContent_MakeAvailable(freetype)

  # Label font: DejaVu Sans (free license, full Latin incl. accents), fetched once.
  set(VRST_FONT_DIR "${CMAKE_BINARY_DIR}/fonts")
  set(VRST_FONT_FILES "${VRST_FONT_DIR}/DejaVuSans.ttf" "${VRST_FONT_DIR}/DejaVuSans-Bold.ttf")
  if(NOT EXISTS "${VRST_FONT_DIR}/DejaVuSans-Bold.ttf")
    set(_dejavu "${CMAKE_BINARY_DIR}/dejavu-fonts-ttf-2.37.tar.bz2")
    file(DOWNLOAD
      https://github.com/dejavu-fonts/dejavu-fonts/releases/download/version_2_37/dejavu-fonts-ttf-2.37.tar.bz2
      "${_dejavu}" STATUS _dl)
    list(GET _dl 0 _dl_code)
    if(NOT _dl_code EQUAL 0)
      message(FATAL_ERROR "DejaVu font download failed: ${_dl}")
    endif()
    file(ARCHIVE_EXTRACT INPUT "${_dejavu}" DESTINATION "${CMAKE_BINARY_DIR}/dejavu"
      PATTERNS "*/ttf/DejaVuSans.ttf" "*/ttf/DejaVuSans-Bold.ttf")
    file(GLOB _ttf "${CMAKE_BINARY_DIR}/dejavu/*/ttf/*.ttf")
    file(COPY ${_ttf} DESTINATION "${VRST_FONT_DIR}")
  endif()
endif()

# --- earcut: polygon triangulation (header only) -----------------------------
# Cuts the map.dat polygons into triangles: once at import (osm2nav), and again for
# the simplified outlines of zoomed-out tiles (navview, iOS app).
include(FetchContent)
FetchContent_Declare(earcut
  GIT_REPOSITORY https://github.com/mapbox/earcut.hpp.git
  GIT_TAG v2.2.4
  GIT_SHALLOW TRUE
  SOURCE_SUBDIR include)  # header only: no CMakeLists there, so its tests are not added
FetchContent_MakeAvailable(earcut)

# --- iOS: only the app (cmake -G Xcode -DCMAKE_SYSTEM_NAME=iOS, see ios/README.md) ---
if(IOS)
  add_subdirectory(ios)
  return()
endif()

find_package(Threads REQUIRED)
find_package(ZLIB REQUIRED)

# --- osm2nav ---------------------------------------------------------------
add_executable(osm2nav src/osm2nav.cpp)
target_include_directories(osm2nav SYSTEM PRIVATE
  third_party/libosmium/include
  third_party/protozero/include
  ${earcut_SOURCE_DIR}/include)
target_link_libraries(osm2nav PRIVATE vrst_types ZLIB::ZLIB Threads::Threads)
nav_warnings(osm2nav)

# --- tile_test: GoogleTest tests of the tiled map (tests/) --------------------
option(NAV_BUILD_TESTS "Build the GoogleTest tests (fetches GoogleTest)" ON)
if(NAV_BUILD_TESTS)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
    GIT_SHALLOW TRUE)
  FetchContent_MakeAvailable(googletest)
  add_executable(tile_test tests/tile_test.cpp)
  target_include_directories(tile_test PRIVATE src)
  target_include_directories(tile_test SYSTEM PRIVATE ${earcut_SOURCE_DIR}/include)
  target_link_libraries(tile_test PRIVATE vrst_types GTest::gtest_main Threads::Threads)
  nav_warnings(tile_test)
endif()

# --- navcheck --------------------------------------------------------------
add_executable(navcheck src/navcheck.cpp)
target_link_libraries(navcheck PRIVATE vrst_types)
nav_warnings(navcheck)

# --- navview ---------------------------------------------------------------
if(NAV_BUILD_VIEWER)
  include(FetchContent)
  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
    GIT_SHALLOW TRUE)
  FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.93.0
    GIT_SHALLOW TRUE)
  FetchContent_MakeAvailable(glfw imgui)

  # Pre-generated GL 3.3 core loader (see third_party/glad/README).
  add_library(glad STATIC third_party/glad/src/glad.c)
  target_include_directories(glad PUBLIC third_party/glad/include)

  # Dear ImGui has no CMake build: core + GLFW/OpenGL3 backends.
  add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp)
  target_include_directories(imgui SYSTEM PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
  target_link_libraries(imgui PUBLIC glfw)

  find_package(OpenGL REQUIRED)
  add_executable(navview src/navview.cpp)
  target_include_directories(navview SYSTEM PRIVATE ${earcut_SOURCE_DIR}/include)
  target_link_libraries(navview PRIVATE vrst_types glad imgui freetype OpenGL::GL Threads::Threads)
  target_compile_definitions(navview PRIVATE VRST_FONT_DIR="${VRST_FONT_DIR}")
  nav_warnings(navview)
endif()

# --- docs ------------------------------------------------------------------
# `cmake --build <build> --target docs` -> <build>/docs/html/index.html
find_package(Doxygen)
if(DOXYGEN_FOUND)
  set(DOXYGEN_PROJECT_NAME verst)
  set(DOXYGEN_PROJECT_BRIEF "Offline OSM map rendering and car navigation")
  set(DOXYGEN_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/docs")
  set(DOXYGEN_USE_MDFILE_AS_MAINPAGE "${CMAKE_SOURCE_DIR}/README.md")
  set(DOXYGEN_STRIP_FROM_PATH "${CMAKE_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/generated")
  set(DOXYGEN_FILE_PATTERNS *.h *.cpp *.mm *.md)
  set(DOXYGEN_EXTENSION_MAPPING mm=Objective-C)
  # Types and fields live in anonymous namespaces and private sections too.
  set(DOXYGEN_EXTRACT_PRIVATE YES)
  set(DOXYGEN_EXTRACT_STATIC YES)
  set(DOXYGEN_EXTRACT_ANON_NSPACES YES)
  set(DOXYGEN_JAVADOC_AUTOBRIEF YES)
  set(DOXYGEN_SOURCE_BROWSER YES)
  set(DOXYGEN_GENERATE_TREEVIEW YES)
  # Everything is documented: warn on any new undocumented entity or parameter.
  set(DOXYGEN_WARN_NO_PARAMDOC YES)
  set(DOXYGEN_WARN_IF_UNDOCUMENTED YES)
  set(DOXYGEN_WARN_IF_INCOMPLETE_DOC YES)
  doxygen_add_docs(docs
    "${CMAKE_SOURCE_DIR}/README.md"
    "${CMAKE_SOURCE_DIR}/docs"
    "${CMAKE_SOURCE_DIR}/src"
    "${CMAKE_SOURCE_DIR}/ios"
    "${CMAKE_SOURCE_DIR}/tests"
    "${CMAKE_BINARY_DIR}/generated/vrst_types.h"
    COMMENT "Generating Doxygen docs")
else()
  message(STATUS "Doxygen not found; docs target disabled")
endif()

# --- tests -----------------------------------------------------------------
enable_testing()
if(EXISTS "${NAV_TEST_PBF}")
  set(_out "${CMAKE_BINARY_DIR}/test_out")
  add_test(NAME convert COMMAND osm2nav "${NAV_TEST_PBF}" "${_out}")
  add_test(NAME check COMMAND navcheck "${_out}")
  if(NAV_BUILD_TESTS)
    add_test(NAME tiles COMMAND tile_test)
    set_tests_properties(tiles PROPERTIES ENVIRONMENT "VRST_TEST_DIR=${_out}" FIXTURES_REQUIRED nav_data)
  endif()
  set_tests_properties(convert PROPERTIES FIXTURES_SETUP nav_data)
  set_tests_properties(check PROPERTIES FIXTURES_REQUIRED nav_data)
else()
  message(STATUS "NAV_TEST_PBF not found (${NAV_TEST_PBF}); tests disabled")
endif()
