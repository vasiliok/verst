cmake_minimum_required(VERSION 3.20)
project(verst LANGUAGES C CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# compile_commands.json in the build dir, for clang-tidy and editors.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(NAV_BUILD_VIEWER "Build navview (fetches GLFW)" ON)
set(NAV_TEST_PBF "${CMAKE_SOURCE_DIR}/data/andorra.osm.pbf" CACHE FILEPATH "PBF used by the CTest")

function(nav_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor -Wold-style-cast
                                             -Woverloaded-virtual -Wnull-dereference -Wformat=2 -Wimplicit-fallthrough
                                             -Wcast-align)
  endif()
endfunction()

# --- sanitizers: -DNAV_SANITIZE=address (ASan + UBSan) or thread (TSan) ---------
# Applied to every target, third-party code included, so the runtime sees all of it.
set(NAV_SANITIZE "OFF" CACHE STRING "Sanitizer build: OFF, address or thread")
set_property(CACHE NAV_SANITIZE PROPERTY STRINGS OFF address thread)
if(NAV_SANITIZE STREQUAL "address")
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=undefined)
  add_link_options(-fsanitize=address,undefined)
elseif(NAV_SANITIZE STREQUAL "thread")
  add_compile_options(-fsanitize=thread -fno-omit-frame-pointer)
  add_link_options(-fsanitize=thread)
elseif(NOT NAV_SANITIZE STREQUAL "OFF")
  message(FATAL_ERROR "NAV_SANITIZE must be OFF, address or thread, not '${NAV_SANITIZE}'")
endif()

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
# earcut decides everything from the sign of a*b - c*d (its area()). A fused multiply-add
# rounds the two products differently, so for nearly collinear points the sign stops
# being consistent and earcut can loop forever. arm64 compilers fuse by default: on the
# iPhone some zoom-10 tiles never finished building (reproduced on the Mac with the
# phone's logged rings and a fused area()). The files that include earcut never fuse.
set(VRST_EARCUT_SOURCES src/map_tiles.cpp src/osm_map_layers.cpp tests/earcut_test.cpp)
set_source_files_properties(${VRST_EARCUT_SOURCES} PROPERTIES COMPILE_OPTIONS -ffp-contract=off)

find_package(Threads REQUIRED)

# --- vrst_core: map provider (nav files, tiles), map frame, GPS tracks (no GL) ----
# Shared by every tool, the desktop viewer and the iPhone app.
add_library(vrst_core STATIC
  src/mapped_file.cpp
  src/nav_files.cpp
  src/map_scene.cpp
  src/map_tiles.cpp
  src/activity_recorder.cpp
  src/map_provider.cpp
  src/nav_map_provider.cpp
  src/gpx_track.cpp
  src/scenario.cpp
  src/session_log.cpp)
target_include_directories(vrst_core PUBLIC src)
target_include_directories(vrst_core SYSTEM PRIVATE ${earcut_SOURCE_DIR}/include)
target_link_libraries(vrst_core PUBLIC vrst_types Threads::Threads)
nav_warnings(vrst_core)

# --- vrst_render: OpenGL map drawing and labels (desktop GL 3.3, iOS GLES 3.0) ---
if(NAV_BUILD_VIEWER OR IOS)
  add_library(vrst_render STATIC
    src/glyph_atlas.cpp
    src/map_labels.cpp
    src/map_renderer.cpp
    src/tile_renderer.cpp)
  target_link_libraries(vrst_render PUBLIC vrst_core freetype)
  if(IOS)
    target_link_libraries(vrst_render PUBLIC "-framework OpenGLES")
  else()
    # Pre-generated GL 3.3 core loader (see third_party/glad/README).
    add_library(glad STATIC third_party/glad/src/glad.c)
    target_include_directories(glad PUBLIC third_party/glad/include)
    target_link_libraries(vrst_render PUBLIC glad)
  endif()
  nav_warnings(vrst_render)
endif()

# --- iOS: only the app (cmake -G Xcode -DCMAKE_SYSTEM_NAME=iOS, see ios/README.md) ---
if(IOS)
  add_subdirectory(ios)
  return()
endif()

find_package(ZLIB REQUIRED)

# --- osm2nav ---------------------------------------------------------------
add_executable(osm2nav src/osm2nav.cpp src/osm_map_layers.cpp)
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
  add_executable(tile_test tests/tile_test.cpp tests/scenario_test.cpp tests/activity_recorder_test.cpp tests/geometry_test.cpp tests/earcut_test.cpp tests/map_provider_test.cpp)
  target_link_libraries(tile_test PRIVATE vrst_core GTest::gtest_main)
  target_include_directories(tile_test SYSTEM PRIVATE ${earcut_SOURCE_DIR}/include)
  nav_warnings(tile_test)
endif()

# --- navcheck --------------------------------------------------------------
add_executable(navcheck src/navcheck.cpp)
target_link_libraries(navcheck PRIVATE vrst_core)
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
  target_link_libraries(navview PRIVATE vrst_render imgui OpenGL::GL)
  target_compile_definitions(navview PRIVATE VRST_FONT_DIR="${VRST_FONT_DIR}")
  nav_warnings(navview)
endif()

# --- docs ------------------------------------------------------------------
# `cmake --build <build> --target docs` -> <build>/docs/html/index.html
find_package(Doxygen)
if(DOXYGEN_FOUND)
  # All settings are in the repo's Doxyfile (also usable standalone: `doxygen` in the repo
  # root). The build only redirects the output into the build dir and adds the generated
  # scalar type header.
  set(_doxyfile "${CMAKE_BINARY_DIR}/Doxyfile.build")
  file(WRITE "${_doxyfile}"
    "@INCLUDE          = ${CMAKE_SOURCE_DIR}/Doxyfile\n"
    "OUTPUT_DIRECTORY  = ${CMAKE_BINARY_DIR}/docs\n"
    "WARN_LOGFILE      = ${CMAKE_BINARY_DIR}/docs/warnings.log\n"
    "INPUT            += ${CMAKE_BINARY_DIR}/generated/vrst_types.h\n"
    "STRIP_FROM_PATH  += ${CMAKE_BINARY_DIR}/generated\n")
  add_custom_target(docs
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/docs"
    COMMAND Doxygen::doxygen "${_doxyfile}"
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    COMMENT "Generating Doxygen docs (warnings: ${CMAKE_BINARY_DIR}/docs/warnings.log)"
    VERBATIM)
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
