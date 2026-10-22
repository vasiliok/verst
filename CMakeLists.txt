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

find_package(Threads REQUIRED)
find_package(ZLIB REQUIRED)

# --- osm2nav ---------------------------------------------------------------
add_executable(osm2nav src/osm2nav.cpp)
target_include_directories(osm2nav SYSTEM PRIVATE
  third_party/libosmium/include
  third_party/protozero/include)
target_link_libraries(osm2nav PRIVATE vrst_types ZLIB::ZLIB Threads::Threads)
nav_warnings(osm2nav)

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
  target_link_libraries(navview PRIVATE vrst_types glad imgui OpenGL::GL)
  nav_warnings(navview)
endif()

# --- tests -----------------------------------------------------------------
enable_testing()
if(EXISTS "${NAV_TEST_PBF}")
  set(_out "${CMAKE_BINARY_DIR}/test_out")
  add_test(NAME convert COMMAND osm2nav "${NAV_TEST_PBF}" "${_out}")
  add_test(NAME check COMMAND navcheck "${_out}")
  set_tests_properties(convert PROPERTIES FIXTURES_SETUP nav_data)
  set_tests_properties(check PROPERTIES FIXTURES_REQUIRED nav_data)
else()
  message(STATUS "NAV_TEST_PBF not found (${NAV_TEST_PBF}); tests disabled")
endif()
