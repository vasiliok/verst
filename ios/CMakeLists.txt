# verst iPhone app: UIKit shell + OpenGL ES 3.0 through GLKView, drawing the tiled
# map with the shared vrst_core and vrst_render libraries. Configured from the
# top-level CMakeLists.txt when CMAKE_SYSTEM_NAME is iOS.
enable_language(OBJCXX)

set(VERST_MAP_DIR "" CACHE PATH "Directory with graph.dat, edges.dat, geom.dat, names.dat (and map.dat) to bundle")
set(VERST_BUNDLE_ID "com.verst.app" CACHE STRING "iOS bundle identifier")
set(VERST_IOS_TEAM "" CACHE STRING "Apple development team id; empty = unsigned (Simulator only)")

set(_map_files)
foreach(f graph.dat edges.dat geom.dat names.dat)
  if(NOT EXISTS "${VERST_MAP_DIR}/${f}")
    message(FATAL_ERROR "VERST_MAP_DIR='${VERST_MAP_DIR}' has no ${f}; run osm2nav first and pass -DVERST_MAP_DIR=<out_dir>")
  endif()
  list(APPEND _map_files "${VERST_MAP_DIR}/${f}")
endforeach()
# Drawing-only layers (water, forest, buildings, railways); without it the app draws roads only.
if(EXISTS "${VERST_MAP_DIR}/map.dat")
  list(APPEND _map_files "${VERST_MAP_DIR}/map.dat")
else()
  message(WARNING "VERST_MAP_DIR has no map.dat: the app will draw roads only")
endif()
# Bundled read-only data: <app>/map/*.dat, mapped in place like on desktop.
set_source_files_properties(${_map_files} PROPERTIES MACOSX_PACKAGE_LOCATION map)
# Label fonts (downloaded by the top-level CMakeLists): <app>/fonts/*.ttf.
set_source_files_properties(${VRST_FONT_FILES} PROPERTIES MACOSX_PACKAGE_LOCATION fonts)

add_executable(verst_ios MACOSX_BUNDLE main.mm debug_overlay.mm ${_map_files} ${VRST_FONT_FILES})
target_link_libraries(verst_ios PRIVATE vrst_render
  "-framework Foundation" "-framework CoreGraphics" "-framework UIKit" "-framework GLKit"
  "-framework QuartzCore" "-framework CoreLocation")
target_compile_options(verst_ios PRIVATE -fobjc-arc -Wall -Wextra)
set_target_properties(verst_ios PROPERTIES
  OUTPUT_NAME Verst
  MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/Info.plist.in"
  MACOSX_BUNDLE_GUI_IDENTIFIER "${VERST_BUNDLE_ID}"
  MACOSX_BUNDLE_BUNDLE_NAME Verst
  MACOSX_BUNDLE_SHORT_VERSION_STRING 0.1
  MACOSX_BUNDLE_BUNDLE_VERSION 1
  XCODE_ATTRIBUTE_PRODUCT_BUNDLE_IDENTIFIER "${VERST_BUNDLE_ID}"
  XCODE_ATTRIBUTE_IPHONEOS_DEPLOYMENT_TARGET 15.0
  XCODE_ATTRIBUTE_TARGETED_DEVICE_FAMILY 1
  XCODE_ATTRIBUTE_CLANG_ENABLE_OBJC_ARC YES)
if(VERST_IOS_TEAM)
  set_target_properties(verst_ios PROPERTIES
    XCODE_ATTRIBUTE_DEVELOPMENT_TEAM "${VERST_IOS_TEAM}"
    XCODE_ATTRIBUTE_CODE_SIGN_STYLE Automatic)
else()
  set_target_properties(verst_ios PROPERTIES
    XCODE_ATTRIBUTE_CODE_SIGNING_ALLOWED NO
    XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY "")
endif()
