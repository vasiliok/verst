# verst iPhone app: UIKit shell + OpenGL ES 3.0 through GLKView, drawing the tiled
# map with the shared vrst_core and vrst_render libraries. Configured from the
# top-level CMakeLists.txt when CMAKE_SYSTEM_NAME is iOS.
enable_language(OBJCXX)

set(VERST_BUNDLE_ID "com.verst.app" CACHE STRING "iOS bundle identifier")
set(VERST_IOS_TEAM "" CACHE STRING "Apple development team id; empty = unsigned (Simulator only)")

# No map is bundled: the app downloads Geofabrik extracts and converts them on the phone
# (Maps screen); for development `--import-region <osm2nav folder>` copies one in.
# Label fonts (downloaded by the top-level CMakeLists): <app>/fonts/*.ttf.
set_source_files_properties(${VRST_FONT_FILES} PROPERTIES MACOSX_PACKAGE_LOCATION fonts)

# Test scenarios (src/scenario.h): <app>/scenarios/*.txt, offered in the debug menu.
file(GLOB _scenarios "${CMAKE_CURRENT_SOURCE_DIR}/scenarios/*.txt")
set_source_files_properties(${_scenarios} PROPERTIES MACOSX_PACKAGE_LOCATION scenarios)

# Asset catalog with the app icon (one 1024 px image; Xcode makes the other sizes).
set(_assets "${CMAKE_CURRENT_SOURCE_DIR}/Assets.xcassets")
set_source_files_properties(${_assets} PROPERTIES MACOSX_PACKAGE_LOCATION Resources)

add_executable(verst_ios MACOSX_BUNDLE main.mm catalog_controller.mm debug_console.mm debug_overlay.mm download_manager.mm maps_controller.mm region_store.mm root_controller.mm ${VRST_FONT_FILES} ${_scenarios}
  ${_assets})
target_link_libraries(verst_ios PRIVATE vrst_render vrst_convert
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
  XCODE_ATTRIBUTE_ASSETCATALOG_COMPILER_APPICON_NAME AppIcon
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
