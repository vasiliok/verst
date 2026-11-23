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

# Test scenarios (src/session/scenario.h): <app>/scenarios/*.txt, offered in the debug menu.
file(GLOB _scenarios "${CMAKE_CURRENT_SOURCE_DIR}/scenarios/*.txt")
set_source_files_properties(${_scenarios} PROPERTIES MACOSX_PACKAGE_LOCATION scenarios)

# Asset catalog with the app icon (one 1024 px image; Xcode makes the other sizes).
set(_assets "${CMAKE_CURRENT_SOURCE_DIR}/Assets.xcassets")
set_source_files_properties(${_assets} PROPERTIES MACOSX_PACKAGE_LOCATION Resources)

add_executable(verst_ios MACOSX_BUNDLE main.mm catalog_controller.mm debug_console.mm debug_overlay.mm download_manager.mm maps_controller.mm region_store.mm root_controller.mm search_controller.mm pin_card.mm ${VRST_FONT_FILES} ${_scenarios}
  ${_assets})
target_link_libraries(verst_ios PRIVATE vrst_render vrst_convert
  "-framework Foundation" "-framework CoreGraphics" "-framework UIKit" "-framework GLKit"
  "-framework QuartzCore" "-framework CoreLocation" "-framework UniformTypeIdentifiers")
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

# Share extension: "Verst" in the iOS share sheet (Google Maps: Share -> Verst). Reads the
# point of a shared link or coordinates with src/search/geo_link.cpp and opens the app with
# verst://share (see ios/share/share_controller.h). Embedded in the app's PlugIns.
add_executable(verst_share MACOSX_BUNDLE share/share_controller.mm ${CMAKE_SOURCE_DIR}/src/search/geo_link.cpp)
target_include_directories(verst_share PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(verst_share PRIVATE vrst_types
  "-framework Foundation" "-framework UIKit" "-framework UniformTypeIdentifiers")
target_compile_options(verst_share PRIVATE -fobjc-arc -Wall -Wextra)
set_target_properties(verst_share PROPERTIES
  OUTPUT_NAME VerstShare
  BUNDLE_EXTENSION appex
  XCODE_PRODUCT_TYPE com.apple.product-type.app-extension
  MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/share/Info.plist.in"
  MACOSX_BUNDLE_GUI_IDENTIFIER "${VERST_BUNDLE_ID}.share"
  MACOSX_BUNDLE_BUNDLE_NAME VerstShare
  MACOSX_BUNDLE_SHORT_VERSION_STRING 0.1
  MACOSX_BUNDLE_BUNDLE_VERSION 1
  XCODE_ATTRIBUTE_PRODUCT_BUNDLE_IDENTIFIER "${VERST_BUNDLE_ID}.share"
  XCODE_ATTRIBUTE_IPHONEOS_DEPLOYMENT_TARGET 15.0
  XCODE_ATTRIBUTE_TARGETED_DEVICE_FAMILY 1
  XCODE_ATTRIBUTE_APPLICATION_EXTENSION_API_ONLY YES
  XCODE_ATTRIBUTE_CLANG_ENABLE_OBJC_ARC YES)
if(VERST_IOS_TEAM)
  set_target_properties(verst_share PROPERTIES
    XCODE_ATTRIBUTE_DEVELOPMENT_TEAM "${VERST_IOS_TEAM}"
    XCODE_ATTRIBUTE_CODE_SIGN_STYLE Automatic)
else()
  set_target_properties(verst_share PROPERTIES
    XCODE_ATTRIBUTE_CODE_SIGNING_ALLOWED NO
    XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY "")
endif()
add_dependencies(verst_ios verst_share)
set_target_properties(verst_ios PROPERTIES
  XCODE_EMBED_APP_EXTENSIONS verst_share
  XCODE_EMBED_APP_EXTENSIONS_REMOVE_HEADERS_ON_COPY YES
  XCODE_EMBED_APP_EXTENSIONS_CODE_SIGN_ON_COPY YES)
