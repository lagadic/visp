set(VISP_APPLE_BUNDLE_NAME "ViSP")
set(VISP_APPLE_BUNDLE_ID "org.visp")

if(IOS)
  configure_file("${VISP_SOURCE_DIR}/platforms/ios/Info.plist.in"
                 "${CMAKE_BINARY_DIR}/ios/Info.plist")
elseif(APPLE)
  configure_file("${VISP_SOURCE_DIR}/platforms/osx/Info.plist.in"
                 "${CMAKE_BINARY_DIR}/osx/Info.plist")
endif()
