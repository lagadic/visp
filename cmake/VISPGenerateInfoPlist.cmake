set(VISP_APPLE_BUNDLE_NAME "ViSP")
set(VISP_APPLE_BUNDLE_ID "org.visp")

if(IOS)
  if (APPLE_FRAMEWORK AND BUILD_SHARED_LIBS)
    configure_file("${VISP_SOURCE_DIR}/platforms/ios/Info.Dynamic.plist.in"
                   "${CMAKE_BINARY_DIR}/ios/Info.plist")
  else()
    configure_file("${VISP_SOURCE_DIR}/platforms/ios/Info.plist.in"
                   "${CMAKE_BINARY_DIR}/ios/Info.plist")
  endif()
elseif(APPLE)
  configure_file("${VISP_SOURCE_DIR}/platforms/osx/Info.plist.in"
                 "${CMAKE_BINARY_DIR}/osx/Info.plist")
endif()
