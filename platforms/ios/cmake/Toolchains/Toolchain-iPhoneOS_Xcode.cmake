message(STATUS "Setting up iPhoneOS toolchain for IOS_ARCH='${IOS_ARCH}'")
set(IPHONEOS TRUE)
include(${CMAKE_CURRENT_LIST_DIR}/common-ios-toolchain.cmake)

# To help opencv2.framework detection
execute_process(COMMAND xcrun --sdk iphoneos --show-sdk-platform-path
                OUTPUT_VARIABLE _IPHONEOS_SDK_PLATFORM_PATH OUTPUT_STRIP_TRAILING_WHITESPACE)
list(APPEND CMAKE_FIND_ROOT_PATH "${_IPHONEOS_SDK_PLATFORM_PATH}/Developer")

message(STATUS "iPhoneOS toolchain loaded")
