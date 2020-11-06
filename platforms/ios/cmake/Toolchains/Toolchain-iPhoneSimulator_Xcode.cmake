message(STATUS "Setting up iPhoneSimulator toolchain for IOS_ARCH='${IOS_ARCH}'")
set(IPHONESIMULATOR TRUE)
include(${CMAKE_CURRENT_LIST_DIR}/common-ios-toolchain.cmake)

# To help opencv2.framework detection
execute_process(COMMAND xcrun --sdk iphonesimulator --show-sdk-platform-path
                   OUTPUT_VARIABLE _IPHONESIMULATOR_SDK_PLATFORM_PATH OUTPUT_STRIP_TRAILING_WHITESPACE)
list(APPEND CMAKE_FIND_ROOT_PATH "${_IPHONESIMULATOR_SDK_PLATFORM_PATH}/Developer")

message(STATUS "iPhoneSimulator toolchain loaded")
