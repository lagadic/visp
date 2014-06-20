project(tutorial-tracking-blob)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED)
if(VISP_FOUND)
  include(${VISP_USE_FILE})
endif(VISP_FOUND)

# build the examples
add_executable(tutorial-blob-auto-tracker tutorial-blob-auto-tracker.cpp)
add_executable(tutorial-blob-tracker-live-firewire tutorial-blob-tracker-live-firewire.cpp)
add_executable(tutorial-blob-tracker-live-v4l2 tutorial-blob-tracker-live-v4l2.cpp)

# copy the data
get_target_property(target_location tutorial-blob-auto-tracker LOCATION)
get_filename_component(target_location "${target_location}" PATH)
set(data "${CMAKE_CURRENT_SOURCE_DIR}/target.pgm" )
add_custom_command(
  TARGET tutorial-blob-auto-tracker
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy "${data}" "${target_location}"
)
