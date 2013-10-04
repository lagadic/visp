project(tutorial-grabber)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED)
if(VISP_FOUND)
  include(${VISP_USE_FILE})
endif(VISP_FOUND)

# build the examples
add_executable(tutorial-grabber-1394 tutorial-grabber-1394.cpp)
add_executable(tutorial-grabber-1394-writer tutorial-grabber-1394-writer.cpp)
add_executable(tutorial-grabber-CMU1394 tutorial-grabber-CMU1394.cpp)
add_executable(tutorial-grabber-opencv tutorial-grabber-opencv.cpp)
add_executable(tutorial-grabber-opencv-bis tutorial-grabber-opencv-bis.cpp)
add_executable(tutorial-grabber-video tutorial-grabber-video.cpp)
add_executable(tutorial-grabber-v4l2 tutorial-grabber-v4l2.cpp)

# copy the data
get_target_property(target_location tutorial-grabber-video LOCATION)
get_filename_component(target_location "${target_location}" PATH)
set(data "${CMAKE_CURRENT_SOURCE_DIR}/video.mpg" )
add_custom_command(
  TARGET tutorial-grabber-video
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy "${data}" "${target_location}"
)
