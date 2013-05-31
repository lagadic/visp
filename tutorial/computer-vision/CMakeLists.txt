project(tutorial-computer-vision)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED)
if(VISP_FOUND)
  include(${VISP_USE_FILE})
endif(VISP_FOUND)

# build the examples
add_executable(tutorial-pose-from-points-image tutorial-pose-from-points-image.cpp)
add_executable(tutorial-pose-from-points-tracking tutorial-pose-from-points-tracking.cpp)

# copy the data
get_target_property(target_location tutorial-pose-from-points-image LOCATION)
get_filename_component(target_location "${target_location}" PATH)
set(data "${CMAKE_CURRENT_SOURCE_DIR}/square.pgm" )
add_custom_command(
  TARGET tutorial-pose-from-points-image
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy "${data}" "${target_location}"
)
