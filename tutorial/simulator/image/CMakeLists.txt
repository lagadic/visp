project(tutorial-simulation-image)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED)
if(VISP_FOUND)
  include(${VISP_USE_FILE})
endif(VISP_FOUND)

# build the examples
add_executable(tutorial-image-simulator tutorial-image-simulator.cpp)

# copy the data
get_target_property(target_location tutorial-image-simulator LOCATION)
get_filename_component(target_location "${target_location}" PATH)
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/target_square.pgm" )
foreach(data ${data2copy})
  add_custom_command(
    TARGET tutorial-image-simulator
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "${data}" "${target_location}"
  )
endforeach()
