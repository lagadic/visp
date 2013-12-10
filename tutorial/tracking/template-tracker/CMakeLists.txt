project(tutorial-tracking-template)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED)
if(VISP_FOUND)
  include(${VISP_USE_FILE})
endif(VISP_FOUND)

# build the examples
add_executable(tutorial-template-tracker tutorial-template-tracker.cpp)

# copy the data
get_target_property(target_location tutorial-template-tracker LOCATION)
get_filename_component(target_location "${target_location}" PATH)
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/pattern-template-tracker.pgm" )
foreach(data ${data2copy})
  add_custom_command(
    TARGET tutorial-template-tracker
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "${data}" "${target_location}"
  )
endforeach()
