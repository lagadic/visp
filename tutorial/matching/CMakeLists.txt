project(tutorial-matching-keypoint)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED)
if(VISP_FOUND)
  include(${VISP_USE_FILE})
endif(VISP_FOUND)

# build the examples
add_executable(tutorial-matching-surf tutorial-matching-surf.cpp)
add_executable(tutorial-matching-surf-homography tutorial-matching-surf-homography.cpp)

# copy the data
get_target_property(target_location tutorial-matching-surf LOCATION)
get_filename_component(target_location "${target_location}" PATH)
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/video-postcard.mpeg" )
foreach(data ${data2copy})
  add_custom_command(
    TARGET tutorial-matching-surf
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "${data}" "${target_location}"
  )
endforeach()
