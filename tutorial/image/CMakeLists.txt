project(tutorial-image)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED)
if(VISP_FOUND)
  include(${VISP_USE_FILE})
endif(VISP_FOUND)

# build the examples
add_executable(tutorial-image-converter tutorial-image-converter.cpp)
add_executable(tutorial-image-manipulation tutorial-image-manipulation.cpp)
add_executable(tutorial-image-reader tutorial-image-reader.cpp)
add_executable(tutorial-image-viewer tutorial-image-viewer.cpp)
add_executable(tutorial-undistort tutorial-undistort.cpp)
add_executable(tutorial-viewer tutorial-viewer.cpp)
add_executable(tutorial-image-filter tutorial-image-filter.cpp)

# copy the data
get_target_property(target_location tutorial-image-reader LOCATION)
get_filename_component(target_location "${target_location}" PATH)
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/lena.bmp" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/lena.ppm" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/lena.pgm" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/chessboard.pgm" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/camera.xml" )
foreach(data ${data2copy})
  add_custom_command(
    TARGET tutorial-image-reader
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "${data}" "${target_location}"
  )
endforeach()
