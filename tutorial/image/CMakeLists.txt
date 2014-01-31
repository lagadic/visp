project(tutorial-image)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED)

set(tutorial_cpp
  tutorial-image-converter.cpp
  tutorial-image-manipulation.cpp
  tutorial-image-reader.cpp
  tutorial-image-viewer.cpp
  tutorial-undistort.cpp
  tutorial-viewer.cpp
  tutorial-image-filter.cpp)

list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/lena.bmp" )
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/lena.ppm" )
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/lena.pgm" )
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/lena.jpeg" )
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/lena.png" )
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/chessboard.pgm" )
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/camera.xml" )

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
endforeach()

# Copy the data files to the same location than the target
foreach(data ${tutorial_data})
  visp_copy_data(tutorial-image-reader.cpp ${data})
endforeach()
