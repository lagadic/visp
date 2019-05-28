project(tutorial-image)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_core visp_io visp_gui)

set(tutorial_cpp
  tutorial-image-converter.cpp
  tutorial-image-manipulation.cpp
  tutorial-image-reader.cpp
  tutorial-image-viewer.cpp
  tutorial-undistort.cpp
  tutorial-viewer.cpp
  tutorial-image-filter.cpp
  tutorial-image-display.cpp
  tutorial-image-display-scaled-manu.cpp
  tutorial-image-display-scaled-auto.cpp
  tutorial-draw-line.cpp
  tutorial-draw-rectangle.cpp
  tutorial-draw-point.cpp
  tutorial-draw-cross.cpp
  tutorial-draw-text.cpp
  tutorial-export-image.cpp
  tutorial-event-keyboard.cpp)

list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/monkey.bmp" )
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/monkey.ppm" )
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/monkey.pgm" )
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/monkey.jpeg" )
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/monkey.png" )
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/chessboard.pgm" )
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/camera.xml" )

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

# Copy the data files to the same location than the target
foreach(data ${tutorial_data})
  visp_copy_data(tutorial-viewer.cpp ${data})
endforeach()
