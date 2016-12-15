project(tutorial-computer-vision)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_core visp_blob visp_vision visp_io visp_gui visp_detection)

set(tutorial_cpp
  tutorial-homography-from-points.cpp
  tutorial-pose-from-points-image.cpp
  tutorial-pose-from-points-tracking.cpp
  tutorial-pose-from-qrcode-image.cpp)

set(tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/square.pgm")
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/bar-code.pgm")

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

# Copy the data files to the same location than the target
foreach(data ${tutorial_data})
  visp_copy_data(tutorial-pose-from-points-image.cpp ${data})
endforeach()
