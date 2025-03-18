cmake_minimum_required(VERSION 3.10)

project(tutorial-computer-vision)

find_package(VISP REQUIRED visp_core visp_blob visp_vision visp_io visp_gui visp_detection visp_sensor)

set(tutorial_cpp
  tutorial-homography-from-points.cpp
  tutorial-pose-from-planar-object.cpp
  tutorial-pose-from-points-image.cpp
  tutorial-pose-from-points-live.cpp
  tutorial-pose-from-points-realsense-T265.cpp
  tutorial-pose-from-qrcode-image.cpp)

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp} pose_helper.h pose_helper.cpp)
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

# Copy the data folder to the same location than the target
visp_copy_dir(tutorial-pose-from-planar-object.cpp "${CMAKE_CURRENT_SOURCE_DIR}" data)
