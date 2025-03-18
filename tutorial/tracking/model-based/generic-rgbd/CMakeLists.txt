cmake_minimum_required(VERSION 3.10)

project(tutorial-tracking-mb-generic-rgbd)

find_package(VISP REQUIRED visp_core visp_mbt visp_io visp_gui visp_sensor)

# set the list of source files
set(tutorial_cpp
  tutorial-mb-generic-tracker-rgbd.cpp
  tutorial-mb-generic-tracker-rgbd-realsense.cpp
  tutorial-mb-generic-tracker-rgbd-structure-core.cpp
  tutorial-mb-generic-tracker-rgbd-realsense-json.cpp)

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})

  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

# Copy the data files
visp_copy_dir(tutorial-mb-generic-tracker-rgbd.cpp "${CMAKE_CURRENT_SOURCE_DIR}" data)
visp_copy_dir(tutorial-mb-generic-tracker-rgbd.cpp "${CMAKE_CURRENT_SOURCE_DIR}" model)
