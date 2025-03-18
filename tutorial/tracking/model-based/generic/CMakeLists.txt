cmake_minimum_required(VERSION 3.10)

project(tutorial-tracking-mb)

find_package(VISP REQUIRED visp_core visp_mbt visp_io visp_gui visp_sensor visp_vision)

# set the list of source files
set(tutorial_cpp
  tutorial-mb-generic-tracker.cpp
  tutorial-mb-generic-tracker-live.cpp
  tutorial-mb-generic-tracker-full.cpp
  tutorial-mb-generic-tracker-save.cpp
  tutorial-mb-generic-tracker-read.cpp
)

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})

  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

# Copy the data files
visp_copy_dir(tutorial-mb-generic-tracker.cpp "${CMAKE_CURRENT_SOURCE_DIR}" model)
