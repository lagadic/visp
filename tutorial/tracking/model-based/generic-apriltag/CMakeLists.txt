cmake_minimum_required(VERSION 3.10)

project(tutorial-tracking-mb-aprilTag-init)

find_package(VISP REQUIRED visp_core visp_sensor visp_gui visp_vision visp_mbt visp_detection)

# set the list of source files
set(tutorial_cpp
  tutorial-mb-generic-tracker-apriltag-webcam.cpp
  tutorial-mb-generic-tracker-apriltag-rs2.cpp
)

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})

  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()
