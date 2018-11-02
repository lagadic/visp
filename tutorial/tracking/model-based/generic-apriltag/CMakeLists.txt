project(tutorial-tracking-mb-aprilTag-init)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_core visp_sensor visp_gui visp_vision visp_mbt visp_detection)

# set the list of source files
set(tutorial_cpp
  tutorial-mb-generic-tracker-apriltag-live-webcam.cpp
  tutorial-mb-generic-tracker-apriltag-live-realsense2.cpp
)

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})

  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

if(UNIX)
  if(VISP_HAVE_OGRE)
    visp_set_source_file_compile_flag(tutorial-mb-generic-tracker-apriltag-live-webcam.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
    visp_set_source_file_compile_flag(tutorial-mb-generic-tracker-apriltag-live-realsense2.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  endif()
else()
  visp_set_source_file_compile_flag(tutorial-mb-generic-tracker-apriltag-live-realsense2.cpp /wd4244)
endif()
