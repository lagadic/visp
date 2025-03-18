cmake_minimum_required(VERSION 3.10)

project(tutorial-grabber)

find_package(VISP REQUIRED visp_core visp_sensor visp_io visp_gui)

# set the list of source files
set(tutorial_cpp
  tutorial-grabber-1394.cpp
  tutorial-grabber-1394-writer.cpp
  tutorial-grabber-basler-pylon.cpp
  tutorial-grabber-bebop2.cpp
  tutorial-grabber-CMU1394.cpp
  tutorial-grabber-flycapture.cpp
  tutorial-grabber-ids-ueye.cpp
  tutorial-grabber-multiple-realsense.cpp
  tutorial-grabber-opencv.cpp
  tutorial-grabber-opencv-threaded.cpp
  tutorial-grabber-rgbd-D435-structurecore.cpp
  tutorial-grabber-realsense.cpp
  tutorial-grabber-realsense-T265.cpp
  tutorial-grabber-structure-core.cpp
  tutorial-grabber-v4l2.cpp
  tutorial-grabber-v4l2-threaded.cpp
  tutorial-video-reader.cpp
  tutorial-video-recorder.cpp
)

set(tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/video.mp4")

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

# Copy the data files to the same location than the target
foreach(data ${tutorial_data})
  visp_copy_data(tutorial-video-reader.cpp ${data})
endforeach()
