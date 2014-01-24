project(tutorial-grabber)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED)

# set the list of source files
set(tutorial_cpp
  tutorial-grabber-1394.cpp
  tutorial-grabber-1394-writer.cpp
  tutorial-grabber-CMU1394.cpp
  tutorial-grabber-opencv.cpp
  tutorial-grabber-opencv-bis.cpp
  tutorial-grabber-video.cpp
  tutorial-grabber-v4l2.cpp)

set(tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/video.mpg")

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
endforeach()

# Copy the data files to the same location than the target
foreach(data ${tutorial_data})
  visp_copy_data(tutorial-grabber-video.cpp ${data})
endforeach()
