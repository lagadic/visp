project(tutorial-grabber)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_core visp_sensor visp_io visp_gui)

# set the list of source files
set(tutorial_cpp
  tutorial-grabber-1394.cpp
  tutorial-grabber-1394-writer.cpp
  tutorial-grabber-basler-pylon.cpp
  tutorial-grabber-CMU1394.cpp
  tutorial-grabber-flycapture.cpp
  tutorial-grabber-opencv.cpp
  tutorial-grabber-opencv-threaded.cpp
  tutorial-grabber-realsense.cpp
  tutorial-grabber-v4l2.cpp
  tutorial-grabber-v4l2-threaded.cpp
  tutorial-video-reader.cpp
  tutorial-video-recorder.cpp
)

set(tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/video.mpg")

if(VISP_HAVE_FLYCAPTURE)
  # Add specific build flag to turn off warnings coming from PointGrey flycapture 3rd party
  visp_set_source_file_compile_flag(tutorial-grabber-flycapture.cpp -Wno-unknown-pragmas -Wno-ignored-qualifiers)
endif()
if(VISP_HAVE_PYLON)
  visp_set_source_file_compile_flag(tutorial-grabber-basler-pylon.cpp -Wno-unknown-pragmas -Wno-overloaded-virtual -Wno-unused-parameter)
endif()

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

