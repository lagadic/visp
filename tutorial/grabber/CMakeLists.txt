project(tutorial-grabber)

cmake_minimum_required(VERSION 3.5)

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

if(VISP_HAVE_FLYCAPTURE)
  # Add specific build flag to turn off warnings coming from PointGrey flycapture 3rd party
  visp_set_source_file_compile_flag(tutorial-grabber-flycapture.cpp -Wno-unknown-pragmas -Wno-ignored-qualifiers)
endif()
if(VISP_HAVE_PYLON)
  visp_set_source_file_compile_flag(tutorial-grabber-basler-pylon.cpp -Wno-unknown-pragmas -Wno-overloaded-virtual -Wno-unused-parameter -Wno-unused-variable)
endif()

if(VISP_HAVE_REALSENSE)
  # Add specific build flag to turn off warnings coming from librealsense 3rd party
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_RS "/wd4244")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_RS "-Wno-overloaded-virtual")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_RS "-Wno-pessimizing-move")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_RS "-Wno-unused-function")
endif()
if(VISP_HAVE_REALSENSE2)
  # Add specific build flag to turn off warnings coming from librealsense 3rd party
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_RS "/wd4244")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_RS "/wd4267")

  list(APPEND CXX_FLAGS_MUTE_WARNINGS_RS "-Wno-deprecated-copy")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_RS "-Wno-deprecated-declarations")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_RS "-Wno-ignored-qualifiers")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_RS "-Wno-overloaded-virtual")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_RS "-Wno-pessimizing-move")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_RS "-Wno-reorder")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_RS "-Wno-sign-compare")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_RS "-Wno-unused-function")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_RS "-Wno-unused-parameter")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_RS "-Wno-unqualified-std-cast-call")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_RS "-Wno-missing-field-initializers")
endif()
if(VISP_HAVE_OCCIPITAL_STRUCTURE)
  # Add specific build flag to turn off warnings coming from Occipital SDK 3rd party
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_SC "/wd4267")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_SC "-Wno-ignored-qualifiers")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_SC "-Wno-maybe-uninitialized")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_SC "-Wno-overloaded-virtual")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_SC "-Wno-reorder")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_SC "-Wno-sign-compare")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_SC "-Wno-unused-function")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_SC "-Wno-unused-parameter")
endif()

if(CXX_FLAGS_MUTE_WARNINGS_RS)
  # Add specific build flag to turn off warnings
  visp_set_source_file_compile_flag(tutorial-grabber-realsense.cpp ${CXX_FLAGS_MUTE_WARNINGS_RS})
  visp_set_source_file_compile_flag(tutorial-grabber-realsense-T265.cpp ${CXX_FLAGS_MUTE_WARNINGS_RS})
  visp_set_source_file_compile_flag(tutorial-grabber-multiple-realsense.cpp ${CXX_FLAGS_MUTE_WARNINGS_RS})
endif()

if(CXX_FLAGS_MUTE_WARNINGS_SC)
  visp_set_source_file_compile_flag(tutorial-grabber-structure-core.cpp ${CXX_FLAGS_MUTE_WARNINGS_SC})
endif()

if(CXX_FLAGS_MUTE_WARNINGS_RS OR CXX_FLAGS_MUTE_WARNINGS_SC)
  visp_set_source_file_compile_flag(tutorial-grabber-rgbd-D435-structurecore.cpp ${CXX_FLAGS_MUTE_WARNINGS_RS} ${CXX_FLAGS_MUTE_WARNINGS_SC})
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
