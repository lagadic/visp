project(tutorial-apriltag)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_core visp_detection visp_io visp_gui visp_sensor)

set(tutorial_cpp
  tutorial-apriltag-detector.cpp
  tutorial-apriltag-detector-live.cpp
)

set(tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/AprilTag.pgm")

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

# Copy the data files to the same location than the target
foreach(data ${tutorial_data})
  visp_copy_data(tutorial-apriltag-detector.cpp ${data})
endforeach()

if(VISP_HAVE_FLYCAPTURE)
  # Add specific build flag to turn off warnings coming from FlyCapture 3rd party
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-ignored-qualifiers")
endif()
if(VISP_HAVE_REALSENSE2)
  # Add specific build flag to turn off warnings coming from librealsense 3rd party
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "/wd4244")
endif()

if(CXX_FLAGS_MUTE_WARNINGS)
  visp_set_source_file_compile_flag(tutorial-apriltag-detector-live.cpp ${CXX_FLAGS_MUTE_WARNINGS})
endif()
