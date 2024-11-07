cmake_minimum_required(VERSION 3.5)

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

if(VISP_HAVE_ZBAR)
  # Add specific build flag to turn off warnings coming from zbar 3rd party
  visp_set_source_file_compile_flag(tutorial-pose-from-qrcode-image.cpp -Wno-unused-parameter -Wno-deprecated-declarations)
endif()

if(VISP_HAVE_FLYCAPTURE)
  # Add specific build flag to turn off warnings coming from FlyCapture 3rd party
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-ignored-qualifiers")
endif()
if(VISP_HAVE_REALSENSE2)
  # Add specific build flag to turn off warnings coming from librealsense 3rd party
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "/wd4244")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "/wd4267")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-deprecated-copy")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-deprecated-declarations")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-ignored-qualifiers")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-overloaded-virtual")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-reorder")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-sign-compare")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-unused-function")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-unqualified-std-cast-call")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-missing-field-initializers")
endif()

if(CXX_FLAGS_MUTE_WARNINGS)
  # Add specific build flag to turn off warnings
  visp_set_source_file_compile_flag(tutorial-pose-from-points-live.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  visp_set_source_file_compile_flag(tutorial-pose-from-points-realsense-T265.cpp ${CXX_FLAGS_MUTE_WARNINGS})
endif()

# Copy the data folder to the same location than the target
visp_copy_dir(tutorial-pose-from-planar-object.cpp "${CMAKE_CURRENT_SOURCE_DIR}" data)
