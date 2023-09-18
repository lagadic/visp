project(tutorial-calibration)

cmake_minimum_required(VERSION 3.5)

find_package(VISP REQUIRED visp_robot visp_vision visp_io visp_gui)

set(tutorial_cpp
  tutorial-chessboard-pose.cpp
  tutorial-franka-acquire-calib-data.cpp
  tutorial-hand-eye-calibration.cpp
  tutorial-universal-robots-acquire-calib-data.cpp
)

file(GLOB tutorial_data "*.yaml" "*.py" "camera.xml" "*.png")

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

if(VISP_HAVE_REALSENSE2)
  # Add specific build flag to turn off warnings coming from librealsense 3rd party and its dependencies
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "/wd4244")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "/wd4267")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-deprecated-copy")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-deprecated-declarations")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-ignored-qualifiers")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-overloaded-virtual")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-pessimizing-move")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-reorder")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-sign-compare")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-unused-function")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-unused-parameter")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-unqualified-std-cast-call")

  visp_set_source_file_compile_flag(tutorial-franka-acquire-calib-data.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  visp_set_source_file_compile_flag(tutorial-universal-robots-acquire-calib-data.cpp ${CXX_FLAGS_MUTE_WARNINGS})
endif()

# Copy the data files to the same location than the target
foreach(data ${tutorial_data})
  visp_copy_data(tutorial-hand-eye-calibration.cpp ${data})
endforeach()
