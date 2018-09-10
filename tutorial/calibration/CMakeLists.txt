project(tutorial-calibration)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_robot visp_vision visp_io visp_gui)

set(tutorial_cpp
  tutorial-chessboard-pose.cpp
  tutorial-franka-acquire-calib-data.cpp
  tutorial-hand-eye.cpp
)

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

if(VISP_HAVE_REALSENSE2)
    visp_set_source_file_compile_flag(tutorial-franka-acquire-calib-data.cpp -Wno-reorder -Wno-unused-function -Wno-sign-compare -Wno-pessimizing-move -Wno-unused-parameter)
endif()

