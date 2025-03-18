cmake_minimum_required(VERSION 3.10)

project(tutorial-render-based-tracking)

find_package(VISP REQUIRED visp_core visp_rbt visp_io visp_gui visp_sensor)

# set the list of source files
set(tutorial_cpp
  tutorial-rbt-sequence.cpp
  tutorial-rbt-realsense.cpp)

list(APPEND tutorial_data_dir "data")

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})

  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

# Copy the data files to the same location than the target
foreach(data_dir ${tutorial_data_dir})
  visp_copy_dir(tutorial-rbt-sequence.cpp "${CMAKE_CURRENT_SOURCE_DIR}" ${data_dir})
  visp_copy_dir(tutorial-rbt-realsense.cpp "${CMAKE_CURRENT_SOURCE_DIR}" ${data_dir})
endforeach()
