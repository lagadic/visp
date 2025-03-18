cmake_minimum_required(VERSION 3.10)

project(tutorial-tracking-mb-generic-rgbd-blender)

find_package(VISP REQUIRED visp_core visp_mbt visp_io visp_gui)

# set the list of source files
set(tutorial_cpp
  tutorial-mb-generic-tracker-rgbd-blender.cpp)

list(APPEND tutorial_data_dir "data")
list(APPEND tutorial_data_dir "model")

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})

  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

# Copy the data files to the same location than the target
foreach(data_dir ${tutorial_data_dir})
  visp_copy_dir(tutorial-mb-generic-tracker-rgbd-blender.cpp "${CMAKE_CURRENT_SOURCE_DIR}" ${data_dir})
endforeach()
