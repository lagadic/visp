cmake_minimum_required(VERSION 3.10)

project(tutorial-panda3d)

find_package(VISP REQUIRED visp_core visp_gui visp_ar visp_io)

# set the list of source files
set(tutorial_cpp
  tutorial-panda3d-renderer.cpp
)

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

# Copy the data folder to the same location than the target
visp_copy_dir(tutorial-panda3d-renderer "${CMAKE_CURRENT_SOURCE_DIR}" data)
