cmake_minimum_required(VERSION 3.10)

project(tutorial-gui-pcl-visualizer)

find_package(VISP REQUIRED visp_core visp_gui visp_io)

# set the list of source files
set(tutorial_cpp
  tutorial-display-pcl.cpp
)

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp} ClassUsingDisplayPCL.h ClassUsingDisplayPCL.cpp)
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()
