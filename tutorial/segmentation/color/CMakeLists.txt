cmake_minimum_required(VERSION 3.5)

project(color_segmentation)

find_package(VISP REQUIRED visp_core visp_sensor visp_io visp_gui)

# set the list of source files
set(tutorial_cpp
  tutorial-hsv-segmentation.cpp
  tutorial-hsv-segmentation-pcl.cpp
  tutorial-hsv-segmentation-pcl-viewer.cpp
  tutorial-hsv-tuner.cpp
)

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()
