cmake_minimum_required(VERSION 3.5)

project(color_segmentation)

find_package(VISP REQUIRED visp_core visp_sensor visp_io visp_gui)

# set the list of source files
set(tutorial_cpp
  tutorial-hsv-range-tuner.cpp
  tutorial-hsv-segmentation.cpp
  tutorial-hsv-segmentation-basic.cpp
  tutorial-hsv-segmentation-pcl.cpp
  tutorial-hsv-segmentation-pcl-viewer.cpp
)

file(GLOB tutorial_data "*.jpg")

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

# Copy the data files to the same location than the target
foreach(data ${tutorial_data})
  visp_copy_data(tutorial-hsv-segmentation-basic.cpp ${data})
endforeach()
