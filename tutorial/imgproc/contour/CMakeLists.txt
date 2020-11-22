project(tutorial-contour)

cmake_minimum_required(VERSION 2.8.12.2)

find_package(VISP REQUIRED visp_core visp_io visp_gui visp_imgproc)

# set the list of source files
set(tutorial_cpp
  tutorial-contour.cpp)

list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/grid36-03.pgm")
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/Contours_tree.pgm")

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

# Copy the data files to the same location than the target
foreach(data ${tutorial_data})
  visp_copy_data(tutorial-contour.cpp ${data})
endforeach()
