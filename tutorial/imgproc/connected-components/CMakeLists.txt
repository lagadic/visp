cmake_minimum_required(VERSION 3.10)

project(tutorial-connected-components)

find_package(VISP REQUIRED visp_core visp_io visp_gui visp_imgproc)

# set the list of source files
set(tutorial_cpp
  tutorial-connected-components.cpp)

list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/img.pgm")

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

# Copy the data files to the same location than the target
foreach(data ${tutorial_data})
  visp_copy_data(tutorial-connected-components.cpp ${data})
endforeach()
