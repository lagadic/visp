project(tutorial-tracking-mb-edges)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_core visp_mbt visp_io visp_gui)

# set the list of source files
set(tutorial_cpp
  tutorial-mb-edge-tracker.cpp)

list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox.mpg" )
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox.xml" )
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox.cao" )
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox.wrl" )
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox.init" )
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox.ppm" )

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

# Copy the data files to the same location than the target
foreach(data ${tutorial_data})
  visp_copy_data(tutorial-mb-edge-tracker.cpp ${data})
endforeach()
