project(tutorial-tracking-dnn)

cmake_minimum_required(VERSION 3.5)

find_package(VISP REQUIRED visp_core visp_detection visp_dnn_tracker visp_io visp_gui visp_sensor)

# set the list of source files
set(tutorial_cpp
  tutorial-megapose-live-single-object-tracking.cpp
)

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

# Copy the data files
visp_copy_dir(tutorial-megapose-live-single-object-tracking.cpp "${CMAKE_CURRENT_SOURCE_DIR}" data)
