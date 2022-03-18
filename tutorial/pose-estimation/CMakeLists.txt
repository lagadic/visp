project(tutorial-pose-estimation)

cmake_minimum_required(VERSION 3.0)

find_package(VISP REQUIRED visp_core visp_io visp_vision)

set(tutorial_cpp)

list(APPEND tutorial_cpp tutorial-planar-object-pose.cpp)

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

# Copy the data files to the same location than the target
visp_copy_dir(tutorial-planar-object-pose.cpp "${CMAKE_CURRENT_SOURCE_DIR}" data)
