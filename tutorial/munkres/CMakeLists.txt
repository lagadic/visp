project(tutorial-munkres)

cmake_minimum_required(VERSION 3.0)

find_package(VISP REQUIRED visp_core visp_gui)

set(tutorial_cpp)

if(VISP_CXX_STANDARD GREATER_EQUAL VISP_CXX_STANDARD_17)
  list(APPEND tutorial_cpp tutorial-munkres-assignment.cpp)
endif()

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()
