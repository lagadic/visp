cmake_minimum_required(VERSION 3.5)

project(tutorial-meandrift)

find_package(VISP REQUIRED visp_core visp_gui)

set(tutorial_cpp)

list(APPEND tutorial_cpp tutorial-meandrift.cpp)

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()
