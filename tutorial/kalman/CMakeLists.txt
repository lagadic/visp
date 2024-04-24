cmake_minimum_required(VERSION 3.5)

project(tutorial-kalman)

find_package(VISP REQUIRED visp_core visp_gui)

set(tutorial_cpp)

list(APPEND tutorial_cpp tutorial-ukf-linear-example.cpp)
list(APPEND tutorial_cpp tutorial-ukf-nonlinear-example.cpp)
list(APPEND tutorial_cpp tutorial-ukf-nonlinear-complex-example.cpp)

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()
