cmake_minimum_required(VERSION 3.5)

project(example-kalman)

find_package(VISP REQUIRED visp_core visp_gui)

set(example_cpp)

list(APPEND example_cpp example-pf-nonlinear.cpp)

foreach(cpp ${example_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "examples")
  endif()
endforeach()

visp_add_test(example-pf-nonlinear example-pf-nonlinear ${OPTION_TO_DESACTIVE_DISPLAY})
