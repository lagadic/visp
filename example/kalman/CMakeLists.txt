cmake_minimum_required(VERSION 3.5)

project(example-kalman)

find_package(VISP REQUIRED visp_core visp_gui)

set(example_cpp)

list(APPEND example_cpp ukf-linear-example.cpp)
list(APPEND example_cpp ukf-nonlinear-example.cpp)
list(APPEND example_cpp ukf-nonlinear-complex-example.cpp)

foreach(cpp ${example_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "examples")
  endif()
endforeach()

visp_add_test(ukf-linear-example ukf-linear-example ${OPTION_TO_DESACTIVE_DISPLAY})
visp_add_test(ukf-nonlinear-example ukf-nonlinear-example ${OPTION_TO_DESACTIVE_DISPLAY})
visp_add_test(ukf-nonlinear-complex-example ukf-nonlinear-complex-example ${OPTION_TO_DESACTIVE_DISPLAY})
