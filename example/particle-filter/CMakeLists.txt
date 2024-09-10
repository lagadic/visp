cmake_minimum_required(VERSION 3.5)

project(example-kalman)

find_package(VISP REQUIRED visp_core visp_gui)

set(example_cpp)

list(APPEND example_cpp pf-nonlinear-example.cpp)

foreach(cpp ${example_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "examples")
  endif()
endforeach()

visp_add_test(pf-nonlinear-example-monothread pf-nonlinear-example --nb-particles 500  --ampli-max-X 0.02 --ampli-max-Y 0.02 --ampli-max-Z 0.01 --nb-steps-main 300 --max-distance-likelihood 10 --nb-threads 1 --seed 4224 ${OPTION_TO_DESACTIVE_DISPLAY})
visp_add_test(pf-nonlinear-example-multithread pf-nonlinear-example --nb-particles 500  --ampli-max-X 0.02 --ampli-max-Y 0.02 --ampli-max-Z 0.01 --nb-steps-main 300 --max-distance-likelihood 10 --nb-threads -1 --seed 4224 ${OPTION_TO_DESACTIVE_DISPLAY})
