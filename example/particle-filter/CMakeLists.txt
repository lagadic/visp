cmake_minimum_required(VERSION 3.10)

project(example-kalman)

find_package(VISP REQUIRED visp_core visp_gui)

set(example_cpp)

list(APPEND example_cpp pf-nonlinear-example.cpp)
list(APPEND example_cpp pf-nonlinear-complex-example.cpp)

foreach(cpp ${example_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "examples")
  endif()
endforeach()

visp_add_test(pf-nonlinear-example-monothread pf-nonlinear-example ${OPTION_TO_DESACTIVE_DISPLAY} --dt 3 --stdev-range 5 --stdev-elev-angle 0.5 --stdev-aircraft-vel 0.2>  --gt-X0 -500 --gt-Y0 1000 --gt-vX0 10 --gt-vY0 5 --max-distance-likelihood 50 -N 500 --seed 4224 --nb-threads 1 --ampli-max-X 20 --ampli-max-Y 200 --ampli-max-vX 1 --ampli-max-vY 0.5)
visp_add_test(pf-nonlinear-example-multithread pf-nonlinear-example ${OPTION_TO_DESACTIVE_DISPLAY} --dt 3 --stdev-range 5 --stdev-elev-angle 0.5 --stdev-aircraft-vel 0.2>  --gt-X0 -500 --gt-Y0 1000 --gt-vX0 10 --gt-vY0 5 --max-distance-likelihood 50 -N 500 --seed 4224 --nb-threads -1 --ampli-max-X 20 --ampli-max-Y 200 --ampli-max-vX 1 --ampli-max-vY 0.5)
visp_add_test(pf-nonlinear-complex-example-monothread pf-nonlinear-complex-example  ${OPTION_TO_DESACTIVE_DISPLAY} --max-distance-likelihood 0.5 --ampli-max-X 0.25 --ampli-max-Y 0.25 --ampli-max-theta 0.1 -N 500 --nb-threads 1)
visp_add_test(pf-nonlinear-complex-example-multithread pf-nonlinear-complex-example  ${OPTION_TO_DESACTIVE_DISPLAY} --max-distance-likelihood 0.5 --ampli-max-X 0.25 --ampli-max-Y 0.25 --ampli-max-theta 0.1 -N 500 --nb-threads -1)
