project(tutorial-robot)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED)
if(VISP_FOUND)
  include(${VISP_USE_FILE})
endif(VISP_FOUND)

# build the examples
add_executable(tutorial-pioneer-robot tutorial-pioneer-robot.cpp)
add_executable(tutorial-simu-pioneer tutorial-simu-pioneer.cpp)
add_executable(tutorial-simu-pioneer-pan tutorial-simu-pioneer-pan.cpp)
